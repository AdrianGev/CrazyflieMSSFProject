import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from typing import List, Tuple, Optional
import os

from ml_planning.data_generator import TrainingExample, examples_to_arrays
from ml_planning.grid_world import GridWorld


# pytorch dataset for pathfinding training examples
class PathfindingDataset(Dataset):
    
    def __init__(self, features: np.ndarray, targets: np.ndarray):
        self.features = torch.FloatTensor(features)
        self.targets = torch.FloatTensor(targets)
        
    def __len__(self):
        return len(self.features)
    
    def __getitem__(self, idx):
        return self.features[idx], self.targets[idx]


# neural network for ranking pathfinding nodes
# flow of the whole thing:
# - input: node position, goal position, local patch
# - hidden layers with relu activation (like abs value but negatives go to 0)
# - output: predicted cost-to-go (single value)
class NeuralRankerNet(nn.Module):
    
    def __init__(self, input_dim: int, hidden_dims: List[int] = [128, 64, 32]):
        super().__init__()
        
        layers = []
        prev_dim = input_dim
        
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, hidden_dim))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(0.1))
            prev_dim = hidden_dim
        
        # output layer
        layers.append(nn.Linear(prev_dim, 1))
        
        self.network = nn.Sequential(*layers)
        
    def forward(self, x):
        # forward pass
        return self.network(x).squeeze(-1)


# wrapper for neural ranker model with training and inference stuff
class NeuralRanker:
    
    def __init__(self, input_dim: int, hidden_dims: List[int] = [128, 64, 32],
                 device: Optional[str] = None):
        if device is None:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = torch.device(device)
        
        self.model = NeuralRankerNet(input_dim, hidden_dims).to(self.device)
        self.input_dim = input_dim
        
        print(f"neural ranker initialized on {self.device}")
        print(f"model parameters: {sum(p.numel() for p in self.model.parameters()):,}")
        
    def train(self, 
              train_examples: List[TrainingExample],
              val_examples: Optional[List[TrainingExample]] = None,
              epochs: int = 50,
              batch_size: int = 256,
              learning_rate: float = 0.001,
              verbose: bool = True,
              export_csv: bool = False,
              export_dir: str = 'export_data'):
        # train the neural ranker
        
        # convert to arrays
        train_features, train_targets = examples_to_arrays(train_examples)
        
        # create datasets
        train_dataset = PathfindingDataset(train_features, train_targets)
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        
        val_loader = None
        if val_examples:
            val_features, val_targets = examples_to_arrays(val_examples)
            val_dataset = PathfindingDataset(val_features, val_targets)
            val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
        
        # setup training
        criterion = nn.MSELoss()
        optimizer = optim.Adam(self.model.parameters(), lr=learning_rate)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', 
                                                         factor=0.5, patience=5)
        
        best_val_loss = float('inf')
        
        # setup csv export if requested
        training_history = []
        if export_csv:
            os.makedirs(export_dir, exist_ok=True)
        
        for epoch in range(epochs):
            # training phase
            self.model.train()
            train_loss = 0.0
            train_batches = 0
            
            for features, targets in train_loader:
                features = features.to(self.device)
                targets = targets.to(self.device)
                
                optimizer.zero_grad()
                predictions = self.model(features)
                loss = criterion(predictions, targets)
                loss.backward()
                optimizer.step()
                
                train_loss += loss.item()
                train_batches += 1
            
            avg_train_loss = train_loss / train_batches
            
            # validation phase
            if val_loader:
                self.model.eval()
                val_loss = 0.0
                val_batches = 0
                
                with torch.no_grad():
                    for features, targets in val_loader:
                        features = features.to(self.device)
                        targets = targets.to(self.device)
                        
                        predictions = self.model(features)
                        loss = criterion(predictions, targets)
                        
                        val_loss += loss.item()
                        val_batches += 1
                
                avg_val_loss = val_loss / val_batches
                scheduler.step(avg_val_loss)
                
                if avg_val_loss < best_val_loss:
                    best_val_loss = avg_val_loss
                
                # record training history
                if export_csv:
                    training_history.append({
                        'epoch': epoch + 1,
                        'train_loss': avg_train_loss,
                        'val_loss': avg_val_loss,
                        'best_val_loss': best_val_loss
                    })
                
                if verbose and (epoch + 1) % 5 == 0:
                    print(f"epoch {epoch+1}/{epochs} - train loss: {avg_train_loss:.4f}, "
                          f"val loss: {avg_val_loss:.4f}, best val: {best_val_loss:.4f}")
            else:
                # record training history (no validation)
                if export_csv:
                    training_history.append({
                        'epoch': epoch + 1,
                        'train_loss': avg_train_loss,
                        'val_loss': None,
                        'best_val_loss': None
                    })
                
                if verbose and (epoch + 1) % 5 == 0:
                    print(f"epoch {epoch+1}/{epochs} - train loss: {avg_train_loss:.4f}")
        
        # export training history to csv
        if export_csv and training_history:
            import csv
            csv_file = os.path.join(export_dir, 'training_history.csv')
            
            with open(csv_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['epoch', 'train_loss', 'val_loss', 'best_val_loss'])
                writer.writeheader()
                writer.writerows(training_history)
            
            print(f"training history exported to {csv_file}")
        
        print(f"training complete. best validation loss: {best_val_loss:.4f}")
    
    def predict(self, features: np.ndarray) -> np.ndarray:
        # predict cost-to-go for features
        self.model.eval()
        with torch.no_grad():
            features_tensor = torch.FloatTensor(features).to(self.device)
            predictions = self.model(features_tensor)
            return predictions.cpu().numpy()
    
    def score_node(self, x: int, y: int, goal_x: int, goal_y: int, 
                   world: GridWorld, patch_size: int = 5) -> float:
        # score a single node for use in a* search
        
        # build feature vector
        dx = goal_x - x
        dy = goal_y - y
        dist = abs(dx) + abs(dy)
        
        local_patch = world.get_local_patch(x, y, patch_size)
        
        features = np.zeros(self.input_dim, dtype=np.float32)
        features[0] = x
        features[1] = y
        features[2] = goal_x
        features[3] = goal_y
        features[4] = dx
        features[5] = dy
        features[6] = dist
        features[7:] = local_patch.flatten()
        
        # predict
        features = features.reshape(1, -1)
        prediction = self.predict(features)[0]
        
        return float(prediction)
    
    def save(self, filepath: str):
        # save model to file
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'input_dim': self.input_dim,
        }, filepath)
        print(f"model saved to {filepath}")
    
    def load(self, filepath: str):
        # load model from file
        checkpoint = torch.load(filepath, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval()
        print(f"model loaded from {filepath}")
    
    def create_from_file(filepath: str, hidden_dims: List[int] = [128, 64, 32],
                        device: Optional[str] = None) -> 'NeuralRanker':
        # create neural ranker from saved file
        checkpoint = torch.load(filepath, map_location='cpu')
        input_dim = checkpoint['input_dim']
        
        ranker = NeuralRanker(input_dim, hidden_dims, device)
        ranker.load(filepath)
        
        return ranker
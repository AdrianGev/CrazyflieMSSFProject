import cv2

def main():
    for idx in range(0, 6):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            continue
        ok, frame = cap.read()
        if not ok:
            cap.release()
            continue

        cv2.imshow(f"cam {idx}  (press any key)", frame)
        k = cv2.waitKey(0)
        cv2.destroyWindow(f"cam {idx}  (press any key)")
        cap.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
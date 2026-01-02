import cv2
import numpy as np
import os

CAM_INDEX = 0
WARP_W, WARP_H = 640, 480
pts = []


def order4(p):
    p = np.array(p, dtype=np.float32)
    s = p.sum(axis=1)
    d = np.diff(p, axis=1).reshape(-1)
    tl = p[np.argmin(s)]
    br = p[np.argmax(s)]
    tr = p[np.argmin(d)]
    bl = p[np.argmax(d)]
    return np.array([tl, tr, br, bl], dtype=np.float32)


def mouse(event, x, y, flags, param):
    global pts
    if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
        pts.append([x, y])


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Could not open camera")
        return 1

    cv2.namedWindow("calibrate")
    cv2.setMouseCallback("calibrate", mouse)

    H = None

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        vis = frame.copy()
        for i, p in enumerate(pts):
            cv2.circle(vis, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)
            cv2.putText(vis, str(i + 1), (int(p[0]) + 8, int(p[1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.putText(
            vis,
            "Click 4 FLOOR corners | Enter=save | c=clear | q=quit",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        if len(pts) == 4 and H is None:
            src = order4(pts)
            dst = np.array([[0, 0], [WARP_W - 1, 0], [WARP_W - 1, WARP_H - 1], [0, WARP_H - 1]], dtype=np.float32)
            H = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(frame, H, (WARP_W, WARP_H))
            cv2.imshow("warped_preview", warped)

        cv2.imshow("calibrate", vis)
        k = cv2.waitKey(1) & 0xFF

        if k in (ord('q'), 27):
            break
        if k == ord('c'):
            pts.clear()
            H = None
            try:
                cv2.destroyWindow("warped_preview")
            except Exception:
                pass
        if k in (13, 10):  # Enter
            if H is None:
                print("Need 4 points first")
            else:
                np.save("H.npy", H)
                print("Saved H.npy")
                break

    cap.release()
    cv2.destroyAllWindows()
    return 0 if os.path.exists("H.npy") else 1


if __name__ == "__main__":
    raise SystemExit(main())

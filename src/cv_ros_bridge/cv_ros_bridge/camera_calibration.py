#!/usr/bin/env python3
import glob

import cv2
import numpy as np
import yaml

def main():
    board_size = (11, 7)
    objpts = []
    imgpts = []
    square_size = 16

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpt = np.zeros((board_size[1] * board_size[0], 3), np.float32)
    objpt[:, :2] = (
        np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2) * square_size
    )

    ext = "*.png"
    images = glob.glob(f"/home/xtal/ros2_ws/pictures/calibration/kinect2/{ext}")
    if not images:
        print("No images found...")
        exit()

    images_used = []
    image_size = None

    cv2.namedWindow("corners", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)

    for fn in images:
        img = cv2.imread(fn)
        if img is None:
            print(f"Could not load image: {fn}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])

        ret, corners = cv2.findChessboardCorners(gray, board_size, None)

        if not ret:
            print(f"Error on image:{fn}")
        else:
            objpts.append(objpt)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpts.append(corners2)
            images_used.append(fn)
            cv2.drawChessboardCorners(img, board_size, corners2, ret)
            cv2.imshow("corners", img)
            cv2.waitKey(0)
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpts, imgpts, image_size, None, None
    )
    print(K)
    print(D)

    camera_parameters = {
        "rms": float(rms),
        "K": K.tolist(),
        "D": D.tolist(),
        "rvecs": [rvec.tolist() for rvec in rvecs],
        "tvecs": [tvec.tolist() for tvec in tvecs],
        "image_size": list(image_size),
        "board_size": list(board_size),
        "square_size": square_size,
    }

    with open("camera_calibration/kinect2.yaml", "w") as f:
        yaml.dump(camera_parameters, f)

if __name__ == "__main__":
    main()
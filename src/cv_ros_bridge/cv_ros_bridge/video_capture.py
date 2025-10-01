#!/usr/bin/env python3
import cv2
import numpy as np

def main():
    cv2.namedWindow("Camera Capture", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit()
    
    print("Frame width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("Frame height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("FPS:", cap.get(cv2.CAP_PROP_FPS))
    print("Frame count:", cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print("Position (ms):", cap.get(cv2.CAP_PROP_POS_MSEC))
    print("Position (frames):", cap.get(cv2.CAP_PROP_POS_FRAMES))
    print("Autofocus:", cap.get(cv2.CAP_PROP_AUTOFOCUS))
    
    print("\nPress 'q' to quit, 's' to save frame")
    
    while True:
        ret, frame = cap.read()   
        cv2.imshow('Camera Capture', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('captured_frame.jpg', frame)
            print("Frame saved as 'captured_frame.jpg'")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
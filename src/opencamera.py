import cv2
import numpy as np

def get_screen_resolution():
    """Returns the screen resolution as (width, height)."""
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()  
        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()
        root.destroy()
        return width, height
    except:
        return 1920, 1080  

def calculate_new_dimensions(frame_width, frame_height, screen_width, screen_height):
    """Calculates new dimensions to fit the screen while maintaining aspect ratio."""
    frame_aspect = frame_width / frame_height
    screen_aspect = screen_width / screen_height

    if frame_aspect > screen_aspect:
        new_width = screen_width
        new_height = int(screen_width / frame_aspect)
    else:
        new_height = screen_height
        new_width = int(screen_height * frame_aspect)

    return new_width, new_height

def main():
    screen_width, screen_height = get_screen_resolution()
    print(f"Screen resolution: {screen_width}x{screen_height}")

    cap = cv2.VideoCapture(0)  
    if not cap.isOpened():
        print("Cannot open camera")
        return
    
    desired_width = 1280
    desired_height = 720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

    window_name = "Camera Fullscreen"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture image")
            break

        frame_height, frame_width = frame.shape[:2]

        new_width, new_height = calculate_new_dimensions(frame_width, frame_height, screen_width, screen_height)

        resized_frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)

        background = np.zeros((screen_height, screen_width, 3), dtype=np.uint8)

        x_offset = (screen_width - new_width) // 2
        y_offset = (screen_height - new_height) // 2

        background[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = resized_frame

        cv2.imshow(window_name, background)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

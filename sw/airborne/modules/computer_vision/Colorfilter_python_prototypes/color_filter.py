
import cv2
import numpy as np
import os

# --- 1. THE FOLDER FIX ---
current_folder = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(current_folder, '83878058.jpg')
result_path = os.path.join(current_folder, 'resultaat_filter.jpg')

# 2. Load the picture
img = cv2.imread(image_path)
img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

if img is None:
    print(f"ERROR: Could not find the picture! I looked here: {image_path}")
else:
    # 3. Convert the picture to YUV
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)

    # 4. Define the color ORANGE in YUV (Y, U, V)
    lower_orange = np.array((105, 52, 180), dtype=np.uint8)
    upper_orange = np.array((205, 140, 255), dtype=np.uint8)

    # 5. Make the black/white mask
    mask = cv2.inRange(img_yuv, lower_orange, upper_orange)

    # 6. Split the screen in half and count the white (orange) pixels
    height, width = mask.shape
    middle = width // 2
    pixels_left = cv2.countNonZero(mask[:, :middle])
    pixels_right = cv2.countNonZero(mask[:, middle:])

    print(f"Orange pixels left: {pixels_left}")
    print(f"Orange pixels right: {pixels_right}")

    # Logic test for navigation later:
    if pixels_left > pixels_right and pixels_left > 100:
        print("Action: Pole is on the left! Steer Right.")
    elif pixels_right > pixels_left and pixels_right > 100:
        print("Action: Pole is on the right! Steer Left.")
    else:
        print("Action: Path is clear, fly straight.")

    # 7. SAVE THE RESULT IN YOUR FOLDER
    cv2.imwrite(result_path, mask)
    print(f"SUCCESS! The filtered picture is saved in as: resultaat_filter.jpg")
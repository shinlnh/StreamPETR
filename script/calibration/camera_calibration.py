import cv2
import numpy as np
import glob
import os
import matplotlib.pyplot as plt

# Get all image file paths
def get_image_paths(image_dir, ext='*.png'):
    return glob.glob(os.path.join(image_dir, ext))

# Generate a 3D array of object points based on the known grid size and physical spacing
def generate_object_points(grid_size, scale=300):
    objp = np.zeros((np.prod(grid_size), 3), np.float32)  
    objp[:, :2] = np.indices(grid_size).T.reshape(-1, 2)  # Fill in x, y coordinates
    objp *= scale  # Scale the coordinates by mm distance between circles
    return objp

# Convert image to grayscale and apply binary thresholding
def preprocess_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
    _, binary = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)  
    return gray, binary

# Find contours in the binary image and draw circular blobs at their centroids
def extract_blob_centers(binary_img):
    contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # print(f"Number of blobs detected: {len(contours)}")

    mask = np.zeros_like(binary_img)  # Create a blank mask
    for contour in contours:
        M = cv2.moments(contour)  
        if M["m00"] != 0:  # Avoid division by zero
            cx = int(M["m10"] / M["m00"])  # X centroid
            cy = int(M["m01"] / M["m00"])  # Y centroid
            cv2.circle(mask, (cx, cy), 15, 255, -1)  # Draw circle at centroid
    return mask

# Invert the blob center image
def find_circles_grid(image, grid_size):
    blob_centers_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    inverted = cv2.bitwise_not(blob_centers_bgr)  # Invert colors (black -> white)
    # plt.imshow(inverted), plt.title("Pattern Found"), plt.axis("off"), plt.show()
    found, centers = cv2.findCirclesGrid(inverted, grid_size, flags=cv2.CALIB_CB_SYMMETRIC_GRID)
    return found, centers, inverted

# Run camera calibration using object and image points
def calibrate_camera(objpoints, imgpoints, image_size):
    return cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

# Save calibration results
def save_calibration_results(path, camera_matrix, dist_coeffs, rvecs, tvecs):
    with open(path, "w") as f:
        f.write("Camera Calibration Results\n")
        f.write("==========================\n\n")

        f.write("Camera Matrix:\n")
        np.savetxt(f, camera_matrix, fmt="%.6f")
        f.write("\nDistortion Coefficients:\n")
        np.savetxt(f, dist_coeffs, fmt="%.6f")
        f.write("\n")

        # Optionally include rotation and translation vectors
        # f.write("Rotation Vectors (rvecs):\n")
        # for idx, rvec in enumerate(rvecs):
        #     f.write(f"rvec[{idx}]:\n")
        #     np.savetxt(f, rvec, fmt="%.6f")
        #     f.write("\n")

        # f.write("Translation Vectors (tvecs):\n")
        # for idx, tvec in enumerate(tvecs):
        #     f.write(f"tvec[{idx}]:\n")
        #     np.savetxt(f, tvec, fmt="%.6f")
        #     f.write("\n")

    print(f"Calibration complete. Parameters saved to {path}")

def main():
    # Define paths and calibration grid properties
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_dir = os.path.join(script_dir, "calibration_images")
    image_paths = get_image_paths(calibration_dir)
    circle_grid_size = (11, 9)

    # Prepare object points template for all images
    object_point_template = generate_object_points(circle_grid_size)
    objpoints = []  # 3D points in real world
    imgpoints = []  # 2D points in image plane

    # Process each calibration image
    for path in image_paths:
        image = cv2.imread(path)
        gray, binary = preprocess_image(image)
        blob_mask = extract_blob_centers(binary)
        found, centers, _ = find_circles_grid(blob_mask, circle_grid_size)

        if found:
            objpoints.append(object_point_template)
            imgpoints.append(centers)

            # Draw and optionally visualize detection
            annotated = cv2.drawChessboardCorners(image, circle_grid_size, centers, found)
            # plt.imshow(annotated), plt.title("Pattern Found"), plt.axis("off"), plt.show()
        else:
            print(f"Pattern not detected in image: {path}")

    cv2.destroyAllWindows()

    if objpoints and imgpoints:
        image_size = gray.shape[::-1] 
        ret, cam_matrix, dist_coeffs, rvecs, tvecs = calibrate_camera(objpoints, imgpoints, image_size)

        print("Camera Matrix:\n", cam_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
        current_dir = script_dir
        while True:
            # Check if "adas_service" exists in the current directory
            potential_path = os.path.join(current_dir, "adas_service")
            if os.path.isdir(potential_path):
                output_path = os.path.join(potential_path, "configs", "camera_calibration_parameters.txt")
                break

            # Move one directory up
            parent_dir = os.path.dirname(current_dir)
            if parent_dir == current_dir:  # Reached the root directory
                raise FileNotFoundError("adas_service folder not found in parent directories.")
            current_dir = parent_dir
        print("Found path:", output_path)
        save_calibration_results(output_path, cam_matrix, dist_coeffs, rvecs, tvecs)

    else:
        print("Insufficient data for calibration.")

if __name__ == "__main__":
    main()

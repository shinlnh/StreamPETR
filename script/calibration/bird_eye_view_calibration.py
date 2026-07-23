import cv2
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import messagebox
import os
import sys

# Configuration Parameters
CAMERA_POSITION = {"x": 1.3, "y": 0.0, "z": 1.3}
CARPET_WIDTH = 5.0  # Width of the carpet (meters)
CARPET_LENGTH = 60  # Length of the carpet (meters)
CAR_TO_CARPET = 2.0  # Distance from car front to carpet start (meters)
CAR_WIDTH = 2.16345  # Width of the car (meters)
HALF_CAR_LENGTH = 2.3958897590637207  # Half of car length (used in camera offset calculation)
IMAGE_EXTENSIONS = (".png", ".jpg", ".jpeg") 

all_sorted_points = []  # Global list to store sorted points for all images
fig = None       # Declare globally
ax = None
tk_root = None   # Declare globally for cleanup

def get_image_paths(folder, extensions=IMAGE_EXTENSIONS):
    return [os.path.join(folder, f) for f in os.listdir(folder) if f.lower().endswith(extensions)]

# Sort 4 corner points in top-left, top-right, bottom-left, bottom-right order
def sort_corners(pts):
    pts = np.array(pts).reshape(4, 2)
    pts = sorted(pts, key=lambda p: (p[1], p[0]))
    top = sorted(pts[:2], key=lambda p: p[0])
    bottom = sorted(pts[2:], key=lambda p: p[0])
    return [
        top[0],      # top-left
        top[1],      # top-right
        bottom[1],   # bottom-right
        bottom[0]    # bottom-left
    ]

# Save calibration result into text file 
def save_bird_eye_view_calibration_results(car_width, carpet_width, camera_to_carpet_distance,
                                           carpet_length, average_points, output_path):
    with open(output_path, "w") as f:
        f.write(f"car_width {car_width:.9f}\n")
        f.write(f"carpet_width {carpet_width:.1f}\n")
        f.write(f"camera_to_carpet_distance {camera_to_carpet_distance:.1f}\n")
        f.write(f"carpet_length {carpet_length:.1f}\n")

        for idx, label in enumerate(["top_left", "top_right", "bottom_right", "bottom_left"]):
            pt = average_points[idx]
            if pt is not None:
                f.write(f"{label}_x {pt[0]}\n")
                f.write(f"{label}_y {pt[1]}\n")
            else:
                f.write(f"{label}_x \n")
                f.write(f"{label}_y \n")

    print(f"\nCalibration results saved to: {output_path}")

# Mouse click callback function to handle point selection
def onclick(event):
    global points
    if event.inaxes != ax or len(points) >= 4:
        return 

    if event.button == 3:  # Right-click only
        x, y = int(event.xdata), int(event.ydata)
        pt_plot, = ax.plot(x, y, 'go')
        fig.canvas.draw()

        confirm = messagebox.askyesno("Confirm Point", f"Use this point?\nX: {x}, Y: {y}")
        if confirm:
            points.append((x, y))
            print(f"Point confirmed: ({x}, {y})")
        else:
            pt_plot.remove()
            fig.canvas.draw()
            print("Point rejected")

        if len(points) == 4:
            # Sort points after selection
            sorted_pts = sort_corners(points)
            all_sorted_points.append(sorted_pts)
            plt.close(fig)
            tk_root.quit()  # Exit the tkinter mainloop to continue the program

# Locate the 'adas_service' directory up the directory tree
def find_output_path(script_dir):
    current_dir = script_dir
    while True:
        potential_path = os.path.join(current_dir, "common")
        if os.path.isdir(potential_path):
            return os.path.join(potential_path, "config/sensor", "camera_calib_carla.txt")

        parent = os.path.dirname(current_dir)
        if parent == current_dir:
            raise FileNotFoundError("adas_service folder not found.")
        current_dir = parent

def main():
    global fig, ax, tk_root, points
    # Get available calibration images
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_dir = os.path.join(script_dir, "bird_eye_view_calibration_images")
    image_files = get_image_paths(calibration_dir)
    if not image_files:
        print("No image files found for calibration")
        return

    # Process each image
    for image_path in image_files:
        image_bgr = cv2.imread(image_path)
        if image_bgr is None:
            print(f"Failed to load image: {image_path}")
            continue
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        # Setup tkinter
        tk_root = tk.Tk()
        tk_root.withdraw()

        # Setup matplotlib in interactive mode
        plt.ion()  # Enable interactive mode (non-blocking)
        fig, ax = plt.subplots()
        ax.imshow(image_rgb)
        ax.set_title(f"Right-click to select 4 points of carpet's corners")
        plt.axis('off')

        # Reset points for new image
        points = []

        # Register right-click handler
        fig.canvas.mpl_connect('button_press_event', onclick)
        fig.canvas.draw()
        fig.canvas.flush_events()
        tk_root.mainloop()  # Use tkinter's mainloop to handle events

        # Clean up
        plt.ioff()  # Turn off interactive mode
        tk_root.destroy()  # Clean up tkinter

    # Calculate average points from sorted points
    if all_sorted_points:
        avg_points = []
        for i in range(4):  # For each of the 4 corners
            x_coords = [pts[i][0] for pts in all_sorted_points if len(pts) == 4]
            y_coords = [pts[i][1] for pts in all_sorted_points if len(pts) == 4]
            if x_coords and y_coords:
                avg_x = int(np.mean(x_coords))
                avg_y = int(np.mean(y_coords))
                avg_points.append([avg_x, avg_y])
            else:
                avg_points.append([0, 0])  # Default if no valid points
                print(f"The point {i} has no valid data")
        print("\nAverage sorted points across all images (top-left, top-right, bottom-right, bottom-left):")
        for i, pt in enumerate(avg_points):
            print(f"Point {i+1}: {pt}")

        # Save average corner points
        camera_to_carpet_distance = CAR_TO_CARPET + HALF_CAR_LENGTH - CAMERA_POSITION["x"] # adjust the distance to the camera coordinate     
        output_path = find_output_path(script_dir)

        # Save calibration data using average points
        save_bird_eye_view_calibration_results(
            CAR_WIDTH, CARPET_WIDTH, camera_to_carpet_distance,
            CARPET_LENGTH, avg_points, output_path
        )
    else:
        print("No points collected from any image.")

if __name__ == "__main__":
    main()
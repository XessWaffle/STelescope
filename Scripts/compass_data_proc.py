import argparse
import numpy as np
from matplotlib.patches import Ellipse
from scipy.optimize import least_squares
from fractions import Fraction

# File: compass_data_proc.py

def process_file(file_path):
    # Initialize arrays to hold split data
    column1 = []
    column2 = []
    column3 = []
    # Open the file and read line by line
    with open(file_path, 'r') as file:
        for line in file:
            # Strip newline characters and split by '|'
            parts = line.strip().split('|')
            
            # Append each part to the respective array
            parts = [int(part) for part in parts]
            if len(parts) >= 1:
                column1.append(parts[0])
            if len(parts) >= 2:
                column2.append(parts[1])
            if len(parts) >= 3:
                column3.append(parts[2])
    
    return column1, column2, column3

def normalize_data(data):
    # Calculate the average of the minimum and maximum values
    min_val = min(data)
    max_val = max(data)
    avg_min_max = (min_val + max_val) / 2
    
    # Subtract the average from each element in the array
    normalized_data = [(x - avg_min_max) for x in data]
    return normalized_data, avg_min_max

# Example usage
if __name__ == "__main__":

    # Set up argument parser
    parser = argparse.ArgumentParser(description="Process a file containing compass data.")
    parser.add_argument("file_path", type=str, help="Path to the input txt file")

    # Parse arguments
    args = parser.parse_args()

    # Process the file
    col1, col2, col3 = process_file(args.file_path)
    
    import matplotlib.pyplot as plt

    # Generate timestamps assuming each index is 500 milliseconds
    timestamps = [i * 0.5 for i in range(len(col1))]

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, col1, label="x raw data")
    plt.plot(timestamps, col2, label="y raw data")
    plt.plot(timestamps, col3, label="z raw data")

    # Add labels, title, and legend
    plt.xlabel("Time (seconds)")
    plt.ylabel("Raw Data")
    plt.title("Magnetometer Raw Data")
    plt.legend()

    # Show the plot
    plt.grid()
    plt.show()

    # Normalize each column
    col1_normalized, col1_avg = normalize_data(col1)
    col2_normalized, col2_avg = normalize_data(col2)
    col3_normalized, col3_avg = normalize_data(col3)

    # Define a function to calculate the moving average
    def moving_average(data, window_size):
        return np.convolve(data, np.ones(window_size) / window_size, mode='valid')

    # Define different window sizes for moving averages
    window_sizes = [3, 5, 10]  # Adjust the window sizes as needed

    for window_size in window_sizes:
        # Calculate moving averages for each column
        col1_moving_avg = moving_average(col1, window_size)
        col2_moving_avg = moving_average(col2, window_size)
        col3_moving_avg = moving_average(col3, window_size)

        # Adjust timestamps for moving average (due to reduced length)
        timestamps_moving_avg = timestamps[:len(col1_moving_avg)]

        # Plot the moving averages
        plt.figure(figsize=(10, 6))
        plt.plot(timestamps_moving_avg, col1_moving_avg, label=f"x moving average (window={window_size})")
        plt.plot(timestamps_moving_avg, col2_moving_avg, label=f"y moving average (window={window_size})")
        plt.plot(timestamps_moving_avg, col3_moving_avg, label=f"z moving average (window={window_size})")

        # Add labels, title, and legend
        plt.xlabel("Time (seconds)")
        plt.ylabel("Moving Average")
        plt.title(f"Magnetometer Moving Averages (Window Size = {window_size})")
        plt.legend()

        # Show the plot
        plt.grid()
        plt.show()

    # Calculate and print the column averages as a vector
    column_averages = [col1_avg, col2_avg, col3_avg]
    print("Hard Iron Calibration:", column_averages)

    # Plot the normalized data
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, col1_normalized, label="x normalized data")
    plt.plot(timestamps, col2_normalized, label="y normalized data")
    plt.plot(timestamps, col3_normalized, label="z normalized data")

    # Add labels, title, and legend
    plt.xlabel("Time (seconds)")
    plt.ylabel("Normalized Data (uT)")
    plt.title("Magnetometer Normalized Data (uT)")
    plt.legend()

    # Show the plot
    plt.grid()
    plt.show()

    # Plot normalized columns against each other
    plt.figure(figsize=(10, 6))
    plt.plot(col1_normalized, col2_normalized, label="x vs y")
    plt.plot(col1_normalized, col3_normalized, label="x vs z")
    plt.plot(col2_normalized, col3_normalized, label="y vs z")

    # Add labels, title, and legend
    plt.xlabel("Normalized Data (uT)")
    plt.ylabel("Normalized Data (uT)")
    plt.title("Normalized Data Comparison")
    plt.legend()

    # Ensure axes are spaced evenly
    plt.axis('equal')

    # Show the plot
    plt.grid()
    plt.show()
    # Perform least squares fit of an ellipse on the normalized x and y data
    # Normalize x and y to a range of -1 to 1
    x = np.array(col1_normalized)
    y = np.array(col2_normalized)


    # Define the ellipse fitting function
    def ellipse_residuals(params, x, y):
        xc, yc, a, b, angle = params
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        x_rot = cos_angle * (x - xc) + sin_angle * (y - yc)
        y_rot = -sin_angle * (x - xc) + cos_angle * (y - yc)
        return (x_rot / a)**2 + (y_rot / b)**2 - 1

    # Initial guess for the parameters: center (0, 0), semi-major axis 1, semi-minor axis 1, angle 0
    initial_guess = [0, 0, 1, 1, 0]

    # Perform least squares fitting
    result = least_squares(ellipse_residuals, initial_guess, args=(x, y))

    # Extract fitted parameters
    xc, yc, a, b, angle = result.x

    # Print the fitted ellipse parameters
    print("Fitted Ellipse Parameters:")
    print(f"Center: ({xc:.3f}, {yc:.3f})")
    print(f"Semi-major axis: {a:.3f}")
    print(f"Semi-minor axis: {b:.3f}")
    print(f"Rotation angle (radians): {angle:.3f}")

    # Plot the fitted ellipse
    ellipse = Ellipse((xc, yc), 2*a, 2*b, angle=np.degrees(angle), edgecolor='r', facecolor='none', label="Fitted Ellipse")
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, 'b.', label="Normalized Data")
    plt.gca().add_patch(ellipse)
    plt.xlabel("Normalized X")
    plt.ylabel("Normalized Y")
    plt.title("Fitted Ellipse on Normalized Data")
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()

    # Calculate the transformation matrix to map the ellipse to a circle
    # The transformation matrix is composed of scaling and rotation
    scaling_matrix = np.array([[1/a, 0], [0, 1/b]]) * max(abs(x).max(), abs(y).max())
    rotation_matrix = np.array([[np.cos(-angle), -np.sin(-angle)], [np.sin(-angle), np.cos(-angle)]])
    transformation_matrix = scaling_matrix @ rotation_matrix
    fractional_matrix = []

    # Print the transformation matrix
    print("Soft Iron Calibration (as fractions):")
    for row in transformation_matrix:
        fraction_row = [Fraction(value).limit_denominator(2**15 - 1) for value in row]
        fractional_matrix.append(fraction_row)
        print([str(fraction) for fraction in fraction_row])

    # Apply the transformation matrix to the normalized data
    normalized_data = np.vstack((x, y))
    transformed_data = fractional_matrix @ normalized_data

    # Plot the transformed data (should form a circle)
    plt.figure(figsize=(10, 6))
    plt.plot(transformed_data[0, :], transformed_data[1, :], 'g.', label="Transformed Data (Circle)")
    plt.xlabel("Transformed X")
    plt.ylabel("Transformed Y")
    plt.title("Transformed Data (Mapped to Circle)")
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()
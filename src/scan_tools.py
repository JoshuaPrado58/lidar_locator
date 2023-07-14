import argparse
import numpy as np
import matplotlib.pyplot as plt
import csv
# matt's system
import matplotlib
matplotlib.use('TkAgg')

from sensor_msgs.msg import LaserScan

def polar2cart(r, theta):
    """
    Converts polar coordinates to cartesian coordinates.
    Inputs: r - radius
            theta - angle in radians
    Outputs: x - x coordinate
             y - y coordinate
    """
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def scan_to_points(scan_msg: LaserScan) -> np.ndarray:
    """
    Converts a LaserScan message to a numpy array of points in the sensor frame.
    """

    # Get the angles of each ray
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))

    # Get the cartesian coordinates
    x, y = polar2cart(scan_msg.ranges, angles)

    # Put in matrix form
    return np.vstack((x, y)).T

def csv_to_points(file_path):
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        data = list(reader)

    # get first scan
    data = data[0]

    ranges = np.array(data).astype(float)
    angles = np.linspace(0., 2.*np.pi, len(ranges))
    xy = polar2cart(ranges, angles)
    xy = np.array(xy).T
    return xy

def create_occupancy_grid_image(X: np.ndarray, img_size:tuple =(10, 10), bin_size: float=0.01, bool_center=False):
    """
    Creates an occupancy grid image from a set of points.
    Inputs: X - (N, 2) numpy array of points
            img_size - (2,) tuple of the size of the image in meters
            bin_size - size of each bin in meters
    Output: grid - (img_size/bin_size, img_size/bin_size) numpy array of the occupancy grid
    """

    # Convert meters to cm
    img_size = tuple(int(i/bin_size) for i in img_size)
    
    # Initialize grid with zeros
    grid = np.zeros(img_size)
    
    # Convert points to centimeters and integers for indexing
    X = (X/bin_size).astype(int)
    if bool_center:
        img_center = np.array(img_size) / -2.
        X = X - img_center.astype(int)
    
    # Make sure all points are within the grid
    X = X[(X[:,0] >= 0) & (X[:,0] < img_size[0]) & (X[:,1] >= 0) & (X[:,1] < img_size[1])]
    
    # Set the points to be occupied (value 1)
    grid[X[:,0], X[:,1]] = 1
    
    return grid

def visualize_occupancy_grid_image(grid_image: np.ndarray, center=(0, 0)):
    """
    Visualizes an occupancy grid image.
    Input: grid_image - (N, M) numpy array of the occupancy grid (0 or 1)
    """

    # Display the occupancy grid as a binary image
    plt.imshow(grid_image, cmap='binary', origin='lower')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Occupancy Grid')
    plt.colorbar()
    plt.show()

def xy_scatter_plot(X: np.ndarray, title: str='Scatter Plot', xlabel: str='X', ylabel: str='Y'):
    """
    Visualizes a scatter plot of points.
    Input: X - (N, 2) numpy array of points
    """

    # Display the occupancy grid as a binary image
    plt.scatter(X[:,0], X[:,1])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.show()

####################
# Tests
####################
def test_visualize_occupancy_grid():
    points = np.random.uniform(0, 10, size=(100, 2))
    grid = create_occupancy_grid_image(points, img_size=(10, 10), bin_size=0.05)
    print(points.shape)
    visualize_occupancy_grid_image(grid)

def test_scatter_plot():
    points = np.random.uniform(0, 10, size=(100, 2))
    print(points.shape)
    xy_scatter_plot(points)

def test_csv(data_path):
    points = csv_to_points(data_path)
    print(points.shape)
    # xy_scatter_plot(points)
    grid = create_occupancy_grid_image(points, img_size=(1, 1), bin_size=0.01, bool_center=True)
    visualize_occupancy_grid_image(grid)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_path", type=str, default=None)
    args = parser.parse_args()
    
    test_visualize_occupancy_grid()
    test_scatter_plot()

    if args.data_path is not None:
        test_csv(data_path=args.data_path)
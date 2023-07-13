import numpy as np
import matplotlib.pyplot as plt
# matt's system
import matplotlib
matplotlib.use('TkAgg')

from sensor_msgs.msg import LaserScan

def scan_to_points(scan_msg: LaserScan) -> np.ndarray:
    """
    Converts a LaserScan message to a numpy array of points in the sensor frame.
    """

    # Get the angles of each ray
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))

    # Get the cartesian coordinates
    x = scan_msg.ranges * np.cos(angles)
    y = scan_msg.ranges * np.sin(angles)

    # Put in matrix form
    return np.vstack((x, y)).T

def create_occupancy_grid_image(X: np.ndarray, img_size:tuple =(10, 10), bin_size: float=0.01):
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
    
    # Make sure all points are within the grid
    X = X[(X[:,0] >= 0) & (X[:,0] < img_size[0]) & (X[:,1] >= 0) & (X[:,1] < img_size[1])]
    
    # Set the points to be occupied (value 1)
    grid[X[:,0], X[:,1]] = 1
    
    return grid

def visualize_occupancy_grid_image(grid_image: np.ndarray):
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

####################
# Tests
####################
def test_visualize_occupancy_grid():
    points = np.random.uniform(0, 10, size=(100, 2))
    grid = create_occupancy_grid_image(points, img_size=(10, 10), bin_size=0.05)
    visualize_occupancy_grid_image(grid)

if __name__ == '__main__':
    test_visualize_occupancy_grid()
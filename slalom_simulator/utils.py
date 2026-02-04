"""
Utility functions for slalom simulator
"""
import numpy as np
from typing import List, Tuple

# Constants
POOL_WIDTH = 600
POOL_HEIGHT = 800
VEHICLE_SIZE = 40

# Colors
COLOR_BACKGROUND = (135, 206, 235)  # Sky blue
COLOR_VEHICLE = (80, 80, 80)  # Dark grey
COLOR_GHOST = (112, 128, 144)  # Blue-grey
COLOR_RED_POLE = (255, 0, 0)
COLOR_WHITE_POLE = (255, 255, 255)
COLOR_FORCE_ARROW = (255, 0, 0)


def generate_gates(num_gates: int = 3,
                   pool_width: float = 600.0,
                   pool_height: float = 800.0,
                   gate_width_mean: float = 70.0,
                   gate_width_std: float = 5.0,
                   pole_separation_mean: float = 100.0,
                   pole_separation_std: float = 5.0) -> List[dict]:
    """
    Generate slalom gates with zig-zag constraint, clustered near the top.

    Args:
        num_gates: Number of gates to generate
        pool_width: Width of the swimming pool in pixels
        pool_height: Height of the swimming pool in pixels
        gate_width_mean: Mean width of the gate opening
        gate_width_std: Standard deviation of gate width
        pole_separation_mean: Mean distance between red and white poles
        pole_separation_std: Standard deviation of pole separation

    Returns:
        List of gate dictionaries with 'center', 'orientation', 'red_pole', 'white_pole'
    """
    gates = []

    # Generate gate centers clustered near the top third of the pool
    y_start = 100
    y_end = pool_height * 0.4  # Focus on top 40% of screen
    y_spacing = (y_end - y_start) / (num_gates - 1) if num_gates > 1 else 0

    for i in range(num_gates):
        # Gate center position - alternating left/right for zig-zag
        if i % 2 == 0:
            center_x = pool_width * 0.40 + np.random.normal(0, 5)
        else:
            center_x = pool_width * 0.50 + np.random.normal(0, 5)
        
        center_y = y_start + i * y_spacing + np.random.normal(0, 3)
        center = np.array([center_x, center_y])

        # Orientation varies within ±20 degrees (±π/9 radians)
        base_orientation = np.pi/2  # Start from vertical (pointing down)
        orientation = base_orientation + np.random.uniform(-np.pi/18, np.pi/18)

        # Forward and right axes based on orientation
        forward_axis = np.array([np.cos(orientation), np.sin(orientation)])
        right_axis = np.array([-np.sin(orientation), np.cos(orientation)])

        # Red pole ALWAYS on the RIGHT side, white pole ALWAYS on the LEFT side
        # when looking in the direction of travel (downward/forward)
        red_offset = np.random.normal(0, 5)
        white_offset = np.random.normal(0, 5)
        
        red_pole = center + (pole_separation_mean/2 + red_offset) * right_axis
        white_pole = center - (pole_separation_mean/2 + white_offset) * right_axis

        gates.append({
            'center': center,
            'orientation': orientation,
            'red_pole': red_pole,
            'white_pole': white_pole
        })

    return gates


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def rotate_point(point: np.ndarray, angle: float, origin: np.ndarray = None) -> np.ndarray:
    """Rotate a point around an origin by an angle"""
    if origin is None:
        origin = np.array([0, 0])

    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    translated = point - origin
    rotated = rotation_matrix @ translated
    return rotated + origin


def distance(p1: np.ndarray, p2: np.ndarray) -> float:
    """Calculate Euclidean distance between two points"""
    return np.linalg.norm(p1 - p2)

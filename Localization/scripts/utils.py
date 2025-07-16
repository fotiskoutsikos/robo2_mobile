import numpy as np

def normalize(vector):
    if np.linalg.norm(vector) < 10e-7:
        return np.array([0, 0])
    else:
        return vector / np.linalg.norm(vector)

def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y])

def points_to_lines(points):
    lines = []
    for i in range(len(points)-1):
        lines.append([points[i], points[i+1]])
    return np.array(lines)

def rotate_vector(v, theta):
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    return rotation_matrix @ v

def angle_between_vectors(u, v):
    u = np.array(u)
    v = np.array(v)
    
    dot_product = np.dot(u, v)
    norms = np.linalg.norm(u) * np.linalg.norm(v)
    
    # Clip to avoid numerical errors outside [-1, 1]
    cos_theta = np.clip(dot_product / norms, -1.0, 1.0)
    
    return np.arccos(cos_theta)  # angle in radians


def closest_vector_to_segment(point, seg_start, seg_end):
    # Convert inputs to numpy arrays for vector operations
    P = np.array(point)
    A = np.array(seg_start)
    B = np.array(seg_end)
    
    # line direction
    AB = B - A
    # starting point is A
    
    s = (np.dot(AB, P) - np.dot(AB, A)) / np.dot(AB, AB)
    s_clamped = np.clip(s, 0, 1)
    closest_point = A + s_clamped * AB
    vector_to_closest = closest_point - P
    
    return vector_to_closest


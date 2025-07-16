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

def line_seg_to_half_line_distance(line_seg, point, unit_vector):
    p1 = line_seg[0]
    p2 = line_seg[1]
    v1 = (p2 - p1).reshape(2, 1)
    v2 = unit_vector.reshape(2, 1)

    if np.linalg.norm(normalize(p2 - p1) - unit_vector) == 0:
        return None  # parallel

    A = np.hstack((v1, -v2))
    b = (point - p1).reshape(2, 1)
    
    try:
        t = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None  # singular matrix (parallel or degenerate)

    t0, t1 = t[0, 0], t[1, 0]

    if 0.0 <= t0 <= 1.0 and t1 >= 0.0:
        return t1  # distance along ray
    else:
        return None

def angle_distance(theta1, theta2):
    vec1 = rotate_vector(np.array([1.0, 0.0]), theta1)
    vec2 = rotate_vector(np.array([1.0, 0.0]), theta2)
    
    return np.linalg.norm(vec1-vec2)

def wrap_angle(angle):
    # Wrap angle to [-pi, pi]
    return (angle + np.pi) % (2 * np.pi) - np.pi

import matplotlib.pyplot as plt
import utils
import numpy as np

if __name__ == "__main__":

    angles = [np.pi/2, np.pi/4, 0, -np.pi/4, -np.pi/2]
    ranges = [1.5, 1.25, 1.5, 1.25, 1.5]



    points = [utils.polar_to_cartesian(r, theta) for r, theta in zip(ranges, angles)]
    points = np.array(points)
    lines = utils.points_to_lines(points)
    print(lines)

    robot_position = np.array([0, 0])
    vectors = []
    for line_segment in lines:
        vector = utils.closest_vector_to_segment(robot_position, line_segment[0], line_segment[1])
        vectors.append(vector)

    vectors = np.array(vectors)
    lengths = np.linalg.norm(vectors, axis=1)
    min_index = np.argmin(lengths)
    min_vector = vectors[min_index]
    print("Closest vector to segment:", min_vector)
    print("Length of closest vector:", lengths[min_index])

    plt.plot(points[:,0], points[:,1], 'ro')
    for line in lines:
        plt.plot(line[:,0], line[:,1], 'b-')

    plt.plot(robot_position[0], robot_position[1], 'go')

    lam = 0.5
    v_error = min_vector
    v_follow = utils.rotate_vector(min_vector, np.pi/2)
    v_total = (1-lam) * v_error + lam * v_follow

    plt.plot(points[:,0], points[:,1], 'ro')
    plt.quiver(robot_position[0], robot_position[1], v_error[0], v_error[1], angles='xy', scale_units='xy', scale=1, color='r')
    plt.quiver(robot_position[0], robot_position[1], v_follow[0], v_follow[1], angles='xy', scale_units='xy', scale=1, color='b')
    plt.quiver(robot_position[0], robot_position[1], v_total[0], v_total[1], angles='xy', scale_units='xy', scale=1, color='g')


    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.show()

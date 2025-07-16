import numpy as np
import utils

class map_model:
    def __init__(self):
        self.walls = np.array(
            [
                [[-2.0, -2.0], [2.0, -2.0]],
                [[2.0, -2.0], [2.0, 2.0]],
                [[2.0, 2.0], [-2.0, 2.0]],
                [[-2.0, 2.0], [-2.0, -2.0]]
            ]
        )
    
    def get_distance(self, point, direction):
        distances = [
            d for d in (utils.line_seg_to_half_line_distance(w, point, direction) for w in self.walls)
            if d is not None
        ]
        return min(distances) if distances else None

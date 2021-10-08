import numpy as np


def fix_angle(e):
    return np.arctan2(np.sin(e), np.cos(e))

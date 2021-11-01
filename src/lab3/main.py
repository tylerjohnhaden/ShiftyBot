#!/usr/bin/env python

import numpy as np

from .line import LineBot

shifty = LineBot()
shifty.cruise_velocity = 0.3
shifty.goal_waypoints = np.array([[100, 0]])
shifty.run()

import numpy as np

# Root Rosbot node
from .Bot import Bot

# Feature Components
from .MetricBot import MetricBot
from .OdomBot import OdomBot
from .GoalBot import GoalBot
from .JoystickBot import JoystickBot
from .ReversibleBot import ReversibleBot
from .RangeBot import RangeBot
from .TrackedBot import TrackedBot
from .VelocityBot import VelocityBot

# Main entry point node
from .ShiftyBot import ShiftyBot

# Secondary io handler node
from .Mapper import Mapper


def fix_angle(e):
    return np.arctan2(np.sin(e), np.cos(e))


__all__ = [
    'Bot',
    'MetricBot',
    'OdomBot',
    'GoalBot',
    'JoystickBot',
    'ReversibleBot',
    'RangeBot',
    'TrackedBot',
    'VelocityBot',
    'ShiftyBot',
    'Mapper',
    'fix_angle'
]

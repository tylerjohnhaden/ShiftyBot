"""A Reversible Bot

Keep track of forward or backwards.
"""

from . import Bot


class ReversibleBot(Bot):
    def __init__(self, name='reversible-bot'):
        super().__init__(name)

        self.forward_direction = True

    def set_forward_direction(self):
        self.forward_direction = True

    def set_backward_direction(self):
        self.forward_direction = False

    def set_reverse_direction(self):
        self.forward_direction = not self.forward_direction

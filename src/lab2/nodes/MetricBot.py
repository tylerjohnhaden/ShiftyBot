"""An Empirical Data Bot

Capture rate of various events.
"""

from collections import defaultdict
import time

from . import Bot


class MetricBot(Bot):
    def __init__(self, name='metric-bot'):
        super().__init__(name)

        self.metric_rate_timeout = 1  # seconds
        self.metric_rate_log = defaultdict(list)
        self.metric_rate_previous = defaultdict(int)
        self.metric_running_rate = defaultdict(float)

    def log_event(self, event_name):
        self.metric_rate_log[event_name].append(time.time())
        if time.time() - self.metric_rate_log[event_name][0] > self.metric_rate_timeout * (10 ** 9):
            current = len(self.metric_rate_log[event_name])
            self.metric_running_rate[event_name] = (current + self.metric_rate_previous[event_name]) / 2
            self.metric_rate_previous[event_name] = current
            self.metric_rate_log[event_name].clear()

    def get_running_rate(self, event_name):
        return self.metric_running_rate[event_name]


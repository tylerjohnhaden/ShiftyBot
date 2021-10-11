import random
import string
import os

import rospy
from std_msgs.msg import String

from .Bot import Bot
from .TrackedBot import TrackedBot
from .GoalBot import GoalBot


class Mapper(Bot):
    def __init__(self, site_path='./site', name='mapper'):
        super(Mapper, self).__init__(name)

        self.goals = {}
        self.trackings = []

        self.mapper_id = (''.join(random.choice(string.ascii_letters) for _ in range(3)) +
                          ''.join(str(random.randint(0, 9)) for _ in range(3))).lower()

        self.site_path = os.path.abspath(site_path)
        self.data_js_file = os.path.join(self.site_path, 'data.js')
        self.data_js_named_file = os.path.join(self.site_path, 'data_' + self.mapper_id + '.js')
        self.data_js_template = '// auto generated\n\n' \
                                'let goals = [\n{goal_rows}\n];\n' \
                                'const tracking = [\n{tracking_rows}\n];'

        if not os.path.exists(self.site_path):
            raise IOError('Missing site folder: ' + self.site_path)

        # reset dt to 2 seconds for slow js file generation
        self.set_dt(2)

        self.init_goal_subscriber()
        self.init_tracking_subscriber()

    def step(self):
        js_generated_file = self.data_js_template.format(
            goal_rows=',\n'.join(self.goals.values()),
            tracking_rows=',\n'.join(self.trackings)
        )

        rospy.loginfo('Mapper writing to %s', self.data_js_file)

        with open(self.data_js_file, 'w') as file:
            file.write(js_generated_file)

        with open(self.data_js_named_file, 'w') as file:
            file.write(js_generated_file)

    def init_goal_subscriber(self):
        def _goal_callback(data):
            try:
                goal_values = str(data)[7: -1].split(';')
                if goal_values[0] and goal_values[0] not in self.goals.keys():
                    rospy.loginfo('\n\nNew Goal: %s\n', data)
                    self.goals[goal_values[0]] = '{{ {0} }}'.format(', '.join(map(
                        (lambda kv: '"{0}": {1}'.format(kv[0], kv[1])),
                        zip(GoalBot.goal_headers, goal_values)
                    )))
            except Exception as e:
                print('Goal Callback Error:', e)

        rospy.Subscriber("goal", String, _goal_callback)

    def init_tracking_subscriber(self):
        def _tracking_callback(data):
            tracking_values = str(data)[7: -1].split(';')
            self.trackings.append('{{ {0} }}'.format(', '.join(map(
                (lambda kv: '"{0}": {1}'.format(kv[0], kv[1])),
                zip(TrackedBot.tracking_headers, tracking_values)
            ))))


        rospy.Subscriber('tracking', String, _tracking_callback)

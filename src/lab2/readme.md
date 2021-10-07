# Shifty Bot

| Goal Configuration | Options | Default |
| ------------------ | ------- | ------- |
| waypoints | array of (x, y) pairs | `[[0, 0]]` |
| waypoint_behavior | `stop and turn` or `maintain speed` | `stop and turn` |
| throttle_behavior | `cruise control` or `distance + bump` | `cruise control` |
| end_behavior | `exit` or `loop` | `exit` |
| radius | float (meters) | `0.05` |
| transformation | `relative` or `global` | `relative` |
| reversible | bool | `False` |

| Control Configuration | Options | Default |
| ------------------ | ------- | ------- |
| steering_kp | float | `0.99` |
| throttle_kp | float | `0.2` |
| throttle_bump | float (meters / second) | `0.03` |
| cruise_velocity | float (meters / second) | `0.5` |
| obstacle_safety_range | float (meters) | `0.34` *uncalibrated |

## Usage

```python
#!/usr/bin/env python

import ShiftyBot

if __name__ == '__main__':
    shifty = ShiftyBot()

    shifty.set_goal(
        ...
    )
    shifty.set_control_constants(
        ...
    )

    shifty.run()
```

## Run

`roslaunch lab2 lab2.launch`

## Website

`open ./site/index.html`

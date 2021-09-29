import os
import json

data_file_name = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'data_9999.log')
html_file_name = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'graph_9999.html')

def build_html():
    with open(data_file_name, 'r') as data_file:
        data = []
        for line in data_file.readlines():
            data.append(json.loads(line))
            assert 'datetime' in data[-1].keys()
            assert 'x' in data[-1].keys()
            assert 'y' in data[-1].keys()
            assert 'theta' in data[-1].keys()
            assert 'linear_velocity' in data[-1].keys()
            assert 'angular_velocity' in data[-1].keys()
            assert 'walls' in data[-1].keys()

        lowest_x = min(d['x'] for d in data)
        lowest_y = min(d['y'] for d in data)
        highest_x = max(d['x'] for d in data)
        highest_y = max(d['y'] for d in data)
        height = highest_x - lowest_x
        width = highest_y - lowest_y

        walls = []
        for d in data:
            walls.extend(d['walls'])

        with open(html_file_name, 'w') as html_file:
            html_file.write('''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Shifty {0}</title>
</head>
<body>
    <canvas id="graph" width="1000" height="1000"></canvas>
</body>
<script>
    var xs = {1};
    var ys = {2};
    var wxs = {5};
    var wys = {6};

    var c = document.getElementById("graph");
    var ctx = c.getContext("2d");
    ctx.moveTo(500, 500);
    for (var i = 0; i < {3}; i++) {{
        ctx.strokeStyle = 'black';
        ctx.lineTo(500 + (xs[i] * 50), 500 + (ys[i] * 50));
        ctx.stroke();
        if (i + 1 === {3}) {{
            ctx.fillStyle = 'green';
            ctx.fillRect(500 + (xs[i] * 50), 500 + (ys[i] * 50), 4, 4);
        }}
    }}

    for (var j = 0; j < {4}; j++) {{
        ctx.fillStyle = 'blue';
        ctx.fillRect(500 + (wxs[j] * 50), 500 + (wys[j] * 50), 4, 4);
    }}
    </script>
</html>'''.format('test', [d['x'] for d in data], [d['y'] for d in data], len(data), len(walls),
                  [w[0] for w in walls], [w[1] for w in walls]))


build_html()
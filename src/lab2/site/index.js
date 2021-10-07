class Map {
    constructor(sideLength, maxDelta, robotRadius = 0.02, goalRadius = 0.01) {
        this.sideLength = sideLength;
        this.canvas = document.createElement('canvas');
        this.canvas.width = this.sideLength;
        this.canvas.height = this.sideLength;
        this.canvas.style.background = 'white';
        this.ctx = this.canvas.getContext('2d');
        this.maxDelta = maxDelta;
        this.robotRadius = robotRadius;
        this.goalRadius = goalRadius;
    }

    drawLine(x0, y0, x1, y1) {
        this.ctx.lineWidth = 2;
        this.ctx.strokeStyle = "#333";
        this.ctx.moveTo(
            (.50 + ((x0 / this.maxDelta) * .40)) * this.sideLength,
            (.50 - ((y0 / this.maxDelta) * .40)) * this.sideLength
        );
        this.ctx.lineTo(
            (.50 + ((x1 / this.maxDelta) * .40)) * this.sideLength,
            (.50 - ((y1 / this.maxDelta) * .40)) * this.sideLength
        );
        this.ctx.stroke();
        this.ctx.beginPath();
    }

    drawGoal(x, y) {
        this.ctx.beginPath();
        this.ctx.fillStyle = 'green';
        this.ctx.arc(
            (.5 + ((x / this.maxDelta) * .40)) * this.sideLength,
            (.5 + ((y / this.maxDelta) * .40)) * this.sideLength,
            this.goalRadius * this.sideLength,
            0,
            2 * Math.PI
        );
        this.ctx.fill();
        this.ctx.beginPath();
    }

    drawRobot(x, y, t) {
        this.ctx.translate(
            (0.5 + (x / this.maxDelta) * .40) * this.sideLength,
            (0.5 + (x / this.maxDelta) * .40) * this.sideLength
        );
        this.ctx.rotate(t + (Math.PI / 2));
        const r = this.robotRadius;
        this.ctx.fillStyle = 'red';
        this.ctx.fillRect(
            (-r) * this.sideLength,
            (-r) * this.sideLength,
            2 * r * this.sideLength,
            2 * r * this.sideLength
        );
        this.ctx.lineWidth = .009 * this.sideLength;
        this.ctx.strokeStyle = "black";
        this.ctx.moveTo((r * (2 / 3)) * this.sideLength, (r * (3 / 5)) * this.sideLength);
        this.ctx.lineTo((-r * (2 / 3)) * this.sideLength, (r * (3 / 5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((r) * this.sideLength, (r * (4 / 5)) * this.sideLength);
        this.ctx.lineTo((r) * this.sideLength, (r * (1 / 5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((r) * this.sideLength, (-r * (1 / 5)) * this.sideLength);
        this.ctx.lineTo((r) * this.sideLength, (-r * (4 / 5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((-r) * this.sideLength, (r * (4 / 5)) * this.sideLength);
        this.ctx.lineTo((-r) * this.sideLength, (r * (1 / 5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((-r) * this.sideLength, (-r * (1 / 5)) * this.sideLength);
        this.ctx.lineTo((-r) * this.sideLength, (-r * (4 / 5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.beginPath();
        this.ctx.fillStyle = 'black';
        this.ctx.arc(0, (-(r * (1 / 4))) * this.sideLength, r * this.sideLength / 2.2, 0, 2 * Math.PI);
        this.ctx.fill();
    }

    drawAxes() {
        this.ctx.beginPath();
        this.ctx.lineWidth = 1;
        this.ctx.strokeStyle = "gray";
        this.ctx.beginPath();
        this.ctx.moveTo(0.10 * this.sideLength, 0.10 * this.sideLength);
        this.ctx.lineTo(0.10 * this.sideLength, 0.90 * this.sideLength);
        this.ctx.stroke();
        this.ctx.font = '20px sans-serif';
        this.ctx.fillStyle = 'gray';
        this.ctx.fillText('Y', 0.05 * this.sideLength, 0.06 * this.sideLength);
        for (let i = 0; i < 9; i++) {
            this.ctx.moveTo(0.10 * this.sideLength, (i + 1) * 0.10 * this.sideLength);
            this.ctx.lineTo(0.08 * this.sideLength, (i + 1) * 0.10 * this.sideLength);
            this.ctx.stroke();
            this.ctx.font = '12px sans-serif';
            this.ctx.fillStyle = 'black';

            let message = (
                ((((i - 4) * 0.1)) / 0.4) * this.maxDelta
            ).toPrecision(2);
            if (i === 4) {
                message = message + ' (m)'
            }
            this.ctx.fillText(message, 0.03 * this.sideLength, ((i + 1) * 0.10 * this.sideLength) + 2);
        }
        this.ctx.beginPath();
        this.ctx.moveTo(0.10 * this.sideLength, 0.90 * this.sideLength);
        this.ctx.lineTo(0.90 * this.sideLength, 0.90 * this.sideLength);
        this.ctx.stroke();
        this.ctx.font = '20px sans-serif';
        this.ctx.fillStyle = 'gray';
        this.ctx.fillText('X', 0.94 * this.sideLength, 0.94 * this.sideLength);
        for (let i = 0; i < 9; i++) {
            this.ctx.moveTo((i + 1) * 0.10 * this.sideLength, 0.90 * this.sideLength);
            this.ctx.lineTo((i + 1) * 0.10 * this.sideLength, 0.92 * this.sideLength);
            this.ctx.stroke();
            this.ctx.font = '12px sans-serif';
            this.ctx.fillStyle = 'black';
            let message = (
                ((((i - 4) * 0.1)) / 0.4) * this.maxDelta
            ).toPrecision(2);
            if (i === 4) {
                message = message + ' (m)'
            }
            this.ctx.fillText(message, ((i + 1) * 0.10 * this.sideLength) - 10, 0.92 * this.sideLength + 40);
        }
        this.ctx.beginPath();
    }
}

const goalListElement = document.getElementById("goalList");
const mapDiv = document.getElementById("mapDiv");
function draw() {
    goalListElement.innerHTML = '';
    for (let j = goals.length - 1; j >= 0; j--) {
        const listItem = document.createElement('li');

        const checkbox = document.createElement('input');
        checkbox.type = "checkbox";
        checkbox.name = "select" + goals[j]["goal_id"];
        checkbox.checked = j + 1 === goals.length;
        listItem.appendChild(checkbox);

        const points = goals[j]['goal_waypoints'];
        let maxDelta = Math.max(...points.map(point => Math.max(...point)));
        const thumbnail = new Map(100, maxDelta, 0.08, 0.05);
        for (let i = 0; i < points.length; i++) {
            thumbnail.drawGoal(points[i][0], points[i][1]);
        }
        thumbnail.drawRobot(0, 0, 0);
        listItem.appendChild(thumbnail.canvas);
        goalListElement.appendChild(listItem);
    }


    let maxDelta = Math.max(...tracking.map(point => Math.max(point['x'], point['y'])));
    const map = new Map(document.body.clientHeight * 0.9, maxDelta, goals[0]['goal_radius'] * 0.3);
    for (let i = 0; i < goals[0]['goal_waypoints'].length; i++) {
        map.drawGoal(goals[0]['goal_waypoints'][i][0], goals[0]['goal_waypoints'][i][1]);
    }
    for (let i = 1; i < tracking.length; i++) {
        map.drawLine(tracking[i - 1]['x'], tracking[i - 1]['y'], tracking[i]['x'], tracking[i]['y']);
    }
    map.drawAxes();
    map.drawRobot(0, 0, 0);
    map.canvas.classList = "mainMap";

    mapDiv.innerHTML = '';
    mapDiv.appendChild(map.canvas);
}

draw();
setInterval(draw, 2000);

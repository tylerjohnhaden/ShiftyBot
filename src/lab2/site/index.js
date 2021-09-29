class Map {
    constructor(sideLength, maxDelta, robotRadius=0.04) {
        this.sideLength = sideLength;
        this.canvas = document.createElement('canvas');
        this.canvas.width = this.sideLength;
        this.canvas.height = this.sideLength;
        this.canvas.style.background = 'white';
        this.ctx = this.canvas.getContext('2d');
        this.maxDelta = maxDelta;
        this.robotRadius = robotRadius
    }

    drawLine(x0, y0, x1, y1) {
        this.ctx.lineWidth = 1;
        this.ctx.moveTo(
            (.50 + ((x0 / this.maxDelta) * .40)) * this.sideLength,
            (.50 - ((y0 / this.maxDelta) * .40)) * this.sideLength
        );
        this.ctx.lineTo(
            (.50 + ((x1 / this.maxDelta) * .40)) * this.sideLength,
            (.50 - ((y1 / this.maxDelta) * .40)) * this.sideLength
        );
        this.ctx.stroke();
    }

    drawGoal(x, y) {
        this.ctx.fillStyle = 'green';
        this.ctx.fillRect(
            (.46 + ((x / this.maxDelta) * .40)) * this.sideLength,
            (.46 - ((y / this.maxDelta) * .40)) * this.sideLength,
            .08 * this.sideLength,
            .08 * this.sideLength
        );
        this.ctx.lineWidth = 1;
        this.ctx.strokeStyle = "#00f000";
        this.ctx.strokeRect(
            (.46 + ((x / this.maxDelta) * .40)) * this.sideLength,
            (.46 - ((y / this.maxDelta) * .40)) * this.sideLength,
            .08 * this.sideLength,
            .08 * this.sideLength
        );
    }

    drawRobot() {
        const r = this.robotRadius;
        this.ctx.fillStyle = 'red';
        this.ctx.fillRect(
            (.50 - r) * this.sideLength,
                (.50 - r) * this.sideLength,
            2 * r * this.sideLength,
            2 * r * this.sideLength
        );
        this.ctx.lineWidth = .009 * this.sideLength;
        this.ctx.strokeStyle = "black";
        this.ctx.moveTo((.50 + r * (2/3)) * this.sideLength, (.50 + r * (3/5)) * this.sideLength);
        this.ctx.lineTo((.50 - r * (2/3)) * this.sideLength, (.50 + r * (3/5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((.50 + r) * this.sideLength, (.50 + r * (4/5)) * this.sideLength);
        this.ctx.lineTo((.50 + r) * this.sideLength, (.50 + r * (1/5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((.50 + r) * this.sideLength, (.50 - r * (1/5)) * this.sideLength);
        this.ctx.lineTo((.50 + r) * this.sideLength, (.50 - r * (4/5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((.50 - r) * this.sideLength, (.50 + r * (4/5)) * this.sideLength);
        this.ctx.lineTo((.50 - r) * this.sideLength, (.50 + r * (1/5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.moveTo((.50 - r) * this.sideLength, (.50 - r * (1/5)) * this.sideLength);
        this.ctx.lineTo((.50 - r) * this.sideLength, (.50 - r * (4/5)) * this.sideLength);
        this.ctx.stroke();
        this.ctx.beginPath();
        this.ctx.fillStyle = 'black';
        this.ctx.arc(.5 * this.sideLength, (.50 - (r * (1 / 4))) * this.sideLength, r * this.sideLength / 2.2, 0, 2 * Math.PI);
        this.ctx.fill();
    }
}

const goalListElement = document.getElementById("goalList");
for (let j = goals.length - 1; j >= 0; j--) {
    const listItem = document.createElement('li');

    const checkbox = document.createElement('input');
    checkbox.type = "checkbox";
    checkbox.name = "select" + goals[j]["goal_id"];
    checkbox.checked = j + 1 === goals.length;
    listItem.appendChild(checkbox);

    const points = goals[j]['points'];
    let maxDelta = Math.max(...points.map(point => Math.max(...point)));
    const thumbnail = new Map(100, maxDelta, 0.08);
    if (points.length > 0) {
        thumbnail.drawGoal(points[0][0], points[0][1]);
    }
    for (let i = 1; i < points.length; i++) {
        thumbnail.drawLine(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1]);
        thumbnail.drawGoal(points[i][0], points[i][1]);
    }
    if (points.length > 2) {
        thumbnail.drawLine(points[points.length - 1][0], points[points.length - 1][1], points[0][0], points[0][1]);
    }
    thumbnail.drawRobot();
    listItem.appendChild(thumbnail.canvas);

    goalListElement.appendChild(listItem);
}

const mapDiv = document.getElementById("mapDiv");
const map = new Map(document.body.clientHeight * 0.9, 1.5);
map.drawRobot();
map.canvas.classList = "mainMap";
mapDiv.appendChild(map.canvas);

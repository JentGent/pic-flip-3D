width = 800;
height = 800;
var canvas;
mousePressed = false;
function setup() {
    canvas = createCanvas(800, 800).canvas;
    canvas.onmousedown = function(e) {
        mousePressed = true;
    };
}

document.onmouseup = function(e) {
    mousePressed = false;
};

let SOLVER_ITERATIONS = 20,
    PARTICLE_COLLISION_ITERATIONS = 1,
    FLIP = 0.5,
    STIFFNESS = 10,
    PUSH_STRENGTH = 15;

const DT = 1/60, MAX_SUBSTEPS = 5;
const GRAVITY = 500;
const VIEW_SPEED = 0.005;
const WIDTH = 30, HEIGHT = 20, DEPTH = 20;
const NUM_PARTICLES = 5000;
const CELL_SIZE = 10;
const PARTICLE_SIZE = 10;
const DENSITY = CELL_SIZE * CELL_SIZE * CELL_SIZE / PARTICLE_SIZE / PARTICLE_SIZE / PARTICLE_SIZE;

function View(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
    
    this.theta = this.phi = 0;
    this.distance = 250;
    this.scale = 300;
}
View.prototype.worldToScreen = function(x, y, z) {
    x += this.x;
    y += this.y;
    z += this.z;
    let c = Math.cos(this.theta), s = Math.sin(this.theta);
    let xp = c * x + s * z, yp = y, zp = -s * x + c * z;
    c = Math.cos(this.phi); s = Math.sin(this.phi);
    yp = c * y - s * zp;
    zp = s * y + c * zp;
    zp += this.distance;
    return {
        x: xp / zp * this.scale,
        y: yp / zp * this.scale,
        z: zp
    };
};
const view = new View(-WIDTH * CELL_SIZE / 2, -HEIGHT * CELL_SIZE / 2, -DEPTH * CELL_SIZE / 2);

function Grid(x, y, z, width, height, depth, size, parent) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.width = width;
    this.height = height;
    this.depth = depth;
    this.cellSize = size;
    this.parent = parent;
    
    this.values = [];
    this.weights = [];
    this.fillValues(0);
}
Grid.prototype.copy = function() {
    const newGrid = new Grid(this.x, this.y, this.z, this.width, this.height, this.depth, this.cellSize, this.parent);
    newGrid.values = this.values.slice(0);
    newGrid.weights = this.weights.slice(0);
    return newGrid;
};
Grid.prototype.copyValues = function(grid) {
    this.values = grid.values.slice(0);
};
Grid.prototype.idx = function(i, j, k) {
    return i + this.width * (j + k * this.height);
};
Grid.prototype.getValueIdx = function(i, j, k) {
    return this.values[this.idx(i, j, k)];
};
Grid.prototype.addValueIdx = function(i, j, k, value) {
    const idx = this.idx(i, j, k);
    this.values[idx] += value;
    this.weights[idx] += 1;
};
Grid.prototype.faceIsAir = function(i, j, k) {
    if (!this.x) {
        return i > 0 && i < this.width - 1 && this.parent.cellIsAir(i - 1, j, k) && this.parent.cellIsAir(i, j, k);
    } else if (!this.y) {
        return j > 0 && j < this.height - 1 && this.parent.cellIsAir(i, j - 1, k) && this.parent.cellIsAir(i, j, k);
    } else if (!this.z) {
        return k > 0 && k < this.depth - 1 && this.parent.cellIsAir(i, j, k - 1) && this.parent.cellIsAir(i, j, k);
    }
};
Grid.prototype.faceIsWall = function(i, j, k) {
    if (!this.x) {
        return i <= 0 || i >= this.width - 1;
    } else if (!this.y) {
        return j <= 0 || j >= this.height - 1;
    } else if (!this.z) {
        return k <= 0 || k >= this.depth - 1;
    }
};
Grid.prototype.getWeights = function(x, y, z, ignoreAir, ignoreWalls) {
    x = Math.max(0, Math.min(this.width - 1, (x - this.x) / this.cellSize));
    y = Math.max(0, Math.min(this.height - 1, (y - this.y) / this.cellSize));
    z = Math.max(0, Math.min(this.depth - 1, (z - this.z) / this.cellSize));
    let i = Math.min(x | 0, this.width - 2),
        j = Math.min(y | 0, this.height - 2),
        k = Math.min(z | 0, this.depth - 2);
    const tx = x - i, ty = y - j, tz = z - k;
    const ix = 1 - tx, iy = 1 - ty, iz = 1 - tz;
    let w000 = ix * iy * iz;
    let w100 = tx * iy * iz;
    let w010 = ix * ty * iz;
    let w110 = tx * ty * iz;
    let w001 = ix * iy * tz;
    let w101 = tx * iy * tz;
    let w011 = ix * ty * tz;
    let w111 = tx * ty * tz;
    if (ignoreAir) {
        if (w000 && this.faceIsAir(i  , j  , k  )) w000 = 0;
        if (w100 && this.faceIsAir(i+1, j  , k  )) w100 = 0;
        if (w010 && this.faceIsAir(i  , j+1, k  )) w010 = 0;
        if (w110 && this.faceIsAir(i+1, j+1, k  )) w110 = 0;
        if (w001 && this.faceIsAir(i  , j  , k+1)) w001 = 0;
        if (w101 && this.faceIsAir(i+1, j  , k+1)) w101 = 0;
        if (w011 && this.faceIsAir(i  , j+1, k+1)) w011 = 0;
        if (w111 && this.faceIsAir(i+1, j+1, k+1)) w111 = 0;
    }
    if (ignoreWalls) {
        if (w000 && this.faceIsWall(i  , j  , k  )) w000 = 0;
        if (w100 && this.faceIsWall(i+1, j  , k  )) w100 = 0;
        if (w010 && this.faceIsWall(i  , j+1, k  )) w010 = 0;
        if (w110 && this.faceIsWall(i+1, j+1, k  )) w110 = 0;
        if (w001 && this.faceIsWall(i  , j  , k+1)) w001 = 0;
        if (w101 && this.faceIsWall(i+1, j  , k+1)) w101 = 0;
        if (w011 && this.faceIsWall(i  , j+1, k+1)) w011 = 0;
        if (w111 && this.faceIsWall(i+1, j+1, k+1)) w111 = 0;
    }
    const total = w000 + w100 + w010 + w110 + w001 + w101 + w011 + w111;
    if (total > 0) {
        const d = 1 / total;
        w000 *= d; w100 *= d; w010 *= d; w110 *= d;
        w001 *= d; w101 *= d; w011 *= d; w111 *= d;
    }
    return {
        i: i, j: j, k: k,
        tx: tx, ty: ty, tz: tz,
        weights: [w000, w100, w010, w110, w001, w101, w011, w111]
    };
};
Grid.prototype.getValue = function(x, y, z, ignoreAir, ignoreWalls) {
    const c = this.getWeights(x, y, z, ignoreAir, ignoreWalls);
    const i = c.i, j = c.j, k = c.k;
    const w = c.weights;
    return  this.values[this.idx(i  , j  , k  )] * w[0] +
            this.values[this.idx(i+1, j  , k  )] * w[1] +
            this.values[this.idx(i  , j+1, k  )] * w[2] +
            this.values[this.idx(i+1, j+1, k  )] * w[3] +
            this.values[this.idx(i  , j  , k+1)] * w[4] +
            this.values[this.idx(i+1, j  , k+1)] * w[5] +
            this.values[this.idx(i  , j+1, k+1)] * w[6] +
            this.values[this.idx(i+1, j+1, k+1)] * w[7];
};
Grid.prototype.addValue = function(x, y, z, value, ignoreAir, ignoreWalls) {
    const c = this.getWeights(x, y, z, ignoreAir, ignoreWalls);
    const i = c.i, j = c.j, k = c.k;
    const w = c.weights;
    this.values[this.idx(i  , j  , k  )] += value * w[0];
    this.values[this.idx(i+1, j  , k  )] += value * w[1];
    this.values[this.idx(i  , j+1, k  )] += value * w[2];
    this.values[this.idx(i+1, j+1, k  )] += value * w[3];
    this.values[this.idx(i  , j  , k+1)] += value * w[4];
    this.values[this.idx(i+1, j  , k+1)] += value * w[5];
    this.values[this.idx(i  , j+1, k+1)] += value * w[6];
    this.values[this.idx(i+1, j+1, k+1)] += value * w[7];
    
    this.weights[this.idx(i  , j  , k  )] += w[0];
    this.weights[this.idx(i+1, j  , k  )] += w[1];
    this.weights[this.idx(i  , j+1, k  )] += w[2];
    this.weights[this.idx(i+1, j+1, k  )] += w[3];
    this.weights[this.idx(i  , j  , k+1)] += w[4];
    this.weights[this.idx(i+1, j  , k+1)] += w[5];
    this.weights[this.idx(i  , j+1, k+1)] += w[6];
    this.weights[this.idx(i+1, j+1, k+1)] += w[7];
};
Grid.prototype.divideWeights = function() {
    for (let i = 0; i < this.weights.length; i++) {
        if (!this.weights[i]) continue;
        this.values[i] /= this.weights[i];
    }
};
Grid.prototype.resetWeights = function() {
    for (let i = 0; i < this.weights.length; i++) {
        this.weights[i] = this.values[i] = 0;
    }
};
Grid.prototype.fillValues = function(value) {
    this.values.length = 0;
    this.weights.length = 0;
    for (let z = 0; z < this.depth; z++) {
        for (let y = 0; y < this.height; y++) {
            for (let x = 0; x < this.width; x++) {
                this.values.push(value);
                this.weights.push(0);
            }
        }
    }
};

function Particle(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
    
    this.xVel = this.yVel = this.zVel = 0;
}
Particle.prototype.update = function(dt) {
    this.yVel -= GRAVITY * dt;
    this.x += this.xVel * dt;
    this.y += this.yVel * dt;
    this.z += this.zVel * dt;
};

function Simulation(width, height, depth, cellSize) {
    this.width = width;
    this.height = height;
    this.depth = depth;
    this.cellSize = cellSize;
    
    this.xVels = new Grid(0, cellSize/2, cellSize/2, width + 1, height, depth, cellSize, this);
    this.yVels = new Grid(cellSize/2, 0, cellSize/2, width, height + 1, depth, cellSize, this);
    this.zVels = new Grid(cellSize/2, cellSize/2, 0, width, height, depth + 1, cellSize, this);
    this.xVels2 = this.xVels.copy();
    this.yVels2 = this.yVels.copy();
    this.zVels2 = this.zVels.copy();
    this.density = new Grid(cellSize/2, cellSize/2, cellSize/2, width, height, depth, cellSize, this);
    this.particles = [];
    this.grid = [];
    
    this.resetGrid();
    this.spawnParticles();
}
Simulation.prototype.idx = function(i, j, k) {
    i = Math.max(0, Math.min(this.width - 1, i | 0));
    j = Math.max(0, Math.min(this.height - 1, j | 0));
    k = Math.max(0, Math.min(this.depth - 1, k | 0));
    return i + this.width * (j + k * this.height);
};
Simulation.prototype.cellIsAir = function(i, j, k) {
    const idx = this.idx(i, j, k);
    return this.grid[idx] && this.grid[idx].length === 0;
};
Simulation.prototype.resetGrid = function() {
    this.grid.length = 0;
    for (let i = 0; i < this.depth * this.height * this.width; i++) {
        this.grid.push([]);
    }
};
Simulation.prototype.zSort = function(a, b) {
    return view.worldToScreen(b.x, b.y, b.z).z - view.worldToScreen(a.x, a.y, a.z).z;
};
Simulation.prototype.spawnParticles = function() {
    for (let i = 0; i < NUM_PARTICLES; i++) {
        this.particles.push(new Particle((0.5 + Math.random() * 0.5) * this.width * this.cellSize, (0.5 + Math.random() * 0.5) * this.height * this.cellSize, Math.random() * this.depth * this.cellSize));
    }
};
Simulation.prototype.particlesToGrid = function() {
    this.resetGrid();
    this.xVels.resetWeights();
    this.yVels.resetWeights();
    this.zVels.resetWeights();
    this.density.resetWeights();
    for (let i = 0; i < this.particles.length; i++) {
        const p = this.particles[i];
        this.grid[this.idx(p.x / this.cellSize, p.y / this.cellSize, p.z / this.cellSize)].push(p);
        this.density.addValue(p.x, p.y, p.z, 1);
        this.xVels.addValue(p.x, p.y, p.z, p.xVel);
        this.yVels.addValue(p.x, p.y, p.z, p.yVel);
        this.zVels.addValue(p.x, p.y, p.z, p.zVel);
    }
    this.xVels.divideWeights();
    this.yVels.divideWeights();
    this.zVels.divideWeights();
};
Simulation.prototype.getCellWalls = function(i, j, k) {
    return {
        left: i === 0, right: i === this.width - 1,
        top: j === 0, bottom: j === this.height - 1,
        front: k === 0, back: k === this.depth - 1
    };
};
Simulation.prototype.eliminateDivergence = function(iterations) {
    const pxVels = this.xVels.copy(), pyVels = this.yVels.copy(), pzVels = this.zVels.copy();
    let maxDiv = 1;
    for (let it = 0; it < iterations; it++) {
        if (maxDiv < 0.001) break;
        maxDiv = 0;
        for (let k = 0; k < this.depth; k++) {
            for (let j = 0; j < this.height; j++) {
                for (let i = 0; i < this.width; i++) {
                    const particles = this.grid[this.idx(i, j, k)];
                    if (!particles || !particles.length) continue;
                    const cellWalls = this.getCellWalls(i, j, k);
                    const denom = 6 - cellWalls.left - cellWalls.right - cellWalls.top - cellWalls.bottom - cellWalls.front - cellWalls.back;
                    if (!denom) continue;
                    const xIdx1 = this.xVels.idx(i, j, k), xIdx2 = this.xVels.idx(i + 1, j, k);
                    const yIdx1 = this.yVels.idx(i, j, k), yIdx2 = this.yVels.idx(i, j + 1, k);
                    const zIdx1 = this.zVels.idx(i, j, k), zIdx2 = this.zVels.idx(i, j, k + 1);
                    let div = this.xVels.values[xIdx2] * !cellWalls.right - this.xVels.values[xIdx1] * !cellWalls.left
                            + this.yVels.values[yIdx2] * !cellWalls.bottom - this.yVels.values[yIdx1] * !cellWalls.top
                            + this.zVels.values[zIdx2] * !cellWalls.back - this.zVels.values[zIdx1] * !cellWalls.front;
                    div = 1 * div - Math.max(0, STIFFNESS * (this.density.getValueIdx(i, j, k) - DENSITY));
                    maxDiv = Math.max(maxDiv, Math.abs(div));
                    div *= 1.5/denom;
                    if (!cellWalls.left) { this.xVels.values[xIdx1] += div; }
                    if (!cellWalls.top) { this.yVels.values[yIdx1] += div; }
                    if (!cellWalls.front) { this.zVels.values[zIdx1] += div; }
                    if (!cellWalls.right) { this.xVels.values[xIdx2] -= div; }
                    if (!cellWalls.bottom) { this.yVels.values[yIdx2] -= div; }
                    if (!cellWalls.back) { this.zVels.values[zIdx2] -= div; }
                }
            }
        }
    }
    for (let i = 0; i < pxVels.values.length; i++) {
        this.xVels2.values[i] = this.xVels.values[i] - pxVels.values[i];
    }
    for (let i = 0; i < pyVels.values.length; i++) {
        this.yVels2.values[i] = this.yVels.values[i] - pyVels.values[i];
    }
    for (let i = 0; i < pzVels.values.length; i++) {
        this.zVels2.values[i] = this.zVels.values[i] - pzVels.values[i];
    }
};
Simulation.prototype.gridToParticles = function(flip) {
    for (let i = 0; i < this.particles.length; i++) {
        const p = this.particles[i];
        p.xVel = this.xVels.getValue(p.x, p.y, p.z, true, true) * (1 - flip)
              + (p.xVel + this.xVels2.getValue(p.x, p.y, p.z, true, true)) * flip;
        p.yVel = this.yVels.getValue(p.x, p.y, p.z, true, true) * (1 - flip)
              + (p.yVel + this.yVels2.getValue(p.x, p.y, p.z, true, true)) * flip;
        p.zVel = this.zVels.getValue(p.x, p.y, p.z, true, true) * (1 - flip)
              + (p.zVel + this.zVels2.getValue(p.x, p.y, p.z, true, true)) * flip;
    }
};
Simulation.prototype.boxParticle = function(p) {
    if (p.x < 0) { p.x = 0; p.xVel = 0; }
    if (p.x > this.width * this.cellSize) { p.x = this.width * this.cellSize; p.xVel = 0; }
    if (p.y < 0) { p.y = 0; p.yVel = 0; }
    if (p.y > this.height * this.cellSize) { p.y = this.height * this.cellSize; p.yVel = 0; }
    if (p.z < 0) { p.z = 0; p.zVel = 0; }
    if (p.z > this.depth * this.cellSize) { p.z = this.depth * this.cellSize; p.zVel = 0; }
}
Simulation.prototype.update = function(dt) {
    for (let i = 0; i < this.particles.length; i++) {
        const p = this.particles[i];
        p.update(dt);
        this.boxParticle(p);
    }
    for (let it = 0; it < PARTICLE_COLLISION_ITERATIONS; it++) {
        let anyCollisions = false;
        let cellX = 0, cellY = 0, cellZ = 0;
        for (let idx1 = 0; idx1 < this.grid.length; idx1++) {
            const cell1 = this.grid[idx1];
            if (cell1 && cell1.length) {
                for (let xIdx = cellX; xIdx <= cellX + 1; xIdx++) {
                    if (xIdx >= this.width) continue;
                    for (let yIdx = cellY; yIdx <= cellY + 1; yIdx++) {
                        if (yIdx >= this.height) continue;
                        for (let zIdx = cellZ; zIdx <= cellZ + 1; zIdx++) {
                            if (zIdx >= this.depth) continue;
                            const idx2 = this.idx(xIdx, yIdx, zIdx);
                            const cell2 = this.grid[idx2];
                            if (!cell2 || !cell2.length) continue;
                            for (let i = 0; i < cell1.length; i++) {
                                const p = cell1[i];
                                for (let j = (cell1 === cell2 ? i + 1 : 0); j < cell2.length; j++) {
                                    const q = cell2[j];
                                    const dx = (q.x - p.x) || (Math.random() * 0.02 - 0.01),
                                        dy = (q.y - p.y) || (Math.random() * 0.02 - 0.01),
                                        dz = (q.z - p.z) || (Math.random() * 0.02 - 0.01);
                                    const sqDist = dx * dx + dy * dy + dz * dz;
                                    if (sqDist < PARTICLE_SIZE * PARTICLE_SIZE) {
                                        anyCollisions = true;
                                        const dist = Math.sqrt(sqDist);
                                        const overlap = (PARTICLE_SIZE - dist) / 8 / dist;
                                        p.x -= dx * overlap;
                                        p.y -= dy * overlap;
                                        p.z -= dz * overlap;
                                        q.x += dx * overlap;
                                        q.y += dy * overlap;
                                        q.z += dz * overlap;
                                        this.boxParticle(p);
                                        this.boxParticle(q);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            cellX++;
            if (cellX >= this.width) {
                cellX = 0;
                cellY++;
                if (cellY >= this.height) {
                    cellY = 0;
                    cellZ++;
                }
            }
        }
        if (!anyCollisions) break;
    }
    this.particlesToGrid();
    this.eliminateDivergence(SOLVER_ITERATIONS);
    this.gridToParticles(FLIP);
};
Simulation.prototype.display = function() {
    this.particles.sort(this.zSort);
    let minDensity = 0.5, maxDensity = 2.5;
    for (let i = 0; i < this.particles.length; i++) {
        const p = this.particles[i];
        const density = this.density.getValue(p.x, p.y, p.z);
        const screenCoords = view.worldToScreen(p.x, p.y, p.z);
        const scl = view.scale / screenCoords.z;
        let grad = constrain((300 - screenCoords.z) / 600 + 0.7, 0, 1);
        grad *= 1.7 - (density - minDensity) / (maxDensity - minDensity) * 0.7;
        stroke(79 * grad, 151 * grad, 214 * grad);
        strokeWeight(PARTICLE_SIZE * scl);
        point(screenCoords.x, screenCoords.y);
    }
};

const sim = new Simulation(WIDTH, HEIGHT, DEPTH, CELL_SIZE);

let time = Date.now() / 1000;
let oldTime = time;
function draw() {
    const realTime = Date.now() / 1000;
    time = Math.max(time, realTime - MAX_SUBSTEPS * DT);
    background(0, 0, 0);
    if (mousePressed) {
        view.theta -= (mouseX - pmouseX) * VIEW_SPEED;
        view.phi -= (mouseY - pmouseY) * VIEW_SPEED;
        view.phi = constrain(view.phi, -Math.PI / 2, Math.PI / 2);
    } else {
        for (let i = 0; i < sim.particles.length; i++) {
            const p = sim.particles[i];
            const screenCoords = view.worldToScreen(p.x, p.y, p.z);
            const dx = mouseX - width/2 - screenCoords.x,
                  dy = -(mouseY - height/2) - screenCoords.y;
            let vx1 = mouseX - pmouseX, vy1 = -(mouseY - pmouseY), vz1 = 0;
            let c = Math.cos(-view.phi); s = Math.sin(-view.phi);
            let vy2 = c * vy1 - s * vz1, vz2 = s * vy1 + c * vz1;
            c = Math.cos(-view.theta), s = Math.sin(-view.theta);
            let vx = c * vx1 + s * vz2, vy = vy2, vz = -s * vx1 + c * vz2;
            const str = PUSH_STRENGTH * 100 / (100 + dx * dx + dy * dy);
            p.xVel += vx * str;
            p.yVel += vy * str;
            p.zVel += vz * str;
        }
    }
    
    push();
    translate(width / 2, height / 2);
    scale(1, -1);
    while (time < realTime) {
        sim.update(DT);
        time += DT;
    }
    sim.display();
    pop();

    fill(255);
    noStroke();
    textAlign(LEFT, TOP);
    textSize(15);
    text(Math.round(1 / (realTime - oldTime) * 10) / 10, 5, 5);
    oldTime = realTime;
}

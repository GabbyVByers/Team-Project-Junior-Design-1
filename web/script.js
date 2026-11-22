const ESP32_IP = "192.168.1.224";  // paste the IP printed on the serial monitor

// World units are meters, z is up. Matches the device's level frame.
const CUBE   = 0.05;               // cube edge length
const AXIS   = 0.15;               // axis line length
const GRID   = 0.1;                // grid spacing
const EXTENT = 5;                  // grid half-width (reads as an infinite plane)
const CAMERA = [1.2, -1.6, 1.0];
const TARGET = [0, 0, 0.25];
const NEAR   = 0.05;

const canvas = document.getElementById("view");
const ctx = canvas.getContext("2d");
const status = document.getElementById("status");

const add   = (a, b) => a.map((v, i) => v + b[i]);
const sub   = (a, b) => a.map((v, i) => v - b[i]);
const scale = (a, s) => a.map(v => v * s);
const dot   = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit  = (a) => scale(a, 1 / Math.hypot(...a));

// Rotates v by unit quaternion [w, x, y, z]
function rotate([w, ...u], v) {
  const t = scale(cross(u, v), 2);
  return add(add(v, scale(t, w)), cross(u, t));
}

// Fixed camera basis
const forward = unit(sub(TARGET, CAMERA));
const right   = unit(cross(forward, [0, 0, 1]));
const up      = cross(right, forward);

function toCamera(p) {
  const d = sub(p, CAMERA);
  return [dot(d, right), dot(d, up), dot(d, forward)];
}

function line(a, b, color, width = 1) {
  a = toCamera(a);
  b = toCamera(b);
  if (a[2] < NEAR && b[2] < NEAR) return;
  if (a[2] < NEAR) a = add(b, scale(sub(a, b), (b[2] - NEAR) / (b[2] - a[2])));
  if (b[2] < NEAR) b = add(a, scale(sub(b, a), (a[2] - NEAR) / (a[2] - b[2])));

  const f = 1.2 * Math.min(canvas.width, canvas.height);
  const project = (p) => [canvas.width / 2 + f * p[0] / p[2], canvas.height / 2 - f * p[1] / p[2]];

  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(...project(a));
  ctx.lineTo(...project(b));
  ctx.stroke();
}

function draw(position, orientation) {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;

  for (let k = -EXTENT / GRID; k <= EXTENT / GRID; k++) {
    const i = k * GRID;
    line([i, -EXTENT, 0], [i, EXTENT, 0], "#ddd");
    line([-EXTENT, i, 0], [EXTENT, i, 0], "#ddd");
  }

  // The tracked point is the cube's center, which rests half a cube above the plane
  const center = add(position, [0, 0, CUBE / 2]);
  const corner = (i) => add(center, rotate(orientation, [i & 1, (i >> 1) & 1, (i >> 2) & 1].map(c => (c - 0.5) * CUBE)));

  for (let i = 0; i < 8; i++) {
    for (const bit of [1, 2, 4]) {
      if (!(i & bit)) line(corner(i), corner(i | bit), "#333", 2);
    }
  }

  line(center, add(center, rotate(orientation, [AXIS, 0, 0])), "red", 2);
  line(center, add(center, rotate(orientation, [0, AXIS, 0])), "green", 2);
  line(center, add(center, rotate(orientation, [0, 0, AXIS])), "blue", 2);
}

let pose = [[0, 0, 0], [1, 0, 0, 0]];
draw(...pose);
window.onresize = () => draw(...pose);

const ws = new WebSocket(`ws://${ESP32_IP}/ws`);
ws.onopen    = () => status.textContent = "Connected";
ws.onclose   = () => status.textContent = "Disconnected - refresh to retry";
ws.onmessage = (e) => {
  const v = e.data.split(" ").map(Number);  // px py pz qw qx qy qz
  pose = [v.slice(0, 3), v.slice(3, 7)];
  draw(...pose);
};

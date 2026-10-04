// ===== config =====
const ROSBRIDGE_PORT = 9090;
const BACKEND_PORT = 8000;
const CAM_PORT = 5000;                   // camera streamer (MJPEG)
const CAMS = [['cam-color', '/video_feed'], ['cam-depth', '/depth_feed']];               // only used for Ask Helio
const T = {
  map:    '/map',
  global: '/global_costmap/costmap',
  local:  '/local_costmap/costmap',
  scan:   '/scan',
  plan:   '/plan',
  navStatus: '/navigate_to_pose/_action/status',
  navCancel: '/navigate_to_pose/_action/cancel_goal',
  goal:   '/goal_pose',
  cmdVel: '/cmd_vel',
  saveMap: '/slam_toolbox/save_map',
  odom:  '/odom',
  globalUpdates: '/global_costmap/costmap_updates',
  localUpdates:  '/local_costmap/costmap_updates',
  clearGlobal: '/global_costmap/clear_entirely_global_costmap',
  clearLocal:  '/local_costmap/clear_entirely_local_costmap',
};
const SPEAK_ON_ROVER = false; // also send Helio replies to the rover speaker topic
const BASE_FRAME = 'base_link';

// joystick defaults; the fields in the Joystick panel override these live
const DEF = { maxLin: 0.3, maxAng: 0.21, accel: 0.5, angAccel: 0.5 };
const field = (id, d) => { const v = parseFloat(document.getElementById(id).value); return v > 0 ? v : d; };
const getCfg = () => ({
  maxLin: field('set-lin', DEF.maxLin), maxAng: field('set-ang', DEF.maxAng),
  accel: field('set-acc', DEF.accel), angAccel: field('set-angacc', DEF.angAccel),
});

// log only navigation / mapping related lines from /rosout
const NAVMAP_RE = /nav|planner|controller|bt_|behavior|costmap|amcl|slam|map|waypoint|smoother|recovery|lifecycle|goal|path|locali/i;

// ===== login (native prompts) + rosbridge connect =====
let ros = null, IP = '';

function connect(ip) {
  return new Promise((resolve, reject) => {
    const r = new ROSLIB.Ros({ url: `ws://${ip}:${ROSBRIDGE_PORT}` });
    r.on('connection', () => resolve(r));
    r.on('error', () => reject());
    r.on('close', () => {
      navEl.textContent = 'Nav: disconnected';
      odomX.textContent = odomY.textContent = odomH.textContent = '--';
    });
  });
}

async function login() {
  while (true) {
    const user = prompt('Username:', 'waregv');
    if (user === null) return false;
    const pass = prompt('Password (rover IP):', localStorage.getItem('ip') || '');
    if (pass === null) return false;
    if (user.trim().toLowerCase() !== 'waregv') { alert('Wrong username.'); continue; }
    try {
      ros = await connect(pass.trim());
      IP = pass.trim();
      localStorage.setItem('ip', IP);
      return true;
    } catch (e) {
      alert('rosbridge did not respond at ' + pass.trim() + ':' + ROSBRIDGE_PORT);
    }
  }
}

const topic = (name, type, opts = {}) =>
  new ROSLIB.Topic({ ros, name, messageType: type, ...opts });

// ===== state =====
const COLORS = {
  slam: [20, 20, 20], global: [255, 140, 0], local: [150, 60, 220],
  scan: [230, 30, 30], path: [0, 160, 60],
};
const rgb = c => `rgb(${c.join(',')})`;
const show = { slam: true, global: true, local: true, scan: true, path: true };
const layers = { slam: null, global: null, local: null };   // {w,h,res,ox,oy,frame,img}
let scan = null;                                            // {frame, angle_min, inc, ranges}
let plan = null;                                            // {frame, pts}
const vel = { v: null, w: null };                          // from /odom
const tfs = {};                                             // child -> {parent,x,y,yaw}

const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const logEl = document.getElementById('log');
const navEl = document.getElementById('nav-state');
const odomX = document.getElementById('odom-x');
const odomY = document.getElementById('odom-y');
const odomH = document.getElementById('odom-h');

const COST_GRADIENT = 'linear-gradient(90deg, #0000ff, #7f007f, #ff0000)';
document.querySelectorAll('.sw').forEach(el => {
  const k = el.dataset.color;
  el.style.background = (k === 'global' || k === 'local') ? COST_GRADIENT : rgb(COLORS[k]);
});
document.querySelectorAll('[data-layer]').forEach(cb =>
  cb.addEventListener('change', () => { show[cb.dataset.layer] = cb.checked; dirty(); }));

// ===== TF =====
const yawOf = q => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
const normDeg = a => ((a * 180 / Math.PI) % 360 + 360) % 360;

function getPose(frame, depth = 0) {
  if (!frame || frame === 'map') return { x: 0, y: 0, yaw: 0 };
  const t = tfs[frame];
  if (!t || depth > 12) return null;
  const p = getPose(t.parent, depth + 1);
  if (!p) return null;
  const c = Math.cos(p.yaw), s = Math.sin(p.yaw);
  return { x: p.x + t.x * c - t.y * s, y: p.y + t.x * s + t.y * c, yaw: p.yaw + t.yaw };
}

function onTF(msg) {
  msg.transforms.forEach(t => {
    tfs[t.child_frame_id.replace(/^\//, '')] = {
      parent: t.header.frame_id.replace(/^\//, ''),
      x: t.transform.translation.x, y: t.transform.translation.y,
      yaw: yawOf(t.transform.rotation),
    };
  });
  dirty();
}

// ===== view: pan / zoom =====
let view = { s: 60, x: canvas.width / 2, y: canvas.height / 2 };
let dragging = false, last = null, goalMode = false, goalDrag = null;
const pos = e => {
  const r = canvas.getBoundingClientRect();
  return [(e.clientX - r.left) * canvas.width / r.width, (e.clientY - r.top) * canvas.height / r.height];
};
const pointers = new Map();
let pinch = null;
const pinchState = () => {
  const [p, q] = [...pointers.values()];
  return { d: Math.hypot(p[0] - q[0], p[1] - q[1]) || 1, cx: (p[0] + q[0]) / 2, cy: (p[1] + q[1]) / 2 };
};
canvas.addEventListener('pointerdown', e => {
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, pos(e));
  if (pointers.size === 2) { dragging = false; goalDrag = null; pinch = pinchState(); dirty(); return; }
  if (pointers.size > 2) return;
  dragging = true; last = pos(e);
  if (goalMode) { goalDrag = { x0: last[0], y0: last[1], x1: last[0], y1: last[1] }; dirty(); }
});
canvas.addEventListener('pointermove', e => {
  if (!pointers.has(e.pointerId)) return;
  const p = pos(e);
  pointers.set(e.pointerId, p);
  if (pinch && pointers.size === 2) {                 // two fingers: pinch zoom + pan
    const n = pinchState(), k = n.d / pinch.d;
    view.x = n.cx - (pinch.cx - view.x) * k;
    view.y = n.cy - (pinch.cy - view.y) * k;
    view.s *= k; pinch = n; dirty(); return;
  }
  if (!dragging) return;
  if (goalMode && goalDrag) { goalDrag.x1 = p[0]; goalDrag.y1 = p[1]; dirty(); return; }
  view.x += p[0] - last[0]; view.y += p[1] - last[1];
  last = p; dirty();
});
const endPointer = e => {
  pointers.delete(e.pointerId);
  pinch = null;
  if (!dragging) return;
  dragging = false;
  const g = goalDrag; goalDrag = null;
  if (g && goalMode && e.type === 'pointerup') sendGoal(g);
  dirty();
};
canvas.addEventListener('pointerup', endPointer);
canvas.addEventListener('pointercancel', endPointer);
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const [px, py] = pos(e);
  const k = e.deltaY < 0 ? 1.15 : 1 / 1.15;
  view.x = px - (px - view.x) * k; view.y = py - (py - view.y) * k;
  view.s *= k; dirty();
}, { passive: false });
document.getElementById('btn-fit').onclick = () => {
  view = { s: 60, x: canvas.width / 2, y: canvas.height / 2 }; dirty();
};

// "Set goal" button toggles goal mode; next click on the map sends a Nav2 goal
const goalBtn = document.getElementById('btn-goal');
function setGoalMode(on) {
  goalMode = on; goalDrag = null;
  goalBtn.classList.toggle('active', on);
  canvas.style.cursor = on ? 'crosshair' : 'grab';
  dirty();
}
goalBtn.onclick = () => setGoalMode(!goalMode);
window.addEventListener('keydown', e => {
  if (e.key === 'Escape') { setGoalMode(false); document.querySelectorAll('.maximized').forEach(el => el.classList.remove('maximized')); }
});

// press = position, drag = heading, release = send (short drag keeps heading 0)
function sendGoal(g) {
  setGoalMode(false);
  const x = (g.x0 - view.x) / view.s, y = -(g.y0 - view.y) / view.s;
  const dx = g.x1 - g.x0, dy = g.y1 - g.y0;
  const deg = Math.hypot(dx, dy) > 12 ? Math.round(Math.atan2(-dy, dx) * 180 / Math.PI) : 0;
  if (!confirm(`Navigate to x=${x.toFixed(2)}, y=${y.toFixed(2)}, heading ${deg}\u00B0?`)) return;
  const yaw = deg * Math.PI / 180;
  topic(T.goal, 'geometry_msgs/msg/PoseStamped').publish({
    header: { frame_id: 'map' },
    pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } },
  });
  addLog(`Goal sent: ${x.toFixed(2)}, ${y.toFixed(2)}, ${deg} deg`);
}

// ===== grids =====
// Foxglove / RViz "costmap" palette:
// 0 transparent, 1-98 blue -> red gradient, 99 inscribed = cyan, 100 lethal = purple
const COST_LUT = (() => {
  const t = [];
  for (let v = 0; v <= 100; v++) {
    if (v === 0) t.push([0, 0, 0, 0]);
    else if (v < 99) { const k = (v - 1) / 97; t.push([Math.round(255 * k), 0, Math.round(255 * (1 - k)), 175]); }
    else if (v === 99) t.push([0, 255, 255, 225]);
    else t.push([255, 0, 255, 235]);
  }
  return t;
})();
const CLEAR = [0, 0, 0, 0];

function cellColor(v, kind) {
  if (kind === 'slam') return v >= 50 ? [20, 20, 20, 255] : v >= 0 ? [240, 240, 240, 255] : CLEAR;
  return v >= 0 && v <= 100 ? COST_LUT[v] : CLEAR;
}

function gridToLayer(msg, kind) {
  const w = msg.info.width, h = msg.info.height;
  const c = document.createElement('canvas'); c.width = w; c.height = h;
  const cx = c.getContext('2d');
  const img = cx.createImageData(w, h);
  const d = msg.data;
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      const [r, g, b, a] = cellColor(d[j * w + i], kind), o = ((h - 1 - j) * w + i) * 4;
      img.data[o] = r; img.data[o + 1] = g; img.data[o + 2] = b; img.data[o + 3] = a;
    }
  }
  cx.putImageData(img, 0, 0);
  return {
    kind, w, h, res: msg.info.resolution,
    ox: msg.info.origin.position.x, oy: msg.info.origin.position.y,
    frame: (msg.header.frame_id || 'map').replace(/^\//, ''), img: c,
    outline: kind === 'local' ? 'rgba(0,119,255,0.9)' : null,   // dashed box marks the local costmap window
  };
}

// Nav2 publishes the full costmap rarely and sends changes on <costmap>_updates (partial patches)
function applyUpdate(layer, u) {
  if (!layer || u.x < 0 || u.y < 0 || u.x + u.width > layer.w || u.y + u.height > layer.h) return;
  const cx = layer.img.getContext('2d');
  const img = cx.createImageData(u.width, u.height);
  for (let j = 0; j < u.height; j++) {
    for (let i = 0; i < u.width; i++) {
      const [r, g, b, a] = cellColor(u.data[j * u.width + i], layer.kind), o = ((u.height - 1 - j) * u.width + i) * 4;
      img.data[o] = r; img.data[o + 1] = g; img.data[o + 2] = b; img.data[o + 3] = a;
    }
  }
  cx.putImageData(img, u.x, layer.h - u.y - u.height);
  dirty();
}

// ===== draw =====
let pending = false;
function dirty() { if (!pending) { pending = true; requestAnimationFrame(draw); } }

function arrowHead(x, y, a, s, color) {
  ctx.fillStyle = color; ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x - s * Math.cos(a - 0.45), y - s * Math.sin(a - 0.45));
  ctx.lineTo(x - s * Math.cos(a + 0.45), y - s * Math.sin(a + 0.45));
  ctx.closePath(); ctx.fill();
}

function drawLayer(g) {
  const p = getPose(g.frame);
  if (!p) return;
  const x = g.ox * view.s, y = -(g.oy + g.h * g.res) * view.s, w = g.w * g.res * view.s, h = g.h * g.res * view.s;
  ctx.save();
  ctx.translate(view.x + p.x * view.s, view.y - p.y * view.s);
  ctx.rotate(-p.yaw);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(g.img, x, y, w, h);
  if (g.outline) { ctx.strokeStyle = g.outline; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]); ctx.strokeRect(x, y, w, h); }
  ctx.restore();
}

const toMap = (p, x, y) => {
  const c = Math.cos(p.yaw), s = Math.sin(p.yaw);
  return [p.x + x * c - y * s, p.y + x * s + y * c];
};
const toScreen = (mx, my) => [view.x + mx * view.s, view.y - my * view.s];

function draw() {
  pending = false;
  ctx.fillStyle = '#fff';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (layers.slam && show.slam) drawLayer(layers.slam);
  if (layers.global && show.global) drawLayer(layers.global);
  if (layers.local && show.local) drawLayer(layers.local);

  if (plan && show.path) {
    const p = getPose(plan.frame);
    if (p) {
      ctx.strokeStyle = rgb(COLORS.path); ctx.lineWidth = 3; ctx.beginPath();
      plan.pts.forEach((q, i) => {
        const [x, y] = toScreen(...toMap(p, q[0], q[1]));
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
      ctx.stroke();
    }
  }

  if (scan && show.scan) {
    const p = getPose(scan.frame) || getPose(BASE_FRAME);
    if (p) {
      ctx.fillStyle = rgb(COLORS.scan);
      scan.ranges.forEach((r, i) => {
        if (!isFinite(r) || r <= 0) return;
        const a = scan.angle_min + i * scan.inc;
        const [x, y] = toScreen(...toMap(p, r * Math.cos(a), r * Math.sin(a)));
        ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
      });
    }
  }

  const rp = getPose(BASE_FRAME);
  if (rp) {
    const [x, y] = toScreen(rp.x, rp.y);
    const ang = -rp.yaw;                               // canvas angle of heading
    const v = vel.v || 0, w = vel.w || 0, cfg = getCfg();

    if (Math.abs(w) > 0.005) {                         // rotational velocity: orange arc + arrowhead
      const sweep = Math.max(-1, Math.min(1, w / cfg.maxAng)) * Math.PI * 0.75;
      const end = ang - sweep;
      ctx.strokeStyle = '#ff9500'; ctx.lineWidth = 3; ctx.beginPath();
      ctx.arc(x, y, 15, ang, end, sweep > 0); ctx.stroke();
      arrowHead(x + 15 * Math.cos(end), y + 15 * Math.sin(end), sweep > 0 ? end - Math.PI / 2 : end + Math.PI / 2, 9, '#ff9500');
    }
    if (Math.abs(v) > 0.005) {                         // forward velocity: green arrow + head
      const len = Math.max(-1, Math.min(1, v / cfg.maxLin)) * 50;
      const tx = x + Math.cos(ang) * len, ty = y + Math.sin(ang) * len;
      ctx.strokeStyle = '#00a03c'; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(tx, ty); ctx.stroke();
      arrowHead(tx, ty, len >= 0 ? ang : ang + Math.PI, 11, '#00a03c');
    }
    ctx.fillStyle = '#0077ff'; ctx.beginPath(); ctx.arc(x, y, 7, 0, 7); ctx.fill();
    ctx.strokeStyle = '#0077ff'; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(ang) * 16, y + Math.sin(ang) * 16); ctx.stroke();

    const label = vel.v === null ? 'v --  \u03C9 --'
      : `v ${v.toFixed(2)} m/s  \u03C9 ${w.toFixed(2)} rad/s`;
    ctx.font = '12px sans-serif'; ctx.textBaseline = 'top';
    ctx.lineWidth = 3; ctx.strokeStyle = '#fff'; ctx.strokeText(label, x + 12, y + 12);
    ctx.fillStyle = '#1a2634'; ctx.fillText(label, x + 12, y + 12);
  }

  if (goalDrag) {                                      // goal pose preview
    const g = goalDrag, dx = g.x1 - g.x0, dy = g.y1 - g.y0;
    ctx.strokeStyle = '#0077ff'; ctx.fillStyle = '#0077ff'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(g.x0, g.y0, 6, 0, 7); ctx.fill();
    if (Math.hypot(dx, dy) > 12) {
      const a2 = Math.atan2(dy, dx);
      ctx.beginPath(); ctx.moveTo(g.x0, g.y0); ctx.lineTo(g.x1, g.y1); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(g.x1, g.y1);
      ctx.lineTo(g.x1 - 12 * Math.cos(a2 - 0.4), g.y1 - 12 * Math.sin(a2 - 0.4));
      ctx.lineTo(g.x1 - 12 * Math.cos(a2 + 0.4), g.y1 - 12 * Math.sin(a2 + 0.4));
      ctx.closePath(); ctx.fill();
    }
  }
}

// ===== log =====
function addLog(text, level = 'INFO', source = 'Dashboard') {
  const atBottom = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 24;
  const row = document.createElement('div');
  row.className = 'row ' + level.toLowerCase();
  const meta = document.createElement('div'); meta.className = 'meta';
  const t = document.createElement('time'); t.textContent = new Date().toLocaleTimeString([], { hour12: false });
  const l = document.createElement('span'); l.className = 'lvl'; l.textContent = level;
  const s = document.createElement('b'); s.textContent = source;
  meta.append(t, l, s);
  const m = document.createElement('div'); m.className = 'msg'; m.textContent = text;
  row.append(meta, m);
  logEl.appendChild(row);
  while (logEl.childElementCount > 300) logEl.firstElementChild.remove();
  if (atBottom) logEl.scrollTop = logEl.scrollHeight;
}
// "global_costmap.global_costmap" -> "Global costmap"
function prettyName(n) {
  const s = [...new Set(String(n).split(/[./]/).filter(Boolean))].join(' / ').replace(/_/g, ' ');
  return s.charAt(0).toUpperCase() + s.slice(1);
}
const LEVELS = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL' };

// ===== subscriptions =====
const NAV = { 1: 'ACCEPTED', 2: 'NAVIGATING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED' };

function subscribeAll() {
  topic('/tf', 'tf2_msgs/msg/TFMessage', { throttle_rate: 50 }).subscribe(onTF);
  topic('/tf_static', 'tf2_msgs/msg/TFMessage').subscribe(onTF);

  topic(T.map, 'nav_msgs/msg/OccupancyGrid', { throttle_rate: 1000 })
    .subscribe(m => { layers.slam = gridToLayer(m, 'slam'); dirty(); });
  topic(T.global, 'nav_msgs/msg/OccupancyGrid')
    .subscribe(m => { layers.global = gridToLayer(m, 'global'); dirty(); });
  topic(T.local, 'nav_msgs/msg/OccupancyGrid')
    .subscribe(m => { layers.local = gridToLayer(m, 'local'); dirty(); });
  topic(T.globalUpdates, 'map_msgs/msg/OccupancyGridUpdate').subscribe(u => applyUpdate(layers.global, u));
  topic(T.localUpdates, 'map_msgs/msg/OccupancyGridUpdate').subscribe(u => applyUpdate(layers.local, u));

  topic(T.scan, 'sensor_msgs/msg/LaserScan', { throttle_rate: 100 }).subscribe(m => {
    scan = { frame: m.header.frame_id.replace(/^\//, ''), angle_min: m.angle_min, inc: m.angle_increment, ranges: m.ranges };
    dirty();
  });
  topic(T.plan, 'nav_msgs/msg/Path', { throttle_rate: 200 }).subscribe(m => {
    plan = { frame: m.header.frame_id.replace(/^\//, ''), pts: m.poses.map(p => [p.pose.position.x, p.pose.position.y]) };
    dirty();
  });

  topic(T.odom, 'nav_msgs/msg/Odometry', { throttle_rate: 100 }).subscribe(m => {
    vel.v = m.twist.twist.linear.x; vel.w = m.twist.twist.angular.z;
    const p = m.pose.pose.position;
    const yaw = yawOf(m.pose.pose.orientation);
    odomX.textContent = p.x.toFixed(2) + ' m';
    odomY.textContent = p.y.toFixed(2) + ' m';
    odomH.textContent = normDeg(yaw).toFixed(1) + '\u00B0';
    dirty();
  });

  topic(T.navStatus, 'action_msgs/msg/GoalStatusArray').subscribe(m => {
    const s = m.status_list;
    navEl.textContent = 'Nav: ' + (s.length ? NAV[s[s.length - 1].status] || '?' : 'IDLE');
  });

  topic('/rosout', 'rcl_interfaces/msg/Log', { queue_length: 100 }).subscribe(m => {
    if (!NAVMAP_RE.test(m.name) && !NAVMAP_RE.test(m.msg)) return;
    addLog(String(m.msg).replace(/\s+/g, ' ').trim(), LEVELS[m.level] || 'INFO', prettyName(m.name));
  });
}

// ===== toolbar =====
document.getElementById('btn-save').onclick = () => {
  const name = prompt('Map name:', 'map');
  if (!name) return;
  new ROSLIB.Service({ ros, name: T.saveMap, serviceType: 'slam_toolbox/srv/SaveMap' })
    .callService({ name: { data: name } },
      r => alert(r.result === 0 ? 'Map saved on rover: ' + name : 'Save failed, code ' + r.result),
      err => alert('Save failed: ' + err));
};

const helioQ = document.getElementById('helio-q');
const helioReply = document.getElementById('helio-reply');

// prefer an en-GB female voice; which voices exist depends on the device / browser
function pickVoice() {
  const gb = speechSynthesis.getVoices().filter(v => /^en[-_]GB$/i.test(v.lang));
  const femaleNames = /female|hazel|serena|kate|libby|sonia|susan|fiona|martha|stephanie|amy|emma|abbi|bella|holly|maisie|mia|olivia|hollie/i;
  const maleNames = /\bmale\b|daniel|arthur|george|ryan|thomas|oliver|alfie|elliot|noah|ollie/i;
  return gb.find(v => /female/i.test(v.name)) || gb.find(v => femaleNames.test(v.name))
    || gb.find(v => !maleNames.test(v.name)) || gb[0] || null;
}

function speak(text) {
  if (SPEAK_ON_ROVER && ros) topic('/robot_operator/speak_device', 'std_msgs/msg/String').publish({ data: text });
  if (!('speechSynthesis' in window)) return;
  const u = new SpeechSynthesisUtterance(text);
  u.lang = 'en-GB';
  const v = pickVoice(); if (v) u.voice = v;
  speechSynthesis.cancel();
  speechSynthesis.speak(u);
}

async function askHelio() {
  const text = helioQ.value.trim();
  if (!text) return;
  helioReply.textContent = 'Thinking\u2026';
  try {
    const r = await fetch(`http://${IP}:${BACKEND_PORT}/helio/command`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ text, lang: 'en' }),
      signal: AbortSignal.timeout(5000),
    }).then(r => { if (!r.ok) throw new Error(r.status); return r.json(); });
    const reply = r.reply || r.text || '(no reply)';
    helioReply.textContent = reply;
    speak(reply);
  } catch (e) {
    helioReply.textContent = 'Helio is not connected.';
    speak('Helio is not connected.');
  }
}
document.getElementById('helio-ask').onclick = askHelio;
helioQ.addEventListener('keydown', e => { if (e.key === 'Enter') askHelio(); });

document.getElementById('btn-cancel').onclick = () => {
  new ROSLIB.Service({ ros, name: T.navCancel, serviceType: 'action_msgs/srv/CancelGoal' })
    .callService({}, () => addLog('Cancel requested'), err => alert('Cancel failed: ' + err));
  haltJoystick();
};

document.getElementById('btn-clear').onclick = () => {
  [['global', T.clearGlobal], ['local', T.clearLocal]].forEach(([label, name]) =>
    new ROSLIB.Service({ ros, name, serviceType: 'nav2_msgs/srv/ClearEntireCostmap' })
      .callService({}, () => addLog(`Cleared the ${label} costmap obstacle layers`, 'INFO', 'Dashboard'),
        err => alert(`Clear ${label} costmap failed: ` + err)));
};

// full screen for the map and every panel (falls back to a fixed overlay if the API is unavailable)
function toggleFull(el) {
  const on = document.fullscreenElement === el || el.classList.contains('maximized');
  if (on) {
    if (document.fullscreenElement) document.exitFullscreen();
    el.classList.remove('maximized');
    return;
  }
  if (el.requestFullscreen) el.requestFullscreen().catch(() => el.classList.add('maximized'));
  else el.classList.add('maximized');
}
document.getElementById('btn-full').onclick = () => toggleFull(document.getElementById('map-wrap'));
document.querySelectorAll('.panel .fs').forEach(b => b.onclick = () => toggleFull(b.closest('.panel')));
document.getElementById('btn-app-full').onclick = () => {
  if (!document.documentElement.requestFullscreen) { alert('Full screen is not supported on this browser.'); return; }
  if (document.fullscreenElement === document.documentElement) document.exitFullscreen();
  else document.documentElement.requestFullscreen();
};

// camera panels: <img> pointed straight at the MJPEG streams, retries if the streamer is down
function startCameras() {
  CAMS.forEach(([id, path]) => {
    const img = document.getElementById(id), msg = img.nextElementSibling;
    const url = `http://${IP}:${CAM_PORT}${path}`;
    const load = () => { img.src = url + '?t=' + Date.now(); };
    img.onload = () => { msg.hidden = true; };
    img.onerror = () => { msg.hidden = false; setTimeout(load, 3000); };
    load();
  });
}

// ===== joystick: single draggable square pad, returns to center on release =====
const JOY_DT = 0.05;                       // 20 Hz cmd_vel loop
const joyTarget = { l: 0, a: 0 };          // -1..1 (l = forward, a = yaw)
const joyCur = { l: 0, a: 0 };             // m/s, rad/s
let joyTimer = null, cmdPub = null;

function drive(lin, ang) {
  if (!cmdPub) cmdPub = topic(T.cmdVel, 'geometry_msgs/msg/Twist');
  cmdPub.publish({ linear: { x: lin, y: 0, z: 0 }, angular: { x: 0, y: 0, z: ang } });
}
const approach = (cur, target, rate) => {
  const d = target - cur, m = rate * JOY_DT;
  return Math.abs(d) <= m ? target : cur + Math.sign(d) * m;
};
function joyTick() {
  const c = getCfg();
  joyCur.l = Math.max(-c.maxLin, Math.min(c.maxLin, approach(joyCur.l, joyTarget.l * c.maxLin, c.accel)));
  joyCur.a = Math.max(-c.maxAng, Math.min(c.maxAng, approach(joyCur.a, joyTarget.a * c.maxAng, c.angAccel)));
  drive(joyCur.l, joyCur.a);
  if (!joyTarget.l && !joyTarget.a && !joyCur.l && !joyCur.a) { clearInterval(joyTimer); joyTimer = null; }
}
function haltJoystick() {
  joyTarget.l = joyTarget.a = 0; joyCur.l = joyCur.a = 0;
  clearInterval(joyTimer); joyTimer = null;
  const box = document.getElementById('joy');
  if (box) { box.classList.remove('dragging'); box.querySelector('.joy-knob').style.transform = 'translate(-50%, -50%)'; }
  if (ros) drive(0, 0);
}

const joyBox = document.getElementById('joy');
const joyKnob = joyBox.querySelector('.joy-knob');
let joyPtr = null;

function joyMove(clientX, clientY) {
  const r = joyBox.getBoundingClientRect();
  const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
  const maxR = r.width / 2 - joyKnob.offsetWidth / 2;   // stay inside the box
  let dx = clientX - cx, dy = clientY - cy;
  const d = Math.hypot(dx, dy);
  if (d > maxR) { dx = dx / d * maxR; dy = dy / d * maxR; }
  joyKnob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
  joyTarget.a = -(dx / maxR);       // right = clockwise (negative yaw)
  joyTarget.l = -(dy / maxR);       // up    = forward
  if (!joyTimer) joyTimer = setInterval(joyTick, JOY_DT * 1000);
}

joyBox.addEventListener('pointerdown', e => {
  e.preventDefault();
  joyPtr = e.pointerId;
  joyBox.setPointerCapture(joyPtr);
  joyBox.classList.add('dragging');
  joyMove(e.clientX, e.clientY);
});
joyBox.addEventListener('pointermove', e => {
  if (e.pointerId !== joyPtr) return;
  joyMove(e.clientX, e.clientY);
});
const joyEnd = e => {
  if (e.pointerId !== joyPtr) return;
  joyPtr = null;
  joyBox.classList.remove('dragging');
  joyKnob.style.transform = 'translate(-50%, -50%)';   // snap back to 0,0
  joyTarget.l = 0; joyTarget.a = 0;                    // ramp down via joyTick
};
joyBox.addEventListener('pointerup', joyEnd);
joyBox.addEventListener('pointercancel', joyEnd);
joyBox.addEventListener('lostpointercapture', joyEnd);

window.addEventListener('blur', haltJoystick);

// ===== start =====
(async () => {
  if (!(await login())) { document.body.textContent = 'Not signed in. Reload to try again.'; return; }
  navEl.textContent = 'Nav: IDLE';
  if (innerWidth < 640) document.getElementById('legend').open = false;
  if ('speechSynthesis' in window) speechSynthesis.getVoices();   // warm up the voice list
  subscribeAll();
  startCameras();
  dirty();
})();
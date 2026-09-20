// =====================================================================
//  CONFIGURATION — adjust topic names / endpoints here
// =====================================================================
const CFG = {
    topics: {
        map: '/map',
        plan: '/plan',
        odom: '/odom',
        tf: '/tf',
        tfStatic: '/tf_static',
        joint: '/joint_states',
        cmd: '/velocity_controller/commands',
        cmdVel: '/cmd_vel',
        joy: '/joy',
        camera: '/camera/image_raw',
        scan: '/scan',
        gcost: '/ui/global_costmap',
        gcostUpd: '/global_costmap/costmap_updates',
        lcost: '/ui/local_costmap',
        lcostUpd: '/local_costmap/costmap_updates',
        rosout: '/rosout'
    },
    mapFrame: 'map',
    baseFrames: ['base_link', 'base_footprint'],
    cmdOrder: ['fr', 'fl', 'br', 'bl'],   // commanded: left<->right swapped
    // /joint_states velocity order: FR, FL, RR, RL
    jointOrder: ['fl', 'fr', 'bl', 'br'],   // actual: left<->right swapped vs original
    jointOverrides: {},
    modeUrl: '/system/mode',
    modeTimeoutMs: 120000,  // Updated to 2 minutes (120000 ms) for autonomous/system mode switches
    modePollMs: 1000,
    modeIdlePollMs: 5000,
    chartWindowSec: 30,
    joyRateHz: 20
};

const MODE_LABELS = {
    manual: 'Manual Driving (Mapping Off)',
    slam: 'Manual Driving + New Mapping',
    slam_update: 'Autonomous Driving + Mapping',
    nav: 'Autonomous Driving + Map Update'
};

// =====================================================================
//  Notifications
// =====================================================================
const NOTIFY_TTL = { ERROR: 12000, WARNING: 8000, INFO: 5000 };

function notify(level, message) {
    level = (level || 'INFO').toUpperCase();
    const stack = document.getElementById('notify-stack');
    for (const el of stack.children) {
        if (el.dataset.key === level + message) { resetNoticeTimer(el, level); return; }
    }
    const el = document.createElement('div');
    el.className = 'notice ' + level.toLowerCase();
    el.dataset.key = level + message;
    el.setAttribute('role', level === 'ERROR' ? 'alert' : 'status');
    el.innerHTML = '<div class="notice-hd"><span class="notice-lvl">' + level +
        '</span><button class="notice-x" aria-label="Dismiss">&times;</button></div><div class="notice-msg"></div>';
    el.querySelector('.notice-msg').textContent = message;
    el.querySelector('.notice-x').onclick = () => dismissNotice(el);
    stack.appendChild(el);
    while (stack.children.length > 5) stack.removeChild(stack.firstChild);
    resetNoticeTimer(el, level);
}
function resetNoticeTimer(el, level) {
    clearTimeout(el._t);
    el._t = setTimeout(() => dismissNotice(el), NOTIFY_TTL[level] || 5000);
}
function dismissNotice(el) {
    clearTimeout(el._t);
    el.classList.add('leaving');
    setTimeout(() => el.remove(), 220);
}

// =====================================================================
//  Theme
// =====================================================================
const theme = {};
function readTheme() {
    const cs = getComputedStyle(document.body);
    ['--map-bg', '--map-free', '--map-occ', '--map-grid', '--series-a', '--series-b', '--border', '--muted', '--text', '--success', '--primary', '--panel']
        .forEach(k => theme[k] = cs.getPropertyValue(k).trim());
}
function applyTheme(isLight) {
    document.body.classList.toggle('light-theme', isLight);
    readTheme();
    mapImageDirty = true; needsDraw = true;
}
function toggleTheme() {
    const isLight = !document.body.classList.contains('light-theme');
    applyTheme(isLight);
    try { localStorage.setItem('milusions-theme', isLight ? 'light' : 'dark'); } catch (e) {}
}

// =====================================================================
//  State & Watchdogs
// =====================================================================
let mapImageDirty = false, needsDraw = true;
let latestMap = null;
let latestPlan = []; 
let navStatus = 'Idle'; 
let distanceRemaining = 0;
const mapCanvasOff = document.createElement('canvas');
const tfTree = {};
let odom = null;
let robot = null;
let latestScan = null;
let showLaserScan = true;

// ---------- Layers legend ----------
const layers = { map: true, scan: true, gcost: true, lcost: true, plan: true };
const LAYER_DEFS = [
    ['map',   'Map',            '#8a8f98'],
    ['scan',  'Laser scan',     '#3b82f6'],
    ['gcost', 'Global costmap', '#e879f9'],
    ['lcost', 'Local costmap',  '#f59e0b'],
    ['plan',  'Path / goal',    '#22c55e']
];
const COST_ONLY = { scan: 1, gcost: 1, lcost: 1, plan: 1 };   // hidden in plain manual mode
function layerOn(k) {
    if (typeof currentMode !== 'undefined' && currentMode === 'manual' && COST_ONLY[k]) return false;
    return !!layers[k];
}
function toggleLayer(k) { layers[k] = !layers[k]; showLaserScan = layers.scan; needsDraw = true; }
function updateLegend() {
    const box = document.getElementById('layer-legend'); if (!box) return;
    if (!box.dataset.built) {
        box.innerHTML = '<div class="ll-title">Layers</div>' + LAYER_DEFS.map(([k, label, col]) =>
            '<label class="ll-row" data-k="' + k + '"><input type="checkbox" data-k="' + k + '"><i style="background:' + col + '"></i>' + label + '</label>').join('');
        box.querySelectorAll('input').forEach(inp => inp.addEventListener('change', () => toggleLayer(inp.dataset.k)));
        box.addEventListener('pointerdown', (e) => { e.stopPropagation(); if (typeof closePoseActionPopup === 'function') closePoseActionPopup(); });
        box.dataset.built = '1';
    }
    LAYER_DEFS.forEach(([k]) => {
        const dis = (typeof currentMode !== 'undefined' && currentMode === 'manual' && COST_ONLY[k]);
        const inp = box.querySelector('input[data-k="' + k + '"]');
        inp.disabled = !!dis; inp.checked = !!layers[k] && !dis;
        inp.parentElement.classList.toggle('dis', !!dis);
    });
}

// ---------- Costmaps ----------
const costmaps = {
    gcost: { g: null, cv: document.createElement('canvas'), dirty: false },
    lcost: { g: null, cv: document.createElement('canvas'), dirty: false }
};
const COST_LUT = (() => {
    const a = new Uint8ClampedArray(101 * 4);
    for (let v = 1; v <= 100; v++) {
        const t = v / 100, k = v * 4;
        a[k] = 60 + 195 * t; a[k + 1] = 200 * (1 - t) * (1 - t); a[k + 2] = 255 - 120 * t;
        a[k + 3] = v >= 99 ? 200 : 60 + 110 * t;
    }
    return a;
})();
function onCostmap(key, m) {
    costmaps[key].g = {
        w: m.info.width, h: m.info.height, res: m.info.resolution,
        ox: m.info.origin.position.x, oy: m.info.origin.position.y,
        yaw: quatYaw(m.info.origin.orientation), data: m.data
    };
    costmaps[key].dirty = true; needsDraw = true;
}
function onCostmapUpdate(key, u) {
    const g = costmaps[key].g; if (!g) return;
    for (let r = 0; r < u.height; r++) {
        const dst = (u.y + r) * g.w + u.x, src = r * u.width;
        for (let c = 0; c < u.width; c++) g.data[dst + c] = u.data[src + c];
    }
    costmaps[key].dirty = true; needsDraw = true;
}
function rebuildCostmap(key) {
    const cm = costmaps[key], g = cm.g; if (!g) return;
    cm.cv.width = g.w; cm.cv.height = g.h;
    const ctx = cm.cv.getContext('2d'), img = ctx.createImageData(g.w, g.h), d = img.data;
    for (let i = 0, n = g.w * g.h; i < n; i++) {
        let v = g.data[i]; if (v <= 0) continue; if (v > 100) v = 100;
        const k = i * 4, l = v * 4;
        d[k] = COST_LUT[l]; d[k + 1] = COST_LUT[l + 1]; d[k + 2] = COST_LUT[l + 2]; d[k + 3] = COST_LUT[l + 3];
    }
    ctx.putImageData(img, 0, 0); cm.dirty = false;
}
function drawCostmap(ctx, key) {
    const cm = costmaps[key], g = cm.g; if (!g) return;
    if (cm.dirty) rebuildCostmap(key);
    const c = Math.cos(g.yaw), s = Math.sin(g.yaw), k = g.res * view.s, { W, H } = mapSize();
    ctx.save(); ctx.imageSmoothingEnabled = false;
    ctx.transform(-k * s, -k * c, -k * c, k * s,
        W / 2 - (g.oy - view.vy) * view.s, H / 2 - (g.ox - view.vx) * view.s);
    ctx.drawImage(cm.cv, 0, 0); ctx.restore();
}

// ---------- Nav2 log bubbles (from /rosout) ----------
const bubbles = [];
const NAV_NODE_RE = /bt_navigator|controller_server|planner_server|behavior_server|waypoint_follower|smoother|nav2/i;
const NAV_MSG_RE = /goal (reached|succeeded)|reached (the )?goal|collision ahead|failed to cancel|failed to|aborted|cancel(l)?ed|no valid (path|control)|stuck|timed? ?out|recover|invalid path|goal (canceled|failed)/i;
function onRosout(m) {
    if (!m || !NAV_NODE_RE.test(m.name || '') || !NAV_MSG_RE.test(m.msg || '')) return;
    const text = String(m.msg).replace(/\s+/g, ' ').slice(0, 70);
    const now = performance.now(), last = bubbles[bubbles.length - 1];
    if (last && last.text === text && now - last.t < 2000) { last.t = now; return; }
    bubbles.push({ text, node: String(m.name).replace(/^\//, ''), t: now });
    if (bubbles.length > 3) bubbles.shift();
    needsDraw = true;
}
function drawBubbles(ctx, p, r) {
    const now = performance.now(), TTL = 7000;
    for (let i = bubbles.length - 1; i >= 0; i--) if (now - bubbles[i].t > TTL) bubbles.splice(i, 1);
    if (!bubbles.length) return;
    ctx.save(); ctx.font = '600 12px Roboto, sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    let y = p[1] - r * 2.6 - 12;
    for (let i = bubbles.length - 1; i >= 0; i--) {
        const b = bubbles[i], age = now - b.t;
        const a = age > TTL - 1500 ? Math.max(0, (TTL - age) / 1500) : 1;
        const w = ctx.measureText(b.text).width + 20, h = 24, x = p[0] - w / 2;
        ctx.globalAlpha = a * (i === bubbles.length - 1 ? 1 : 0.8);
        ctx.fillStyle = '#16a34a'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.roundRect(x, y - h, w, h, 8); ctx.fill(); ctx.stroke();
        if (i === bubbles.length - 1) {          // tail pointing at the rover
            ctx.beginPath(); ctx.moveTo(p[0] - 6, y - 0.5); ctx.lineTo(p[0], y + 7); ctx.lineTo(p[0] + 6, y - 0.5);
            ctx.closePath(); ctx.fill();
        }
        ctx.fillStyle = '#fff'; ctx.fillText(b.text, p[0], y - h / 2 + 1);
        y -= h + 5;
    }
    ctx.restore();
}
let currentCmdVel = { linear: 0, angular: 0 };

let activeNavBtn = null;
let navInitTimeout = null;

// "Waiting for Nav2" placeholder animation (drawn on the map canvas until /plan arrives)
let planAnim = null;
const PLAN_DRAW_MS = 1800, PLAN_HOLD_MS = 600, PLAN_FADE_MS = 300;

function startPlanAnim(targets) {
    const list = (targets || []).filter(t => t && isFinite(t.x) && isFinite(t.y)).map(t => ({ x: t.x, y: t.y }));
    planAnim = list.length ? { targets: list, t0: performance.now() } : null;
    const badge = document.getElementById('map-plan-badge');
    if (badge) badge.classList.toggle('show', !!planAnim);
    needsDraw = true;
}
function stopPlanAnim() {
    planAnim = null;
    const badge = document.getElementById('map-plan-badge');
    if (badge) badge.classList.remove('show');
    needsDraw = true;
}

function setNavLoading(btn) {
    if (activeNavBtn) {
        activeNavBtn.classList.remove('loading');
        activeNavBtn.disabled = false;
    }
    activeNavBtn = btn;
    if (activeNavBtn) {
        activeNavBtn.classList.add('loading');
        activeNavBtn.disabled = true;
    }
}

function clearNavLoading() {
    stopPlanAnim();
    if (navInitTimeout) {
        clearTimeout(navInitTimeout);
        navInitTimeout = null;
    }
    if (activeNavBtn) {
        activeNavBtn.classList.remove('loading');
        activeNavBtn.disabled = false;
        activeNavBtn = null;
    }
}

function startNavWatchdog() {
    if (navInitTimeout) clearTimeout(navInitTimeout);
    navInitTimeout = setTimeout(async () => {
        clearNavLoading();
        navStatus = 'Aborted';
        latestPlan = [];
        try {
            await postJSON('/abort');
        } catch (e) {}
        notify('ERROR', 'something is wrong with the navigation server.');
    }, 20000);
}

const wheelIds = ['fl', 'fr', 'bl', 'br'];
const MAX_WHEEL_RAD_S = 20;
const RAD_S_TO_RPM = 60 / (2 * Math.PI);
const MAX_WHEEL_RPM = 95;   // fixed chart range: -95 .. +95 RPM
const series = {};
wheelIds.forEach(id => series[id] = { a: [], c: [] });

// =====================================================================
//  ROS connection
// =====================================================================
// ---- Rover IP (stored in localStorage as "rover_ip") ----
const ROVER_IP_KEY = 'rover_ip';
function normalizeRoverIp(v) {
    return (v || '').trim().replace(/^[a-z]+:\/\//i, '').replace(/[\/?#].*$/, '').replace(/:\d+$/, '');
}
function isValidRoverHost(h) {
    return /^(\d{1,3}(\.\d{1,3}){3}|[a-z0-9]([a-z0-9.-]*[a-z0-9])?)$/i.test(h);
}
function getRoverIp() {
    try { return normalizeRoverIp(localStorage.getItem(ROVER_IP_KEY)); } catch (e) { return ''; }
}
function saveRoverIpAndReload(v) {
    const ip = normalizeRoverIp(v);
    if (!isValidRoverHost(ip)) return false;
    try { localStorage.setItem(ROVER_IP_KEY, ip); } catch (e) {}
    location.reload();
    return true;
}
const ROVER_IP = getRoverIp();
const ROVER_HOST = ROVER_IP;   // fallback only while the IP dialog is showing

const ROS_URL = 'ws://' + ROVER_HOST + ':9090';
const ros = new ROSLIB.Ros({ url: ROS_URL });
const statusEl = document.getElementById('ros-status');
let rosState = 'init';
let subs = [];
let joyTopic = null;

ros.on('connection', () => {
    statusEl.textContent = 'ROS Connected';
    statusEl.classList.add('connected');
    const overlay = document.getElementById('connection-overlay');
    if (overlay) overlay.style.display = 'none';
    if (rosState !== 'up') notify('INFO', 'Connected to rosbridge.');
    rosState = 'up';
    subscribeAll();
    fetchMaps({ notifyError: false });
});
ros.on('error', () => {
    statusEl.textContent = 'Connection Error';
    statusEl.classList.remove('connected');
});
ros.on('close', () => {
    statusEl.textContent = 'Disconnected';
    statusEl.classList.remove('connected');
    const overlay = document.getElementById('connection-overlay');
    if (overlay) overlay.style.display = 'flex';
    if (rosState === 'up') notify('WARNING', 'Rosbridge connection lost. Retrying…');
    else if (rosState === 'init') notify('ERROR', 'Unable to connect to rosbridge at ' + ROS_URL + '.');
    rosState = 'down';
    setTimeout(() => { try { ros.connect(ROS_URL); } catch (e) {} }, 5000);
});

function strip(f) { return (f || '').replace(/^\//, ''); }
function quatYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

function subscribeAll() {
    subs.forEach(s => { try { s.unsubscribe(); } catch (e) {} });
    subs = [];
    const T = CFG.topics;

    const mkT = (name, type, opts, cb) => {
        const t = new ROSLIB.Topic(Object.assign({ ros, name, messageType: type }, opts));
        t.subscribe(cb); subs.push(t); return t;
    };
    const mk = (name, type, cb) => {
        const t = new ROSLIB.Topic({ ros, name, messageType: type });
        t.subscribe(cb); subs.push(t); return t;
    };

    mkT(T.map, 'nav_msgs/OccupancyGrid', { throttle_rate: 500, queue_length: 1 }, (m) => {
        latestMap = {
            w: m.info.width, h: m.info.height, res: m.info.resolution,
            ox: m.info.origin.position.x, oy: m.info.origin.position.y,
            yaw: quatYaw(m.info.origin.orientation), data: m.data
        };
        mapImageDirty = true; needsDraw = true;
    });

    mk(T.plan, 'nav_msgs/Path', (m) => {
        if (m && m.poses) {
            latestPlan = m.poses.map(p => ({
                x: p.pose.position.x,
                y: p.pose.position.y,
                yaw: quatYaw(p.pose.orientation)
            }));
            if (latestPlan.length > 0) {
                if (navStatus === 'Idle') {
                    navStatus = 'Navigating';
                }
                clearNavLoading();
            }
            needsDraw = true;
        }
    });

    mk(T.odom, 'nav_msgs/Odometry', (m) => {
        const v = m.twist.twist.linear;
        odom = {
            x: m.pose.pose.position.x, y: m.pose.pose.position.y,
            yaw: quatYaw(m.pose.pose.orientation), speed: Math.hypot(v.x, v.y)
        };
        needsDraw = true;
    });

    mk(T.cmdVel, 'geometry_msgs/Twist', (m) => {
        currentCmdVel = { linear: m.linear.x, angular: m.angular.z };
        needsDraw = true;
    });

    const onTf = (m) => {
        m.transforms.forEach(t => {
            tfTree[strip(t.child_frame_id)] = {
                parent: strip(t.header.frame_id),
                x: t.transform.translation.x, y: t.transform.translation.y,
                yaw: quatYaw(t.transform.rotation)
            };
        });
        needsDraw = true;
    };
    mk(T.tf, 'tf2_msgs/TFMessage', onTf);
    mk(T.tfStatic, 'tf2_msgs/TFMessage', onTf);

    mk(T.joint, 'sensor_msgs/JointState', onJointState);

    mk(T.scan, 'sensor_msgs/LaserScan', (m) => {
        latestScan = {
            frame: strip(m.header && m.header.frame_id),
            stamp: m.header && m.header.stamp ? m.header.stamp : null,
            angleMin: Number(m.angle_min) || 0,
            angleInc: Number(m.angle_increment) || 0,
            rangeMin: Number(m.range_min) || 0,
            rangeMax: Number(m.range_max) || 0,
            ranges: Array.isArray(m.ranges) ? m.ranges.slice() : []
        };
        needsDraw = true;
    });

    mkT(T.gcost, 'nav_msgs/OccupancyGrid', { throttle_rate: 1500, queue_length: 1 }, (m) => layerOn('gcost') && onCostmap('gcost', m));
    mkT(T.lcost, 'nav_msgs/OccupancyGrid', { throttle_rate: 1000, queue_length: 1 }, (m) => layerOn('lcost') && onCostmap('lcost', m));
    subs.push((() => { const t = new ROSLIB.Topic({ ros, name: T.rosout, messageType: 'rcl_interfaces/msg/Log', queue_length: 200 });
        t.subscribe(onRosout); return t; })());

    const cam = new ROSLIB.Topic({ ros, name: T.camera, messageType: 'sensor_msgs/Image', throttle_rate: 100, queue_length: 1 });
    cam.subscribe(onCameraImage); subs.push(cam);

    ros.getTopicType(T.cmd, (type) => {
        if (!type) return;
        mk(T.cmd, type, onCommand);
    }, () => notify('WARNING', 'Command topic ' + T.cmd + ' not found; wheel command traces will be empty.'));

    joyTopic = new ROSLIB.Topic({ ros, name: T.joy, messageType: 'sensor_msgs/Joy' });
}

function resolveFramePose(frame) {
    frame = strip(frame);
    if (!frame) return null;
    if (frame === CFG.mapFrame) return { x: 0, y: 0, yaw: 0 };

    let cur = frame;
    let acc = { x: 0, y: 0, yaw: 0 };
    let depth = 0;
    while (cur !== CFG.mapFrame && depth++ < 20) {
        const t = tfTree[cur];
        if (!t) return null;
        const c = Math.cos(t.yaw), ss = Math.sin(t.yaw);
        acc = {
            x: t.x + c * acc.x - ss * acc.y,
            y: t.y + ss * acc.x + c * acc.y,
            yaw: t.yaw + acc.yaw
        };
        cur = t.parent;
    }
    return cur === CFG.mapFrame ? acc : null;
}

function drawLaserScan(ctx) {
    if (!layerOn('scan') || !latestScan || !latestScan.ranges.length) return;

    const scan = latestScan;
    const pose = resolveFramePose(scan.frame) || robot;
    if (!pose) return;

    const framePose = resolveFramePose(scan.frame);
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    const step = Math.max(1, Math.ceil(scan.ranges.length / 1800));

    ctx.save();
    ctx.fillStyle = theme['--primary'];
    ctx.globalAlpha = 0.72;

    for (let i = 0; i < scan.ranges.length; i += step) {
        const r = Number(scan.ranges[i]);
        if (!Number.isFinite(r) || r < scan.rangeMin || r > scan.rangeMax) continue;

        const a = scan.angleMin + i * scan.angleInc;
        let lx = r * Math.cos(a), ly = r * Math.sin(a);

        if (framePose) {
            const fc = Math.cos(framePose.yaw), fs = Math.sin(framePose.yaw);
            const mx = framePose.x + fc * lx - fs * ly;
            const my = framePose.y + fs * lx + fc * ly;
            const p = w2s(mx, my);
            ctx.fillRect(Math.round(p[0]) - 1, Math.round(p[1]) - 1, 2, 2);
        } else {
            const mx = pose.x + c * lx - s * ly;
            const my = pose.y + s * lx + c * ly;
            const p = w2s(mx, my);
            ctx.fillRect(Math.round(p[0]) - 1, Math.round(p[1]) - 1, 2, 2);
        }
    }
    ctx.restore();
}

function resolveRobotPose() {
    for (const base of CFG.baseFrames) {
        if (!tfTree[base]) continue;
        let cur = base, acc = { x: 0, y: 0, yaw: 0 }, depth = 0;
        while (cur !== CFG.mapFrame && depth++ < 12) {
            const t = tfTree[cur];
            if (!t) { acc = null; break; }
            const c = Math.cos(t.yaw), s = Math.sin(t.yaw);
            acc = { x: t.x + c * acc.x - s * acc.y, y: t.y + s * acc.x + c * acc.y, yaw: t.yaw + acc.yaw };
            cur = t.parent;
        }
        if (acc && cur === CFG.mapFrame) return { x: acc.x, y: acc.y, yaw: acc.yaw, src: 'map' };
    }
    if (odom) return { x: odom.x, y: odom.y, yaw: odom.yaw, src: 'odom' };
    return null;
}

function classifyJoint(name) {
    if (CFG.jointOverrides[name]) return CFG.jointOverrides[name];
    const n = name.toLowerCase();
    if (/(^|[_\-])(fl|lf)([_\-]|$)|front[_\-]?left|left[_\-]?front/.test(n)) return 'fl';
    if (/(^|[_\-])(fr|rf)([_\-]|$)|front[_\-]?right|right[_\-]?front/.test(n)) return 'fr';
    if (/(^|[_\-])(bl|lb|rl|lr)([_\-]|$)|(rear|back)[_\-]?left|left[_\-]?(rear|back)/.test(n)) return 'bl';
    if (/(^|[_\-])(br|rb|rr)([_\-]|$)|(rear|back)[_\-]?right|right[_\-]?(rear|back)/.test(n)) return 'br';
    return null;
}

function formatSigned(v, digits = 2) {
    const n = Number(v);
    if (!Number.isFinite(n)) return '—';
    if (Math.abs(n) < 0.0005) return '0.' + '0'.repeat(digits);
    return (n > 0 ? '+' : '') + n.toFixed(digits);
}

function pushPoint(arr, v) {
    const n = Number(v);
    if (!Number.isFinite(n)) return;
    const now = performance.now() / 1000;
    arr.push({ t: now, v: n });
    const cutoff = now - CFG.chartWindowSec - 5;
    while (arr.length && arr[0].t < cutoff) arr.shift();
}
function onJointState(m) {
    if (!m.velocity) return;
    CFG.jointOrder.forEach((id, i) => {
        if (typeof m.velocity[i] === 'number') pushPoint(series[id].a, m.velocity[i] * RAD_S_TO_RPM);
    });
}
function onCommand(m) {
    if (!m.data) return;
    CFG.cmdOrder.forEach((id, i) => { if (typeof m.data[i] === 'number') pushPoint(series[id].c, m.data[i] * RAD_S_TO_RPM); });
}

const camCanvas = document.getElementById('cam-canvas');
let lastCamFrame = 0;
function onCameraImage(m) {
    const w = m.width, h = m.height;
    if (!w || !h) return;
    const bin = atob(m.data), step = m.step || 0;
    const enc = (m.encoding || '').toLowerCase();
    const ch = { rgb8: 3, bgr8: 3, rgba8: 4, bgra8: 4, mono8: 1, '8uc1': 1, '8uc3': 3 }[enc];
    if (!ch) {
        if (!onCameraImage.warned) { onCameraImage.warned = true; notify('WARNING', 'Unsupported camera encoding: ' + m.encoding + '.'); }
        return;
    }
    if (camCanvas.width !== w || camCanvas.height !== h) { camCanvas.width = w; camCanvas.height = h; }
    const ctx = camCanvas.getContext('2d'), img = ctx.createImageData(w, h), d = img.data;
    const stride = step || w * ch, swap = (enc === 'bgr8' || enc === 'bgra8');
    for (let y = 0; y < h; y++) {
        let si = y * stride, di = y * w * 4;
        for (let x = 0; x < w; x++, si += ch, di += 4) {
            if (ch === 1) { d[di] = d[di + 1] = d[di + 2] = bin.charCodeAt(si); }
            else {
                const r = bin.charCodeAt(si), g = bin.charCodeAt(si + 1), b = bin.charCodeAt(si + 2);
                d[di] = swap ? b : r; d[di + 1] = g; d[di + 2] = swap ? r : b;
            }
            d[di + 3] = 255;
        }
    }
    ctx.putImageData(img, 0, 0);
    lastCamFrame = performance.now();
    document.getElementById('cam-empty').style.display = 'none';
    document.getElementById('cam-meta').textContent = '· ' + w + '×' + h;
}

const chartCanvases = {};
wheelIds.forEach(id => chartCanvases[id] = document.getElementById('chart-' + id));

function fitCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
        canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w, h };
}

function drawChart(id) {
    const { ctx, w, h } = fitCanvas(chartCanvases[id]);
    if (!w || !h) return;
    ctx.clearRect(0, 0, w, h);
    const now = performance.now() / 1000, win = CFG.chartWindowSec, t0 = now - win;
    const L = 46, R = 10, T = 10, B = 20;
    const pw = w - L - R, ph = h - T - B;
    const A = series[id].a.filter(p => p.t >= t0 - 1), C = series[id].c;

    const lo = -MAX_WHEEL_RPM;
    const hi = MAX_WHEEL_RPM;

    const X = t => L + ((t - t0) / win) * pw;
    const Y = v => T + (1 - (v - lo) / (hi - lo)) * ph;

    ctx.font = '400 10px Roboto, sans-serif';
    ctx.lineWidth = 1;
    ctx.strokeStyle = theme['--border']; ctx.fillStyle = theme['--muted'];
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (let i = 0; i <= 4; i++) {
        const v = lo + (hi - lo) * (i / 4), y = Math.round(Y(v)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
        ctx.fillText(v.toFixed(0), L - 6, y);
    }
    ctx.save();
    ctx.translate(11, T + ph / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('RPM', 0, 0);
    ctx.restore();
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let s = 0; s <= win; s += 10) {
        const x = Math.round(X(t0 + s)) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
        ctx.fillText(s === win ? 'now' : '-' + (win - s) + 's', x, T + ph + 4);
    }
    if (lo < 0 && hi > 0) {
        ctx.strokeStyle = theme['--muted']; ctx.globalAlpha = 0.6;
        const y0 = Math.round(Y(0)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y0); ctx.lineTo(w - R, y0); ctx.stroke(); ctx.globalAlpha = 1;
    }

    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    if (C.length) {
        ctx.strokeStyle = theme['--series-b']; ctx.lineWidth = 1.6; ctx.setLineDash([5, 3]);
        ctx.beginPath();
        let started = false, prevY = 0;
        C.forEach(p => {
            const x = X(p.t), y = Y(p.v);
            if (!started) { ctx.moveTo(x, y); started = true; } else { ctx.lineTo(x, prevY); ctx.lineTo(x, y); }
            prevY = y;
        });
        ctx.lineTo(X(now), prevY); ctx.stroke(); ctx.setLineDash([]);
    }
    if (A.length) {
        ctx.strokeStyle = theme['--series-a']; ctx.lineWidth = 1.6;
        ctx.beginPath();
        A.forEach((p, i) => { const x = X(p.t), y = Y(p.v); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
        ctx.stroke();
    }
    ctx.restore();

    const la = A.length ? A[A.length - 1].v : null, lc = C.length ? C[C.length - 1].v : null;
    document.getElementById('lg-' + id + '-a').textContent = formatSigned(la);
    document.getElementById('lg-' + id + '-c').textContent = formatSigned(lc);
}
setInterval(() => wheelIds.forEach(drawChart), 100);

// =====================================================================
//  SLAM map canvas & Interaction
// =====================================================================
const mapWrap = document.getElementById('map-wrapper');
const mapCanvas = document.getElementById('map-canvas');
const view = { vx: 0, vy: 0, s: 30 };
let follow = false, viewFitted = false;
let clickMode = 'single';
let singleTarget = null;
let waypointsData = [];
let activeGoal = null, panState = null;

const mapSize = () => ({ W: mapCanvas.clientWidth, H: mapCanvas.clientHeight });
function w2s(x, y) { const { W, H } = mapSize(); return [W / 2 - (y - view.vy) * view.s, H / 2 - (x - view.vx) * view.s]; }
function s2w(sx, sy) { const { W, H } = mapSize(); return { x: view.vx - (sy - H / 2) / view.s, y: view.vy - (sx - W / 2) / view.s }; }

function fitMapView() {
    if (!latestMap) return;
    const m = latestMap, c = Math.cos(m.yaw), s = Math.sin(m.yaw);
    const cs = [[0, 0], [m.w, 0], [0, m.h], [m.w, m.h]].map(([i, j]) =>
        [m.ox + m.res * (i * c - j * s), m.oy + m.res * (i * s + j * c)]);
    const xs = cs.map(p => p[0]), ys = cs.map(p => p[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    const { W, H } = mapSize();
    view.vx = (minX + maxX) / 2; view.vy = (minY + maxY) / 2;
    view.s = Math.max(2, Math.min(W / (maxY - minY || 1), H / (maxX - minX || 1)) * 0.94);
    follow = false; document.getElementById('follow-btn').classList.remove('on');
    viewFitted = true; needsDraw = true;
}
function toggleFollow() {
    follow = !follow;
    document.getElementById('follow-btn').classList.toggle('on', follow);
    needsDraw = true;
}
function toggleLaserScan() { toggleLayer('scan'); return; }

function rebuildMapImage() {
    const m = latestMap; if (!m) return;
    mapCanvasOff.width = m.w; mapCanvasOff.height = m.h;
    const ctx = mapCanvasOff.getContext('2d');
    const img = ctx.createImageData(m.w, m.h), d = img.data;
    const parse = (hex) => {
        const c = document.createElement('canvas').getContext('2d'); c.fillStyle = hex; c.fillRect(0, 0, 1, 1);
        return c.getImageData(0, 0, 1, 1).data;
    };
    const free = parse(theme['--map-free']), occ = parse(theme['--map-occ']);
    for (let j = 0; j < m.h; j++) {
        for (let i = 0; i < m.w; i++) {
            const v = m.data[j * m.w + i], k = (j * m.w + i) * 4;
            if (v < 0) { d[k + 3] = 0; }
            else {
                const t = Math.min(v, 100) / 100;
                d[k] = free[0] + (occ[0] - free[0]) * t;
                d[k + 1] = free[1] + (occ[1] - free[1]) * t;
                d[k + 2] = free[2] + (occ[2] - free[2]) * t;
                d[k + 3] = 255;
            }
        }
    }
    ctx.putImageData(img, 0, 0);
    document.getElementById('map-empty').style.display = 'none';
    document.getElementById('map-meta').textContent =
        '· ' + m.w + '×' + m.h + ' · ' + m.res.toFixed(3) + ' m/cell';
}

function drawArrow(ctx, x, y, yawRad, color, label, size) {
    const p = w2s(x, y), q = w2s(x + Math.cos(yawRad), y + Math.sin(yawRad));
    const ang = Math.atan2(q[1] - p[1], q[0] - p[0]);
    ctx.save();
    ctx.translate(p[0], p[1]); ctx.rotate(ang);
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(size, 0); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(size + 9, 0); ctx.lineTo(size - 2, -6); ctx.lineTo(size - 2, 6); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(p[0], p[1], 5, 0, 7); ctx.fill();
    ctx.strokeStyle = theme['--panel']; ctx.lineWidth = 1.5; ctx.stroke();
    if (label) {
        ctx.font = '700 11px Roboto, sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
        ctx.fillStyle = color; ctx.fillText(label, p[0], p[1] - 9);
    }
}

function drawPathWithArrows(ctx, points, color) {
    if (!points || points.length < 2) return;
    
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    points.forEach((pt, idx) => {
        const sPt = w2s(pt.x, pt.y);
        if (idx === 0) ctx.moveTo(sPt[0], sPt[1]);
        else ctx.lineTo(sPt[0], sPt[1]);
    });
    ctx.stroke();
    ctx.setLineDash([]);

    const spacingPx = 45;
    let accumulatedDist = 0;
    
    for (let i = 0; i < points.length - 1; i++) {
        const p1 = w2s(points[i].x, points[i].y);
        const p2 = w2s(points[i+1].x, points[i+1].y);
        const segLen = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
        if (segLen === 0) continue;

        const angle = Math.atan2(p2[1] - p1[1], p2[0] - p1[0]);
        let currentDist = spacingPx - (accumulatedDist % spacingPx);

        while (currentDist < segLen) {
            const t = currentDist / segLen;
            const cx = p1[0] + (p2[0] - p1[0]) * t;
            const cy = p1[1] + (p2[1] - p1[1]) * t;

            ctx.save();
            ctx.translate(cx, cy);
            ctx.rotate(angle);
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(-5, -4);
            ctx.lineTo(4, 0);
            ctx.lineTo(-5, 4);
            ctx.closePath();
            ctx.fill();
            ctx.restore();

            currentDist += spacingPx;
        }
        accumulatedDist += segLen;
    }
}

function drawPlanAnim(ctx, now) {
    if (!planAnim) return;
    const nodesW = robot ? [{ x: robot.x, y: robot.y }].concat(planAnim.targets || []) : (planAnim.targets || []);
    if (nodesW.length < 2) return;

    const S = nodesW.map(n => w2s(n.x, n.y));
    const segments = [];
    let total = 0;
    for (let i = 0; i < S.length - 1; i++) {
        const a = S[i], b = S[i + 1];
        const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
        if (len < 0.5) continue;
        segments.push({ a, b, len, start: total });
        total += len;
    }
    if (total < 1) return;

    const el = now - planAnim.t0;
    const cycle = PLAN_DRAW_MS + PLAN_HOLD_MS + PLAN_FADE_MS;
    const p = el % cycle;
    let progress = 1;
    let alpha = 1;

    if (p < PLAN_DRAW_MS) {
        const u = p / PLAN_DRAW_MS;
        progress = u * u * (3 - 2 * u);
    } else if (p > PLAN_DRAW_MS + PLAN_HOLD_MS) {
        alpha = Math.max(0, 1 - (p - PLAN_DRAW_MS - PLAN_HOLD_MS) / PLAN_FADE_MS);
    }

    const col = theme['--primary'];
    const headDist = progress * total;

    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    ctx.globalAlpha = 0.16 * alpha;
    ctx.strokeStyle = col;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 7]);
    ctx.beginPath();
    segments.forEach((seg, i) => {
        if (i === 0) ctx.moveTo(seg.a[0], seg.a[1]);
        ctx.lineTo(seg.b[0], seg.b[1]);
    });
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.globalAlpha = 0.95 * alpha;
    ctx.strokeStyle = col;
    ctx.lineWidth = 3;
    ctx.beginPath();
    let started = false;
    for (const seg of segments) {
        if (headDist <= seg.start) break;
        const travelled = Math.min(headDist - seg.start, seg.len);
        const t = travelled / seg.len;
        const x = seg.a[0] + (seg.b[0] - seg.a[0]) * t;
        const y = seg.a[1] + (seg.b[1] - seg.a[1]) * t;
        if (!started) {
            ctx.moveTo(seg.a[0], seg.a[1]);
            started = true;
        }
        ctx.lineTo(x, y);
        if (travelled < seg.len) break;
    }
    if (started) ctx.stroke();

    let head = S[0];
    for (const seg of segments) {
        if (headDist <= seg.start + seg.len) {
            const t = Math.max(0, Math.min(1, (headDist - seg.start) / seg.len));
            head = [
                seg.a[0] + (seg.b[0] - seg.a[0]) * t,
                seg.a[1] + (seg.b[1] - seg.a[1]) * t
            ];
            break;
        }
    }

    ctx.globalAlpha = alpha;
    ctx.fillStyle = col;
    ctx.beginPath();
    ctx.arc(head[0], head[1], 4, 0, Math.PI * 2);
    ctx.fill();

    for (let i = 1; i < S.length; i++) {
        let distanceToNode = 0;
        for (const seg of segments) {
            if (Math.abs(seg.b[0] - S[i][0]) < 0.5 && Math.abs(seg.b[1] - S[i][1]) < 0.5) {
                distanceToNode = seg.start + seg.len;
                break;
            }
        }
        if (distanceToNode <= headDist + 0.5) {
            ctx.globalAlpha = 0.8 * alpha;
            ctx.beginPath();
            ctx.arc(S[i][0], S[i][1], 3, 0, Math.PI * 2);
            ctx.fill();
        }
    }
    ctx.restore();
}

function drawMap() {
    const dpr = window.devicePixelRatio || 1;
    const W = mapCanvas.clientWidth, H = mapCanvas.clientHeight;
    if (!W || !H) return;
    if (mapCanvas.width !== Math.round(W * dpr) || mapCanvas.height !== Math.round(H * dpr)) {
        mapCanvas.width = Math.round(W * dpr); mapCanvas.height = Math.round(H * dpr);
    }
    const ctx = mapCanvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = theme['--map-bg']; ctx.fillRect(0, 0, W, H);

    if (follow && robot) { view.vx = robot.x; view.vy = robot.y; }

    if (latestMap && layerOn('map')) {
        const m = latestMap, c = Math.cos(m.yaw), s = Math.sin(m.yaw), k = m.res * view.s;
        ctx.save();
        ctx.imageSmoothingEnabled = false;
        ctx.transform(-k * s, -k * c, -k * c, k * s,
            W / 2 - (m.oy - view.vy) * view.s, H / 2 - (m.ox - view.vx) * view.s);
        ctx.drawImage(mapCanvasOff, 0, 0);
        ctx.restore();
    }

    const step = view.s >= 6 ? 0.6 : (view.s >= 1.5 ? 3.0 : 6.0);
    document.getElementById('map-grid-label').textContent = 'Grid ' + Math.round(step * 100) + ' cm';
    const a = s2w(0, 0), b = s2w(W, H);
    const xLo = Math.min(a.x, b.x), xHi = Math.max(a.x, b.x), yLo = Math.min(a.y, b.y), yHi = Math.max(a.y, b.y);
    ctx.strokeStyle = theme['--map-grid']; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(xLo / step) * step; x <= xHi; x += step) {
        const sy = Math.round(w2s(x, 0)[1]) + 0.5; ctx.moveTo(0, sy); ctx.lineTo(W, sy);
    }
    for (let y = Math.ceil(yLo / step) * step; y <= yHi; y += step) {
        const sx = Math.round(w2s(0, y)[0]) + 0.5; ctx.moveTo(sx, 0); ctx.lineTo(sx, H);
    }
    ctx.stroke();

    if (layerOn('gcost')) drawCostmap(ctx, 'gcost');
    if (layerOn('lcost')) drawCostmap(ctx, 'lcost');
    drawLaserScan(ctx);

    const o = w2s(0, 0), ax = w2s(1, 0), ay = w2s(0, 1);
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#e5484d'; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(ax[0], ax[1]); ctx.stroke();
    ctx.strokeStyle = '#30a46c'; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(ay[0], ay[1]); ctx.stroke();

    if (layerOn('plan') && latestPlan && latestPlan.length > 0) {
        drawPathWithArrows(ctx, latestPlan, theme['--primary']);
        const finalPt = latestPlan[latestPlan.length - 1];
        drawArrow(ctx, finalPt.x, finalPt.y, finalPt.yaw, theme['--danger'], 'GOAL', 28);
    }

    drawPlanAnim(ctx, performance.now());

    const WPCOL = '#E8A317', TCOL = '#0077ff';
    if (waypointsData.length > 1) {
        ctx.strokeStyle = WPCOL; ctx.lineWidth = 1.2; ctx.setLineDash([4, 4]); ctx.beginPath();
        waypointsData.forEach((wp, i) => { const p = w2s(wp.x, wp.y); i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
        ctx.stroke(); ctx.setLineDash([]);
    }
    waypointsData.forEach((wp, i) => drawArrow(ctx, wp.x, wp.y, wp.yaw * Math.PI / 180, WPCOL, String(i + 1), 26));
    if (singleTarget) drawArrow(ctx, singleTarget.x, singleTarget.y, singleTarget.yaw * Math.PI / 180, TCOL, 'T', 26);
    if (activeGoal) drawArrow(ctx, activeGoal.x, activeGoal.y, activeGoal.yaw * Math.PI / 180,
        clickMode === 'single' ? TCOL : WPCOL, clickMode === 'single' ? 'T' : String(waypointsData.length + 1), 26);

    if (robot) {
        const p = w2s(robot.x, robot.y), q = w2s(robot.x + Math.cos(robot.yaw), robot.y + Math.sin(robot.yaw));
        const ang = Math.atan2(q[1] - p[1], q[0] - p[0]);
        const r = Math.max(8, Math.min(16, 0.3 * view.s));
        const col = theme['--success'];
        
        ctx.fillStyle = col; ctx.globalAlpha = 0.18;
        ctx.beginPath(); ctx.arc(p[0], p[1], r * 2, 0, 7); ctx.fill(); ctx.globalAlpha = 1;
        
        ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(ang);
        ctx.fillStyle = col; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(r * 1.6, 0); ctx.lineTo(-r * 0.9, -r * 0.95); ctx.lineTo(-r * 0.4, 0); ctx.lineTo(-r * 0.9, r * 0.95); ctx.closePath();
        ctx.fill(); ctx.stroke();
        
        if (Math.abs(currentCmdVel.linear) > 0.02) {
            const linLen = currentCmdVel.linear * 35;
            const dir = Math.sign(currentCmdVel.linear);
            ctx.strokeStyle = '#38bdf8';
            ctx.fillStyle = '#38bdf8';
            ctx.lineWidth = 2.5;
            ctx.beginPath();
            ctx.moveTo(r * 1.8, 0);
            ctx.lineTo(r * 1.8 + linLen, 0);
            ctx.stroke();
            
            const headX = r * 1.8 + linLen;
            ctx.beginPath();
            ctx.moveTo(headX + dir * 6, 0);
            ctx.lineTo(headX, -5);
            ctx.lineTo(headX, 5);
            ctx.fill();
        }

        if (Math.abs(currentCmdVel.angular) > 0.05) {
            const arcR = r * 2.5;
            ctx.strokeStyle = '#facc15';
            ctx.fillStyle = '#facc15';
            ctx.lineWidth = 2.5;
            
            const dir = Math.sign(currentCmdVel.angular);
            const sweepAng = Math.min(Math.max(Math.abs(currentCmdVel.angular) * 0.5, 0.3), Math.PI / 1.2); 
            
            ctx.beginPath();
            ctx.arc(0, 0, arcR, 0, -dir * sweepAng, dir > 0);
            ctx.stroke();
            
            const endAngle = -dir * sweepAng;
            const ax = arcR * Math.cos(endAngle);
            const ay = arcR * Math.sin(endAngle);
            
            ctx.save();
            ctx.translate(ax, ay);
            ctx.rotate(endAngle - (dir * Math.PI / 2));
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(-5, -3);
            ctx.lineTo(-5, 3);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }

        ctx.restore();
        drawBubbles(ctx, p, r);
    }
}

function evtPos(evt) { const r = mapCanvas.getBoundingClientRect(); return [evt.clientX - r.left, evt.clientY - r.top]; }

let poseActionPopup = null;

function openPoseActionPopup(pose, evt) {
    const popup = document.getElementById('map-pose-popup');
    const poseText = document.getElementById('map-pose-popup-pose');
    if (!popup || !pose) return;

    poseActionPopup = { x: pose.x, y: pose.y, yaw: pose.yaw || 0 };

    poseText.textContent =
        'X ' + pose.x.toFixed(2) +
        ' | Y ' + pose.y.toFixed(2) +
        ' | Yaw ' + (pose.yaw || 0).toFixed(1) + '°';

    const margin = 10;
    const pw = 230;
    const ph = 175;
    let left = evt.clientX + margin;
    let top = evt.clientY + margin;

    if (left + pw > window.innerWidth - margin) left = evt.clientX - pw - margin;
    if (top + ph > window.innerHeight - margin) top = evt.clientY - ph - margin;

    popup.style.left = Math.max(margin, left) + 'px';
    popup.style.top = Math.max(margin, top) + 'px';
    popup.classList.add('show');
}

function closePoseActionPopup() {
    const popup = document.getElementById('map-pose-popup');
    if (popup) popup.classList.remove('show');
    poseActionPopup = null;
}

async function popupNavigateToPoint() {
    if (!poseActionPopup) return;
    const p = { ...poseActionPopup };
    closePoseActionPopup();

    singleTarget = p;
    updateUIInputs(p.x, p.y, p.yaw);
    needsDraw = true;

    await sendGoalPose();
}

async function popupSetInitialCheckpoint() {
    if (!poseActionPopup) return;
    const p = { ...poseActionPopup };
    closePoseActionPopup();

    singleTarget = p;
    updateUIInputs(p.x, p.y, p.yaw);
    needsDraw = true;

    await sendInitialPose();
}

function popupAddWaypoint() {
    if (!poseActionPopup) return;
    const p = { ...poseActionPopup };
    closePoseActionPopup();

    waypointsData.push(p);
    updateWaypointsUI();
    setClickMode('waypoint');
    document.querySelectorAll('input[name="click_mode"]').forEach(i => i.checked = (i.value === 'waypoint'));
    needsDraw = true;
    notify('INFO', 'Waypoint ' + waypointsData.length + ' added at (' +
        p.x.toFixed(2) + ', ' + p.y.toFixed(2) + ').');
}

document.addEventListener('pointerdown', (evt) => {
    const popup = document.getElementById('map-pose-popup');
    if (popup && popup.classList.contains('show') && !popup.contains(evt.target) && !mapWrap.contains(evt.target)) {
        closePoseActionPopup();
    }
});

mapWrap.addEventListener('contextmenu', e => e.preventDefault());
mapWrap.addEventListener('pointerdown', (evt) => {
    if (evt.target.closest('#map-pose-popup, #layer-legend')) return;
    mapWrap.setPointerCapture(evt.pointerId);
    const [sx, sy] = evtPos(evt);
    if (evt.button === 1 || evt.button === 2 || evt.shiftKey) {
        panState = { sx, sy, vx: view.vx, vy: view.vy };
        follow = false; document.getElementById('follow-btn').classList.remove('on');
        return;
    }
    const w = s2w(sx, sy);
    activeGoal = { x: w.x, y: w.y, yaw: 0 };
    needsDraw = true;
});
mapWrap.addEventListener('pointermove', (evt) => {
    if (evt.target.closest('#map-pose-popup')) return;
    const [sx, sy] = evtPos(evt);
    const w = s2w(sx, sy);
    document.getElementById('map-cursor').textContent = 'x ' + w.x.toFixed(2) + '  y ' + w.y.toFixed(2) + ' m';
    if (panState) {
        view.vy = panState.vy + (sx - panState.sx) / view.s;
        view.vx = panState.vx + (sy - panState.sy) / view.s;
        needsDraw = true;
    } else if (activeGoal) {
        const dx = w.x - activeGoal.x, dy = w.y - activeGoal.y;
        if (Math.abs(dx) > 0.05 || Math.abs(dy) > 0.05) activeGoal.yaw = Math.atan2(dy, dx) * 180 / Math.PI;
        needsDraw = true;
    }
});
const endPointer = (evt) => {
    if (evt.target.closest('#map-pose-popup')) return;
    if (panState) { panState = null; return; }
    if (!activeGoal) return;
    const g = activeGoal; activeGoal = null;

    if (clickMode === 'single') {
        singleTarget = g;
        updateUIInputs(g.x, g.y, g.yaw);
    } else {
        waypointsData.push(g);
        updateWaypointsUI();
    }

    needsDraw = true;
    openPoseActionPopup(g, evt);
};
mapWrap.addEventListener('pointerup', endPointer);

document.addEventListener('keydown', (evt) => {
    if (evt.key === 'Escape') closePoseActionPopup();
});
mapWrap.addEventListener('pointercancel', () => { panState = null; activeGoal = null; needsDraw = true; });
mapWrap.addEventListener('wheel', (evt) => {
    if (evt.target.closest('#layer-legend')) return;
    evt.preventDefault();
    const [sx, sy] = evtPos(evt), { W, H } = mapSize();
    const w = s2w(sx, sy);
    view.s = Math.max(1, Math.min(500, view.s * (evt.deltaY < 0 ? 1.12 : 1 / 1.12)));
    view.vy = w.y + (sx - W / 2) / view.s;
    view.vx = w.x + (sy - H / 2) / view.s;
    needsDraw = true;
}, { passive: false });
new ResizeObserver(() => { needsDraw = true; }).observe(mapWrap);

let lastStatus = 0;
function frame(ts) {
    robot = resolveRobotPose();
    if (mapImageDirty && latestMap) {
        rebuildMapImage(); mapImageDirty = false;
        if (!viewFitted) fitMapView();
        needsDraw = true;
    }
    if (planAnim || bubbles.length) needsDraw = true;
    if (follow || needsDraw) { drawMap(); needsDraw = false; }
    if (ts - lastStatus > 200) { lastStatus = ts; updateStatus(); }
    requestAnimationFrame(frame);
}

function calculateDistanceRemaining() {
    if (!latestPlan || latestPlan.length < 2) return 0;
    let dist = 0;
    for (let i = 0; i < latestPlan.length - 1; i++) {
        dist += Math.hypot(latestPlan[i+1].x - latestPlan[i].x, latestPlan[i+1].y - latestPlan[i].y);
    }
    return dist;
}

function updateStatus() {
    if (lastCamFrame && performance.now() - lastCamFrame > 3000) {
        const e = document.getElementById('cam-empty');
        e.textContent = 'No signal from /camera/image_raw'; e.style.display = 'flex';
    }
    const spd = odom ? odom.speed : 0;
    document.getElementById('stat-speed').textContent = spd.toFixed(2);

    distanceRemaining = calculateDistanceRemaining();
    document.getElementById('stat-distance').textContent = latestPlan.length > 0 ? distanceRemaining.toFixed(2) : '—';

    if (navStatus === 'Navigating' && latestPlan.length > 0 && distanceRemaining < 0.25) {
        navStatus = 'Reached';
        notify('INFO', 'Navigation destination reached successfully.');
        latestPlan = [];
    }

    const statusEl = document.getElementById('stat-nav-status');
    statusEl.textContent = navStatus;
    if (navStatus === 'Reached') statusEl.style.color = 'var(--success)';
    else if (navStatus === 'Aborted') statusEl.style.color = 'var(--danger)';
    else if (navStatus === 'Navigating') statusEl.style.color = 'var(--primary)';
    else statusEl.style.color = 'var(--muted)';

    if (robot) {
        let deg = robot.yaw * 180 / Math.PI;
        deg = ((deg + 180) % 360 + 360) % 360 - 180;
        document.getElementById('stat-heading').textContent = deg.toFixed(1);
        document.getElementById('hdg-arrow').style.transform = 'rotate(' + (-deg) + 'deg)';
        document.getElementById('stat-location').innerHTML =
            'X ' + robot.x.toFixed(2) + ' &nbsp; Y ' + robot.y.toFixed(2);
    }
}

function setClickMode(mode) {
    clickMode = mode;
    document.getElementById('single-actions').style.display = mode === 'single' ? 'flex' : 'none';
    document.getElementById('waypoint-actions').style.display = mode === 'waypoint' ? 'flex' : 'none';
    document.getElementById('waypoints-container').style.display = mode === 'waypoint' ? 'block' : 'none';
    document.querySelectorAll('input[name="click_mode"]').forEach(i => i.checked = (i.value === mode));
}
function updateUIInputs(x, y, yaw) {
    document.getElementById('target-x').value = x.toFixed(2);
    document.getElementById('target-y').value = y.toFixed(2);
    document.getElementById('target-yaw').value = yaw.toFixed(2);
    document.getElementById('lbl-x').textContent = x.toFixed(2);
    document.getElementById('lbl-y').textContent = y.toFixed(2);
    document.getElementById('lbl-yaw').textContent = yaw.toFixed(2);
}
function updateSingleFromInputs() {
    const x = parseFloat(document.getElementById('target-x').value) || 0;
    const y = parseFloat(document.getElementById('target-y').value) || 0;
    const yaw = parseFloat(document.getElementById('target-yaw').value) || 0;
    document.getElementById('lbl-x').textContent = x.toFixed(2);
    document.getElementById('lbl-y').textContent = y.toFixed(2);
    document.getElementById('lbl-yaw').textContent = yaw.toFixed(2);
    singleTarget = { x, y, yaw }; needsDraw = true;
}
function updateWaypointsUI() {
    const container = document.getElementById('waypoints-container');
    container.innerHTML = waypointsData.length === 0 ? '<em>No waypoints added.</em>' : '';
    waypointsData.forEach((wp, i) => {
        const div = document.createElement('div');
        div.innerHTML = '<strong>WP ' + (i + 1) + ':</strong> X:' + wp.x.toFixed(2) + ' Y:' + wp.y.toFixed(2) + ' Yaw:' + wp.yaw.toFixed(0) + '&deg;';
        container.appendChild(div);
    });
}
function clearMarkers() {
    closePoseActionPopup();
    singleTarget = null; waypointsData = []; activeGoal = null; latestPlan = [];
    navStatus = 'Idle';
    stopPlanAnim();
    updateWaypointsUI(); needsDraw = true;
}

async function postJSON(url, body) {
    const r = await fetch("http://"+ROVER_HOST+":8000"+url, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: body === undefined ? undefined : JSON.stringify(body)
    });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return r;
}
async function withBusy(btn, fn) {
    if (btn) { if (btn.classList.contains('loading')) return; btn.classList.add('loading'); btn.disabled = true; }
    try { return await fn(); }
    finally { if (btn) { btn.classList.remove('loading'); btn.disabled = false; } }
}

function run(btn, fn) { return withBusy(btn, fn); }
function runNavButton(btn, fn) { return withBusy(btn, fn); }

window.run = run;
window.runNavButton = runNavButton;

function readTarget() {
    return {
        x: parseFloat(document.getElementById('target-x').value),
        y: parseFloat(document.getElementById('target-y').value),
        yawDeg: parseFloat(document.getElementById('target-yaw').value) || 0
    };
}
async function sendGoalPose() {
    const btn = document.getElementById('nav-pose-btn');
    setNavLoading(btn);
    startNavWatchdog();
    const t = readTarget();
    latestPlan = [];
    startPlanAnim([t]);
    try {
        await postJSON('/navigate_to_pose', { x: t.x, y: t.y, yaw_deg: t.yawDeg });
        navStatus = 'Navigating';
        notify('INFO', 'Navigation goal sent to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) {
        clearNavLoading();
        notify('ERROR', 'Failed to send navigation goal: ' + e.message);
    }
}
async function sendInitialPose() {
    const t = readTarget();
    try {
        await postJSON('/set_initial_pose', { x: t.x, y: t.y, yaw_deg: t.yawDeg });
        notify('INFO', 'Initial pose set to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) { notify('ERROR', 'Failed to set initial pose: ' + e.message); }
}
async function sendWaypoints() {
    if (waypointsData.length === 0) return notify('WARNING', 'No waypoints added. Click the map to add some.');
    const btn = document.getElementById('follow-wp-btn');
    setNavLoading(btn);
    startNavWatchdog();
    latestPlan = [];
    startPlanAnim(waypointsData);
    const waypoints = waypointsData.map(wp => ({ x: wp.x, y: wp.y, yaw_deg: wp.yaw }));
    try {
        await postJSON('/follow_waypoints', { waypoints });
        navStatus = 'Navigating';
        notify('INFO', 'Following ' + waypoints.length + ' waypoint(s).');
    } catch (e) {
        clearNavLoading();
        notify('ERROR', 'Failed to send waypoints: ' + e.message);
    }
}
async function sendAbort() {
    try {
        await postJSON('/abort');
        navStatus = 'Aborted';
        latestPlan = [];
        clearNavLoading();
        notify('WARNING', 'Mission abort requested.');
    }
    catch (e) { notify('ERROR', 'Failed to abort mission: ' + e.message); }
}
let availableMaps = [];
let mapsLoading = false;

async function fetchMaps(options = {}) {
    if (mapsLoading) return;
    const select = document.getElementById('map-name-select');
    if (!select) return;

    if (!ROVER_HOST) {
        select.disabled = true;
        select.innerHTML = '<option value="">Connect to rover to load maps</option>';
        return;
    }

    mapsLoading = true;
    try {
        const ctl = new AbortController();
        const to = setTimeout(() => ctl.abort(), 5000);
        const r = await fetch('http://' + ROVER_HOST + ':8000/maps?t=' + Date.now(), {
            cache: 'no-store',
            signal: ctl.signal,
            headers: { 'Accept': 'application/json' }
        });
        clearTimeout(to);
        if (!r.ok) throw new Error('HTTP ' + r.status);

        const data = await r.json();
        if (!Array.isArray(data.maps)) throw new Error('Invalid /maps response');
        availableMaps = data.maps;

        const previous = select.value;
        select.innerHTML = '';

        if (!availableMaps.length) {
            const opt = document.createElement('option');
            opt.value = '';
            opt.textContent = 'No saved maps';
            select.appendChild(opt);
        } else {
            availableMaps.forEach(name => {
                const opt = document.createElement('option');
                opt.value = name;
                opt.textContent = name;
                select.appendChild(opt);
            });
            if (availableMaps.includes(previous)) {
                select.value = previous;
            } else {
                select.selectedIndex = 0;
            }
        }

        select.disabled = false;
        select.dataset.loaded = 'true';
        updateMapButtons();
        if (options.notifySuccess) notify('INFO', 'Loaded ' + availableMaps.length + ' saved map(s).');
    } catch (e) {
        select.innerHTML = '<option value="">Unable to load maps</option>';
        select.disabled = false;
        if (options.notifyError !== false) {
            notify('WARNING', 'Could not fetch saved maps: ' + (e.name === 'AbortError' ? 'request timed out' : e.message));
        }
    } finally {
        mapsLoading = false;
    }
}

function openMapSaveModal() {
    if (!ROVER_HOST) return notify('WARNING', 'Connect to the rover before saving a map.');
    const modal = document.getElementById('map-save-modal');
    const input = document.getElementById('save-map-name');
    const error = document.getElementById('map-save-error');
    const select = document.getElementById('map-name-select');
    if (error) error.textContent = '';
    if (input) input.value = select && select.value && availableMaps.includes(select.value) ? select.value : '';
    if (modal) modal.hidden = false;
    setTimeout(() => input && input.focus(), 30);
}

function closeMapSaveModal() {
    const modal = document.getElementById('map-save-modal');
    if (modal) modal.hidden = true;
}

function setMapSaveError(message) {
    const el = document.getElementById('map-save-error');
    if (el) el.textContent = message;
}

async function saveMapFromModal() {
    const input = document.getElementById('save-map-name');
    const submit = document.getElementById('save-map-submit');
    const mapName = (input ? input.value : '').trim();
    setMapSaveError('');

    if (!mapName) return setMapSaveError('Enter a map name.');
    if (!/^[A-Za-z0-9 _.-]+$/.test(mapName) || mapName === '.' || mapName === '..') {
        return setMapSaveError('Use only letters, numbers, spaces, _, -, and . in the map name.');
    }

    if (submit) { submit.disabled = true; submit.classList.add('loading'); }
    try {
        await postJSON('/system/save_map', { map_name: mapName });
        closeMapSaveModal();
        notify('INFO', 'Saving "' + mapName + '" map + SLAM continuation data…');
        setTimeout(async () => {
            await fetchMaps({ notifyError: false });
            const select = document.getElementById('map-name-select');
            if (select && availableMaps.includes(mapName)) select.value = mapName;
            updateMapButtons();
        }, 1500);
    } catch (e) {
        setMapSaveError('Failed to save map: ' + e.message);
    } finally {
        if (submit) { submit.disabled = false; submit.classList.remove('loading'); }
    }
}

async function saveCurrentMap() {
    openMapSaveModal();
}

let currentMode = null, pending = null, modeSelectTouched = false, modeTimer = null, polling = false;

function normalizeMode(raw) {
    if (raw === undefined || raw === null) return null;
    const s = String(raw).toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_|_$/g, '');
    if (['slam_update', 'slam_update_nav', 'autonomous_driving_map_update', 'autonomous_map_update',
         'mapping_navigation_update'].includes(s)) return 'slam_update';
    if (['nav', 'nav_only', 'navigation', 'navigation_only', 'amcl', 'localization',
         'autonomous_driving_fixed_map', 'fixed_map', 'autonomous_fixed_map'].includes(s)) return 'nav';
    if (['slam', 'slam_only', 'mapping', 'mapping_only', 'manual_mapping',
         'manual_driving_new_mapping', 'new_mapping'].includes(s)) return 'slam';
    if (['manual', 'teleop', 'joystick', 'manual_driving_mapping_off', 'manual_mapping_off'].includes(s)) return 'manual';
    return null;
}

async function fetchMode() {
    const ctl = new AbortController();
    const to = setTimeout(() => ctl.abort(), 4000);
    try {
        const r = await fetch("http://"+ROVER_HOST+":8000"+CFG.modeUrl, { cache: 'no-store', signal: ctl.signal });
        if (!r.ok) return undefined;
        const text = await r.text();
        let raw = text;
        try {
            const j = JSON.parse(text);
            raw = (j && typeof j === 'object') ? (j.mode ?? j.current_mode ?? j.system_mode) : j;
        } catch (e) {}
        return normalizeMode(raw);
    } catch (e) { return undefined; }
    finally { clearTimeout(to); }
}

function renderNavCardVisibility() {
    const card = document.getElementById('map-nav-card');
    if (!card) return;
    const autonomous = currentMode === 'nav' || currentMode === 'slam_update';
    card.classList.toggle('mode-hidden', !autonomous);
}

function renderMode() {
    const chip = document.getElementById('mode-chip'), txt = document.getElementById('stat-mode');
    const btn = document.getElementById('deploy-btn');
    chip.classList.remove('known', 'pending');
    if (pending) {
        chip.classList.add('pending');
        txt.textContent = 'Switching to ' + MODE_LABELS[pending.target] + '…';
    } else if (currentMode) {
        chip.classList.add('known');
        txt.textContent = MODE_LABELS[currentMode];
    } else {
        txt.textContent = 'Unknown';
    }
    btn.disabled = !!pending;
    btn.classList.toggle('loading', !!pending);

    renderNavCardVisibility();
    updateLegend();

    document.querySelectorAll('.mode-loader').forEach(el => {
        el.classList.toggle('show', !!pending);
        if (pending) el.querySelector('.ml-title').textContent = 'Switching to ' + MODE_LABELS[pending.target];
    });
}

function setCurrentMode(m) {
    currentMode = m;
    if (m && !modeSelectTouched && !pending) document.getElementById('sys-mode-select').value = m;
    renderMode();
}

function clearPending() {
    if (pending) clearTimeout(pending.timer);
    pending = null; renderMode();
    scheduleModePoll();
}

async function pollMode() {
    if (polling) return;
    polling = true;
    try {
        const m = await fetchMode();
        if (m !== undefined) {
            const prev = currentMode;
            setCurrentMode(m);
            if (pending && m === pending.target) {
                const label = MODE_LABELS[m]; clearPending();
                notify('INFO', 'Mode changed to ' + label + '.');
            } else if (!pending && prev && m && prev !== m) {
                notify('INFO', 'Mode is now ' + MODE_LABELS[m] + '.');
            }
        }
    } finally { polling = false; }
}

function scheduleModePoll() {
    clearTimeout(modeTimer);
    modeTimer = setTimeout(async () => { await pollMode(); scheduleModePoll(); },
        pending ? CFG.modePollMs : CFG.modeIdlePollMs);
}

async function applySystemMode() {
    if (pending) return;
    const mode = document.getElementById('sys-mode-select').value;
    const mapName = document.getElementById('map-name-select').value || '';
    if (mode === currentMode) return notify('INFO', 'Already in ' + MODE_LABELS[mode] + ' mode.');

    const p = { target: mode, timer: null };
    p.timer = setTimeout(async () => {
        if (pending !== p) return;
        const m = await fetchMode();
        if (pending !== p) return;
        if (m !== undefined) setCurrentMode(m);
        const seconds = CFG.modeTimeoutMs / 1000;
        if (m === p.target) { clearPending(); notify('INFO', 'Mode changed to ' + MODE_LABELS[mode] + '.'); }
        else {
            clearPending();
            notify('ERROR', 'Mode change to ' + MODE_LABELS[mode] + ' timed out after ' + seconds + ' s. Current mode: ' +
                (currentMode ? MODE_LABELS[currentMode] : 'unknown') + '.');
        }
    }, CFG.modeTimeoutMs);
    pending = p; renderMode();
    notify('INFO', 'Switching to ' + MODE_LABELS[mode] + '… waiting up to ' + (CFG.modeTimeoutMs / 1000) + ' s for confirmation.');
    scheduleModePoll();

    try { await postJSON(CFG.modeUrl, { mode, map_name: mapName }); }
    catch (e) {
        if (pending === p) { clearPending(); notify('ERROR', 'Mode request failed: ' + e.message); }
    }
}

const joyPad = document.getElementById('joy-pad'), joyKnob = document.getElementById('joy-knob');
const joyEnableBtn = document.getElementById('joy-enable-btn');
const joy = { x: 0, y: 0, active: false, enabled: false, timer: null };

function publishJoy() {
    if (!joyTopic || rosState !== 'up') return;
    const ms = Date.now();
    const msg = new ROSLIB.Message({
        header: { stamp: { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 }, frame_id: 'joy' },
        axes: [-joy.x, -joy.y, 0, 0, 0, 0, 0, 0],
        buttons: [0, 0, 0, 0, joy.enabled ? 1 : 0, 0, 0, 0, 0, 0, 0, 0]
    });
    joyTopic.publish(msg);
}

function startJoyPublishing() {
    clearInterval(joy.timer);
    if (!joy.enabled) return;
    joy.timer = setInterval(publishJoy, 1000 / CFG.joyRateHz);
    publishJoy();
}

function stopJoyPublishing() {
    clearInterval(joy.timer);
    joy.timer = null;
}

function updateJoyEnableButton() {
    if (!joyEnableBtn) return;
    joyEnableBtn.textContent = joy.enabled ? 'Disable' : 'Enable';
    joyEnableBtn.classList.toggle('enabled', joy.enabled);
    joyEnableBtn.classList.toggle('btn-success', !joy.enabled);
    joyEnableBtn.classList.toggle('btn-danger', joy.enabled);
    joyEnableBtn.setAttribute('aria-pressed', joy.enabled ? 'true' : 'false');
}

function toggleJoyEnabled() {
    joy.enabled = !joy.enabled;
    if (!joy.enabled) {
        joy.x = 0; joy.y = 0;
        joyShow();
    }
    updateJoyEnableButton();
    if (joy.enabled) {
        startJoyPublishing();
        notify('INFO', 'Joystick enabled. Publishing sensor_msgs/Joy on /joy with button[4] pressed.');
    } else {
        if (joyTopic && rosState === 'up') {
            publishJoy();
            setTimeout(publishJoy, 50);
            setTimeout(publishJoy, 100);
        }
        stopJoyPublishing();
        notify('INFO', 'Joystick disabled.');
    }
}

function joyShow() {
    joyKnob.style.transform = 'translate(' + (joy.x * joyR()) + 'px,' + (joy.y * joyR()) + 'px)';
    document.getElementById('joy-x').textContent = (-joy.x).toFixed(2);
    document.getElementById('joy-y').textContent = (-joy.y).toFixed(2);
}
function joyR() { return joyPad.clientWidth / 2 - 26; }
function joyMove(evt) {
    const r = joyPad.getBoundingClientRect(), R = joyR();
    let dx = evt.clientX - (r.left + r.width / 2), dy = evt.clientY - (r.top + r.height / 2);
    const mag = Math.hypot(dx, dy);
    
    if (mag > R) { dx *= R / mag; dy *= R / mag; }
    
    if (mag < R * 0.1) { dx = 0; dy = 0; }
    
    joy.x = dx / R; joy.y = dy / R; joyShow();
}
joyPad.addEventListener('pointerdown', (e) => {
    if (!joy.enabled) {
        notify('WARNING', 'Enable the joystick before driving.');
        return;
    }
    joyPad.setPointerCapture(e.pointerId); joyPad.classList.add('active');
    joy.active = true; joyMove(e);
});
joyPad.addEventListener('pointermove', (e) => { if (joy.active && joy.enabled) joyMove(e); });
const joyEnd = () => {
    if (!joy.active) return;
    joy.active = false; joyPad.classList.remove('active');
    joy.x = 0; joy.y = 0; joyShow();
    if (joy.enabled) {
        publishJoy(); setTimeout(publishJoy, 50); setTimeout(publishJoy, 100);
    }
};
joyPad.addEventListener('pointerup', joyEnd);
joyPad.addEventListener('pointercancel', joyEnd);
updateJoyEnableButton();

(function init() {
    updateLegend();
    let saved = null;
    try { saved = localStorage.getItem('milusions-theme'); } catch (e) {}
    applyTheme(saved !== 'dark');
    renderMode();
    if (ROVER_HOST) fetchMaps({ notifyError: false });
    pollMode().then(scheduleModePoll);
    requestAnimationFrame(frame);
})();

let audioCtx = null;

function initAudio() {
    if (!audioCtx) {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    }
    if (audioCtx.state === 'suspended') {
        audioCtx.resume();
    }
}

function playStartupSound() {
    try {
        initAudio();
        const now = audioCtx.currentTime;
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(440, now);
        osc.frequency.exponentialRampToValueAtTime(880, now + 0.15);
        gain.gain.setValueAtTime(0.45, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.35);
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.start(now);
        osc.stop(now + 0.35);
    } catch (e) { console.log(e); }
}

function playListenSound() {
    try {
        initAudio();
        const now = audioCtx.currentTime;
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(587.33, now);
        osc.frequency.setValueAtTime(880, now + 0.08);
        gain.gain.setValueAtTime(0.12, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.2);
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.start(now);
        osc.stop(now + 0.2);
    } catch (e) { console.log(e); }
}

function playProcessingSound() {
    try {
        initAudio();
        const now = audioCtx.currentTime;
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(320, now);
        osc.frequency.linearRampToValueAtTime(160, now + 0.1);
        gain.gain.setValueAtTime(0.1, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.12);
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.start(now);
        osc.stop(now + 0.12);
    } catch (e) { console.log(e); }
}

const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
let recognition = null;
let inactivityTimer = null;
const LISTEN_TIMEOUT_MS = 0;
let listenDeadline = 0, heardSpeech = false;
let helioSocket = null;
let helioSocketPromise = null;
let helioConversationId = null;
let helioSocketClosing = false; 

const modal = document.getElementById('voice-modal');
const robotFace = document.getElementById('robot-face');
const captionStatus = document.getElementById('caption-status');
const captionText = document.getElementById('caption-text');
const historyListEl = document.getElementById('history-list');

let agentState = 'idle';
let thinkingTimer = null;
let thinkingStageIndex = 0;
window.currentUtterance = null;

const thinkingPanel = document.getElementById('thinking-panel');
const thinkingStageText = document.getElementById('thinking-stage-text');

const THINKING_STAGES = ['Analyzing prompt', 'Thinking...', 'Accessing rover', 'Finalizing'];

function startThinkingIndicator() {
    stopThinkingIndicator();
    if (!thinkingPanel) return;
    thinkingPanel.classList.add('active');
    thinkingStageIndex = 0;
    const update = () => {
        if (thinkingStageText) thinkingStageText.textContent = THINKING_STAGES[Math.min(thinkingStageIndex, THINKING_STAGES.length - 1)];
        thinkingStageIndex++;
        if (thinkingStageIndex >= THINKING_STAGES.length && thinkingTimer) { clearInterval(thinkingTimer); thinkingTimer = null; }
    };
    update();
    thinkingTimer = setInterval(update, 1600);
}

function stopThinkingIndicator() {
    if (thinkingTimer) { clearInterval(thinkingTimer); thinkingTimer = null; }
    if (thinkingPanel) thinkingPanel.classList.remove('active');
}

function appendHistory(sender, text) {
    const item = document.createElement('div');
    item.className = `history-item ${sender}`;
    
    const label = document.createElement('div');
    label.className = 'history-label';
    label.textContent = sender === 'you' ? 'you:' : 'agent:';
    
    const content = document.createElement('div');
    if (typeof marked !== 'undefined') {
        content.innerHTML = marked.parse(text);
    } else {
        content.textContent = text;
    }
    
    item.appendChild(label);
    item.appendChild(content);
    historyListEl.appendChild(item);
    historyListEl.scrollTop = historyListEl.scrollHeight;
}

function clearAgentLiveStatus() {
    const list = document.getElementById('agent-event-list');
    if (list) list.innerHTML = '';
}
function appendAgentEvent(kind, message, detail = null) {
    const list = document.getElementById('agent-event-list'); if (!list) return;
    const item = document.createElement('div'); item.className = 'agent-event ' + String(kind || 'status').replace(/[^a-z-]/gi, '-');
    const label = document.createElement('div'); label.className = 'agent-event-kind'; label.textContent = kind || 'status';
    const body = document.createElement('div'); body.className = 'agent-event-message'; body.textContent = message || '';
    if (detail !== null && detail !== undefined) { const pre = document.createElement('pre'); try { pre.textContent = typeof detail === 'string' ? detail : JSON.stringify(detail, null, 2); } catch(e) { pre.textContent = String(detail); } body.appendChild(pre); }
    item.appendChild(label); item.appendChild(body); list.appendChild(item); while (list.children.length > 60) list.removeChild(list.firstChild); list.scrollTop = list.scrollHeight;
}
function setAgentConnectionState(connected, text) {
    const el = document.getElementById('agent-connection-state'); if (!el) return;
    el.textContent = text || (connected ? 'Connected' : 'Offline'); el.classList.toggle('connected', !!connected);
}
function helioWsUrl() { const scheme = location.protocol === 'https:' ? 'wss:' : 'ws:'; return scheme + '//' + ROVER_HOST + ':8001/ws/helio'; }
function connectHelioWebSocket() {
    if (helioSocket && helioSocket.readyState === WebSocket.OPEN) return Promise.resolve(helioSocket);
    if (helioSocketPromise) return helioSocketPromise;
    helioSocketPromise = new Promise((resolve, reject) => {
        const ws = new WebSocket(helioWsUrl()); helioSocket = ws; helioSocketClosing = false;
        const timeout = setTimeout(() => { try { ws.close(); } catch(e) {} reject(new Error('Helio WebSocket connection timeout.')); }, 6000);
        ws.onopen = () => { clearTimeout(timeout); setAgentConnectionState(true, 'Connected'); appendAgentEvent('status', 'Helio WebSocket connected.'); resolve(ws); };
        ws.onmessage = event => { let data; try { data = JSON.parse(event.data); } catch(e) { return; } handleHelioSocketEvent(data); };
        ws.onerror = () => { clearTimeout(timeout); setAgentConnectionState(false, 'Connection error'); if (ws.readyState !== WebSocket.OPEN) reject(new Error('Helio WebSocket connection failed.')); };
        ws.onclose = () => { clearTimeout(timeout); if (helioSocket === ws) helioSocket = null; helioSocketPromise = null; setAgentConnectionState(false, 'Disconnected'); if (!helioSocketClosing && modal.classList.contains('active')) { appendAgentEvent('error', 'Helio WebSocket disconnected. Reconnecting…'); setTimeout(() => { if (modal.classList.contains('active')) connectHelioWebSocket().catch(() => {}); }, 500); } };
    }).finally(() => { helioSocketPromise = null; });
    return helioSocketPromise;
}
function handleHelioSocketEvent(data) {
    if (data.conversation_id) helioConversationId = data.conversation_id;
    switch (data.type) {
        case 'session': setAgentConnectionState(true, 'Ready'); appendAgentEvent('status', 'Conversation started.', data.conversation_id); break;
        case 'status': {
            const stage = String(data.stage || 'status').toLowerCase();
            const liveLabel = stage === 'thinking' ? 'Thinking' : stage === 'tool_running' ? 'Using tool' : stage === 'listening' ? 'Listening' : stage === 'speaking' ? 'Speaking' : 'Working';
            setAgentConnectionState(true, liveLabel);
            appendAgentEvent(data.stage || 'status', data.message || '');
            if (data.stage === 'thinking') setRobotMood('thinking', 'Thinking', data.message || 'Processing…');
            else if (data.stage === 'tool_running') setRobotMood('thinking', 'Using tool', data.message || 'Accessing rover…');
            break;
        }
        case 'model': setAgentConnectionState(true, data.message || 'Helio'); appendAgentEvent('model', data.message || ''); break;
        case 'tool_call': setAgentConnectionState(true, 'Using tool'); appendAgentEvent('tool-call', `${data.tool || 'tool'}()`, data.inputs || {}); setRobotMood('thinking', 'Using tool', `Calling ${data.tool || 'tool'}…`); break;
        case 'tool_result': setAgentConnectionState(true, 'Working'); appendAgentEvent('tool-result', `${data.tool || 'tool'} completed`, data.result); break;
        case 'final': { setAgentConnectionState(true, 'Speaking'); stopThinkingIndicator(); const finalText = data.output || "I didn't receive a valid response."; appendHistory('agent', finalText); speakAndLoop(finalText); break; }
        case 'error': stopThinkingIndicator(); setAgentConnectionState(true, 'Ready'); appendAgentEvent('error', data.message || 'Unknown Helio error.'); setRobotMood('listening', 'Ready', data.message || 'I ran into a problem.'); break;
    }
}
function closeHelioWebSocket() { helioSocketClosing = true; if (helioSocket) { try { helioSocket.close(1000, 'Helio closed by user'); } catch(e) {} } helioSocket = null; helioSocketPromise = null; helioConversationId = null; setAgentConnectionState(false, 'Offline'); }

const SPEECH_LANG = (navigator.language || '').toLowerCase().startsWith('en') ? navigator.language : 'en-US';
const SILENCE_COMMIT_MS = 1200;
const MAX_UTTERANCE_MS = 30000;
let lastText = '', lastResultAt = 0, listenStartedAt = 0, committed = false, silenceTimer = null;

function clearListenTimers() {
    if (inactivityTimer) { clearTimeout(inactivityTimer); inactivityTimer = null; }
    if (silenceTimer) { clearTimeout(silenceTimer); silenceTimer = null; }
}

function startRecognition(retry = true) {
    if (!recognition) return;
    stopWake();
    try { recognition.start(); }
    catch (e) {
        if (retry) setTimeout(() => {
            if (modal.classList.contains('active') && (agentState === 'listening' || agentState === 'waking')) startRecognition(false);
        }, 250);
    }
}

function commitUtterance(text) {
    text = (text || '').replace(/\s+/g, ' ').trim();
    if (committed || agentState !== 'listening') return;
    if (text.length < 2) return;
    committed = true;
    clearListenTimers();
    captionText.textContent = text;
    appendHistory('you', text);
    processVoiceCommand(text);
}

function bestAlt(res) {
    let best = res[0];
    for (let j = 1; j < res.length; j++) if (res[j].confidence > best.confidence) best = res[j];
    return best.transcript;
}

if (SpeechRecognitionImpl) {
    recognition = new SpeechRecognitionImpl();
    recognition.continuous = false;
    recognition.interimResults = true;
    recognition.maxAlternatives = 3;
    recognition.lang = SPEECH_LANG;

    recognition.onstart = () => {
        agentState = 'listening'; committed = false; heardSpeech = false; lastText = '';
        listenStartedAt = Date.now(); listenDeadline = 0; setRobotMood('listening', 'Listening', 'Speak your command...'); setAgentConnectionState(true, 'Listening'); playListenSound(); clearListenTimers();
    };

    recognition.onerror = (e) => {
        if (e.error === 'aborted' || e.error === 'no-speech') return;
        if (agentState !== 'listening') return;
        if (heardSpeech && lastText && !committed) { commitUtterance(lastText); return; }
        if (e.error === 'not-allowed' || e.error === 'service-not-allowed' || e.error === 'audio-capture') {
            setRobotMood('listening', 'Error', `Microphone problem: ${e.error}`);
            setTimeout(closeVoiceModal, 3000);
        }
    };

    recognition.onresult = (event) => {
        if (agentState !== 'listening' || committed) return;
        let full = '', allFinal = true;
        for (let i = 0; i < event.results.length; i++) {
            full += bestAlt(event.results[i]) + ' ';
            if (!event.results[i].isFinal) allFinal = false;
        }
        full = full.replace(/\s+/g, ' ').trim();
        if (!full) return;

        heardSpeech = true; lastText = full; lastResultAt = Date.now();
        clearListenTimers();
        captionText.textContent = full;

        if (allFinal) commitUtterance(full);
        else silenceTimer = setTimeout(() => commitUtterance(lastText), SILENCE_COMMIT_MS);
    };

    recognition.onend = () => {
        if (agentState !== 'listening' || committed) return;
        if (heardSpeech && lastText) { commitUtterance(lastText); return; }
        setTimeout(() => { if (modal.classList.contains('active') && agentState === 'listening' && !committed) startRecognition(); }, 120);
    };

    setInterval(() => {
        if (agentState !== 'listening' || committed) return;
        const now = Date.now();
        if (heardSpeech && lastText && now - lastResultAt > SILENCE_COMMIT_MS + 700) commitUtterance(lastText);
        else if (listenStartedAt && now - listenStartedAt > MAX_UTTERANCE_MS) {
            if (lastText) commitUtterance(lastText); else { committed = true; setTimeout(() => { if (modal.classList.contains('active') && agentState === 'listening') startRecognition(); }, 100); }
        }
    }, 400);
}

function setRobotMood(className, statusMsg, captionMsg) {
    robotFace.className = 'robot-face ' + className;
    if (statusMsg !== null) {
        captionStatus.textContent = statusMsg;
        const live = document.getElementById('agent-connection-state');
        if (live && modal && modal.classList.contains('active')) {
            live.textContent = statusMsg;
            live.classList.add('connected');
        }
    }
    if (captionMsg !== null) {
        captionText.innerHTML = (typeof marked !== 'undefined' && captionMsg.length > 20) ? marked.parse(captionMsg) : captionMsg;
    }
}

function openVoiceModal() {
    if (!recognition) return alert("Voice not supported on this browser.");
    if (modal.classList.contains('active')) return;
    stopWake(); modal.classList.add('active'); playStartupSound(); clearAgentLiveStatus(); setAgentConnectionState(false, 'Connecting…');
    const bubble = document.getElementById('speech-bubble'), greeting = 'Hello! How can I help?';
    if (bubble) { bubble.textContent = greeting; bubble.classList.add('show'); setTimeout(() => bubble.classList.remove('show'), 3200); }
    agentState = 'speaking'; setRobotMood('speaking', 'Helio', greeting);
    connectHelioWebSocket().catch(err => { appendAgentEvent('error', err.message || 'Unable to connect to Helio.'); setRobotMood('listening', 'Ready', 'Connection failed. Retrying…'); });
    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel(); const u = new SpeechSynthesisUtterance(greeting); u.lang='en-US'; u.rate=1.0;
        const startListening = () => { if (modal.classList.contains('active') && agentState === 'speaking') { agentState='listening'; startRecognition(); } };
        u.onend=startListening; u.onerror=startListening; setTimeout(startListening,1500); window.currentUtterance=u; window.currentSpokenText=greeting; window.speechSynthesis.speak(u);
    } else { agentState='listening'; startRecognition(); }
}
function closeVoiceModal() {
    agentState='idle'; listenDeadline=0; clearListenTimers(); modal.classList.remove('active'); historyListEl.innerHTML=''; clearAgentLiveStatus();
    const bubble=document.getElementById('speech-bubble'); if (bubble) bubble.classList.remove('show');
    if (recognition) { try { recognition.stop(); } catch(e){} } if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    stopThinkingIndicator(); closeHelioWebSocket(); const ui=document.getElementById('user-input'); if(ui) ui.value=''; syncWake();
}

async function sendManualText() {
    const input=document.getElementById('user-input'), btn=document.getElementById('manual-send-btn'), text=(input&&input.value?input.value:'').trim();
    if(!text){notify('WARNING','Enter a command before pressing SEND.');if(input)input.focus();return;}
    if(!modal.classList.contains('active')) openVoiceModal(); appendHistory('you',text); if(input)input.value='';
    if(btn){btn.disabled=true;btn.textContent='SENDING…';} try{await processVoiceCommand(text);}finally{if(btn){btn.disabled=false;btn.textContent='SEND';}}
}
async function processVoiceCommand(text) {
    agentState='thinking'; listenDeadline=0; clearListenTimers(); try{recognition.stop();}catch(e){}
    setRobotMood('thinking','Processing',text); startThinkingIndicator(); playProcessingSound();
    const isHinglish=document.getElementById('hinglish-toggle')&&document.getElementById('hinglish-toggle').checked;
    const userField=document.getElementById('user-input'); const baseMessage=`MAIN PROMPT: ${text}\nUSER INPUT FIELD: ${(userField&&userField.value||'').trim()}`;
    const payloadMessage=isHinglish?`[CRITICAL INSTRUCTION: The user is speaking in Hinglish (Hindi written in English alphabet). Translate the user's query to English internally to understand it, formulate your response, and then TRANSLATE YOUR ENTIRE FINAL RESPONSE BACK TO HINGLISH. Your final output must be exclusively in Hinglish (Hindi words written with English letters) so the text-to-speech sounds like conversational Hindi.]\n\n${baseMessage}`:baseMessage;
    try{const ws=await connectHelioWebSocket();if(ws.readyState!==WebSocket.OPEN)throw new Error('Helio WebSocket is not connected.');ws.send(JSON.stringify({type:'message',message:payloadMessage}));appendAgentEvent('status','Request sent over WebSocket.');}
    catch(err){stopThinkingIndicator();appendAgentEvent('error',err.message||'Helio connection error.');speakAndLoop('Sorry, I am having trouble connecting to Helio.');}
}

function speakThenClose(text) { speakAndLoop(text); }

function speakAndLoop(text) {
    agentState = 'speaking';
    setRobotMood('speaking', 'Response', text);

    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
        const utterance = new SpeechSynthesisUtterance(text);
        
        const isHinglish = document.getElementById('hinglish-toggle') && document.getElementById('hinglish-toggle').checked;
        utterance.lang = isHinglish ? 'hi-IN' : 'en-US'; 
        utterance.rate = 1.0;
        
        utterance.onend = () => {
            if (window.currentUtterance !== utterance || agentState !== 'speaking') return;
            if (modal.classList.contains('active')) {
                agentState = 'listening';
                setRobotMood('listening', 'Listening', 'Listening for next command...');
                setAgentConnectionState(true, 'Listening');
                startRecognition();
            }
        };

        window.currentUtterance = utterance;
        window.currentSpokenText = text;
        window.speechSynthesis.speak(utterance);
        syncWake();
    } else {
        setTimeout(() => { if (modal.classList.contains('active')) { agentState='listening'; startRecognition(); } }, 300); 
    }
}

const manualInput = document.getElementById('user-input');
if (manualInput) {
    manualInput.addEventListener('keydown', (event) => {
        if (event.key === 'Enter' && !event.shiftKey) {
            event.preventDefault();
            sendManualText();
        }
    });
}

const WAKE_OPTIONS = {
    rover: ['rover', 'rova', 'rovar', 'ro ver'],
    helio: ['helio', 'heleo', 'healio', 'hilio', 'hello'],
    computer: ['computer'],
    car: ['car'],
    robot: ['robot']
};
let wakeWord = 'rover';
try { const w = localStorage.getItem('milusions-wake-word'); if (w && WAKE_OPTIONS[w]) wakeWord = w; } catch (e) {}
let WAKE_RE = null;
function buildWakeRe() { WAKE_RE = new RegExp('\\b(' + WAKE_OPTIONS[wakeWord].join('|') + ')\\b', 'i'); }
buildWakeRe();
let wakeRec = null, wakeRunning = false, wakeWanted = false, wakeBlocked = false;
let wakeEnabled = true;
try { wakeEnabled = localStorage.getItem('milusions-wake') !== 'off'; } catch (e) {}

function updateWakeBtn() {
    const btn = document.getElementById('wake-btn');
    if (!btn) return;
    const on = wakeEnabled && !wakeBlocked;
    btn.classList.toggle('on', on); btn.classList.toggle('off', !on);
    document.getElementById('wake-label').textContent = wakeWord;
    btn.title = 'Wake word "' + wakeWord + '": ' + (wakeBlocked ? 'blocked (microphone)' : (wakeEnabled ? 'on' : 'off'));
    renderWakeMenu();
}

function renderWakeMenu() {
    const m = document.getElementById('wake-menu');
    if (!m) return;
    m.innerHTML = '<div class="wm-title">Wake word</div>';
    Object.keys(WAKE_OPTIONS).forEach(w => {
        const b = document.createElement('button');
        b.setAttribute('role', 'menuitemradio');
        b.setAttribute('aria-checked', w === wakeWord ? 'true' : 'false');
        if (w === wakeWord) b.classList.add('sel');
        b.textContent = w;
        b.onclick = () => { setWakeWord(w); closeWakeMenu(); };
        m.appendChild(b);
    });
    m.appendChild(document.createElement('hr'));
    const t = document.createElement('button');
    t.className = 'wm-toggle' + (wakeEnabled ? ' on' : '');
    t.dataset.state = wakeEnabled ? 'ON' : 'OFF';
    t.textContent = 'Listen for wake word';
    t.onclick = () => { toggleWake(); };
    m.appendChild(t);
}

function setWakeWord(w) {
    if (!WAKE_OPTIONS[w] || w === wakeWord) return;
    wakeWord = w; buildWakeRe();
    try { localStorage.setItem('milusions-wake-word', w); } catch (e) {}
    updateWakeBtn();
    if (typeof modal !== 'undefined' && modal.classList.contains('active') && agentState === 'asleep') {
        captionText.textContent = 'Say "' + wakeWord + '" to wake me up';
    }
    notify('INFO', 'Wake word set to "' + wakeWord + '".');
}

function toggleWakeMenu(ev) {
    ev.stopPropagation();
    const m = document.getElementById('wake-menu'), btn = document.getElementById('wake-btn');
    const open = m.hidden;
    m.hidden = !open;
    btn.setAttribute('aria-expanded', open ? 'true' : 'false');
}
function closeWakeMenu() {
    const m = document.getElementById('wake-menu');
    if (m) m.hidden = true;
    const btn = document.getElementById('wake-btn');
    if (btn) btn.setAttribute('aria-expanded', 'false');
}
document.addEventListener('click', (e) => { if (!e.target.closest('.wake-menu-wrap')) closeWakeMenu(); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') closeWakeMenu(); });

function syncWake() {
    const popupOpen = modal.classList.contains('active');
    const should = wakeEnabled && !wakeBlocked && SpeechRecognitionImpl && !popupOpen;
    if (should) startWake(); else stopWake();
}
function startWake() {
    wakeWanted = true;
    if (wakeRunning || !wakeRec) return;
    try { wakeRec.start(); } catch (e) {}
}
function stopWake() {
    wakeWanted = false;
    if (wakeRunning && wakeRec) { try { wakeRec.abort(); } catch (e) {} }
}
function toggleWake() {
    wakeEnabled = !wakeEnabled;
    if (wakeEnabled) wakeBlocked = false;
    try { localStorage.setItem('milusions-wake', wakeEnabled ? 'on' : 'off'); } catch (e) {}
    updateWakeBtn(); syncWake();
    notify('INFO', 'Wake word "' + wakeWord + '" ' + (wakeEnabled ? 'enabled.' : 'disabled.'));
}

if (SpeechRecognitionImpl) {
    wakeRec = new SpeechRecognitionImpl();
    wakeRec.continuous = true;
    wakeRec.interimResults = true;
    wakeRec.maxAlternatives = 5;
    wakeRec.lang = SPEECH_LANG;
    wakeRec.onstart = () => { wakeRunning = true; };
    wakeRec.onend = () => {
        wakeRunning = false;
        if (wakeWanted) setTimeout(() => { if (wakeWanted && !wakeRunning) { try { wakeRec.start(); } catch (e) {} } }, 400);
    };
    wakeRec.onerror = (e) => {
        if (e.error === 'not-allowed' || e.error === 'service-not-allowed') {
            wakeBlocked = true; wakeWanted = false; updateWakeBtn();
            notify('WARNING', 'Microphone access is blocked, so the wake word is disabled.');
        }
    };
    wakeRec.onresult = (ev) => {
        for (let i = ev.resultIndex; i < ev.results.length; i++) {
            const res = ev.results[i];
            for (let j = 0; j < res.length; j++) {
                const t = res[j].transcript;
                if (agentState === 'speaking') {
                    if (isEchoOfSpeech(t)) continue;
                    if (!bargeInAllowed()) continue;
                    if (t.trim().length >= 2) { onWakeWord(t); return; }
                    continue;
                }

                if (!WAKE_RE.test(t)) continue;
                onWakeWord(); return;
            }
        }
    };
}

function isEchoOfSpeech(t) {
    const words = (t.toLowerCase().match(/[a-z\u0900-\u097f']+/g) || []);
    if (!words.length) return true;
    const spoken = new Set((String(window.currentSpokenText || '').toLowerCase().match(/[a-z\u0900-\u097f']+/g) || []));
    const hit = words.filter(w => spoken.has(w)).length;
    return hit / words.length >= 0.5;
}

let vadStream = null, vadCtx = null, vadAn = null, vadBuf = null, vadTimer = null;
let vadResidual = 0.02, vadLoudSince = 0, vadLastLoud = 0, speakStartAt = 0;
async function startVad() {
    if (vadTimer) return;
    speakStartAt = Date.now(); vadResidual = 0.02; vadLoudSince = 0; vadLastLoud = 0;
    try {
        if (!vadStream) vadStream = await navigator.mediaDevices.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: false } });
        if (!vadCtx) {
            vadCtx = new (window.AudioContext || window.webkitAudioContext)();
            vadAn = vadCtx.createAnalyser(); vadAn.fftSize = 1024;
            vadCtx.createMediaStreamSource(vadStream).connect(vadAn);
            vadBuf = new Float32Array(vadAn.fftSize);
        }
        if (vadCtx.state === 'suspended') vadCtx.resume();
    } catch (e) { return; }
    vadTimer = setInterval(() => {
        vadAn.getFloatTimeDomainData(vadBuf);
        let sum = 0; for (let i = 0; i < vadBuf.length; i++) sum += vadBuf[i] * vadBuf[i];
        const rms = Math.sqrt(sum / vadBuf.length), now = Date.now();
        const thr = Math.max(0.09, vadResidual * 3.5);
        if (rms > thr) {
            if (!vadLoudSince) vadLoudSince = now;
            if (now - vadLoudSince > 450) vadLastLoud = now;
        } else {
            vadLoudSince = 0;
            vadResidual = vadResidual * 0.97 + rms * 0.03;
        }
    }, 50);
}
function stopVad() { if (vadTimer) { clearInterval(vadTimer); vadTimer = null; } }
function bargeInAllowed() {
    if (Date.now() - speakStartAt < 1500) return false;
    if (!vadTimer) return false;
    return Date.now() - vadLastLoud < 1500;
}
setInterval(() => { if (agentState === 'speaking' && modal.classList.contains('active')) startVad(); else stopVad(); }, 300);

function onWakeWord(initialText = '') {
    stopWake();
    if (modal.classList.contains('active')) {
        window.currentUtterance = null;
        if ('speechSynthesis' in window) window.speechSynthesis.cancel();
        stopThinkingIndicator();
        if (agentState === 'asleep') {
            wakeFromSleep();
        } else if (agentState === 'speaking' || agentState === 'thinking') {
            window.currentUtterance = null; agentState='listening'; listenDeadline=0;
            if ('speechSynthesis' in window) window.speechSynthesis.cancel(); stopThinkingIndicator();
            heardSpeech=!!initialText; lastText=initialText; committed=false; if(initialText) captionText.textContent=initialText;
            setRobotMood('listening','Listening','I heard you. Listening…'); playListenSound();
            setTimeout(() => { if(agentState==='listening') startRecognition(); },80);
        }
    } else {
        openVoiceModal();
    }
}

function goToSleep() {
    if (!modal.classList.contains('active')) return; agentState='listening'; listenDeadline=0; clearListenTimers();
    try{recognition.stop();}catch(e){} setRobotMood('listening','Listening','Still listening…');
    setTimeout(()=>{if(modal.classList.contains('active')&&agentState==='listening')startRecognition();},100);
}

function wakeFromSleep() {
    if(!modal.classList.contains('active')) return; stopWake(); agentState='listening'; setRobotMood('listening','Listening','Listening…'); startRecognition();
}

updateWakeBtn();
syncWake();

function stopTalking() {
    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
    }
    window.currentUtterance = null;
    stopThinkingIndicator();
    if (agentState === 'speaking' || agentState === 'thinking') {
        agentState = 'listening';
        listenDeadline = 0; heardSpeech = false; lastText = '';
        setRobotMood('listening', 'Listening', 'Stopped talking. Listening...');
        setTimeout(() => { if (agentState === 'listening') startRecognition(); }, 150);
    }
    notify('INFO', 'Announcer stopped.');
}

function setLang(lang) {
    const isHi = lang === 'hi';
    const toggle = document.getElementById('hinglish-toggle');
    if (toggle) toggle.checked = isHi;
    
    document.getElementById('btn-en').classList.toggle('active', !isHi);
    document.getElementById('btn-hi').classList.toggle('active', isHi);
}

function mapApiUrl(path) {
    return 'http://' + ROVER_HOST + ':8000' + path;
}

function updateMapButtons() {
    const select = document.getElementById('map-name-select');
    const name = select ? select.value.trim() : '';
    const valid = !!name && availableMaps.includes(name);
    const view = document.getElementById('view-map-btn');
    const del = document.getElementById('delete-map-btn');
    if (view) view.disabled = !valid;
    if (del) del.disabled = !valid;
    const dl = ensureMapDownloadBtn();
    if (dl) dl.disabled = !valid;
}

function ensureMapDownloadBtn() {
    let b = document.getElementById('download-map-btn');
    if (b) return b;
    const del = document.getElementById('delete-map-btn');
    if (!del) return null;
    b = document.createElement('button');
    b.className = 'btn btn-secondary'; b.id = 'download-map-btn'; b.type = 'button';
    b.textContent = 'Download ZIP'; b.title = 'Download the selected map as a .zip';
    b.onclick = downloadSelectedMap;
    del.parentElement.insertBefore(b, del);
    return b;
}

function downloadSelectedMap() {
    const select = document.getElementById('map-name-select');
    const name = select ? select.value.trim() : '';
    if (!name || !availableMaps.includes(name)) return notify('WARNING', 'Select a saved map first.');
    const a = document.createElement('a');
    a.href = mapApiUrl('/maps/' + encodeURIComponent(name) + '/zip');
    a.download = name + '.zip';
    document.body.appendChild(a); a.click(); a.remove();
    notify('INFO', 'Downloading ' + name + '.zip…');
}

function openMapAddModal() {
    if (!ROVER_HOST) return notify('WARNING', 'Connect to the rover before adding a map.');
    const modal = document.getElementById('map-add-modal');
    const name = document.getElementById('new-map-name');
    ['new-map-posegraph','new-map-data'].forEach(function (i) { const e = document.getElementById(i); if (e) e.value = ''; });
    const pgm = document.getElementById('new-map-pgm');
    const yaml = document.getElementById('new-map-yaml');
    const err = document.getElementById('map-add-error');
    if (name) name.value = '';
    if (pgm) pgm.value = '';
    if (yaml) yaml.value = '';
    if (err) err.textContent = '';
    if (modal) modal.hidden = false;
    setTimeout(() => name && name.focus(), 30);
}

function closeMapAddModal() {
    const modal = document.getElementById('map-add-modal');
    if (modal) modal.hidden = true;
}

function closeMapPreview() {
    const modal = document.getElementById('map-preview-modal');
    if (modal) modal.hidden = true;
    const canvas = document.getElementById('map-preview-canvas');
    if (canvas) { canvas.width = 1; canvas.height = 1; }
}

async function addMapFromFiles() {
    const nameEl = document.getElementById('new-map-name');
    const pgmEl = document.getElementById('new-map-pgm');
    const yamlEl = document.getElementById('new-map-yaml');
    const errEl = document.getElementById('map-add-error');
    const submit = document.getElementById('add-map-submit');
    const name = (nameEl ? nameEl.value : '').trim();
    const pgm = pgmEl && pgmEl.files ? pgmEl.files[0] : null;
    const yaml = yamlEl && yamlEl.files ? yamlEl.files[0] : null;

    if (errEl) errEl.textContent = '';
    if (!name) return setMapAddError('Enter a map name.');
    if (!/^[A-Za-z0-9 _.-]+$/.test(name) || name === '.' || name === '..') {
        return setMapAddError('Use only letters, numbers, spaces, _, -, and . in the map name.');
    }
    if (!pgm || !yaml) return setMapAddError('Select both a PGM file and a YAML file.');
    if (!/\.pgm$/i.test(pgm.name)) return setMapAddError('The image file must be a .pgm file.');
    if (!/\.ya?ml$/i.test(yaml.name)) return setMapAddError('The metadata file must be a .yaml or .yml file.');

    if (submit) { submit.disabled = true; submit.classList.add('loading'); }
    let created = false;
    try {
        let r = await fetch(mapApiUrl('/maps'), {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ map_name: name })
        });
        if (!r.ok) throw new Error(await responseError(r));
        created = true;

        r = await fetch(mapApiUrl('/maps/' + encodeURIComponent(name) + '/pgm'), {
            method: 'PUT', headers: { 'Content-Type': 'application/octet-stream' },
            body: await pgm.arrayBuffer()
        });
        if (!r.ok) throw new Error(await responseError(r));

        r = await fetch(mapApiUrl('/maps/' + encodeURIComponent(name) + '/yaml'), {
            method: 'PUT', headers: { 'Content-Type': 'text/yaml; charset=utf-8' },
            body: await yaml.text()
        });
        if (!r.ok) throw new Error(await responseError(r));

        // optional SLAM pose graph files (enable "Autonomous + Map Update")
        const pg = document.getElementById('new-map-posegraph'), dt = document.getElementById('new-map-data');
        const pgF = pg && pg.files ? pg.files[0] : null, dtF = dt && dt.files ? dt.files[0] : null;
        if (!pgF || !dtF) throw new Error('Select both the .posegraph and .data files.');
        {
            for (const [ext, f] of [['posegraph', pgF], ['data', dtF]]) {
                r = await fetch(mapApiUrl('/maps/' + encodeURIComponent(name) + '/' + ext), {
                    method: 'PUT', headers: { 'Content-Type': 'application/octet-stream' },
                    body: await f.arrayBuffer()
                });
                if (!r.ok) throw new Error(await responseError(r));
            }
        }

        closeMapAddModal();
        notify('INFO', 'Map "' + name + '" added successfully.');
        await fetchMaps({ notifyError: true });
        const select = document.getElementById('map-name-select');
        if (select && availableMaps.includes(name)) select.value = name;
        updateMapButtons();
    } catch (e) {
        if (created) {
            try { await fetch(mapApiUrl('/maps/' + encodeURIComponent(name)), { method: 'DELETE' }); } catch (_) {}
        }
        setMapAddError(e.message || 'Failed to add map.');
    } finally {
        if (submit) { submit.disabled = false; submit.classList.remove('loading'); }
    }
}

function setMapAddError(message) {
    const el = document.getElementById('map-add-error');
    if (el) el.textContent = message;
}

async function responseError(r) {
    let message = 'HTTP ' + r.status;
    try {
        const data = await r.json();
        if (data && data.detail) message = data.detail;
    } catch (_) {}
    return message;
}

async function deleteSelectedMap() {
    const select = document.getElementById('map-name-select');
    const name = select ? select.value.trim() : '';
    if (!name || !availableMaps.includes(name)) return;
    if (!window.confirm('Delete map "' + name + '" and all of its files? This cannot be undone.')) return;

    const btn = document.getElementById('delete-map-btn');
    if (btn) { btn.disabled = true; btn.classList.add('loading'); }
    try {
        const r = await fetch(mapApiUrl('/maps/' + encodeURIComponent(name)), { method: 'DELETE' });
        if (!r.ok) throw new Error(await responseError(r));
        notify('INFO', 'Map "' + name + '" deleted.');
        await fetchMaps({ notifyError: true });
        updateMapButtons();
    } catch (e) {
        notify('ERROR', 'Failed to delete map: ' + e.message);
        updateMapButtons();
    } finally {
        if (btn) btn.classList.remove('loading');
    }
}

async function openSelectedMapPreview() {
    const select = document.getElementById('map-name-select');
    const name = select ? select.value.trim() : '';
    if (!name || !availableMaps.includes(name)) return;

    const modal = document.getElementById('map-preview-modal');
    const title = document.getElementById('map-preview-title');
    const meta = document.getElementById('map-preview-meta');
    const loading = document.getElementById('map-preview-loading');
    const error = document.getElementById('map-preview-error');
    const canvas = document.getElementById('map-preview-canvas');
    if (!modal || !canvas) return;

    title.textContent = name;
    meta.textContent = 'PGM occupancy image — loading…';
    error.textContent = '';
    loading.hidden = false;
    canvas.hidden = true;
    modal.hidden = false;

    try {
        const r = await fetch(mapApiUrl('/maps/' + encodeURIComponent(name) + '/pgm?t=' + Date.now()), { cache: 'no-store' });
        if (!r.ok) throw new Error(await responseError(r));
        const buffer = await r.arrayBuffer();
        const info = renderPgmToCanvas(buffer, canvas);
        loading.hidden = true;
        canvas.hidden = true;
        initPreviewViewer(canvas, name);
        meta.textContent = info.width + ' × ' + info.height + ' px · PGM ' + info.magic;
    } catch (e) {
        loading.hidden = true;
        error.textContent = 'Could not load PGM: ' + e.message;
    }
}

function renderPgmToCanvas(buffer, canvas) {
    const bytes = new Uint8Array(buffer);
    let pos = 0;

    function nextToken() {
        while (pos < bytes.length) {
            const c = bytes[pos];
            if (c === 35) {
                while (pos < bytes.length && bytes[pos] !== 10 && bytes[pos] !== 13) pos++;
            } else if (c <= 32) {
                pos++;
            } else break;
        }
        const start = pos;
        while (pos < bytes.length && bytes[pos] > 32 && bytes[pos] !== 35) pos++;
        return new TextDecoder().decode(bytes.subarray(start, pos));
    }

    const magic = nextToken();
    if (magic !== 'P2' && magic !== 'P5') throw new Error('Unsupported PGM format ' + magic);
    const width = Number(nextToken()), height = Number(nextToken()), maxval = Number(nextToken());
    if (!Number.isFinite(width) || !Number.isFinite(height) || !Number.isFinite(maxval) || width <= 0 || height <= 0 || maxval <= 0 || maxval > 65535) {
        throw new Error('Invalid PGM header');
    }

    const count = width * height;
    const pixels = new Uint8ClampedArray(count);
    if (magic === 'P2') {
        for (let i = 0; i < count; i++) {
            const v = Number(nextToken());
            if (!Number.isFinite(v)) throw new Error('PGM pixel data is incomplete');
            pixels[i] = Math.max(0, Math.min(255, Math.round(v * 255 / maxval)));
        }
    } else {
        while (pos < bytes.length && bytes[pos] <= 32) pos++;
        if (maxval <= 255) {
            if (bytes.length - pos < count) throw new Error('PGM pixel data is incomplete');
            for (let i = 0; i < count; i++) pixels[i] = Math.round(bytes[pos + i] * 255 / maxval);
        } else {
            if (bytes.length - pos < count * 2) throw new Error('PGM pixel data is incomplete');
            for (let i = 0; i < count; i++) {
                const v = (bytes[pos + i * 2] << 8) | bytes[pos + i * 2 + 1];
                pixels[i] = Math.round(v * 255 / maxval);
            }
        }
    }

    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');
    const image = ctx.createImageData(width, height);
    for (let i = 0, j = 0; i < count; i++, j += 4) {
        const v = pixels[i];
        image.data[j] = v; image.data[j + 1] = v; image.data[j + 2] = v; image.data[j + 3] = 255;
    }
    ctx.putImageData(image, 0, 0);
    return { width, height, magic };
}

window.openMapAddModal = openMapAddModal;
window.closeMapAddModal = closeMapAddModal;
window.openMapSaveModal = openMapSaveModal;
window.closeMapSaveModal = closeMapSaveModal;
window.saveMapFromModal = saveMapFromModal;
window.addMapFromFiles = addMapFromFiles;
window.openSelectedMapPreview = openSelectedMapPreview;
window.closeMapPreview = closeMapPreview;
window.deleteSelectedMap = deleteSelectedMap;
window.updateMapButtons = updateMapButtons;

document.addEventListener('keydown', e => {
    if (e.key !== 'Escape') return;
    const add = document.getElementById('map-add-modal');
    const save = document.getElementById('map-save-modal');
    const preview = document.getElementById('map-preview-modal');
    if (add && !add.hidden) closeMapAddModal();
    if (save && !save.hidden) closeMapSaveModal();
    if (preview && !preview.hidden) closeMapPreview();
});

(function () {
    const lg = document.getElementById('layer-legend'); if (!lg) return;
    ['pointerup', 'pointermove', 'click', 'contextmenu'].forEach(t => lg.addEventListener(t, e => e.stopPropagation()));
})();

(function () {
    const st = document.createElement('style');
    st.textContent = '.chart-full-btn{margin-left:8px;border:1px solid var(--border);background:transparent;color:var(--muted);border-radius:4px;font-size:12px;line-height:1;padding:3px 6px;cursor:pointer}' +
        '.chart-full-btn:hover{color:var(--text);border-color:var(--muted)}' +
        '.panel.chart-full{position:fixed!important;inset:12px;z-index:9999;height:auto!important;width:auto!important;background:var(--panel);box-shadow:0 10px 40px rgba(0,0,0,.35)}' +
        '#graphs-resizer{display:none!important}';
    document.head.appendChild(st);
    let full = null;
    function setFull(panel) {
        if (full) { full.classList.remove('chart-full'); full.querySelector('.chart-full-btn').textContent = '⛶'; }
        full = (panel && panel !== full) ? panel : null;
        if (full) { full.classList.add('chart-full'); full.querySelector('.chart-full-btn').textContent = '✕'; }
    }
    wheelIds.forEach(id => {
        const cv = document.getElementById('chart-' + id); if (!cv) return;
        const panel = cv.closest('.panel'), hd = panel.querySelector('.panel-hd');
        const b = document.createElement('button');
        b.className = 'chart-full-btn'; b.textContent = '⛶'; b.title = 'Full screen (Esc to close)';
        b.addEventListener('click', () => setFull(panel));
        hd.appendChild(b);
        hd.addEventListener('dblclick', () => setFull(panel));
    });
    document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && full) setFull(null); });
})();

ensureMapDownloadBtn();

(function () {
    const st = document.createElement('style');
    st.textContent = '.pv-wrap{display:flex;flex-direction:column;width:100%;gap:8px}' +
        '.pv-bar{display:flex;gap:6px;align-items:center;flex-wrap:wrap}' +
        '.pv-btn{border:1px solid var(--border);background:var(--panel);color:var(--text);border-radius:5px;padding:4px 10px;font-size:12px;cursor:pointer}' +
        '.pv-btn:hover{border-color:var(--muted)}.pv-btn.on{color:var(--primary);border-color:var(--primary)}' +
        '.pv-info{margin-left:auto;font-size:12px;color:var(--muted);font-family:monospace}' +
        '.pv-canvas{width:100%;height:62vh;border:1px solid var(--border);background:#e9ecef;border-radius:4px;touch-action:none;cursor:grab}' +
        '.pv-canvas.measure{cursor:crosshair}';
    document.head.appendChild(st);
})();

async function initPreviewViewer(srcCanvas, mapName) {
    const body = srcCanvas.parentElement;
    let wrap = body.querySelector('.pv-wrap'); if (wrap) wrap.remove();
    wrap = document.createElement('div'); wrap.className = 'pv-wrap';
    wrap.innerHTML = '<div class="pv-bar">' +
        '<button class="pv-btn" data-a="in" title="Zoom in">＋</button><button class="pv-btn" data-a="out" title="Zoom out">－</button>' +
        '<button class="pv-btn" data-a="fit">Fit</button>' +
        '<button class="pv-btn" data-a="measure" title="Click two points to measure distance">📏 Measure</button>' +
        '<button class="pv-btn" data-a="clear">Clear</button><span class="pv-info" id="pv-info">scroll = zoom · drag = pan</span></div>' +
        '<canvas class="pv-canvas"></canvas>';
    body.appendChild(wrap);
    const cv = wrap.querySelector('canvas'), info = wrap.querySelector('.pv-info');
    const src = srcCanvas, iw = src.width, ih = src.height;

    let res = null;
    try {
        const r = await fetch(mapApiUrl('/maps/' + encodeURIComponent(mapName) + '/yaml?t=' + Date.now()), { cache: 'no-store' });
        if (r.ok) { const m = /resolution:\s*([0-9.eE+-]+)/.exec(await r.text()); if (m) res = parseFloat(m[1]); }
    } catch (_) {}

    let s = 1, tx = 0, ty = 0, measure = false, pts = [], drag = null, moved = false;
    const dpr = window.devicePixelRatio || 1;
    const size = () => { const w = cv.clientWidth, h = cv.clientHeight;
        if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
        return [w, h]; };
    const toImg = (sx, sy) => [(sx - tx) / s, (sy - ty) / s];
    function fit() { const [w, h] = size(); s = Math.min(w / iw, h / ih) * 0.95; tx = (w - iw * s) / 2; ty = (h - ih * s) / 2; draw(); }
    function fmt(p, q) {
        const px = Math.hypot(q[0] - p[0], q[1] - p[1]);
        return res ? (px * res).toFixed(2) + ' m  (' + (px * res * 100).toFixed(0) + ' cm)' : px.toFixed(1) + ' px (no resolution)';
    }
    function draw() {
        const [w, h] = size(), ctx = cv.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
        ctx.imageSmoothingEnabled = false; ctx.drawImage(src, tx, ty, iw * s, ih * s);
        if (pts.length) {
            const P = pts.map(p => [tx + p[0] * s, ty + p[1] * s]);
            ctx.strokeStyle = '#e5484d'; ctx.fillStyle = '#e5484d'; ctx.lineWidth = 2;
            if (P.length === 2) { ctx.beginPath(); ctx.moveTo(P[0][0], P[0][1]); ctx.lineTo(P[1][0], P[1][1]); ctx.stroke(); }
            P.forEach(q => { ctx.beginPath(); ctx.arc(q[0], q[1], 4, 0, 7); ctx.fill(); });
            if (P.length === 2) {
                const label = fmt(pts[0], pts[1]), mx = (P[0][0] + P[1][0]) / 2, my = (P[0][1] + P[1][1]) / 2;
                ctx.font = '600 13px Roboto, sans-serif'; const tw = ctx.measureText(label).width + 14;
                ctx.fillStyle = '#e5484d'; ctx.beginPath(); ctx.roundRect(mx - tw / 2, my - 26, tw, 22, 6); ctx.fill();
                ctx.fillStyle = '#fff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(label, mx, my - 15);
                info.textContent = 'distance: ' + label;
            }
        }
    }
    function zoomAt(f, sx, sy) { const [ix, iy] = toImg(sx, sy); s = Math.max(0.05, Math.min(80, s * f)); tx = sx - ix * s; ty = sy - iy * s; draw(); }
    cv.addEventListener('wheel', (e) => { e.preventDefault(); const r = cv.getBoundingClientRect(); zoomAt(e.deltaY < 0 ? 1.15 : 1 / 1.15, e.clientX - r.left, e.clientY - r.top); }, { passive: false });
    cv.addEventListener('pointerdown', (e) => { cv.setPointerCapture(e.pointerId); drag = { x: e.clientX, y: e.clientY, tx, ty }; moved = false; if (!measure) cv.style.cursor = 'grabbing'; });
    cv.addEventListener('pointermove', (e) => {
        if (drag) { const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
            if (Math.abs(dx) + Math.abs(dy) > 4) moved = true;
            if (moved && !measure) { tx = drag.tx + dx; ty = drag.ty + dy; draw(); } }
        else if (measure && pts.length === 1) {
            const r = cv.getBoundingClientRect(), q = toImg(e.clientX - r.left, e.clientY - r.top);
            info.textContent = 'distance: ' + fmt(pts[0], q);
        }
    });
    cv.addEventListener('pointerup', (e) => {
        cv.style.cursor = '';
        if (drag && !moved && measure) {
            const r = cv.getBoundingClientRect(), q = toImg(e.clientX - r.left, e.clientY - r.top);
            if (pts.length >= 2) pts = [];
            pts.push(q); if (pts.length === 1) info.textContent = 'click the second point'; draw();
        }
        drag = null;
    });
    wrap.querySelectorAll('.pv-btn').forEach(b => b.addEventListener('click', () => {
        const a = b.dataset.a, [w, h] = size();
        if (a === 'in') zoomAt(1.4, w / 2, h / 2);
        else if (a === 'out') zoomAt(1 / 1.4, w / 2, h / 2);
        else if (a === 'fit') fit();
        else if (a === 'clear') { pts = []; info.textContent = 'scroll = zoom · drag = pan'; draw(); }
        else if (a === 'measure') { measure = !measure; b.classList.toggle('on', measure); cv.classList.toggle('measure', measure);
            info.textContent = measure ? 'click two points to measure' + (res ? '' : ' (resolution unknown → pixels)') : 'scroll = zoom · drag = pan'; }
    }));
    new ResizeObserver(draw).observe(cv);
    fit();
}
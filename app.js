// =====================================================================
//  Milusions WareGV Suite - dashboard logic
//  UI (index.html / style.css) + live rover integration:
//    - rosbridge  (/map /odom /tf /cmd_vel /plan /wheel_states, publishes /joy)
//    - REST API   (navigation, initial pose, waypoints, abort, mode, save map)
//    - WebRTC     (RealSense RGB + Depth, camera_webrtc_streamer.py)
//    - Helio      (continuous voice assistant, dual wake words, local commands)
// =====================================================================

const SESSION_HOST_KEY = 'Rover host';
const SESSION_EXPIRY_KEY = 'Rover host expires';

function isValidRoverHost(value) {
    const host = String(value || '').trim();
    if (host === 'localhost') return true;
    const parts = host.split('.');
    return parts.length === 4 && parts.every((part) =>
        /^\d+$/.test(part) && Number(part) >= 0 && Number(part) <= 255
    );
}

function readAuthenticatedRoverHost() {
    const configuredHost = String(window.ROVER_IP || window.ROVER_HOST || '').trim();
    if (isValidRoverHost(configuredHost)) return configuredHost;
    try {
        const storedHost = String(localStorage.getItem(SESSION_HOST_KEY) || '').trim();
        const expiry = Number(localStorage.getItem(SESSION_EXPIRY_KEY) || 0);
        if (isValidRoverHost(storedHost) && Date.now() < expiry) return storedHost;
    } catch (_) {}
    return '';
}

const ROVER_IP = readAuthenticatedRoverHost();
if (!ROVER_IP) {
    window.location.replace('index.html');
    throw new Error('Rover session is missing or expired. Redirecting to login.');
}

const REST_API_BASE = `http://${ROVER_IP}:8000`;
const ROSBRIDGE_WS_URL = `ws://${ROVER_IP}:9090`;
const WEBRTC_SIGNAL_URL = `http://${ROVER_IP}:8081`;
const HELIO_WS_URL = `ws://${ROVER_IP}:8001/ws/helio`;

const CFG = {
    chartWindowSec: 30,
    joyRateHz: 20,
    reachTolM: 0.30,
    planTimeoutMs: 20000,
    cmdVelStaleMs: 600,
    rosRetryMs: 3000
};

const TOPICS = {
    map: '/map',
    odom: '/odom',
    cmdVel: '/cmd_vel',
    plan: '/plan',
    wheel: '/wheel_states',
    jointStates: '/joint_states',
    localCostmap: '/local_costmap/costmap',
    globalCostmap: '/global_costmap/costmap',
    joy: '/joy',
    tf: '/tf',
    tfStatic: '/tf_static'
};

const layerVis = { map: true, plan: true, localCostmap: true, globalCostmap: true };
function setLayerVisible(key, visible) {
    layerVis[key] = !!visible;
    needsDraw = true;
}
const MAP_FRAME = 'map';
const MODE_LABELS = { auto_nav: 'Autonomous Driving and Mapping' };
const modeLabel = (m) => MODE_LABELS[m] || String(m || 'Unknown');

// =====================================================================
//  Notifications
// =====================================================================
const NOTIFY_TTL = { ERROR: 12000, WARNING: 8000, INFO: 5000 };

function notify(level, message) {
    level = (level || 'INFO').toUpperCase();
    const stack = document.getElementById('notify-stack');
    if (!stack) return;
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
//  Helio State Synchronization to Backend
// =====================================================================
async function notifyHelioState(event, state, text = '', sound_name = '') {
    try {
        await fetch(REST_API_BASE + '/helio/state', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ event, state, text, sound_name })
        });
    } catch (_) {}
}

// =====================================================================
//  Theme & Fullscreen
// =====================================================================
const theme = {};
function readTheme() {
    const cs = getComputedStyle(document.body);
    ['--map-bg', '--map-free', '--map-occ', '--map-grid', '--series-a', '--series-b', '--border', '--muted',
     '--text', '--success', '--primary', '--panel', '--danger', '--warning']
        .forEach(k => theme[k] = cs.getPropertyValue(k).trim());
}
function applyTheme(isLight) {
    document.body.classList.toggle('light-theme', isLight);
    readTheme();
    mapImageDirty = true; needsDraw = true;
    if (typeof mvDraw === 'function') mvDraw();
}
function toggleTheme() {
    const isLight = !document.body.classList.contains('light-theme');
    applyTheme(isLight);
    try { localStorage.setItem('milusions-theme', isLight ? 'light' : 'dark'); } catch (_) {}
}

function updateDocumentFullscreenButton() {
    const button = document.getElementById('document-fullscreen-btn');
    if (!button) return;
    const active = Boolean(document.fullscreenElement);
    button.title = active ? 'Exit fullscreen' : 'Enter fullscreen';
    button.setAttribute('aria-label', button.title);
    button.querySelector('span').textContent = '⛶';
}

async function toggleDocumentFullscreen() {
    try {
        if (document.fullscreenElement) {
            await document.exitFullscreen();
        } else if (document.documentElement.requestFullscreen) {
            await document.documentElement.requestFullscreen();
        }
    } catch (_) {}
    updateDocumentFullscreenButton();
}
document.addEventListener('fullscreenchange', updateDocumentFullscreenButton);
window.toggleDocumentFullscreen = toggleDocumentFullscreen;

// =====================================================================
//  State
// =====================================================================
let mapImageDirty = false, needsDraw = true;
let latestMap = null;
let latestPlan = [];
let navStatus = 'Idle';
let missionGoal = null;
let ignorePlansUntil = 0;
let distanceRemaining = null;
const mapCanvasOff = document.createElement('canvas');
let odom = null;
let odomPose = null;
let robot = null;
let poseDirty = false;
let baseFrame = 'base_link';
let currentCmdVel = { linear: 0, angular: 0 };
let cmdVelAt = 0;

let activeNavBtn = null;
let navInitTimeout = null;

function setNavLoading(btn) {
    if (activeNavBtn && activeNavBtn !== btn) {
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
    if (navInitTimeout) { clearTimeout(navInitTimeout); navInitTimeout = null; }
    if (activeNavBtn) {
        activeNavBtn.classList.remove('loading');
        activeNavBtn.disabled = false;
        activeNavBtn = null;
    }
}

function startNavWatchdog() {
    if (navInitTimeout) clearTimeout(navInitTimeout);
    navInitTimeout = setTimeout(async () => {
        navInitTimeout = null;
        clearNavLoading();
        navStatus = 'Aborted';
        missionGoal = null;
        latestPlan = [];
        ignorePlansUntil = Date.now() + 1500;
        needsDraw = true;
        try { await postJSON('/abort'); } catch (_) {}
        notify('ERROR', 'Nav2 did not produce a path within ' + (CFG.planTimeoutMs / 1000) + ' s. Mission aborted.');
    }, CFG.planTimeoutMs);
}

const wheelIds = ['fl', 'fr', 'bl', 'br'];
const series = {};
wheelIds.forEach(id => series[id] = { a: [], c: [] });

function formatSigned(v, digits = 2) {
    const n = Number(v);
    if (!Number.isFinite(n)) return '—';
    if (Math.abs(n) < 0.0005) return '0.' + '0'.repeat(digits);
    return (n > 0 ? '+' : '') + n.toFixed(digits);
}

function yawFromQuat(q) {
    if (!q) return 0;
    const x = q.x || 0, y = q.y || 0, z = q.z || 0, w = (q.w === undefined ? 1 : q.w);
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function toInt8Array(d) {
    if (typeof d === 'string') {
        const bin = atob(d), a = new Int8Array(bin.length);
        for (let i = 0; i < bin.length; i++) a[i] = bin.charCodeAt(i);
        return a;
    }
    return d || [];
}

// =====================================================================
//  ROS 2 BRIDGE
// =====================================================================
let ros = null, rosConnected = false, rosTopics = [], joyTopic = null, rosRetryTimer = null;
let lastMapAt = 0, rosConnectedAt = 0;

function setRosBadge(connected, text) {
    const b = document.getElementById('ros-status');
    if (!b) return;
    b.textContent = text;
    b.className = 'status-badge' + (connected ? ' connected' : '');
}

function rosSubscribe(name, type, cb, opts) {
    const t = new ROSLIB.Topic(Object.assign({ ros, name, messageType: type }, opts || {}));
    t.subscribe(cb);
    rosTopics.push(t);
    return t;
}

const tfEdges = new Map();
const stripSlash = (s) => String(s || '').replace(/^\/+/, '');

function onTF(msg) {
    const list = (msg && msg.transforms) || [];
    for (const t of list) {
        const child = stripSlash(t.child_frame_id), parent = stripSlash(t.header && t.header.frame_id);
        if (!child || !parent) continue;
        const tr = t.transform.translation;
        tfEdges.set(child, { parent, x: tr.x, y: tr.y, yaw: yawFromQuat(t.transform.rotation) });
    }
    poseDirty = true;
}

function lookupInMap(frame) {
    frame = stripSlash(frame);
    if (frame === MAP_FRAME) return { x: 0, y: 0, z: 0, yaw: 0 };
    const chain = [];
    let f = frame, guard = 0;
    while (f !== MAP_FRAME && guard++ < 16) {
        const e = tfEdges.get(f);
        if (!e) return null;
        chain.push(e);
        f = e.parent;
    }
    if (f !== MAP_FRAME) return null;
    let x = 0, y = 0, z = 0, yaw = 0;
    for (let i = chain.length - 1; i >= 0; i--) {
        const e = chain[i], c = Math.cos(yaw), s = Math.sin(yaw);
        x += c * e.x - s * e.y;
        y += s * e.x + c * e.y;
        yaw += e.yaw;
    }
    return { x, y, z: 0, yaw };
}

function refreshRobotPose() {
    poseDirty = false;
    const p = lookupInMap(baseFrame);
    if (p) robot = p;
    else if (odomPose) robot = odomPose;
    needsDraw = true;
}

function ingestWheel(d, now) {
    if (!d) return;
    const id = String(d.id || '').toLowerCase();
    const s = series[id];
    if (!s) return;
    const a = Number(d.a !== undefined ? d.a : d.actual);
    const c = Number(d.c !== undefined ? d.c : (d.cmd !== undefined ? d.cmd : d.command));
    if (Number.isFinite(a)) s.a.push({ t: now, v: a });
    if (Number.isFinite(c)) s.c.push({ t: now, v: c });
    const cutoff = now - CFG.chartWindowSec - 2;
    while (s.a.length > 1 && s.a[1].t < cutoff) s.a.shift();
    while (s.c.length > 1 && s.c[1].t < cutoff) s.c.shift();
}

function onWheelStates(msg) {
    let data;
    try { data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data; } catch (_) { return; }
    const now = performance.now() / 1000;
    if (Array.isArray(data)) data.forEach(d => ingestWheel(d, now));
    else if (data && Array.isArray(data.wheels)) data.wheels.forEach(d => ingestWheel(d, now));
    else if (data && data.id) ingestWheel(data, now);
    else if (data && typeof data === 'object') {
        wheelIds.forEach(id => { if (data[id]) ingestWheel(Object.assign({ id }, data[id]), now); });
    }
}

function matchWheelIdFromJointName(name) {
    const n = String(name || '').toLowerCase();
    const isFront = /front|\bfl\b|\bfr\b|_f_|^f_|_f$/.test(n) && !/rear|back/.test(n);
    const isRear = /rear|back|\bbl\b|\bbr\b|_r_|^r_|_r$/.test(n) && !/front/.test(n);
    const isLeft = /left|\bfl\b|\bbl\b|_l_|^l_|_l$/.test(n) && !/right/.test(n);
    const isRight = /right|\bfr\b|\bbr\b|_r_(?!ear)|^r_(?!ear)|_r$/.test(n) && !/left/.test(n);

    if (/\bfl\b/.test(n) || (isFront && isLeft)) return 'fl';
    if (/\bfr\b/.test(n) || (isFront && isRight)) return 'fr';
    if (/\bbl\b/.test(n) || (isRear && isLeft)) return 'bl';
    if (/\bbr\b/.test(n) || (isRear && isRight)) return 'br';
    return null;
}

function onJointStates(msg) {
    const names = msg.name || [];
    const vel = msg.velocity || [];
    const pos = msg.position || [];
    if (!names.length) return;
    const now = performance.now() / 1000;
    names.forEach((name, idx) => {
        const id = matchWheelIdFromJointName(name);
        if (!id) return;
        const v = vel.length ? Number(vel[idx]) : NaN;
        const p = pos.length ? Number(pos[idx]) : NaN;
        const value = Number.isFinite(v) ? v : p;
        if (Number.isFinite(value)) ingestWheel({ id, a: value }, now);
    });
}

// ----- Foxglove-style Costmap Color Palette Generator -----
// Maps costs (0-100) to distinct Foxglove colors:
// 0: transparent/free, 1-25: cyan, 26-50: blue, 51-75: magenta/purple, 76-100: red (lethal)
function costmapColorFoxglove(v) {
    if (v <= 0) return [0, 0, 0, 0];
    const t = Math.min(v, 100) / 100;
    let r = 0, g = 0, b = 0, a = Math.round(80 + t * 155);
    if (t < 0.25) { // Cyan spectrum
        r = 0; g = Math.round(255 * (t / 0.25)); b = 255;
    } else if (t < 0.5) { // Blue-Magenta spectrum
        r = Math.round(255 * ((t - 0.25) / 0.25)); g = 0; b = 255;
    } else if (t < 0.75) { // Magenta-Pink spectrum
        r = 255; g = 0; b = Math.round(255 * (1 - (t - 0.5) / 0.25));
    } else { // Red lethal obstacle spectrum
        r = 255; g = Math.round(50 * (1 - (t - 0.75) / 0.25)); b = 0;
    }
    return [r, g, b, a];
}

let latestLocalCostmap = null, latestGlobalCostmap = null;
let localCostmapDirty = false, globalCostmapDirty = false;
const localCostmapCanvasOff = document.createElement('canvas');
const globalCostmapCanvasOff = document.createElement('canvas');

function onLocalCostmapMsg(msg) {
    const info = msg.info;
    if (!info || !info.width || !info.height) return;
    latestLocalCostmap = {
        w: info.width, h: info.height, res: info.resolution,
        ox: info.origin.position.x, oy: info.origin.position.y,
        yaw: yawFromQuat(info.origin.orientation), data: toInt8Array(msg.data)
    };
    localCostmapDirty = true; needsDraw = true;
}

function onGlobalCostmapMsg(msg) {
    const info = msg.info;
    if (!info || !info.width || !info.height) return;
    latestGlobalCostmap = {
        w: info.width, h: info.height, res: info.resolution,
        ox: info.origin.position.x, oy: info.origin.position.y,
        yaw: yawFromQuat(info.origin.orientation), data: toInt8Array(msg.data)
    };
    globalCostmapDirty = true; needsDraw = true;
}

function rebuildCostmapImage(costmap, canvasOff) {
    if (!costmap) return;
    canvasOff.width = costmap.w; canvasOff.height = costmap.h;
    const ctx = canvasOff.getContext('2d');
    const img = ctx.createImageData(costmap.w, costmap.h), d = img.data;
    for (let j = 0; j < costmap.h; j++) {
        for (let i = 0; i < costmap.w; i++) {
            const v = costmap.data[j * costmap.w + i], k = (j * costmap.w + i) * 4;
            const rgba = costmapColorFoxglove(v);
            d[k] = rgba[0]; d[k + 1] = rgba[1]; d[k + 2] = rgba[2]; d[k + 3] = rgba[3];
        }
    }
    ctx.putImageData(img, 0, 0);
}
function rebuildLocalCostmapImage() { rebuildCostmapImage(latestLocalCostmap, localCostmapCanvasOff); }
function rebuildGlobalCostmapImage() { rebuildCostmapImage(latestGlobalCostmap, globalCostmapCanvasOff); }

function onMapMsg(msg) {
    const info = msg.info;
    if (!info || !info.width || !info.height) return;
    lastMapAt = Date.now();
    latestMap = {
        w: info.width, h: info.height, res: info.resolution,
        ox: info.origin.position.x, oy: info.origin.position.y,
        yaw: yawFromQuat(info.origin.orientation), data: toInt8Array(msg.data)
    };
    mapImageDirty = true;
    needsDraw = true;
}

function onOdomMsg(msg) {
    const pos = msg.pose.pose.position;
    const tw = msg.twist.twist.linear;
    if (msg.child_frame_id) baseFrame = stripSlash(msg.child_frame_id);
    odomPose = { x: pos.x, y: pos.y, z: 0, yaw: yawFromQuat(msg.pose.pose.orientation) };
    odom = { speed: Math.hypot(tw.x || 0, tw.y || 0) };
    poseDirty = true;
}

function onPlanMsg(msg) {
    if (Date.now() < ignorePlansUntil) return;
    const poses = (msg && msg.poses) || [];
    latestPlan = poses.map(p => ({
        x: p.pose.position.x, y: p.pose.position.y, z: 0, yaw: yawFromQuat(p.pose.orientation)
    }));
    if (latestPlan.length > 1) {
        clearNavLoading();
        if (navStatus !== 'Navigating') {
            navStatus = 'Navigating';
            if (!missionGoal) {
                const last = latestPlan[latestPlan.length - 1];
                missionGoal = { x: last.x, y: last.y, z: 0 };
            }
        }
    }
    needsDraw = true;
}

function subscribeAllTopics() {
    rosSubscribe(TOPICS.map, 'nav_msgs/OccupancyGrid', onMapMsg, { queue_length: 1 });
    rosSubscribe(TOPICS.odom, 'nav_msgs/Odometry', onOdomMsg, { queue_length: 1 });
    rosSubscribe(TOPICS.tf, 'tf2_msgs/TFMessage', onTF);
    rosSubscribe(TOPICS.tfStatic, 'tf2_msgs/TFMessage', onTF);
    rosSubscribe(TOPICS.cmdVel, 'geometry_msgs/Twist', (msg) => {
        currentCmdVel = { linear: msg.linear.x, angular: msg.angular.z };
        cmdVelAt = performance.now();
        needsDraw = true;
    }, { queue_length: 1 });
    rosSubscribe(TOPICS.plan, 'nav_msgs/Path', onPlanMsg, { queue_length: 1 });
    rosSubscribe(TOPICS.wheel, 'std_msgs/String', onWheelStates);
    rosSubscribe(TOPICS.jointStates, 'sensor_msgs/JointState', onJointStates, { queue_length: 1 });
    rosSubscribe(TOPICS.localCostmap, 'nav_msgs/OccupancyGrid', onLocalCostmapMsg, { queue_length: 1 });
    rosSubscribe(TOPICS.globalCostmap, 'nav_msgs/OccupancyGrid', onGlobalCostmapMsg, { queue_length: 1 });

    joyTopic = new ROSLIB.Topic({ ros, name: TOPICS.joy, messageType: 'sensor_msgs/Joy' });
    joyTopic.advertise();
}

function initROSBridge() {
    clearTimeout(rosRetryTimer);
    if (typeof ROSLIB === 'undefined') {
        setRosBadge(false, 'ROSLIB missing');
        notify('ERROR', 'roslib.js failed to load - ROS features are unavailable.');
        return;
    }
    setRosBadge(false, 'ROS Connecting…');
    const thisRos = new ROSLIB.Ros({ url: ROSBRIDGE_WS_URL });
    ros = thisRos;

    thisRos.on('connection', () => {
        if (thisRos !== ros) return;
        rosConnected = true;
        rosConnectedAt = Date.now();
        setRosBadge(true, 'ROS Connected');
        notify('INFO', 'Connected to ROS bridge.');
        subscribeAllTopics();
    });
    thisRos.on('error', (err) => {
        if (thisRos !== ros) return;
        if (!rosConnected) setRosBadge(false, 'ROS Error');
    });
    thisRos.on('close', () => {
        if (thisRos !== ros) return;
        const wasUp = rosConnected;
        rosConnected = false;
        rosTopics = [];
        joyTopic = null;
        setRosBadge(false, 'ROS Disconnected');
        if (wasUp) notify('WARNING', 'ROS bridge connection lost. Reconnecting…');
        rosRetryTimer = setTimeout(initROSBridge, CFG.rosRetryMs);
    });
}

// =====================================================================
//  REST API
// =====================================================================
async function postJSON(urlPath, body = {}) {
    let response;
    try {
        response = await fetch(REST_API_BASE + urlPath, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body)
        });
    } catch (_) {
        throw new Error('Cannot reach the rover API (' + REST_API_BASE + ')');
    }
    const raw = await response.text();
    let data = {};
    try { data = raw ? JSON.parse(raw) : {}; } catch (_) { data = { raw }; }
    if (!response.ok) {
        let detail = data && data.detail;
        if (detail && typeof detail !== 'string') detail = JSON.stringify(detail);
        throw new Error(detail || (data && data.message) || ('HTTP ' + response.status));
    }
    return data;
}

// =====================================================================
//  REALSENSE WEBRTC VIDEO
// =====================================================================
const WEBRTC_CFG = { signalUrl: WEBRTC_SIGNAL_URL, baseRetryMs: 1500, maxRetryMs: 8000 };

const webrtcStreams = {
    rgb: {
        video: document.getElementById('rgb-video'),
        empty: document.getElementById('rgb-video-empty'),
        state: document.getElementById('rgb-webrtc-state'),
        panel: document.getElementById('rgb-video') && document.getElementById('rgb-video').closest('.video-panel'),
        endpoint: '/offer/color', pc: null, retry: null, tries: 0
    },
    depth: {
        video: document.getElementById('depth-video'),
        empty: document.getElementById('depth-video-empty'),
        state: document.getElementById('depth-webrtc-state'),
        panel: document.getElementById('depth-video') && document.getElementById('depth-video').closest('.video-panel'),
        endpoint: '/offer/depth', pc: null, retry: null, tries: 0
    }
};

function setWebRTCState(s, live, label) {
    if (!s.state) return;
    s.state.textContent = label || (live ? 'LIVE' : 'OFFLINE');
    s.state.classList.toggle('live', !!live);
    if (s.panel) s.panel.classList.toggle('live', !!live);
}

function waitIceGathering(pc, ms) {
    return new Promise((resolve) => {
        if (pc.iceGatheringState === 'complete') return resolve();
        let t = null;
        const done = () => { clearTimeout(t); pc.removeEventListener('icegatheringstatechange', check); resolve(); };
        const check = () => { if (pc.iceGatheringState === 'complete') done(); };
        pc.addEventListener('icegatheringstatechange', check);
        t = setTimeout(done, ms);
    });
}

function scheduleWebRTCRetry(kind) {
    const s = webrtcStreams[kind];
    if (s.retry) clearTimeout(s.retry);
    s.tries += 1;
    const delay = Math.min(WEBRTC_CFG.baseRetryMs * Math.pow(1.5, s.tries - 1), WEBRTC_CFG.maxRetryMs);
    s.retry = setTimeout(() => startWebRTCStream(kind), delay);
}

async function startWebRTCStream(kind) {
    const s = webrtcStreams[kind];
    if (!s || !s.video || typeof RTCPeerConnection === 'undefined') return;
    if (s.retry) { clearTimeout(s.retry); s.retry = null; }
    if (s.pc) { try { s.pc.close(); } catch (_) {} s.pc = null; }
    setWebRTCState(s, false, 'CONNECTING');

    const pc = new RTCPeerConnection({ iceServers: [] });
    s.pc = pc;
    pc.addTransceiver('video', { direction: 'recvonly' });

    pc.ontrack = (event) => {
        const stream = (event.streams && event.streams[0]) || new MediaStream([event.track]);
        s.video.srcObject = stream;
        const p = s.video.play();
        if (p && p.catch) p.catch(() => {});
        s.tries = 0;
        setWebRTCState(s, true, 'LIVE');
    };
    pc.onconnectionstatechange = () => {
        const state = pc.connectionState;
        if (s.pc !== pc) return;
        if (state === 'connected') { s.tries = 0; setWebRTCState(s, true, 'LIVE'); }
        if (state === 'failed' || state === 'disconnected' || state === 'closed') {
            setWebRTCState(s, false, state.toUpperCase());
            try { pc.close(); } catch (_) {}
            s.pc = null;
            scheduleWebRTCRetry(kind);
        }
    };

    try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        await waitIceGathering(pc, 1500);
        const response = await fetch(WEBRTC_CFG.signalUrl + s.endpoint, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type })
        });
        if (!response.ok) throw new Error('WebRTC signaling HTTP ' + response.status);
        await pc.setRemoteDescription(await response.json());
    } catch (_) {
        setWebRTCState(s, false, 'RETRYING');
        try { pc.close(); } catch (_) {}
        if (s.pc === pc) s.pc = null;
        scheduleWebRTCRetry(kind);
    }
}

function stopWebRTCStream(kind) {
    const s = webrtcStreams[kind];
    if (!s) return;
    if (s.retry) clearTimeout(s.retry);
    s.retry = null;
    if (s.pc) { try { s.pc.close(); } catch (_) {} }
    s.pc = null;
    if (s.video) s.video.srcObject = null;
    setWebRTCState(s, false, 'OFFLINE');
}

function startRealSenseWebRTC() {
    startWebRTCStream('rgb');
    startWebRTCStream('depth');
}

window.addEventListener('beforeunload', () => {
    stopWebRTCStream('rgb');
    stopWebRTCStream('depth');
});

// =====================================================================
//  Wheel graphs
// =====================================================================
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

    let lo = Infinity, hi = -Infinity;
    const scan = p => { if (p.t >= t0 - 1) { lo = Math.min(lo, p.v); hi = Math.max(hi, p.v); } };
    A.forEach(scan); C.forEach(scan);
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    if (hi - lo < 0.2) { const mid = (hi + lo) / 2; lo = mid - 0.1; hi = mid + 0.1; }
    const pad = (hi - lo) * 0.1; lo -= pad; hi += pad;

    const X = t => L + ((t - t0) / win) * pw;
    const Y = v => T + (1 - (v - lo) / (hi - lo)) * ph;

    ctx.font = '400 10px Roboto, sans-serif';
    ctx.lineWidth = 1;
    ctx.strokeStyle = theme['--border']; ctx.fillStyle = theme['--muted'];
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (let i = 0; i <= 4; i++) {
        const v = lo + (hi - lo) * (i / 4), y = Math.round(Y(v)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
        ctx.fillText(v.toFixed(2), L - 6, y);
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let s = 0; s <= win; s += 10) {
        const x = Math.round(X(t0 + s)) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
        ctx.fillText(s === win ? 'now' : '-' + (win - s) + 's', x, T + ph + 4);
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
//  SLAM map canvas & interaction (3D-like tilt projection & Z=0 planar)
// =====================================================================
const mapWrap = document.getElementById('map-wrapper');
const mapCanvas = document.getElementById('map-canvas');
const view = { vx: 0, vy: 0, s: 30, tilt: 0.48 }; // 3D-like tilt angle matching Foxglove
let follow = true, viewFitted = false;
let clickMode = 'single';
let singleTarget = null;
let waypointsData = [];
let activeGoal = null, panState = null;

const mapSize = () => ({ W: mapCanvas.clientWidth, H: mapCanvas.clientHeight });
function w2s(x, y) {
    const { W, H } = mapSize();
    const dx = y - view.vy;
    const dy = x - view.vx;
    // 3D projection transformation with tilt
    const sx = W / 2 - dx * view.s;
    const sy = H / 2 - dy * view.s * Math.cos(view.tilt) + (dx * view.s * Math.sin(view.tilt) * 0.3);
    return [sx, sy];
}
function s2w(sx, sy) {
    const { W, H } = mapSize();
    const dx = (W / 2 - sx) / view.s;
    const dy = (H / 2 - sy) / (view.s * Math.cos(view.tilt));
    return { x: view.vx + dy, y: view.vy + dx, z: 0 };
}

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
    follow = true; document.getElementById('follow-btn').classList.add('on');
    viewFitted = true; needsDraw = true;
}
function toggleFollow() {
    follow = !follow;
    document.getElementById('follow-btn').classList.toggle('on', follow);
    needsDraw = true;
}

function toggleSidebar() {
    document.body.classList.toggle('sidebar-collapsed');
    const button = document.getElementById('sidebar-toggle-btn');
    const collapsed = document.body.classList.contains('sidebar-collapsed');
    if (button) {
        button.setAttribute('aria-label', collapsed ? 'Show navigation' : 'Hide navigation');
        button.title = collapsed ? 'Show navigation' : 'Hide navigation';
    }
    requestAnimationFrame(() => {
        mapImageDirty = true;
        needsDraw = true;
        if (typeof mvDraw === 'function') mvDraw();
    });
}

function setupPanelUtilities() {
    const panels = Array.from(document.querySelectorAll('.panel'));
    panels.forEach((panel, index) => {
        const header = panel.querySelector(':scope > .panel-hd');
        if (!header) return;
        panel.dataset.panelId = panel.dataset.panelId || 'panel-' + index;

        let tools = header.querySelector(':scope > .panel-tools');
        if (!tools) {
            tools = document.createElement('div');
            tools.className = 'panel-tools';
            header.appendChild(tools);
        }
    });
}

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
    const OCC_THRESHOLD = 65;
    for (let j = 0; j < m.h; j++) {
        for (let i = 0; i < m.w; i++) {
            const v = m.data[j * m.w + i], k = (j * m.w + i) * 4;
            if (v < 0) { d[k + 3] = 0; }
            else {
                const col = v >= OCC_THRESHOLD ? occ : free;
                d[k] = col[0]; d[k + 1] = col[1]; d[k + 2] = col[2];
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

    function drawGrid(m, offCanvas) {
        if (!m) return;
        const c = Math.cos(m.yaw), s = Math.sin(m.yaw), k = m.res * view.s;
        ctx.save();
        ctx.imageSmoothingEnabled = false;
        ctx.transform(-k * s, -k * c, -k * c, k * s,
            W / 2 - (m.oy - view.vy) * view.s, H / 2 - (m.ox - view.vx) * view.s);
        ctx.drawImage(offCanvas, 0, 0);
        ctx.restore();
    }
    if (layerVis.map && latestMap) drawGrid(latestMap, mapCanvasOff);
    if (layerVis.globalCostmap && latestGlobalCostmap) drawGrid(latestGlobalCostmap, globalCostmapCanvasOff);
    if (layerVis.localCostmap && latestLocalCostmap) drawGrid(latestLocalCostmap, localCostmapCanvasOff);

    const step = view.s >= 6 ? 1 : (view.s >= 1.5 ? 5 : 10);
    document.getElementById('map-grid-label').textContent = 'Grid ' + step + ' m';
    
    if (layerVis.plan && latestPlan && latestPlan.length > 0) {
        drawPathWithArrows(ctx, latestPlan, theme['--primary']);
        const finalPt = latestPlan[latestPlan.length - 1];
        drawArrow(ctx, finalPt.x, finalPt.y, finalPt.yaw, theme['--danger'], 'GOAL (Z=0)', 28);
    }

    const WPCOL = '#E8A317', TCOL = '#0077ff';
    if (waypointsData.length > 1) {
        ctx.strokeStyle = WPCOL; ctx.lineWidth = 1.2; ctx.setLineDash([4, 4]); ctx.beginPath();
        waypointsData.forEach((wp, i) => { const p = w2s(wp.x, wp.y); i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
        ctx.stroke(); ctx.setLineDash([]);
    }
    waypointsData.forEach((wp, i) => drawArrow(ctx, wp.x, wp.y, wp.yaw * Math.PI / 180, WPCOL, String(i + 1), 26));
    if (singleTarget) drawArrow(ctx, singleTarget.x, singleTarget.y, singleTarget.yaw * Math.PI / 180, TCOL, 'T', 26);
    if (activeGoal) drawArrow(ctx, activeGoal.x, activeGoal.y, activeGoal.yaw * Math.PI / 180, TCOL, 'T', 26);

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
        ctx.restore();
    }
}

function evtPos(evt) { const r = mapCanvas.getBoundingClientRect(); return [evt.clientX - r.left, evt.clientY - r.top]; }

let poseActionPopup = null;

function openPoseActionPopup(pose, evt, waypointIndex = -1) {
    const popup = document.getElementById('map-pose-popup');
    const poseText = document.getElementById('map-pose-popup-pose');
    const singleActions = document.getElementById('map-pose-single-actions');
    const waypointActions = document.getElementById('map-pose-waypoint-actions');
    if (!popup || !pose) return;

    poseActionPopup = { x: pose.x, y: pose.y, z: 0, yaw: pose.yaw || 0, waypointIndex };
    const isWaypoint = waypointIndex >= 0;
    if (singleActions) singleActions.hidden = isWaypoint;
    if (waypointActions) waypointActions.hidden = !isWaypoint;
    poseText.textContent =
        'X ' + pose.x.toFixed(2) +
        ' | Y ' + pose.y.toFixed(2) +
        ' | Z 0.00 | Yaw ' + (pose.yaw || 0).toFixed(1) + '°';

    const margin = 10, pw = 230, ph = 175;
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
    needsDraw = true;
    notify('INFO', 'Waypoint ' + waypointsData.length + ' added at Z=0 (' + p.x.toFixed(2) + ', ' + p.y.toFixed(2) + ').');
}

function popupCancelWaypoint() {
    if (!poseActionPopup || poseActionPopup.waypointIndex < 0) return;
    const index = poseActionPopup.waypointIndex;
    const removed = waypointsData[index];
    closePoseActionPopup();
    if (!removed) return;
    waypointsData.splice(index, 1);
    updateWaypointsUI();
    needsDraw = true;
    notify('INFO', 'Waypoint cancelled.');
}

document.addEventListener('pointerdown', (evt) => {
    const popup = document.getElementById('map-pose-popup');
    if (popup && popup.classList.contains('show') && !popup.contains(evt.target) && !mapWrap.contains(evt.target)) {
        closePoseActionPopup();
    }
});

mapWrap.addEventListener('contextmenu', e => e.preventDefault());
mapWrap.addEventListener('pointerdown', (evt) => {
    if (evt.target.closest('#map-pose-popup') || evt.target.closest('#map-legend')) return;
    mapWrap.setPointerCapture(evt.pointerId);
    const [sx, sy] = evtPos(evt);
    if (evt.button === 1 || evt.button === 2 || evt.shiftKey) {
        panState = { sx, sy, vx: view.vx, vy: view.vy };
        follow = false; document.getElementById('follow-btn').classList.remove('on');
        return;
    }
    const w = s2w(sx, sy);
    activeGoal = { x: w.x, y: w.y, z: 0, yaw: 0 };
    needsDraw = true;
});
mapWrap.addEventListener('pointermove', (evt) => {
    if (evt.target.closest('#map-pose-popup') || evt.target.closest('#map-legend')) return;
    const [sx, sy] = evtPos(evt);
    const w = s2w(sx, sy);
    document.getElementById('map-cursor').textContent = 'x ' + w.x.toFixed(2) + '  y ' + w.y.toFixed(2) + '  z 0.00 m';
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
    if (evt.target.closest('#map-pose-popup') || evt.target.closest('#map-legend')) return;
    if (panState) { panState = null; return; }
    if (!activeGoal) return;
    const g = activeGoal; activeGoal = null;

    let waypointIndex = -1;
    if (clickMode === 'single') {
        singleTarget = g;
        updateUIInputs(g.x, g.y, g.yaw);
    } else {
        waypointsData.push(g);
        waypointIndex = waypointsData.length - 1;
        updateWaypointsUI();
    }
    needsDraw = true;
    openPoseActionPopup(g, evt, waypointIndex);
};
mapWrap.addEventListener('pointerup', endPointer);
mapWrap.addEventListener('pointercancel', () => { panState = null; activeGoal = null; needsDraw = true; });
mapWrap.addEventListener('wheel', (evt) => {
    evt.preventDefault();
    const [sx, sy] = evtPos(evt), { W, H } = mapSize();
    const w = s2w(sx, sy);
    view.s = Math.max(1, Math.min(500, view.s * (evt.deltaY < 0 ? 1.12 : 1 / 1.12)));
    view.vy = w.y + (sx - W / 2) / view.s;
    view.vx = w.x + (sy - H / 2) / view.s;
    needsDraw = true;
}, { passive: false });
new ResizeObserver(() => { needsDraw = true; }).observe(mapWrap);

// =====================================================================
//  Animation frame & status panel
// =====================================================================
let lastStatus = 0;
function frame(ts) {
    if (poseDirty) refreshRobotPose();
    if (mapImageDirty && latestMap) {
        rebuildMapImage(); mapImageDirty = false;
        if (!viewFitted) fitMapView();
        needsDraw = true;
    }
    if (localCostmapDirty && latestLocalCostmap) { rebuildLocalCostmapImage(); localCostmapDirty = false; needsDraw = true; }
    if (globalCostmapDirty && latestGlobalCostmap) { rebuildGlobalCostmapImage(); globalCostmapDirty = false; needsDraw = true; }
    if (follow || needsDraw) { drawMap(); needsDraw = false; }
    if (ts - lastStatus > 200) { lastStatus = ts; updateStatus(); }
    requestAnimationFrame(frame);
}

function calcDistanceRemaining() {
    if (!latestPlan || latestPlan.length < 2) return null;
    let start = 0, lead = 0;
    if (robot) {
        let best = Infinity;
        for (let i = 0; i < latestPlan.length; i++) {
            const d = Math.hypot(latestPlan[i].x - robot.x, latestPlan[i].y - robot.y);
            if (d < best) { best = d; start = i; }
        }
        lead = best;
    }
    let dist = lead;
    for (let i = start; i < latestPlan.length - 1; i++) {
        dist += Math.hypot(latestPlan[i + 1].x - latestPlan[i].x, latestPlan[i + 1].y - latestPlan[i].y);
    }
    return dist;
}

function updateStatus() {
    const now = performance.now();
    if (cmdVelAt && now - CFG.cmdVelStaleMs > cmdVelAt && (currentCmdVel.linear || currentCmdVel.angular)) {
        currentCmdVel = { linear: 0, angular: 0 }; needsDraw = true;
    }

    document.getElementById('stat-speed').textContent = (odom ? odom.speed : 0).toFixed(2);
    distanceRemaining = calcDistanceRemaining();
    document.getElementById('stat-distance').textContent =
        (distanceRemaining !== null && (navStatus === 'Navigating' || navStatus === 'Planning')) ? distanceRemaining.toFixed(2) : '—';

    if (navStatus === 'Navigating') {
        let reached = false;
        if (robot && missionGoal) reached = Math.hypot(robot.x - missionGoal.x, robot.y - missionGoal.y) < CFG.reachTolM;
        else if (distanceRemaining !== null) reached = distanceRemaining < 0.25;
        if (reached) {
            navStatus = 'Reached';
            missionGoal = null;
            latestPlan = [];
            ignorePlansUntil = Date.now() + 1500;
            clearNavLoading();
            needsDraw = true;
            notify('INFO', 'Navigation destination reached.');
        }
    }

    const statusEl = document.getElementById('stat-nav-status');
    statusEl.textContent = navStatus;
    statusEl.style.color = { Reached: 'var(--success)', Aborted: 'var(--danger)', Navigating: 'var(--primary)', Planning: 'var(--warning)' }[navStatus] || 'var(--muted)';

    if (robot) {
        let deg = robot.yaw * 180 / Math.PI;
        deg = ((deg + 180) % 360 + 360) % 360 - 180;
        document.getElementById('stat-heading').textContent = deg.toFixed(1);
        document.getElementById('hdg-arrow').style.transform = 'rotate(' + (-deg) + 'deg)';
        document.getElementById('stat-location').innerHTML =
            'X ' + robot.x.toFixed(2) + ' &nbsp; Y ' + robot.y.toFixed(2) + ' (Z=0)';
    }
}

function setClickMode(mode) {
    clickMode = mode;
    document.getElementById('single-actions').style.display = mode === 'single' ? 'flex' : 'none';
    document.getElementById('waypoint-actions').style.display = mode === 'waypoint' ? 'flex' : 'none';
    document.getElementById('waypoints-container').style.display = mode === 'waypoint' ? 'block' : 'none';
    document.querySelectorAll('input[name="click_mode"]').forEach(i => i.checked = (i.value === mode));
    needsDraw = true;
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
    singleTarget = { x, y, z: 0, yaw }; needsDraw = true;
}

function updateWaypointsUI() {
    const container = document.getElementById('waypoints-container');
    container.innerHTML = waypointsData.length === 0 ? '<em>No waypoints added.</em>' : '';
    waypointsData.forEach((wp, i) => {
        const div = document.createElement('div');
        div.innerHTML = '<strong>WP ' + (i + 1) + ':</strong> X:' + wp.x.toFixed(2) + ' Y:' + wp.y.toFixed(2) + ' Z:0.0 Yaw:' + wp.yaw.toFixed(0) + '&deg;';
        container.appendChild(div);
    });
}

function clearMarkers() {
    closePoseActionPopup();
    singleTarget = null; waypointsData = []; activeGoal = null;
    if (navStatus !== 'Planning' && navStatus !== 'Navigating') { latestPlan = []; navStatus = 'Idle'; }
    updateWaypointsUI(); needsDraw = true;
}

async function withBusy(btn, fn) {
    if (btn) { if (btn.classList.contains('loading')) return; btn.classList.add('loading'); btn.disabled = true; }
    try { return await fn(); }
    finally { if (btn) { btn.classList.remove('loading'); btn.disabled = false; } }
}
function run(btn, fn) { return withBusy(btn, fn); }
function runNavButton(btn, fn) { return fn(); }

window.run = run;
window.runNavButton = runNavButton;

function readTarget() {
    const x = parseFloat(document.getElementById('target-x').value);
    const y = parseFloat(document.getElementById('target-y').value);
    const yaw_deg = parseFloat(document.getElementById('target-yaw').value) || 0;
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
    return { x, y, z: 0, yaw_deg };
}

async function sendGoalPose() {
    const t = readTarget();
    if (!t) { notify('WARNING', 'Enter valid X and Y target values.'); return; }
    const btn = document.getElementById('nav-pose-btn');
    setNavLoading(btn);
    startNavWatchdog();
    navStatus = 'Planning';
    missionGoal = { x: t.x, y: t.y, z: 0 };
    latestPlan = [];
    try {
        await postJSON('/navigate_to_pose', { x: t.x, y: t.y, yaw_deg: t.yaw_deg });
        notify('INFO', 'Goal sent to Z=0 (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) {
        clearNavLoading();
        navStatus = 'Idle'; missionGoal = null;
        notify('ERROR', 'Navigation failed: ' + e.message);
    }
}

async function sendInitialPose() {
    const t = readTarget();
    if (!t) { notify('WARNING', 'Enter valid X and Y values.'); return; }
    try {
        await postJSON('/set_initial_pose', { x: t.x, y: t.y, yaw_deg: t.yaw_deg });
        notify('INFO', 'Initial pose set to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) { notify('ERROR', 'Set initial pose failed: ' + e.message); }
}

async function sendWaypoints() {
    if (waypointsData.length === 0) { notify('WARNING', 'No waypoints added.'); return; }
    const btn = document.getElementById('follow-wp-btn');
    setNavLoading(btn);
    startNavWatchdog();
    navStatus = 'Planning';
    const last = waypointsData[waypointsData.length - 1];
    missionGoal = { x: last.x, y: last.y, z: 0 };
    latestPlan = [];
    const waypoints = waypointsData.map(wp => ({ x: wp.x, y: wp.y, yaw_deg: wp.yaw }));
    try {
        await postJSON('/follow_waypoints', { waypoints });
        notify('INFO', 'Running ' + waypoints.length + ' waypoint(s).');
    } catch (e) {
        clearNavLoading();
        navStatus = 'Idle'; missionGoal = null;
        notify('ERROR', 'Waypoint mission failed: ' + e.message);
    }
}

async function sendAbort() {
    clearNavLoading();
    navStatus = 'Aborted';
    missionGoal = null;
    latestPlan = [];
    ignorePlansUntil = Date.now() + 1500;
    needsDraw = true;
    try {
        await postJSON('/abort');
        notify('WARNING', 'Mission abort requested.');
    } catch (e) { notify('ERROR', 'Abort request failed: ' + e.message); }
}

function downloadBlob(blob, filename) {
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = filename;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 10000);
}

async function saveCurrentMap() {
    const input = document.getElementById('map-name-input');
    const mapName = ((input && input.value) || 'small_warehouse').trim().replace(/[^\w.\-]+/g, '_') || 'map';
    const url = REST_API_BASE + '/map/save?name=' + encodeURIComponent(mapName);
    notify('INFO', 'Saving map "' + mapName + '" on the rover…');
    try {
        const res = await fetch(url);
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const blob = await res.blob();
        downloadBlob(blob, mapName + '.zip');
        notify('INFO', 'Map "' + mapName + '" saved and downloaded.');
    } catch (_) {
        const f = document.createElement('iframe');
        f.style.display = 'none'; f.src = url;
        document.body.appendChild(f);
        setTimeout(() => f.remove(), 30000);
    }
}

async function loadMapNames() {
    const select = document.getElementById('map-name-input');
    if (!select) return;
    try {
        const response = await fetch(REST_API_BASE + '/maps', { cache: 'no-store' });
        if (!response.ok) return;
        const data = await response.json();
        const names = Array.isArray(data) ? data : (data.maps || []);
        if (!names.length) return;
        select.replaceChildren(...names.map((name) => {
            const option = document.createElement('option');
            option.value = name; option.textContent = name;
            return option;
        }));
    } catch (_) {}
}

let currentMode = null, pending = null, modeSelectTouched = false;
const modeLoaders = [];

function initMapPanelExtras() {
    ['.p-map', '.ctrl-panel'].forEach(sel => {
        const host = document.querySelector(sel);
        if (!host) return;
        const l = document.createElement('div');
        l.className = 'mode-loader';
        l.innerHTML = '<div class="mode-loader-card"><div class="ml-spinner"></div>' +
            '<div class="ml-title">Switching mode</div>' +
            '<div class="ml-sub">Deploying</div></div>';
        host.appendChild(l);
        modeLoaders.push(l);
    });
}
function showModeLoaders(on) { modeLoaders.forEach(l => l.classList.toggle('show', !!on)); }

async function fetchMode() {
    try {
        const res = await fetch(REST_API_BASE + '/system/mode');
        if (res.ok) { const data = await res.json(); return data.mode || null; }
    } catch (_) {}
    return null;
}

function renderMode() {
    const chip = document.getElementById('mode-chip'), txt = document.getElementById('stat-mode');
    const btn = document.getElementById('deploy-btn');
    chip.classList.remove('known', 'pending');
    if (pending) {
        chip.classList.add('pending');
        txt.textContent = 'Switching…';
    } else if (currentMode) {
        chip.classList.add('known');
        txt.textContent = modeLabel(currentMode);
    } else {
        txt.textContent = 'Unknown';
    }
    btn.disabled = !!pending;
    btn.classList.toggle('loading', !!pending);
}

function setCurrentMode(m) {
    currentMode = m;
    renderMode();
}

async function applySystemMode() {
    if (pending) return;
    const mode = document.getElementById('sys-mode-select').value;
    const mapName = (document.getElementById('map-name-input').value || 'small_warehouse').trim();
    pending = { target: mode };
    renderMode(); showModeLoaders(true);
    try {
        await postJSON('/system/mode', { mode, map_name: mapName });
        pending = null;
        setCurrentMode(mode);
        notify('INFO', 'Switched mode successfully.');
    } catch (e) {
        pending = null; renderMode();
        notify('ERROR', 'Mode switch failed: ' + e.message);
    } finally {
        showModeLoaders(false);
    }
}

// =====================================================================
//  Joystick (Fixed publishing /joy topic)
// =====================================================================
const joyPad = document.getElementById('joy-pad'), joyKnob = document.getElementById('joy-knob');
const joy = { x: 0, y: 0, active: false, timer: null };
let joyEnabled = false;

function publishJoyRaw(turn, fwd) {
    if (!joyTopic || !rosConnected) return false;
    joyTopic.publish(new ROSLIB.Message({
        header: { stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 }, frame_id: 'joy' },
        axes: [turn, fwd],
        buttons: []
    }));
    return true;
}
function publishJoy() {
    if (!joyEnabled) return;
    publishJoyRaw(-joy.x, -joy.y);
}

window.toggleJoyEnable = function () {
    if (!joyEnabled && !rosConnected) {
        notify('WARNING', 'ROS bridge is offline - joystick commands cannot be published.');
    }
    joyEnabled = !joyEnabled;
    const btn = document.getElementById('joy-enable-btn');
    btn.textContent = joyEnabled ? 'Disable' : 'Enable';
    btn.classList.toggle('btn-primary', joyEnabled);
    btn.classList.toggle('btn-secondary', !joyEnabled);
    if (!joyEnabled) {
        joyEnd();
        publishJoyRaw(0, 0);
    }
    notify('INFO', joyEnabled ? 'Joystick enabled.' : 'Joystick disabled.');
};

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
    joy.x = dx / R; joy.y = dy / R; joyShow();
}

joyPad.addEventListener('pointerdown', (e) => {
    joyPad.setPointerCapture(e.pointerId); joyPad.classList.add('active');
    joy.active = true; joyMove(e);
    clearInterval(joy.timer);
    joy.timer = setInterval(publishJoy, 1000 / CFG.joyRateHz);
    publishJoy();
});
joyPad.addEventListener('pointermove', (e) => { if (joy.active) joyMove(e); });
function joyEnd() {
    if (!joy.active) return;
    joy.active = false; joyPad.classList.remove('active');
    clearInterval(joy.timer); joy.timer = null;
    joy.x = 0; joy.y = 0; joyShow();
    publishJoy();
}
joyPad.addEventListener('pointerup', joyEnd);
joyPad.addEventListener('pointercancel', joyEnd);

// =====================================================================
//  MAP VIEWER
// =====================================================================
const mv = { ready: false, open: false, root: null, canvas: null, ctx: null, body: null, img: null, gray: null, w: 0, h: 0, res: 0.05, s: 1, tx: 0, ty: 0, measure: false, pts: [] };
const $mv = (id) => document.getElementById(id);

function openMapViewer() {
    mv.root = $mv('map-viewer');
    mv.canvas = $mv('mv-canvas');
    mv.ctx = mv.canvas.getContext('2d');
    mv.body = $mv('mv-body');
    mv.root.hidden = false;
    mv.open = true;
}
function closeMapViewer() { if (mv.root) mv.root.hidden = true; mv.open = false; }
window.openMapViewer = openMapViewer; window.closeMapViewer = closeMapViewer;

// =====================================================================
//  HELIO VOICE ASSISTANT (Dual Wake Words & Responsive Face & Feelings)
// =====================================================================
const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
const modal = document.getElementById('voice-modal');
const robotFace = document.getElementById('robot-face');
const helioDots = document.getElementById('helio-dots');
const historyListEl = document.getElementById('history-list');

let recognition = null;
let wakeRecRobot = null, wakeRecJojo = null;
let helioOpen = false;
let agentState = 'idle';
let selectedLanguage = 'en';
let commandSeq = 0;

const MicLevels = {
    stream: null, analyser: null, data: null, active: false,
    async start() {
        if (this.active) return;
        try {
            const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
            this.stream = await navigator.mediaDevices.getUserMedia({ audio: true });
            const src = audioCtx.createMediaStreamSource(this.stream);
            this.analyser = audioCtx.createAnalyser();
            this.analyser.fftSize = 256;
            src.connect(this.analyser);
            this.data = new Uint8Array(this.analyser.frequencyBinCount);
            this.active = true;
        } catch (_) {}
    },
    level() {
        if (!this.active || !this.analyser) return 0;
        this.analyser.getByteFrequencyData(this.data);
        let sum = 0;
        for (let i = 0; i < this.data.length; i++) sum += this.data[i];
        return Math.min(1, (sum / this.data.length) / 85);
    },
    stop() {
        this.active = false;
        if (this.stream) { this.stream.getTracks().forEach(t => t.stop()); this.stream = null; }
    }
};

// Enhanced Responsive Face & Feelings Engine
const EyeMotion = {
    raf: null,
    gazeX: 0, gazeY: 0, targetX: 0, targetY: 0,
    start() {
        this.stop();
        this.loop();
    },
    stop() {
        if (this.raf) cancelAnimationFrame(this.raf);
        this.raf = null;
        if (robotFace) robotFace.querySelectorAll('.eye').forEach(e => { e.style.transform = ''; });
    },
    loop() {
        if (!helioOpen || agentState !== 'listening') { this.raf = null; return; }
        this.targetX = (Math.random() * 2 - 1) * 32;
        this.targetY = (Math.random() * 2 - 1) * 16;
        this.gazeX += (this.targetX - this.gazeX) * 0.05;
        this.gazeY += (this.targetY - this.gazeY) * 0.05;
        const level = MicLevels.level();
        if (robotFace) {
            const scale = 1 + level * 0.28;
            robotFace.querySelectorAll('.eye').forEach(eye => {
                eye.style.transform = `translate(${this.gazeX.toFixed(1)}px, ${this.gazeY.toFixed(1)}px) scaleY(${scale.toFixed(3)})`;
            });
        }
        this.raf = requestAnimationFrame(() => this.loop());
    }
};

function setFace(state, feeling = 'neutral') {
    if (!robotFace) return;
    robotFace.className = 'robot-face ' + state + ' ' + feeling;
    if (helioDots) helioDots.classList.toggle('show', state === 'thinking');
    if (state === 'listening') { MicLevels.start(); EyeMotion.start(); }
    else { MicLevels.stop(); EyeMotion.stop(); }
}

function appendHistory(sender, text) {
    if (!historyListEl || !text) return;
    const item = document.createElement('div');
    item.className = 'history-item ' + sender;
    item.innerHTML = `<div class="history-label">${sender === 'you' ? 'you:' : 'agent:'}</div><div>${text}</div>`;
    historyListEl.appendChild(item);
    historyListEl.scrollTop = historyListEl.scrollHeight;
}

function openVoiceModal(lang = 'en') {
    selectedLanguage = lang;
    helioOpen = true;
    if (modal) { modal.classList.add('active'); modal.setAttribute('aria-hidden', 'false'); }
    setFace('waking', 'happy');
    speakText(lang === 'hi' ? 'नमस्ते, बोलिए।' : 'Hello, listening.', { pitch: 1.1, rate: 0.8 }).then(() => {
        if (!helioOpen) return;
        setFace('listening', 'neutral');
        if (recognition) {
            recognition.lang = lang === 'hi' ? 'hi-IN' : 'en-US';
            try { recognition.start(); } catch (_) {}
        }
    });
}
window.openVoiceModal = openVoiceModal;

function closeVoiceModal() {
    helioOpen = false;
    agentState = 'idle';
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    if (modal) { modal.classList.remove('active'); modal.setAttribute('aria-hidden', 'true'); }
    if (recognition) { try { recognition.abort(); } catch (_) {} }
    MicLevels.stop();
    EyeMotion.stop();
}
window.closeVoiceModal = closeVoiceModal;

function speakText(text, opts = {}) {
    if (!('speechSynthesis' in window) || !text) return Promise.resolve();
    return new Promise((resolve) => {
        const u = new SpeechSynthesisUtterance(text);
        u.lang = selectedLanguage === 'hi' ? 'hi-IN' : 'en-US';
        u.pitch = opts.pitch || 1.0;
        u.rate = opts.rate || 1.0;
        u.onend = resolve;
        u.onerror = resolve;
        window.speechSynthesis.speak(u);
    });
}

function bindRecognition() {
    if (!SpeechRecognitionImpl) return;
    recognition = new SpeechRecognitionImpl();
    recognition.continuous = false;
    recognition.interimResults = false;

    recognition.onresult = async (event) => {
        const text = event.results[0][0].transcript.trim();
        appendHistory('you', text);
        setFace('thinking', 'neutral');
        
        // Understand feeling & respond with empathy/expressions
        let feeling = 'neutral';
        if (/happy|good|awesome|great|नमस्ते/i.test(text)) feeling = 'happy';
        else if (/bad|problem|error|help|issue/i.test(text)) feeling = 'sad';

        const reply = await localAssistant(text);
        setFace('speaking', feeling);
        appendHistory('agent', reply.text);
        await speakText(reply.text);
        if (helioOpen) {
            setFace('listening', 'neutral');
            try { recognition.start(); } catch (_) {}
        }
    };
    recognition.onerror = () => {
        if (helioOpen) { try { recognition.start(); } catch (_) {} }
    };
}

// Dual Wake Words ("Hey Robot" & "Hey JoJo") with Improved Detection
function bindWakeWords() {
    if (!SpeechRecognitionImpl) return;

    // Robot (English)
    try {
        wakeRecRobot = new SpeechRecognitionImpl();
        wakeRecRobot.continuous = true;
        wakeRecRobot.interimResults = true;
        wakeRecRobot.lang = 'en-US';
        wakeRecRobot.onresult = (e) => {
            const heard = e.results[e.results.length - 1][0].transcript.toLowerCase();
            if (/\bhey\s+robot\b/.test(heard)) {
                try { wakeRecRobot.abort(); } catch (_) {}
                openVoiceModal('en');
            }
        };
        wakeRecRobot.onend = () => { setTimeout(() => { try { wakeRecRobot.start(); } catch (_) {} }, 500); };
        wakeRecRobot.start();
    } catch (_) {}

    // JoJo (Hindi)
    try {
        wakeRecJojo = new SpeechRecognitionImpl();
        wakeRecJojo.continuous = true;
        wakeRecJojo.interimResults = true;
        wakeRecJojo.lang = 'hi-IN';
        wakeRecJojo.onresult = (e) => {
            const heard = e.results[e.results.length - 1][0].transcript.toLowerCase();
            if (/\bhey\s+jojo\b|\bहैलो जोजो\b/.test(heard)) {
                try { wakeRecJojo.abort(); } catch (_) {}
                openVoiceModal('hi');
            }
        };
        wakeRecJojo.onend = () => { setTimeout(() => { try { wakeRecJojo.start(); } catch (_) {} }, 500); };
        wakeRecJojo.start();
    } catch (_) {}
}

function numbersIn(s) {
    const t = s.replace(/\bminus\b/g, '-').replace(/\bpoint\b/g, '.');
    return (t.match(/-?\d+(?:\.\d+)?/g) || []).map(Number);
}

async function localAssistant(raw) {
    const t = ' ' + raw.toLowerCase().replace(/[^\w\s.\-,]/g, ' ').replace(/\s+/g, ' ').trim() + ' ';
    const has = (re) => re.test(t);
    const nums = numbersIn(t);

    if (has(/\b(abort|cancel|halt|रद्द)\b/)) {
        await sendAbort();
        return { text: selectedLanguage === 'hi' ? 'मिशन रद्द कर दिया गया है।' : 'Mission aborted.' };
    }
    if (has(/\bwaypoint|waypoints|वेपॉइंट\b/) && has(/\b(run|execute|follow|चलाओ)\b/)) {
        await sendWaypoints();
        return { text: selectedLanguage === 'hi' ? 'वेपॉइंट्स निष्पादित किए जा रहे हैं।' : 'Running waypoints.' };
    }
    if (has(/\b(navigate|go|चलो)\b/) && nums.length >= 2) {
        updateUIInputs(nums[0], nums[1], nums[2] || 0);
        singleTarget = { x: nums[0], y: nums[1], z: 0, yaw: nums[2] || 0 };
        needsDraw = true;
        await sendGoalPose();
        return { text: selectedLanguage === 'hi' ? `x ${nums[0]}, y ${nums[1]} पर नेविगेट कर रहे हैं।` : `Navigating to x ${nums[0]}, y ${nums[1]}.` };
    }
    if (has(/\bsave map|मैप सेव करो\b/)) {
        await saveCurrentMap();
        return { text: selectedLanguage === 'hi' ? 'मैप सहेजा जा रहा है।' : 'Saving map.' };
    }
    return { text: selectedLanguage === 'hi' ? 'समझ नहीं आया। कृपया दोबारा कहें।' : "I didn't quite catch that. Try asking to navigate or run waypoints." };
}

window.toggleHistory = function () {
    const el = document.getElementById('modal-history-column'); if (el) el.classList.toggle('show');
};
window.closeHistory = function () {
    const el = document.getElementById('modal-history-column'); if (el) el.classList.remove('show');
};
window.toggleWakeMenu = function (e) { if (e) e.stopPropagation(); document.getElementById('wake-menu').hidden ^= true; };

// =====================================================================
//  Initialization
// =====================================================================
(function init() {
    let saved = null;
    try { saved = localStorage.getItem('milusions-theme'); } catch (_) {}
    applyTheme(saved !== 'dark');

    setupPanelUtilities();
    loadMapNames();
    updateWaypointsUI();
    setClickMode('single');
    initMapPanelExtras();
    renderMode();

    initROSBridge();
    startRealSenseWebRTC();

    fetchMode().then(m => { if (m && !pending) setCurrentMode(m); });

    bindRecognition();
    bindWakeWords();

    requestAnimationFrame(frame);
})();
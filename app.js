// =====================================================================
//  Milusions WareGV Suite - dashboard logic
//  UI (index.html / style.css) + live rover integration:
//    - rosbridge  (/map /odom /tf /cmd_vel /plan /wheel_states, publishes /joy)
//    - REST API   (navigation, initial pose, waypoints, abort, mode, save map)
//    - WebRTC     (RealSense RGB + Depth, camera_webrtc_streamer.py)
//    - Helio      (continuous voice assistant, wake word, local command engine)
//
//  Endpoints default to the host the page was loaded from, and can be
//  overridden with URL parameters, e.g.
//    index.html?rover=192.168.1.50
//    index.html?api=http://host:8000&ros=ws://host:9090&webrtc=http://host:8081
// =====================================================================

// =====================================================================
//  CONFIGURATION
// =====================================================================
const _qs = new URLSearchParams(window.location.search);
const ROVER_IP = _qs.get('rover') || window.ROVER_IP || window.location.hostname || '127.0.0.1';
const REST_API_BASE = (_qs.get('api') || window.REST_API_BASE || `http://${ROVER_IP}:8000`).replace(/\/+$/, '');
const ROSBRIDGE_WS_URL = _qs.get('ros') || window.ROSBRIDGE_WS_URL || `ws://${ROVER_IP}:9090`;
const WEBRTC_SIGNAL_URL = (_qs.get('webrtc') || window.WEBRTC_SIGNAL_URL || `http://${ROVER_IP}:8081`).replace(/\/+$/, '');
const HELIO_ENDPOINT = _qs.get('helio') || window.HELIO_ENDPOINT || '/helio/command';

const CFG = {
    chartWindowSec: 30,
    joyRateHz: 20,
    reachTolM: 0.30,          // "Reached" when the rover is this close to the mission goal
    planTimeoutMs: 20000,     // abort if Nav2 publishes no /plan after a goal
    cmdVelStaleMs: 600,       // hide the cmd_vel arrows when /cmd_vel goes quiet
    rosRetryMs: 3000
};

const TOPICS = {
    map: '/map',
    odom: '/odom',
    cmdVel: '/cmd_vel',
    plan: '/plan',
    wheel: '/wheel_states',
    joy: '/joy',
    tf: '/tf',
    tfStatic: '/tf_static'
};
const MAP_FRAME = 'map';

const MODE_LABELS = { auto_nav: 'Autonomous Navigation and Driving' };
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
//  Theme
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
    try { localStorage.setItem('milusions-theme', isLight ? 'light' : 'dark'); } catch (e) {}
}

// =====================================================================
//  State
// =====================================================================
let mapImageDirty = false, needsDraw = true;
let latestMap = null;
let latestPlan = [];
let navStatus = 'Idle';            // Idle | Planning | Navigating | Reached | Aborted
let missionGoal = null;            // {x, y} final goal of the running mission
let ignorePlansUntil = 0;
let distanceRemaining = null;
const mapCanvasOff = document.createElement('canvas');
let odom = null;                   // {speed}
let odomPose = null;               // pose straight from /odom (odom frame)
let robot = null;                  // best pose estimate, in the map frame when TF allows
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

// If Nav2 never publishes a /plan after we send a goal, give up and abort.
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
        try { await postJSON('/abort'); } catch (e) {}
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
    if (typeof d === 'string') {                     // rosbridge may base64-encode byte arrays
        const bin = atob(d), a = new Int8Array(bin.length);
        for (let i = 0; i < bin.length; i++) a[i] = bin.charCodeAt(i);
        return a;
    }
    return d || [];
}

// =====================================================================
//  ROS 2 BRIDGE (rosbridge_websocket via roslib)
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

// ----- TF: keep a small child->parent table and resolve base_link in the map frame -----
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
    if (frame === MAP_FRAME) return { x: 0, y: 0, yaw: 0 };
    const chain = [];
    let f = frame, guard = 0;
    while (f !== MAP_FRAME && guard++ < 16) {
        const e = tfEdges.get(f);
        if (!e) return null;
        chain.push(e);
        f = e.parent;
    }
    if (f !== MAP_FRAME) return null;
    let x = 0, y = 0, yaw = 0;
    for (let i = chain.length - 1; i >= 0; i--) {     // compose map -> ... -> frame (planar)
        const e = chain[i], c = Math.cos(yaw), s = Math.sin(yaw);
        x += c * e.x - s * e.y;
        y += s * e.x + c * e.y;
        yaw += e.yaw;
    }
    return { x, y, yaw };
}

function refreshRobotPose() {
    poseDirty = false;
    const p = lookupInMap(baseFrame);
    if (p) robot = p;
    else if (odomPose) robot = odomPose;
    needsDraw = true;
}

// ----- wheel telemetry: accepts {id,a,c}, [{...}], or {fl:{a,c},...} -----
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
    // keep one point older than the window so a step line stays anchored
    while (s.a.length > 1 && s.a[1].t < cutoff) s.a.shift();
    while (s.c.length > 1 && s.c[1].t < cutoff) s.c.shift();
}
function onWheelStates(msg) {
    let data;
    try { data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data; } catch (e) { return; }
    const now = performance.now() / 1000;
    if (Array.isArray(data)) data.forEach(d => ingestWheel(d, now));
    else if (data && Array.isArray(data.wheels)) data.wheels.forEach(d => ingestWheel(d, now));
    else if (data && data.id) ingestWheel(data, now);
    else if (data && typeof data === 'object') {
        wheelIds.forEach(id => { if (data[id]) ingestWheel(Object.assign({ id }, data[id]), now); });
    }
}

function onMapMsg(msg) {
    const info = msg.info;
    if (!info || !info.width || !info.height) return;
    lastMapAt = Date.now();
    latestMap = {
        w: info.width,
        h: info.height,
        res: info.resolution,
        ox: info.origin.position.x,
        oy: info.origin.position.y,
        yaw: yawFromQuat(info.origin.orientation),
        data: toInt8Array(msg.data)
    };
    mapImageDirty = true;
    needsDraw = true;
}

function onOdomMsg(msg) {
    const pos = msg.pose.pose.position;
    const tw = msg.twist.twist.linear;
    if (msg.child_frame_id) baseFrame = stripSlash(msg.child_frame_id);
    odomPose = { x: pos.x, y: pos.y, yaw: yawFromQuat(msg.pose.pose.orientation) };
    odom = { speed: Math.hypot(tw.x || 0, tw.y || 0) };
    poseDirty = true;
}

function onPlanMsg(msg) {
    if (Date.now() < ignorePlansUntil) return;
    const poses = (msg && msg.poses) || [];
    latestPlan = poses.map(p => ({
        x: p.pose.position.x, y: p.pose.position.y, yaw: yawFromQuat(p.pose.orientation)
    }));
    if (latestPlan.length > 1) {
        // Nav2 answered: stop the spinner and the watchdog
        clearNavLoading();
        if (navStatus !== 'Navigating') {
            navStatus = 'Navigating';
            if (!missionGoal) {                       // a goal sent by another client
                const last = latestPlan[latestPlan.length - 1];
                missionGoal = { x: last.x, y: last.y };
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
        console.error('ROS bridge error:', err);
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
    } catch (e) {
        throw new Error('Cannot reach the rover API (' + REST_API_BASE + ')');
    }
    const raw = await response.text();
    let data = {};
    try { data = raw ? JSON.parse(raw) : {}; } catch (e) { data = { raw }; }
    if (!response.ok) {
        let detail = data && data.detail;
        if (detail && typeof detail !== 'string') detail = JSON.stringify(detail);
        throw new Error(detail || (data && data.message) || ('HTTP ' + response.status));
    }
    return data;
}

// =====================================================================
//  REALSENSE WEBRTC VIDEO
//  Two independent PeerConnections: RGB (/offer/color) and Depth (/offer/depth),
//  matching camera_webrtc_streamer.py. Media never travels through rosbridge.
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

    const pc = new RTCPeerConnection({ iceServers: [] });   // rover is on the LAN
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
    } catch (err) {
        if (s.tries < 2) console.warn('[WebRTC:' + kind + ']', err.message || err);
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
//  SLAM map canvas & interaction
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
        const p2 = w2s(points[i + 1].x, points[i + 1].y);
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
            ctx.moveTo(-5, -4); ctx.lineTo(4, 0); ctx.lineTo(-5, 4);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
            currentDist += spacingPx;
        }
        accumulatedDist += segLen;
    }
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

    if (latestMap) {
        const m = latestMap, c = Math.cos(m.yaw), s = Math.sin(m.yaw), k = m.res * view.s;
        ctx.save();
        ctx.imageSmoothingEnabled = false;
        ctx.transform(-k * s, -k * c, -k * c, k * s,
            W / 2 - (m.oy - view.vy) * view.s, H / 2 - (m.ox - view.vx) * view.s);
        ctx.drawImage(mapCanvasOff, 0, 0);
        ctx.restore();
    }

    const step = view.s >= 6 ? 1 : (view.s >= 1.5 ? 5 : 10);
    document.getElementById('map-grid-label').textContent = 'Grid ' + step + ' m';
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

    const o = w2s(0, 0), ax = w2s(1, 0), ay = w2s(0, 1);
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#e5484d'; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(ax[0], ax[1]); ctx.stroke();
    ctx.strokeStyle = '#30a46c'; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(ay[0], ay[1]); ctx.stroke();

    if (latestPlan && latestPlan.length > 0) {
        drawPathWithArrows(ctx, latestPlan, theme['--primary']);
        const finalPt = latestPlan[latestPlan.length - 1];
        drawArrow(ctx, finalPt.x, finalPt.y, finalPt.yaw, theme['--danger'], 'GOAL', 28);
    }

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
            const ex = arcR * Math.cos(endAngle);
            const ey = arcR * Math.sin(endAngle);

            ctx.save();
            ctx.translate(ex, ey);
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
    notify('INFO', 'Waypoint ' + waypointsData.length + ' added at (' + p.x.toFixed(2) + ', ' + p.y.toFixed(2) + ').');
}

document.addEventListener('pointerdown', (evt) => {
    const popup = document.getElementById('map-pose-popup');
    if (popup && popup.classList.contains('show') && !popup.contains(evt.target) && !mapWrap.contains(evt.target)) {
        closePoseActionPopup();
    }
});

mapWrap.addEventListener('contextmenu', e => e.preventDefault());
mapWrap.addEventListener('pointerdown', (evt) => {
    if (evt.target.closest('#map-pose-popup')) return;
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
    if (follow || needsDraw) { drawMap(); needsDraw = false; }
    if (ts - lastStatus > 200) { lastStatus = ts; updateStatus(); }
    requestAnimationFrame(frame);
}

// Distance left along the plan, measured from the rover's nearest point on it.
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

let planBadgeEl = null;

function updateStatus() {
    const now = performance.now();
    if (cmdVelAt && now - cmdVelAt > CFG.cmdVelStaleMs && (currentCmdVel.linear || currentCmdVel.angular)) {
        currentCmdVel = { linear: 0, angular: 0 }; needsDraw = true;
    }

    document.getElementById('stat-speed').textContent = (odom ? odom.speed : 0).toFixed(2);

    distanceRemaining = calcDistanceRemaining();
    document.getElementById('stat-distance').textContent =
        (distanceRemaining !== null && (navStatus === 'Navigating' || navStatus === 'Planning')) ? distanceRemaining.toFixed(2) : '—';

    // Mission complete: rover is within tolerance of the final goal
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
    const col = { Reached: 'var(--success)', Aborted: 'var(--danger)', Navigating: 'var(--primary)', Planning: 'var(--warning)' }[navStatus];
    statusEl.style.color = col || 'var(--muted)';
    if (planBadgeEl) planBadgeEl.classList.toggle('show', navStatus === 'Planning');

    if (robot) {
        let deg = robot.yaw * 180 / Math.PI;
        deg = ((deg + 180) % 360 + 360) % 360 - 180;
        document.getElementById('stat-heading').textContent = deg.toFixed(1);
        document.getElementById('hdg-arrow').style.transform = 'rotate(' + (-deg) + 'deg)';
        document.getElementById('stat-location').innerHTML =
            'X ' + robot.x.toFixed(2) + ' &nbsp; Y ' + robot.y.toFixed(2);
    }

    const empty = document.getElementById('map-empty');
    if (empty && !latestMap) {
        empty.textContent = !rosConnected ? 'Map Data Unavailable (ROS bridge offline)'
            : (Date.now() - rosConnectedAt > 8000 ? 'Waiting for ' + TOPICS.map + '…' : 'Waiting for map…');
    }
}

// =====================================================================
//  Navigation panel helpers
// =====================================================================
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
    singleTarget = null; waypointsData = []; activeGoal = null;
    if (navStatus !== 'Planning' && navStatus !== 'Navigating') { latestPlan = []; navStatus = 'Idle'; }
    updateWaypointsUI(); needsDraw = true;
}

// =====================================================================
//  Button helpers
// =====================================================================
async function withBusy(btn, fn) {
    if (btn) { if (btn.classList.contains('loading')) return; btn.classList.add('loading'); btn.disabled = true; }
    try { return await fn(); }
    finally { if (btn) { btn.classList.remove('loading'); btn.disabled = false; } }
}
function run(btn, fn) { return withBusy(btn, fn); }
// Navigation buttons manage their own spinner (it stays until Nav2 answers with a /plan).
function runNavButton(btn, fn) { return fn(); }

window.run = run;
window.runNavButton = runNavButton;

// =====================================================================
//  Navigation commands (REST)
// =====================================================================
function readTarget() {
    const x = parseFloat(document.getElementById('target-x').value);
    const y = parseFloat(document.getElementById('target-y').value);
    const yaw_deg = parseFloat(document.getElementById('target-yaw').value) || 0;
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
    return { x, y, yaw_deg };
}

async function sendGoalPose() {
    const t = readTarget();
    if (!t) { notify('WARNING', 'Enter valid X and Y target values.'); return; }
    const btn = document.getElementById('nav-pose-btn');
    setNavLoading(btn);
    startNavWatchdog();
    navStatus = 'Planning';
    missionGoal = { x: t.x, y: t.y };
    latestPlan = [];
    try {
        await postJSON('/navigate_to_pose', { x: t.x, y: t.y, yaw_deg: t.yaw_deg });
        notify('INFO', 'Goal sent to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
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
    if (waypointsData.length === 0) { notify('WARNING', 'No waypoints added. Click the map to add some.'); return; }
    const btn = document.getElementById('follow-wp-btn');
    setNavLoading(btn);
    startNavWatchdog();
    navStatus = 'Planning';
    const last = waypointsData[waypointsData.length - 1];
    missionGoal = { x: last.x, y: last.y };
    latestPlan = [];
    const waypoints = waypointsData.map(wp => ({ x: wp.x, y: wp.y, yaw_deg: wp.yaw }));
    try {
        await postJSON('/follow_waypoints', { waypoints });
        notify('INFO', 'Following ' + waypoints.length + ' waypoint(s).');
    } catch (e) {
        clearNavLoading();
        navStatus = 'Idle'; missionGoal = null;
        notify('ERROR', 'Waypoint mission failed: ' + e.message);
    }
}

async function sendAbort() {
    // Local state first: the operator must see the abort immediately.
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

// =====================================================================
//  Save map (downloads the PGM + YAML package from the rover)
// =====================================================================
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
        if (!res.ok) {
            let detail = '';
            try { const j = await res.json(); detail = j.detail || j.message || ''; } catch (_) {}
            throw new Error(detail || ('HTTP ' + res.status));
        }
        const blob = await res.blob();
        const cd = res.headers.get('Content-Disposition') || '';
        const m = /filename\*?=(?:UTF-8'')?"?([^";]+)"?/i.exec(cd);
        downloadBlob(blob, m ? decodeURIComponent(m[1]) : mapName + '.zip');
        notify('INFO', 'Map "' + mapName + '" saved and downloaded.');
    } catch (e) {
        if (e instanceof TypeError) {
            // Network/CORS: let the browser handle the download directly.
            notify('WARNING', 'Direct fetch was blocked - asking the browser to download the map instead.');
            const f = document.createElement('iframe');
            f.style.display = 'none'; f.src = url;
            document.body.appendChild(f);
            setTimeout(() => f.remove(), 30000);
        } else {
            notify('ERROR', 'Save map failed: ' + e.message);
        }
    }
}

// =====================================================================
//  System mode
// =====================================================================
let currentMode = null, pending = null, modeSelectTouched = false;
const modeLoaders = [];

function initMapPanelExtras() {
    const wrap = document.getElementById('map-wrapper');
    if (wrap) {
        planBadgeEl = document.createElement('div');
        planBadgeEl.className = 'map-plan-badge';
        planBadgeEl.innerHTML = '<span class="plan-spinner"></span><span>Waiting for Nav2 path…</span>';
        wrap.appendChild(planBadgeEl);
    }
    ['.p-map', '.ctrl-panel'].forEach(sel => {
        const host = document.querySelector(sel);
        if (!host) return;
        const l = document.createElement('div');
        l.className = 'mode-loader';
        l.innerHTML = '<div class="mode-loader-card"><div class="ml-spinner"></div>' +
            '<div class="ml-title">Switching mode</div>' +
            '<div class="ml-sub">Deploying<span class="ml-dots"><i>.</i><i>.</i><i>.</i></span></div></div>';
        host.appendChild(l);
        modeLoaders.push(l);
    });
}
function showModeLoaders(on) { modeLoaders.forEach(l => l.classList.toggle('show', !!on)); }

async function fetchMode() {
    try {
        const res = await fetch(REST_API_BASE + '/system/mode');
        if (res.ok) {
            const data = await res.json();
            return data.mode || data.current_mode || null;
        }
    } catch (e) {}
    return null;
}

function renderMode() {
    const chip = document.getElementById('mode-chip'), txt = document.getElementById('stat-mode');
    const btn = document.getElementById('deploy-btn');
    chip.classList.remove('known', 'pending');
    if (pending) {
        chip.classList.add('pending');
        txt.textContent = 'Switching to ' + modeLabel(pending.target) + '…';
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
    if (m && !modeSelectTouched && !pending) {
        const sel = document.getElementById('sys-mode-select');
        if (sel && Array.from(sel.options).some(o => o.value === m)) sel.value = m;
    }
    renderMode();
}

async function applySystemMode() {
    if (pending) return;
    const mode = document.getElementById('sys-mode-select').value;
    const mapName = (document.getElementById('map-name-input').value || 'small_warehouse').trim();
    pending = { target: mode };
    renderMode(); showModeLoaders(true);
    try {
        const data = await postJSON('/system/mode', { mode, map_name: mapName });
        pending = null;
        setCurrentMode((data && data.mode) || mode);
        notify('INFO', 'Switched to ' + modeLabel(mode) + ' (map: ' + mapName + ').');
    } catch (e) {
        pending = null; renderMode();
        notify('ERROR', 'Mode switch failed: ' + e.message);
    } finally {
        showModeLoaders(false);
    }
}

// =====================================================================
//  Joystick  (publishes sensor_msgs/Joy on /joy, axes = [turn, forward])
// =====================================================================
const joyPad = document.getElementById('joy-pad'), joyKnob = document.getElementById('joy-knob');
const joy = { x: 0, y: 0, active: false, timer: null };
let joyEnabled = false;

function publishJoyRaw(turn, fwd) {
    if (!joyTopic || !rosConnected) return false;
    joyTopic.publish(new ROSLIB.Message({ header: { frame_id: 'joy' }, axes: [turn, fwd], buttons: [] }));
    return true;
}
function publishJoy() {
    if (!joyEnabled) return;
    publishJoyRaw(-joy.x, -joy.y);
}

window.toggleJoyEnable = function () {
    if (!joyEnabled && !rosConnected) {
        notify('WARNING', 'ROS bridge is offline - the joystick cannot send commands yet.');
    }
    joyEnabled = !joyEnabled;
    const btn = document.getElementById('joy-enable-btn');
    btn.textContent = joyEnabled ? 'Disable' : 'Enable';
    btn.classList.toggle('btn-primary', joyEnabled);
    btn.classList.toggle('btn-secondary', !joyEnabled);
    if (!joyEnabled) {                         // always leave the rover stopped
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
    publishJoy(); setTimeout(publishJoy, 50); setTimeout(publishJoy, 100);
}
joyPad.addEventListener('pointerup', joyEnd);
joyPad.addEventListener('pointercancel', joyEnd);
joyPad.addEventListener('lostpointercapture', joyEnd);
window.addEventListener('blur', joyEnd);
document.addEventListener('visibilitychange', () => { if (document.hidden) joyEnd(); });

// =====================================================================
//  MAP VIEWER  (PGM view, zoom / pan / two-click distance measurement)
//  Shows the live map exactly as map_saver would write it (free 254,
//  unknown 205, occupied 0), or any .pgm (+ optional .yaml) from disk.
// =====================================================================
const PGM_FREE = 254, PGM_UNKNOWN = 205, PGM_OCC = 0;
const OCC_THRESH = 65, FREE_THRESH = 25;      // same defaults as nav2_map_server

const mv = {
    ready: false, open: false,
    root: null, canvas: null, ctx: null, body: null,
    img: null, gray: null, w: 0, h: 0, res: 0.05, origin: null, label: '',
    s: 1, tx: 0, ty: 0,
    measure: false, pts: [], hover: null, drag: null
};

const $mv = (id) => document.getElementById(id);

function mvEnsureInit() {
    if (mv.ready) return;
    mv.ready = true;
    mv.root = $mv('map-viewer');
    mv.canvas = $mv('mv-canvas');
    mv.ctx = mv.canvas.getContext('2d');
    mv.body = $mv('mv-body');

    new ResizeObserver(() => { if (mv.open) mvDraw(); }).observe(mv.body);
    mv.root.addEventListener('pointerdown', (e) => { if (e.target === mv.root) closeMapViewer(); });

    mv.canvas.addEventListener('contextmenu', e => e.preventDefault());
    mv.canvas.addEventListener('pointerdown', (e) => {
        mv.canvas.setPointerCapture(e.pointerId);
        mv.drag = { sx: e.offsetX, sy: e.offsetY, tx: mv.tx, ty: mv.ty, moved: false };
    });
    mv.canvas.addEventListener('pointermove', (e) => {
        const sx = e.offsetX, sy = e.offsetY;
        mv.hover = { sx, sy };
        if (mv.drag) {
            const dx = sx - mv.drag.sx, dy = sy - mv.drag.sy;
            if (!mv.drag.moved && Math.hypot(dx, dy) > 3) mv.drag.moved = true;
            if (mv.drag.moved) { mv.tx = mv.drag.tx + dx; mv.ty = mv.drag.ty + dy; }
        }
        mvReadout(); mvDraw();
    });
    const up = (e) => {
        const d = mv.drag; mv.drag = null;
        if (!d || d.moved || e.type === 'pointercancel') { mvDraw(); return; }
        if (mv.measure && mv.img) mvAddPoint(e.offsetX, e.offsetY);
    };
    mv.canvas.addEventListener('pointerup', up);
    mv.canvas.addEventListener('pointercancel', up);
    mv.canvas.addEventListener('pointerleave', () => { mv.hover = null; mvReadout(); mvDraw(); });
    mv.canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        if (!mv.img) return;
        mvZoomAt(e.offsetX, e.offsetY, e.deltaY < 0 ? 1.15 : 1 / 1.15);
    }, { passive: false });

    $mv('mv-file').addEventListener('change', (e) => mvLoadFiles(Array.from(e.target.files || [])));
    document.addEventListener('keydown', (e) => {
        if (!mv.open) return;
        if (e.key === 'Escape') closeMapViewer();
        if (e.key === 'm' || e.key === 'M') mvToggleMeasure();
    });
}

// ----- image source -----
function mvSetGray(gray, w, h, res, origin, label) {
    mv.gray = gray; mv.w = w; mv.h = h; mv.res = res; mv.origin = origin; mv.label = label;
    const c = document.createElement('canvas');
    c.width = w; c.height = h;
    const cx = c.getContext('2d');
    const img = cx.createImageData(w, h), d = img.data;
    for (let i = 0, n = w * h; i < n; i++) {
        const g = gray[i], k = i * 4;
        d[k] = d[k + 1] = d[k + 2] = g; d[k + 3] = 255;
    }
    cx.putImageData(img, 0, 0);
    mv.img = c;
    mv.pts = [];
    mvFit();
    $mv('mv-sub').textContent = label + ' · ' + w + '×' + h + ' px · ' + res.toFixed(3) + ' m/px';
    mvReadout();
}

function mvLoadLive() {
    if (!latestMap) {
        mv.img = null; mv.gray = null;
        $mv('mv-sub').textContent = 'No map received yet';
        mvDraw();
        notify('WARNING', 'No live map yet - waiting for ' + TOPICS.map + '. You can also open a .pgm file.');
        return;
    }
    const m = latestMap, gray = new Uint8Array(m.w * m.h);
    for (let j = 0; j < m.h; j++) {
        for (let i = 0; i < m.w; i++) {
            const v = m.data[j * m.w + i];
            let g = PGM_UNKNOWN;
            if (v >= 0) g = v >= OCC_THRESH ? PGM_OCC : (v <= FREE_THRESH ? PGM_FREE : PGM_UNKNOWN);
            gray[(m.h - 1 - j) * m.w + i] = g;        // PGM row 0 is the top (highest y)
        }
    }
    mvSetGray(gray, m.w, m.h, m.res, { x: m.ox, y: m.oy }, 'Live map (PGM view)');
}

function parsePGM(buf) {
    const u8 = new Uint8Array(buf);
    let pos = 0;
    const tok = () => {
        while (pos < u8.length) {
            const ch = u8[pos];
            if (ch === 35) { while (pos < u8.length && u8[pos] !== 10) pos++; }
            else if (ch <= 32) pos++;
            else break;
        }
        let s = '';
        while (pos < u8.length && u8[pos] > 32) s += String.fromCharCode(u8[pos++]);
        return s;
    };
    const magic = tok();
    if (magic !== 'P5' && magic !== 'P2') throw new Error('not a PGM file (expected P5 or P2)');
    const w = parseInt(tok(), 10), h = parseInt(tok(), 10), maxv = parseInt(tok(), 10);
    if (!(w > 0 && h > 0 && maxv > 0 && maxv < 65536)) throw new Error('invalid PGM header');
    const gray = new Uint8Array(w * h);
    if (magic === 'P5') {
        pos++;                                          // single whitespace after maxval
        const bytes = maxv < 256 ? 1 : 2;
        if (pos + w * h * bytes > u8.length) throw new Error('PGM data is truncated');
        for (let i = 0; i < w * h; i++) {
            const v = bytes === 1 ? u8[pos + i] : ((u8[pos + 2 * i] << 8) | u8[pos + 2 * i + 1]);
            gray[i] = Math.round(v * 255 / maxv);
        }
    } else {
        for (let i = 0; i < w * h; i++) gray[i] = Math.round(parseInt(tok(), 10) * 255 / maxv);
    }
    return { gray, w, h };
}

function parseMapYaml(text) {
    const out = {};
    let m = /resolution:\s*([-+\d.eE]+)/.exec(text); if (m) out.res = parseFloat(m[1]);
    m = /origin:\s*\[\s*([-+\d.eE]+)\s*,\s*([-+\d.eE]+)/.exec(text); if (m) out.origin = { x: parseFloat(m[1]), y: parseFloat(m[2]) };
    m = /negate:\s*(\d)/.exec(text); if (m) out.negate = m[1] === '1';
    return out;
}

function mvImageToGray(file) {
    return new Promise((resolve, reject) => {
        const url = URL.createObjectURL(file), im = new Image();
        im.onload = () => {
            const c = document.createElement('canvas'); c.width = im.naturalWidth; c.height = im.naturalHeight;
            const cx = c.getContext('2d'); cx.drawImage(im, 0, 0);
            const d = cx.getImageData(0, 0, c.width, c.height).data, g = new Uint8Array(c.width * c.height);
            for (let i = 0; i < g.length; i++) g[i] = d[i * 4];
            URL.revokeObjectURL(url);
            resolve({ gray: g, w: c.width, h: c.height });
        };
        im.onerror = () => { URL.revokeObjectURL(url); reject(new Error('cannot decode image')); };
        im.src = url;
    });
}

async function mvLoadFiles(files) {
    if (!files.length) return;
    const img = files.find(f => /\.(pgm|png|jpe?g)$/i.test(f.name));
    const yml = files.find(f => /\.ya?ml$/i.test(f.name));
    $mv('mv-file').value = '';
    if (!img) { notify('WARNING', 'Select a .pgm file (and optionally its .yaml).'); return; }
    try {
        const parsed = /\.pgm$/i.test(img.name) ? parsePGM(await img.arrayBuffer()) : await mvImageToGray(img);
        let res = latestMap ? latestMap.res : 0.05, origin = null, note = '';
        if (yml) {
            const y = parseMapYaml(await yml.text());
            if (y.res) res = y.res;
            if (y.origin) origin = y.origin;
            if (y.negate) for (let i = 0; i < parsed.gray.length; i++) parsed.gray[i] = 255 - parsed.gray[i];
        } else {
            note = ' (no .yaml selected - assuming ' + res.toFixed(3) + ' m/px)';
        }
        mvSetGray(parsed.gray, parsed.w, parsed.h, res, origin, img.name);
        mvDraw();
        if (note) notify('INFO', 'Loaded ' + img.name + note);
    } catch (e) {
        notify('ERROR', 'Could not open map: ' + e.message);
    }
}

function mvDownloadPGM() {
    if (!mv.gray) { notify('WARNING', 'No map to download yet.'); return; }
    const head = 'P5\n# CREATOR: Milusions WareGV Suite ' + mv.res.toFixed(3) + ' m/pix\n' + mv.w + ' ' + mv.h + '\n255\n';
    const hb = new TextEncoder().encode(head), out = new Uint8Array(hb.length + mv.gray.length);
    out.set(hb, 0); out.set(mv.gray, hb.length);
    const base = ($mv('map-name-input').value || 'map').trim().replace(/[^\w.\-]+/g, '_') || 'map';
    downloadBlob(new Blob([out], { type: 'image/x-portable-graymap' }), base + '.pgm');
}

// ----- view transform -----
function mvFit() {
    if (!mv.img) return;
    const W = mv.canvas.clientWidth || 800, H = mv.canvas.clientHeight || 500;
    mv.s = Math.min(W / mv.w, H / mv.h) * 0.96;
    mv.tx = (W - mv.w * mv.s) / 2;
    mv.ty = (H - mv.h * mv.s) / 2;
}
function mvFitBtn() { mvFit(); mvDraw(); }
function mvZoomAt(sx, sy, f) {
    const s0 = mv.s;
    const fit = Math.min((mv.canvas.clientWidth || 800) / mv.w, (mv.canvas.clientHeight || 500) / mv.h);
    mv.s = Math.max(fit * 0.2, Math.min(60, s0 * f));
    const k = mv.s / s0;
    mv.tx = sx - (sx - mv.tx) * k;
    mv.ty = sy - (sy - mv.ty) * k;
    mvReadout(); mvDraw();
}
function mvZoomBtn(f) { mvZoomAt((mv.canvas.clientWidth || 800) / 2, (mv.canvas.clientHeight || 500) / 2, f); }
const mvToImg = (sx, sy) => ({ u: (sx - mv.tx) / mv.s, v: (sy - mv.ty) / mv.s });

// ----- measure -----
function mvToggleMeasure() {
    mv.measure = !mv.measure;
    $mv('mv-measure-btn').classList.toggle('on', mv.measure);
    mv.canvas.style.cursor = mv.measure ? 'crosshair' : 'grab';
    if (!mv.measure) mv.pts = [];
    $mv('mv-hint').textContent = mv.measure
        ? 'Measure: click a first point, then a second point. Drag to pan, wheel to zoom.'
        : 'Drag to pan · wheel to zoom · press Measure (or M) to measure distances.';
    mvDraw();
}
function mvClearMeasure() { mv.pts = []; mvDraw(); }
function mvAddPoint(sx, sy) {
    const p = mvToImg(sx, sy);
    if (p.u < 0 || p.v < 0 || p.u > mv.w || p.v > mv.h) return;
    if (mv.pts.length >= 2) mv.pts = [];
    mv.pts.push(p);
    mvReadout(); mvDraw();
}
const mvDist = (a, b) => Math.hypot(a.u - b.u, a.v - b.v) * mv.res;

function mvReadout() {
    const el = $mv('mv-readout');
    if (!el) return;
    let txt = '';
    if (mv.hover && mv.img) {
        const p = mvToImg(mv.hover.sx, mv.hover.sy);
        if (p.u >= 0 && p.v >= 0 && p.u < mv.w && p.v < mv.h) {
            txt = 'px ' + Math.floor(p.u) + ', ' + Math.floor(p.v);
            if (mv.origin) txt += ' · x ' + (mv.origin.x + p.u * mv.res).toFixed(2) + ' y ' + (mv.origin.y + (mv.h - p.v) * mv.res).toFixed(2) + ' m';
        }
    }
    if (mv.pts.length === 2) txt = 'Distance ' + mvDist(mv.pts[0], mv.pts[1]).toFixed(3) + ' m   ' + (txt ? '· ' + txt : '');
    el.textContent = txt;
}

// ----- drawing -----
function mvDraw() {
    if (!mv.ready || !mv.open) return;
    const c = mv.canvas, dpr = window.devicePixelRatio || 1;
    const W = c.clientWidth, H = c.clientHeight;
    if (!W || !H) return;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) {
        c.width = Math.round(W * dpr); c.height = Math.round(H * dpr);
        if (mv.img && !mv._fitted) { mvFit(); mv._fitted = true; }
    }
    const ctx = mv.ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = theme['--map-bg'] || '#111'; ctx.fillRect(0, 0, W, H);

    if (!mv.img) {
        ctx.fillStyle = theme['--muted'] || '#888'; ctx.font = '500 13px Roboto, sans-serif';
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText('No map data yet - waiting for ' + TOPICS.map + ' (or open a .pgm file)', W / 2, H / 2);
        return;
    }

    ctx.save();
    ctx.imageSmoothingEnabled = mv.s < 1;
    ctx.drawImage(mv.img, mv.tx, mv.ty, mv.w * mv.s, mv.h * mv.s);
    ctx.strokeStyle = theme['--border'] || '#444'; ctx.lineWidth = 1;
    ctx.strokeRect(mv.tx + 0.5, mv.ty + 0.5, mv.w * mv.s, mv.h * mv.s);
    ctx.restore();

    // measurement overlay
    const S = (p) => [mv.tx + p.u * mv.s, mv.ty + p.v * mv.s];
    const pts = mv.pts.slice();
    let live = false;
    if (mv.measure && pts.length === 1 && mv.hover) { pts.push(mvToImg(mv.hover.sx, mv.hover.sy)); live = true; }
    if (pts.length) {
        const col = '#ff3b30';
        ctx.lineWidth = 2; ctx.strokeStyle = col; ctx.fillStyle = col;
        if (pts.length === 2) {
            const a = S(pts[0]), b = S(pts[1]);
            ctx.save(); if (live) ctx.setLineDash([6, 4]);
            ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); ctx.restore();
            const label = mvDist(pts[0], pts[1]).toFixed(2) + ' m';
            ctx.font = '700 12px Roboto, sans-serif';
            const tw = ctx.measureText(label).width + 14, mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
            ctx.fillStyle = 'rgba(20,22,26,.92)'; ctx.strokeStyle = col; ctx.lineWidth = 1;
            ctx.beginPath(); ctx.roundRect ? ctx.roundRect(mx - tw / 2, my - 24, tw, 20, 5) : ctx.rect(mx - tw / 2, my - 24, tw, 20);
            ctx.fill(); ctx.stroke();
            ctx.fillStyle = '#fff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(label, mx, my - 14);
        }
        pts.forEach((p, i) => {
            if (live && i === 1) return;
            const q = S(p);
            ctx.fillStyle = col; ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
            ctx.beginPath(); ctx.arc(q[0], q[1], 5, 0, 7); ctx.fill(); ctx.stroke();
        });
    }

    // scale bar
    const pxPerM = mv.s / mv.res;
    const nice = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100].find(n => n * pxPerM >= 70) || 100;
    const len = nice * pxPerM, bx = 14, by = H - 16;
    ctx.fillStyle = 'rgba(20,22,26,.8)'; ctx.fillRect(bx - 6, by - 20, len + 12, 28);
    ctx.strokeStyle = '#fff'; ctx.fillStyle = '#fff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by); ctx.lineTo(bx + len, by); ctx.lineTo(bx + len, by - 4); ctx.stroke();
    ctx.font = '600 11px Roboto, sans-serif'; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText(nice + ' m', bx, by - 6);
}

// ----- open / close -----
function openMapViewer() {
    mvEnsureInit();
    mv.root.hidden = false;
    mv.open = true;
    mv._fitted = false;
    $mv('mv-measure-btn').classList.toggle('on', mv.measure);
    mv.canvas.style.cursor = mv.measure ? 'crosshair' : 'grab';
    // Always show the freshest live map when opening, unless a file was loaded on purpose.
    if (!mv.gray || mv.label.indexOf('Live map') === 0) mvLoadLive();
    requestAnimationFrame(() => { if (mv.img) mvFit(); mvDraw(); });
}
function closeMapViewer() {
    if (!mv.ready) return;
    mv.open = false;
    mv.root.hidden = true;
    mv.drag = null;
}
function mvUseLive() { mvLoadLive(); mvDraw(); }

Object.assign(window, { openMapViewer, closeMapViewer, mvToggleMeasure, mvClearMeasure, mvFitBtn, mvZoomBtn, mvUseLive, mvDownloadPGM });

// =====================================================================
//  Sound FX
// =====================================================================
let audioCtx = null;
function initAudio() {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    if (audioCtx.state === 'suspended') audioCtx.resume();
}
function tone(type, f0, f1, gain0, dur, ramp) {
    try {
        initAudio();
        const now = audioCtx.currentTime;
        const osc = audioCtx.createOscillator(), gain = audioCtx.createGain();
        osc.type = type;
        osc.frequency.setValueAtTime(f0, now);
        if (f1) osc.frequency[ramp || 'linearRampToValueAtTime'](f1, now + dur * 0.6);
        gain.gain.setValueAtTime(gain0, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + dur);
        osc.connect(gain); gain.connect(audioCtx.destination);
        osc.start(now); osc.stop(now + dur);
    } catch (e) {}
}
function playStartupSound() { tone('sine', 440, 880, 0.45, 0.35, 'exponentialRampToValueAtTime'); }
function playListenSound() { tone('sine', 587.33, 880, 0.12, 0.2); }
function playProcessingSound() { tone('triangle', 320, 160, 0.1, 0.12); }

// =====================================================================
//  HELIO VOICE ASSISTANT
//  Continuous transcription. A command is submitted after a short,
//  deliberate pause in speech (never on a recognition segment boundary).
//  What you said is shown in the teleprompter and read back aloud while the
//  command is already on its way to Helio.
// =====================================================================
const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
const BASE_SPEECH_LANG = (navigator.language || '').toLowerCase().startsWith('en') ? navigator.language : 'en-US';
const SILENCE_COMMIT_MS = 1500;        // pause after the last word before sending (was 3600)
const LOW_CONFIDENCE_GRACE_MS = 800;   // small extra wait when recognition is unsure (was 3200)
const CONFIDENCE_THRESHOLD = 0.6;
const HELIO_TIMEOUT_MS = 20000;

const modal = document.getElementById('voice-modal');
const robotFace = document.getElementById('robot-face');
const teleprompter = document.getElementById('teleprompter');
const captionStatus = document.getElementById('caption-status');
const captionText = document.getElementById('caption-text');
const historyListEl = document.getElementById('history-list');
const thinkingPanel = document.getElementById('thinking-panel');
const thinkingStageText = document.getElementById('thinking-stage-text');

let recognition = null;
let recognitionRestartTimer = null;
let recognitionRestartDelay = 80;
let silenceCommitTimer = null;
let lowConfidenceTimer = null;
let transcriptText = '';
let interimText = '';
let confidenceSum = 0;
let confidenceCount = 0;
let speechActive = false;
let helioOpen = false;
let agentState = 'idle';               // idle | listening | thinking | speaking
let currentCallId = null;
let selectedLanguage = 'en';
let thinkingTimer = null;
let thinkingStageIndex = 0;
let commandSeq = 0;
let lastSpeechNetworkWarn = 0;
window.currentUtterance = null;

const THINKING_STAGES = ['Processing command', 'Working...', 'Preparing response'];
const speechLang = () => selectedLanguage === 'hi' ? 'hi-IN' : BASE_SPEECH_LANG;

const PeekController = {
    messages: [
        'Ever wondered if you could control a rover hands free?',
        'Hey there, how are you?',
        'Controlling a rover, I see!',
        'Hey, what is this button?'
    ],
    timer: null,
    hideTimer: null,
    start() {
        this.stop();
        this.timer = setInterval(() => this.show(), 6500);
        setTimeout(() => this.show(), 1400);
    },
    stop() {
        if (this.timer) clearInterval(this.timer);
        if (this.hideTimer) clearTimeout(this.hideTimer);
        this.timer = null; this.hideTimer = null;
        const c = document.getElementById('helio-peek'), b = document.getElementById('peek-bubble');
        if (c) c.classList.remove('peeking');
        if (b) b.classList.remove('show');
    },
    show() {
        const container = document.getElementById('helio-peek');
        const bubble = document.getElementById('peek-bubble');
        if (!container || !bubble || helioOpen) return;
        bubble.textContent = this.messages[Math.floor(Math.random() * this.messages.length)];
        container.classList.add('peeking');
        bubble.classList.add('show');
        if (this.hideTimer) clearTimeout(this.hideTimer);
        this.hideTimer = setTimeout(() => {
            container.classList.remove('peeking');
            setTimeout(() => bubble.classList.remove('show'), 650);
        }, 3500);
    }
};

// ----- small UI helpers -----
function setVoiceStatus(text, live = false) {
    if (!captionStatus) return;
    captionStatus.textContent = text;
    captionStatus.classList.toggle('live', live);
}

function renderTranscript(empty = 'Speak whenever you are ready...') {
    if (!captionText) return;
    const value = [transcriptText.trim(), interimText.trim()].filter(Boolean).join(' ');
    captionText.textContent = value || empty;
    captionText.scrollTop = captionText.scrollHeight;          // long prompt: always follow the newest words
}

function setFace(state) { if (robotFace) robotFace.className = 'robot-face ' + state; }

// who: 'user' (grey, your words) | 'helio' (white, Helio's reply)
function setTeleprompter(text, who) {
    if (!teleprompter) return;
    teleprompter.className = 'teleprompter ' + (who === 'helio' ? 'helio-speaking' : 'user-speaking');
    teleprompter.textContent = text || '';
    teleprompter.scrollTop = 0;
}

function appendHistory(sender, text) {
    if (!historyListEl || !text) return;
    const item = document.createElement('div');
    item.className = 'history-item ' + sender;
    const label = document.createElement('div');
    label.className = 'history-label';
    label.textContent = sender === 'you' ? 'you:' : 'agent:';
    const content = document.createElement('div');
    if (sender !== 'you' && typeof marked !== 'undefined' && marked.parse) {
        try { content.innerHTML = marked.parse(String(text)); } catch (_) { content.textContent = text; }
    } else {
        content.textContent = text;
    }
    item.append(label, content);
    historyListEl.appendChild(item);
    historyListEl.scrollTop = historyListEl.scrollHeight;
}

function clearVoiceTimers() {
    if (silenceCommitTimer) clearTimeout(silenceCommitTimer);
    if (lowConfidenceTimer) clearTimeout(lowConfidenceTimer);
    if (recognitionRestartTimer) clearTimeout(recognitionRestartTimer);
    silenceCommitTimer = null; lowConfidenceTimer = null; recognitionRestartTimer = null;
}

function startThinkingIndicator() {
    stopThinkingIndicator();
    if (!thinkingPanel) return;
    thinkingPanel.classList.add('active');
    thinkingStageIndex = 0;
    const update = () => {
        if (thinkingStageText) thinkingStageText.textContent = THINKING_STAGES[Math.min(thinkingStageIndex, THINKING_STAGES.length - 1)];
        thinkingStageIndex += 1;
    };
    update();
    thinkingTimer = setInterval(update, 1200);
}
function stopThinkingIndicator() {
    if (thinkingTimer) clearInterval(thinkingTimer);
    thinkingTimer = null;
    if (thinkingPanel) thinkingPanel.classList.remove('active');
}

// ----- recognition -----
function recognitionStart() {
    if (!recognition || !helioOpen || agentState !== 'listening') return;
    try { recognition.start(); }
    catch (_) { recognitionRestartTimer = setTimeout(() => recognitionStart(), 180); }
}
function recognitionStop() {
    try { recognition && recognition.abort(); } catch (_) {}
}
function scheduleRecognitionRestart() {
    if (!helioOpen || agentState !== 'listening') return;
    if (recognitionRestartTimer) clearTimeout(recognitionRestartTimer);
    recognitionRestartTimer = setTimeout(() => { recognitionRestartTimer = null; recognitionStart(); }, recognitionRestartDelay);
}

function pendingText() {
    return [transcriptText.trim(), interimText.trim()].filter(Boolean).join(' ').replace(/\s+/g, ' ').trim();
}

// Called on every recognition result. Restarts the silence countdown, so a
// command is only sent once the speaker has really stopped for SILENCE_COMMIT_MS.
function scheduleSilenceCommit() {
    if (silenceCommitTimer) clearTimeout(silenceCommitTimer);
    if (lowConfidenceTimer) { clearTimeout(lowConfidenceTimer); lowConfidenceTimer = null; }
    silenceCommitTimer = setTimeout(() => {
        silenceCommitTimer = null;
        if (!helioOpen || agentState !== 'listening' || !pendingText()) return;
        const confidence = confidenceCount ? confidenceSum / confidenceCount : 1;
        if (confidence >= CONFIDENCE_THRESHOLD) {
            commitTranscript();
        } else {
            setVoiceStatus('WAITING FOR CLEAR SPEECH', true);
            lowConfidenceTimer = setTimeout(() => {
                lowConfidenceTimer = null;
                if (helioOpen && agentState === 'listening' && pendingText()) commitTranscript();
            }, LOW_CONFIDENCE_GRACE_MS);
        }
    }, SILENCE_COMMIT_MS);
}

function commitTranscript() {
    const text = pendingText();
    if (!text || agentState !== 'listening') return;
    clearVoiceTimers();
    recognitionStop();                      // mic off while Helio talks, so it never hears itself
    interimText = ''; transcriptText = '';
    confidenceSum = 0; confidenceCount = 0;
    renderTranscript();
    appendHistory('you', text);
    handleCommand(text);
}

function bindRecognition() {
    if (!SpeechRecognitionImpl) return;
    recognition = new SpeechRecognitionImpl();
    recognition.continuous = true;
    recognition.interimResults = true;
    recognition.maxAlternatives = 3;
    recognition.lang = speechLang();

    recognition.onstart = () => {
        if (!helioOpen) return;
        agentState = 'listening';
        speechActive = true;
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        setFace('listening');
    };

    recognition.onresult = (event) => {
        if (!helioOpen || agentState !== 'listening') return;
        recognitionRestartDelay = 80;
        let currentInterim = '';
        for (let i = event.resultIndex; i < event.results.length; i += 1) {
            const result = event.results[i];
            const alt = result[0];
            const text = ((alt && alt.transcript) || '').trim();
            if (!text) continue;
            if (result.isFinal) {
                transcriptText = (transcriptText + ' ' + text).replace(/\s+/g, ' ').trim();
                const confidence = Number(alt && alt.confidence);
                if (Number.isFinite(confidence) && confidence > 0) { confidenceSum += confidence; confidenceCount += 1; }
            } else {
                currentInterim = (currentInterim + ' ' + text).trim();
            }
        }
        interimText = currentInterim;
        speechActive = true;
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        renderTranscript();
        if (pendingText()) scheduleSilenceCommit();
    };

    recognition.onerror = (event) => {
        if (!helioOpen) return;
        if (event.error === 'not-allowed' || event.error === 'service-not-allowed' || event.error === 'audio-capture') {
            agentState = 'idle';
            speechActive = false;
            setVoiceStatus('MICROPHONE UNAVAILABLE');
            setFace('listening');
            setTeleprompter('Microphone permission is required (use https or localhost).', 'helio');
            return;
        }
        if (event.error === 'network') {
            recognitionRestartDelay = 2500;
            if (Date.now() - lastSpeechNetworkWarn > 15000) {
                lastSpeechNetworkWarn = Date.now();
                notify('ERROR', 'Speech recognition needs an internet connection (Chrome sends audio to Google).');
            }
        }
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
    };

    recognition.onend = () => {
        speechActive = false;
        if (!helioOpen || agentState !== 'listening') return;
        // Browsers end recognition on their own now and then. That is NOT the end of
        // a command: restart quietly and keep the words collected so far.
        scheduleRecognitionRestart();
    };
}

function resumeListening() {
    if (!helioOpen) return;
    agentState = 'listening';
    transcriptText = ''; interimText = ''; confidenceSum = 0; confidenceCount = 0;
    renderTranscript();
    setVoiceStatus('LISTENING CONTINUOUSLY', true);
    setFace('listening');
    setTeleprompter('', 'user');
    recognitionStart();
}

// ----- open / close -----
function openVoiceModal() {
    if (!SpeechRecognitionImpl) {
        alert('Continuous speech recognition is not supported in this browser. Use Chrome or Edge.');
        return;
    }
    stopWake();
    helioOpen = true;
    if (modal) { modal.classList.add('active'); modal.setAttribute('aria-hidden', 'false'); }
    const hist = document.getElementById('modal-history-column'); if (hist) hist.classList.remove('show');
    const drawer = document.getElementById('prompt-drawer'); if (drawer) drawer.classList.remove('show');
    PeekController.stop();

    transcriptText = ''; interimText = ''; confidenceSum = 0; confidenceCount = 0;
    clearVoiceTimers();
    commandSeq += 1;
    currentCallId = currentCallId || String(Date.now());
    recognition.lang = speechLang();
    renderTranscript();
    setVoiceStatus('LISTENING CONTINUOUSLY', true);
    setFace('listening');
    setTeleprompter('', 'user');
    agentState = 'listening';

    recognitionStop();
    setTimeout(() => recognitionStart(), 80);
}

function closeVoiceModal() {
    helioOpen = false;
    agentState = 'idle';
    speechActive = false;
    commandSeq += 1;                        // invalidate any command still in flight
    clearVoiceTimers();
    recognitionStop();
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    stopThinkingIndicator();
    if (modal) { modal.classList.remove('active'); modal.setAttribute('aria-hidden', 'true'); }
    currentCallId = null;
    transcriptText = ''; interimText = ''; confidenceSum = 0; confidenceCount = 0;
    renderTranscript();
    PeekController.start();
    syncWake();
}

// ----- speech output -----
function cleanForSpeech(t) {
    return String(t || '')
        .replace(/```[\s\S]*?```/g, ' ')
        .replace(/!?\[([^\]]*)\]\([^)]*\)/g, '$1')
        .replace(/[*_`#>~]+/g, '')
        .replace(/\s+/g, ' ')
        .trim();
}

// Speaks `text` and resolves when finished (or cancelled). While speaking, the
// teleprompter scrolls along with the words so long text stays readable.
function speakText(text, opts) {
    opts = opts || {};
    return new Promise((resolve) => {
        const spoken = cleanForSpeech(text);
        if (!('speechSynthesis' in window) || !spoken) { resolve(); return; }
        let finished = false, wd = null;
        const finish = () => { if (finished) return; finished = true; clearTimeout(wd); resolve(); };
        try { window.speechSynthesis.cancel(); } catch (_) {}
        const u = new SpeechSynthesisUtterance(spoken);
        u.lang = speechLang();
        u.rate = 1.02;
        u.onend = finish;
        u.onerror = finish;
        if (opts.follow && teleprompter) {
            const total = opts.follow.length || 1;
            u.onboundary = (e) => {
                const idx = Math.max(0, e.charIndex - (opts.offset || 0));
                const max = teleprompter.scrollHeight - teleprompter.clientHeight;
                if (max > 0) teleprompter.scrollTop = Math.min(max, (idx / total) * max);
            };
        }
        wd = setTimeout(finish, 2500 + spoken.length * 110);   // some engines never fire onend
        window.currentUtterance = u;
        window.speechSynthesis.speak(u);
    });
}

function stopTalking() {
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
}

const announcePrefix = () => selectedLanguage === 'hi' ? 'आपने कहा, ' : 'You said, ';

// ----- one full command turn: read back -> Helio -> reply -----
async function handleCommand(text) {
    const seq = ++commandSeq;
    const alive = () => helioOpen && seq === commandSeq;

    agentState = 'speaking';
    setVoiceStatus('HELIO SPEAKING', false);
    setFace('speaking');
    setTeleprompter(text, 'user');
    playProcessingSound();

    // The request goes out immediately; the read-back happens in parallel.
    let done = false;
    const replyP = queryHelio(text)
        .catch((e) => ({ text: 'Sorry, something went wrong: ' + (e && e.message ? e.message : e) }))
        .then((r) => { done = true; return r; });

    const prefix = announcePrefix();
    await speakText(prefix + text, { follow: text, offset: prefix.length });
    if (!alive()) return;

    if (!done) {
        agentState = 'thinking';
        setVoiceStatus('SENDING TO HELIO', false);
        setFace('thinking');
        startThinkingIndicator();
    }
    const reply = await replyP;
    stopThinkingIndicator();
    if (!alive()) return;

    const answer = (reply && reply.text) || 'Done.';
    appendHistory('agent', answer);
    agentState = 'speaking';
    setVoiceStatus('HELIO SPEAKING', false);
    setFace('speaking');
    setTeleprompter(answer, 'helio');
    await speakText(answer, { follow: cleanForSpeech(answer) });
    if (!alive()) return;
    resumeListening();
}

function sendManualText() {
    const input = document.getElementById('user-input');
    const text = ((input && input.value) || '').trim();
    if (!text) return;
    if (!helioOpen) openVoiceModal();
    if (agentState !== 'listening') return;          // Helio is busy with another command
    input.value = '';
    transcriptText = text;
    interimText = '';
    confidenceSum = 1; confidenceCount = 1;
    commitTranscript();
}

function setLang(lang) {
    selectedLanguage = lang === 'hi' ? 'hi' : 'en';
    const en = document.getElementById('btn-en'), hi = document.getElementById('btn-hi');
    if (en) en.classList.toggle('active', selectedLanguage === 'en');
    if (hi) hi.classList.toggle('active', selectedLanguage === 'hi');
    if (recognition) {
        recognition.lang = speechLang();
        if (helioOpen && agentState === 'listening') { recognitionStop(); scheduleRecognitionRestart(); }
    }
}

window.toggleHistory = function () {
    const el = document.getElementById('modal-history-column'); if (el) el.classList.toggle('show');
};
window.closeHistory = function () {
    const el = document.getElementById('modal-history-column'); if (el) el.classList.remove('show');
};
window.togglePrompt = function (event) {
    if (event) event.stopPropagation();
    const drawer = document.getElementById('prompt-drawer');
    if (!drawer) return;
    drawer.classList.toggle('show');
    const btn = document.getElementById('prompt-toggle-btn');
    if (btn) btn.setAttribute('aria-expanded', drawer.classList.contains('show') ? 'true' : 'false');
    if (drawer.classList.contains('show')) setTimeout(() => { const i = document.getElementById('user-input'); if (i) i.focus(); }, 80);
};
document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') { const d = document.getElementById('prompt-drawer'); if (d) d.classList.remove('show'); }
});
(function () {
    const input = document.getElementById('user-input');
    if (input) input.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); sendManualText(); } });
})();

// =====================================================================
//  HELIO BRAIN
//  1) POST the text to the rover's Helio endpoint (HELIO_ENDPOINT).
//  2) If that endpoint does not exist or fails, the built-in command engine
//     below drives the dashboard directly (navigate, waypoints, abort, ...).
// =====================================================================
let helioBackendDownUntil = 0;

function pickReply(data) {
    if (!data) return '';
    if (typeof data === 'string') return data;
    const v = data.response || data.reply || data.message || data.text || data.answer || data.output;
    return typeof v === 'string' ? v : '';
}

async function queryHelio(text) {
    if (Date.now() >= helioBackendDownUntil) {
        const ctrl = new AbortController();
        const to = setTimeout(() => ctrl.abort(), HELIO_TIMEOUT_MS);
        try {
            const url = /^https?:/i.test(HELIO_ENDPOINT) ? HELIO_ENDPOINT : REST_API_BASE + HELIO_ENDPOINT;
            const res = await fetch(url, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ text, message: text, lang: selectedLanguage, call_id: currentCallId }),
                signal: ctrl.signal
            });
            if (res.ok) {
                const raw = await res.text();
                let data = raw;
                try { data = JSON.parse(raw); } catch (_) {}
                const reply = pickReply(data);
                if (reply) return { text: reply, source: 'backend' };
            } else {
                helioBackendDownUntil = Date.now() + 60000;      // 404 etc: skip the endpoint for a minute
            }
        } catch (_) {
            helioBackendDownUntil = Date.now() + 60000;
        } finally {
            clearTimeout(to);
        }
    }
    return localAssistant(text);
}

function numbersIn(s) {
    const t = s.replace(/\bminus\b/g, '-').replace(/\bpoint\b/g, '.');
    return (t.match(/-?\d+(?:\.\d+)?/g) || []).map(Number);
}

function statusReport() {
    const parts = [];
    parts.push('Mode: ' + (currentMode ? modeLabel(currentMode) : 'unknown') + '.');
    parts.push('Navigation is ' + navStatus.toLowerCase() + '.');
    if (robot) parts.push('The rover is at x ' + robot.x.toFixed(2) + ', y ' + robot.y.toFixed(2) + ' meters, heading ' +
        (((robot.yaw * 180 / Math.PI) + 360) % 360).toFixed(0) + ' degrees.');
    else parts.push('I have no position yet.');
    if (odom) parts.push('Speed is ' + odom.speed.toFixed(2) + ' meters per second.');
    if (distanceRemaining !== null && (navStatus === 'Navigating' || navStatus === 'Planning'))
        parts.push(distanceRemaining.toFixed(1) + ' meters to go.');
    parts.push(rosConnected ? 'ROS bridge is connected.' : 'ROS bridge is offline.');
    return parts.join(' ');
}

async function localAssistant(raw) {
    const t = ' ' + raw.toLowerCase().replace(/[^\w\s.\-,]/g, ' ').replace(/\s+/g, ' ').trim() + ' ';
    const has = (re) => re.test(t);
    const nums = numbersIn(t);

    if (has(/\b(abort|cancel|halt|emergency|stop)\b/) && !has(/\bstop (the )?joystick\b/)) {
        await sendAbort();
        return { text: 'Mission aborted.' };
    }
    if (has(/\b(enable|activate|turn on|start)\b.*\bjoystick\b/)) {
        if (!joyEnabled) window.toggleJoyEnable();
        return { text: 'Joystick enabled.' };
    }
    if (has(/\b(disable|deactivate|turn off|stop)\b.*\bjoystick\b/)) {
        if (joyEnabled) window.toggleJoyEnable();
        return { text: 'Joystick disabled.' };
    }
    if (has(/\bfollow\b.*\bwaypoints?\b|\b(start|run|execute)\b.*\bwaypoints?\b/)) {
        if (!waypointsData.length) return { text: 'There are no waypoints yet. Click the map to add some first.' };
        await sendWaypoints();
        return { text: 'Following ' + waypointsData.length + ' waypoints.' };
    }
    if (has(/\b(clear|remove|delete|reset)\b.*\b(markers?|waypoints?|targets?)\b/)) {
        clearMarkers();
        return { text: 'Markers cleared.' };
    }
    if (has(/\b(set|update)\b.*\b(initial|start)\b.*\b(pose|position)\b/)) {
        if (nums.length >= 2) { updateUIInputs(nums[0], nums[1], nums[2] || 0); singleTarget = { x: nums[0], y: nums[1], yaw: nums[2] || 0 }; }
        await sendInitialPose();
        return { text: 'Initial pose sent.' };
    }
    if (has(/\b(navigate|go|drive|move|head|take me)\b/) && nums.length >= 2) {
        const yaw = nums[2] || 0;
        updateUIInputs(nums[0], nums[1], yaw);
        singleTarget = { x: nums[0], y: nums[1], yaw };
        needsDraw = true;
        await sendGoalPose();
        return { text: 'Navigating to x ' + nums[0] + ', y ' + nums[1] + '.' };
    }
    if (has(/\b(navigate|go|drive)\b.*\b(target|marker|selected|there)\b/)) {
        if (!singleTarget) return { text: 'No target is selected. Click the map to choose one.' };
        await sendGoalPose();
        return { text: 'Navigating to the selected target.' };
    }
    if (has(/\bsave\b.*\bmap\b/)) {
        const m = /\b(?:as|named|called)\s+([\w\-]+)/.exec(t);
        if (m) document.getElementById('map-name-input').value = m[1];
        await saveCurrentMap();
        return { text: 'Saving the map.' };
    }
    if (has(/\b(view|open|show)\b.*\bmap\b/)) {
        openMapViewer();
        return { text: 'Opening the map viewer.' };
    }
    if (has(/\b(fit|reset|center|centre)\b.*\b(map|view)\b/)) {
        fitMapView();
        return { text: 'Map fitted to the panel.' };
    }
    if (has(/\bfollow\b.*\b(rover|robot|me)\b/)) {
        toggleFollow();
        return { text: follow ? 'Following the rover.' : 'Stopped following the rover.' };
    }
    if (has(/\b(dark|night)\b.*\b(mode|theme)\b/)) { if (document.body.classList.contains('light-theme')) toggleTheme(); return { text: 'Dark theme on.' }; }
    if (has(/\b(light|day)\b.*\b(mode|theme)\b/)) { if (!document.body.classList.contains('light-theme')) toggleTheme(); return { text: 'Light theme on.' }; }
    if (has(/\b(status|where|position|location|report|speed|how far|distance)\b/)) return { text: statusReport() };
    if (has(/\b(hello|hi|hey|namaste)\b/)) return { text: 'Hello! I am Helio. Tell me where to drive the rover.' };
    if (has(/\b(help|what can you do)\b/)) {
        return { text: 'You can say: go to x 2 y 3, follow waypoints, abort, save map, view map, enable joystick, or status.' };
    }
    return { text: "Sorry, I didn't understand that command. Try: go to x 2 y 3, follow waypoints, abort, save map, or status." };
}

// =====================================================================
//  Wake word  (dashboard-level: say the word and Helio opens by itself)
// =====================================================================
const WAKE_OPTIONS = { rover: ['rover'], helio: ['helio'], computer: ['computer'], car: ['car'], robot: ['robot'] };
let wakeWord = 'rover';
let wakeEnabled = true;
let wakeBlocked = false;
let wakeRec = null, wakeRunning = false, wakeWanted = false, wakeRestartTimer = null, wakeRestartDelay = 300;
try {
    const savedWord = localStorage.getItem('milusions-wake-word');
    const savedState = localStorage.getItem('milusions-wake');
    if (savedWord && WAKE_OPTIONS[savedWord]) wakeWord = savedWord;
    if (savedState === 'off') wakeEnabled = false;
} catch (_) {}

function renderWakeMenu() {
    const menu = document.getElementById('wake-menu');
    if (!menu) return;
    menu.innerHTML = '<div class="wm-title">Wake word</div>';
    Object.keys(WAKE_OPTIONS).forEach((key) => {
        const button = document.createElement('button');
        button.type = 'button';
        button.className = key === wakeWord ? 'sel' : '';
        button.textContent = key;
        button.onclick = () => {
            wakeWord = key;
            try { localStorage.setItem('milusions-wake-word', key); } catch (_) {}
            updateWakeBtn(); closeWakeMenu();
        };
        menu.appendChild(button);
    });
    menu.appendChild(document.createElement('hr'));
    const toggle = document.createElement('button');
    toggle.type = 'button';
    toggle.className = 'wm-toggle' + (wakeEnabled ? ' on' : '');
    toggle.dataset.state = wakeEnabled ? 'ON' : 'OFF';
    toggle.textContent = 'Listen for wake word';
    toggle.onclick = toggleWake;
    menu.appendChild(toggle);
}
function updateWakeBtn() {
    const btn = document.getElementById('wake-btn');
    const label = document.getElementById('wake-label');
    if (label) label.textContent = wakeWord;
    const on = wakeEnabled && !wakeBlocked;
    if (btn) {
        btn.classList.toggle('on', on); btn.classList.toggle('off', !on);
        btn.title = wakeBlocked ? 'Microphone blocked - wake word unavailable' : 'Wake word settings';
    }
    renderWakeMenu();
    syncWake();
}
function toggleWakeMenu(event) {
    if (event) event.stopPropagation();
    const menu = document.getElementById('wake-menu');
    const btn = document.getElementById('wake-btn');
    if (!menu) return;
    menu.hidden = !menu.hidden;
    if (btn) btn.setAttribute('aria-expanded', menu.hidden ? 'false' : 'true');
}
function closeWakeMenu() {
    const menu = document.getElementById('wake-menu');
    const btn = document.getElementById('wake-btn');
    if (menu) menu.hidden = true;
    if (btn) btn.setAttribute('aria-expanded', 'false');
}
function toggleWake() {
    wakeEnabled = !wakeEnabled;
    try { localStorage.setItem('milusions-wake', wakeEnabled ? 'on' : 'off'); } catch (_) {}
    updateWakeBtn();
}
document.addEventListener('click', (event) => { if (!event.target.closest('.wake-menu-wrap')) closeWakeMenu(); });
document.addEventListener('keydown', (event) => { if (event.key === 'Escape') closeWakeMenu(); });

function syncWake() {
    const should = wakeEnabled && !wakeBlocked && !!wakeRec && !helioOpen;
    if (should) startWake(); else stopWake();
}
function startWake() {
    wakeWanted = true;
    if (wakeRunning || !wakeRec) return;
    try { wakeRec.start(); } catch (_) {}
}
function stopWake() {
    wakeWanted = false;
    if (wakeRestartTimer) { clearTimeout(wakeRestartTimer); wakeRestartTimer = null; }
    if (wakeRunning && wakeRec) { try { wakeRec.abort(); } catch (_) {} }
}
function bindWake() {
    if (!SpeechRecognitionImpl) return;
    wakeRec = new SpeechRecognitionImpl();
    wakeRec.continuous = true;
    wakeRec.interimResults = true;
    wakeRec.lang = BASE_SPEECH_LANG;
    wakeRec.onstart = () => { wakeRunning = true; };
    wakeRec.onend = () => {
        wakeRunning = false;
        if (wakeWanted && !wakeBlocked && !helioOpen) {
            wakeRestartTimer = setTimeout(() => { wakeRestartTimer = null; if (wakeWanted) startWake(); }, wakeRestartDelay);
        }
    };
    wakeRec.onerror = (e) => {
        if (e.error === 'not-allowed' || e.error === 'service-not-allowed') {
            wakeBlocked = true;
            notify('WARNING', 'Microphone access is blocked, so the wake word is off. Helio still works from the Ask Helio button.');
            updateWakeBtn();
        } else if (e.error === 'network') {
            wakeRestartDelay = 5000;
        }
    };
    wakeRec.onresult = (ev) => {
        wakeRestartDelay = 300;
        const re = new RegExp('\\b' + wakeWord + '\\b', 'i');
        for (let i = ev.resultIndex; i < ev.results.length; i++) {
            if (re.test(ev.results[i][0].transcript)) { openVoiceModal(); return; }
        }
    };
}

// =====================================================================
//  INIT
// =====================================================================
(function init() {
    let saved = null;
    try { saved = localStorage.getItem('milusions-theme'); } catch (e) {}
    applyTheme(saved !== 'dark');

    updateWaypointsUI();
    setClickMode('single');
    initMapPanelExtras();
    renderMode();

    initROSBridge();
    startRealSenseWebRTC();

    fetchMode().then(m => { if (m && !pending) setCurrentMode(m); });
    setInterval(async () => {
        if (pending) return;
        const m = await fetchMode();
        if (m) setCurrentMode(m);
    }, 8000);

    bindRecognition();
    bindWake();
    updateWakeBtn();
    PeekController.start();

    requestAnimationFrame(frame);
})();

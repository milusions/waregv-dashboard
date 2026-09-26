// =====================================================================
//  Milusions WareGV Suite - dashboard logic
//  UI (index.html / style.css) + live rover integration:
//    - rosbridge  (/map /odom /tf /cmd_vel /plan, publishes /cmd_vel_joy)
//    - REST API   (navigation, initial pose, waypoints, abort, mode, save map)
//    - Helio      (continuous voice assistant, wake word, local command engine)
//
//  All service endpoints use the authenticated rover host. The dashboard
//  never falls back to the browser hostname or URL endpoint overrides.
// =====================================================================

// =====================================================================
//  CONFIGURATION
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
    window.location.replace('login.html');
    throw new Error('Rover session is missing or expired. Redirecting to login.');
}

const REST_API_BASE = `http://${ROVER_IP}:8000`;
const ROSBRIDGE_WS_URL = `ws://${ROVER_IP}:9090`;
const HELIO_WS_URL = `ws://${ROVER_IP}:8001/ws/helio`;

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
    jointStates: '/joint_states',
    localCostmap: '/local_costmap/costmap',
    globalCostmap: '/global_costmap/costmap',
    joy: '/cmd_vel_joy',
    tf: '/tf',
    tfStatic: '/tf_static',
    nav2Status: '/navigate_to_pose/_action/status',
    btLog: '/behavior_tree_log',
    rosout: '/rosout'
};

// --- Layer visibility, driven by the map legend checkboxes ---
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
    console.log(`[Helio State] Event: ${event}, State: ${state}`, { text, sound_name });
    try {
        await fetch(REST_API_BASE + '/helio/state', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ event, state, text, sound_name })
        });
    } catch (e) {
        console.error('[Helio State] Sync failed', e);
    }
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
        } else {
            notify('WARNING', 'Fullscreen is not supported by this browser.');
        }
    } catch (error) {
        notify('ERROR', 'Could not change fullscreen mode.');
    }
    updateDocumentFullscreenButton();
}

document.addEventListener('fullscreenchange', updateDocumentFullscreenButton);
document.addEventListener('DOMContentLoaded', () => renderNav2Logs());
window.toggleDocumentFullscreen = toggleDocumentFullscreen;

// =====================================================================
//  State
// =====================================================================
let mapImageDirty = false, needsDraw = true;
let latestMap = null;
let latestPlan = [];
let navStatus = 'Idle';            // Idle | Planning | Navigating | Recovering | Reached | Aborted | Failed
let missionGoal = null;            // {x, y} final goal of the running mission
let ignorePlansUntil = 0;
let distanceRemaining = null;

// --- Nav2 fine-grained state (from real Nav2 topics, not guessed) ---
// GoalStatus codes per action_msgs/msg/GoalStatus.
const NAV2_GOAL_STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'
};
let nav2GoalStatusCode = null;     // last GoalStatus string, e.g. 'EXECUTING'
let nav2ActiveNode = null;         // last BT leaf/control node reported RUNNING
let nav2Stage = null;              // human label shown in the map tag, e.g. 'Recovery: Spin'
let nav2StatusMsgAt = 0;           // last time we heard a *real* Nav2 status/BT message
let nav2LastError = null;          // { text, node, level, ts } - most recent rosout WARN/ERROR from a nav2 node
const nav2LogHistory = [];          // stored Nav2 WARN/ERROR messages for the Logs carousel
let nav2LogIndex = -1;
const NAV2_LOG_HISTORY_MAX = 50;
const NAV2_STATUS_FRESH_MS = 4000; // how long a real Nav2 signal is considered authoritative
const NAV2_ERROR_BUBBLE_MS = 12000; // how long the chat-bubble stays up after an error
const NAV2_NODE_RE = /bt_navigator|controller_server|planner_server|recoveries_server|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|costmap/i;
const NAV2_RECOVERY_NODE_RE = /recover|spin|back ?up|wait|clear ?costmap|assisted_teleop/i;
const NAV2_PLAN_NODE_RE = /computepathtopose|compute_path|planner|smoothpath|smooth_path/i;

function nav2StatusFresh() { return Date.now() - nav2StatusMsgAt < NAV2_STATUS_FRESH_MS; }

// Turns a BT node name like "RecoveryNode" / "Spin" / "ComputePathToPose" into
// something readable to sit next to the location arrow.
function nav2StageLabel(nodeName) {
    if (!nodeName) return null;
    if (NAV2_RECOVERY_NODE_RE.test(nodeName)) return 'Recovering: ' + nodeName;
    if (NAV2_PLAN_NODE_RE.test(nodeName)) return 'Planning path';
    if (/followpath|follow_path/i.test(nodeName)) return 'Following path';
    return nodeName;
}

// Reconciles the last GoalStatus + last active BT node into the single
// navStatus the rest of the UI reads, and the human-readable stage tag.
function applyNav2Status() {
    const isRecovery = nav2ActiveNode && NAV2_RECOVERY_NODE_RE.test(nav2ActiveNode);
    nav2Stage = nav2StageLabel(nav2ActiveNode) || (nav2GoalStatusCode ? nav2GoalStatusCode : null);
    switch (nav2GoalStatusCode) {
        case 'ACCEPTED':
        case 'EXECUTING':
            navStatus = isRecovery ? 'Recovering' : (nav2ActiveNode && NAV2_PLAN_NODE_RE.test(nav2ActiveNode) ? 'Planning' : 'Navigating');
            break;
        case 'SUCCEEDED':
            if (navStatus !== 'Reached') {
                navStatus = 'Reached'; missionGoal = null; latestPlan = [];
                ignorePlansUntil = Date.now() + 1500; clearNavLoading();
                notify('INFO', 'Navigation destination reached.');
            }
            nav2Stage = 'Reached';
            break;
        case 'ABORTED':
            if (navStatus !== 'Aborted') {
                navStatus = 'Aborted'; missionGoal = null; clearNavLoading();
                notify('ERROR', 'Nav2 aborted the mission' + (nav2LastError ? ': ' + nav2LastError.text : '.'));
            }
            nav2Stage = 'Aborted' + (nav2LastError ? ': ' + nav2LastError.text : '');
            break;
        case 'CANCELED':
            navStatus = 'Idle'; missionGoal = null; clearNavLoading();
            nav2Stage = 'Canceled';
            break;
        default:
            break;
    }
    needsDraw = true;
}

// rosout Log levels (rcl_interfaces/msg/Log).
const ROSOUT_WARN = 30, ROSOUT_ERROR = 40, ROSOUT_FATAL = 50;
function onRosoutMsg(msg) {
    if (!msg || msg.level < ROSOUT_WARN) return;
    if (!NAV2_NODE_RE.test(msg.name || '')) return;

    const entry = {
        text: String(msg.msg || '').trim(),
        node: String(msg.name || 'Nav2'),
        level: Number(msg.level) >= ROSOUT_FATAL ? 'FATAL' : (Number(msg.level) >= ROSOUT_ERROR ? 'ERROR' : 'WARN'),
        levelCode: Number(msg.level),
        ts: Date.now()
    };
    if (!entry.text) return;

    nav2LastError = entry;
    nav2StatusMsgAt = entry.ts;

    // Avoid filling the carousel with an identical burst from the same node.
    const previous = nav2LogHistory[nav2LogHistory.length - 1];
    if (!previous || previous.text !== entry.text || previous.node !== entry.node || previous.level !== entry.level) {
        nav2LogHistory.push(entry);
        while (nav2LogHistory.length > NAV2_LOG_HISTORY_MAX) nav2LogHistory.shift();
        nav2LogIndex = nav2LogHistory.length - 1;
        renderNav2Logs();
    } else {
        // Refresh the timestamp of the current repeated message.
        previous.ts = entry.ts;
        nav2LogIndex = nav2LogHistory.length - 1;
        renderNav2Logs();
    }

    needsDraw = true;
    if (Number(msg.level) >= ROSOUT_ERROR) notify('ERROR', '[' + msg.name + '] ' + msg.msg);
}

function formatNav2LogTime(ts) {
    try {
        return new Date(ts).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    } catch (_) { return '--:--:--'; }
}

function renderNav2Logs() {
    const card = document.getElementById('nav2-log-card');
    const count = document.getElementById('nav2-log-count');
    const prev = document.getElementById('nav2-log-prev');
    const next = document.getElementById('nav2-log-next');
    if (!card) return;

    const total = nav2LogHistory.length;
    if (count) count.textContent = total + (total === 1 ? ' stored' : ' stored');

    if (!total) {
        card.innerHTML = '<div class="nav2-log-empty">No Nav2 warnings or errors yet.</div>';
        if (prev) prev.disabled = true;
        if (next) next.disabled = true;
        return;
    }

    nav2LogIndex = Math.max(0, Math.min(nav2LogIndex, total - 1));
    const entry = nav2LogHistory[nav2LogIndex];
    const levelClass = String(entry.level || 'WARN').toLowerCase();
    card.innerHTML = `
        <div class="nav2-log-top">
            <span class="nav2-log-level ${levelClass}">${entry.level}</span>
            <span class="nav2-log-position mono">${nav2LogIndex + 1} / ${total}</span>
        </div>
        <div class="nav2-log-node">${escapeHtml(entry.node)}</div>
        <div class="nav2-log-message">${escapeHtml(entry.text)}</div>
        <div class="nav2-log-time mono">${formatNav2LogTime(entry.ts)}</div>`;

    if (prev) prev.disabled = total <= 1;
    if (next) next.disabled = total <= 1;
}

function previousNav2Log() {
    if (!nav2LogHistory.length) return;
    nav2LogIndex = (nav2LogIndex - 1 + nav2LogHistory.length) % nav2LogHistory.length;
    renderNav2Logs();
}

function nextNav2Log() {
    if (!nav2LogHistory.length) return;
    nav2LogIndex = (nav2LogIndex + 1) % nav2LogHistory.length;
    renderNav2Logs();
}

function escapeHtml(value) {
    return String(value ?? '').replace(/[&<>'"]/g, ch => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;'
    }[ch]));
}

window.previousNav2Log = previousNav2Log;
window.nextNav2Log = nextNav2Log;

function onBTLogMsg(msg) {
    const events = (msg && msg.event_log) || [];
    let changed = false;
    events.forEach((e) => {
        if (e.current_status === 'RUNNING') { nav2ActiveNode = e.node_name; changed = true; }
    });
    if (!changed) return;
    nav2StatusMsgAt = Date.now();
    applyNav2Status();
}

function onNav2GoalStatusMsg(msg) {
    const list = (msg && msg.status_list) || [];
    if (!list.length) return;
    const last = list[list.length - 1];
    const code = NAV2_GOAL_STATUS_NAMES[last.status] || null;
    if (!code) return;
    nav2GoalStatusCode = code;
    nav2StatusMsgAt = Date.now();
    applyNav2Status();
}
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
        if (nav2StatusFresh() && (nav2GoalStatusCode === 'EXECUTING' || nav2GoalStatusCode === 'ACCEPTED')) {
            return; // Nav2 itself says the goal is still live - trust it over the /plan heuristic.
        }
        navStatus = 'Aborted';
        missionGoal = null;
        latestPlan = [];
        ignorePlansUntil = Date.now() + 1500;
        needsDraw = true;
        try { await postJSON('/abort'); } catch (e) {}
        notify('ERROR', 'Nav2 did not produce a path within ' + (CFG.planTimeoutMs / 1000) + ' s. Mission aborted.');
    }, CFG.planTimeoutMs);
}

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

// ----- Local / global costmaps (nav_msgs/OccupancyGrid), rendered as toggleable overlays with Foxglove-style colormap -----
let latestLocalCostmap = null, latestGlobalCostmap = null;
let localCostmapDirty = false, globalCostmapDirty = false;
const localCostmapCanvasOff = document.createElement('canvas');
const globalCostmapCanvasOff = document.createElement('canvas');

function getCostmapColor(v) {
    if (v <= 0 || v === 255 || v === -1) return [0, 0, 0, 0];
    if (v >= 100) return [227, 0, 53, 220]; // Lethal obstacle (bright red)
    const t = Math.min(v, 99) / 99;
    let r, g, b;
    if (t < 0.33) {
        const f = t / 0.33;
        r = 0; g = Math.round(200 * f); b = 255;
    } else if (t < 0.66) {
        const f = (t - 0.33) / 0.33;
        r = Math.round(255 * f); g = 200; b = Math.round(255 * (1 - f));
    } else {
        const f = (t - 0.66) / 0.34;
        r = 255; g = Math.round(200 * (1 - f)); b = 0;
    }
    const alpha = Math.round(70 + t * 150);
    return [r, g, b, alpha];
}

function rebuildCostmapImage(costmap, canvasOff) {
    if (!costmap) return;
    canvasOff.width = costmap.w; canvasOff.height = costmap.h;
    const ctx = canvasOff.getContext('2d');
    const img = ctx.createImageData(costmap.w, costmap.h), d = img.data;
    for (let j = 0; j < costmap.h; j++) {
        for (let i = 0; i < costmap.w; i++) {
            const v = costmap.data[j * costmap.w + i], k = (j * costmap.w + i) * 4;
            const [r, g, b, a] = getCostmapColor(v);
            d[k] = r; d[k + 1] = g; d[k + 2] = b; d[k + 3] = a;
        }
    }
    ctx.putImageData(img, 0, 0);
}
function rebuildLocalCostmapImage() { rebuildCostmapImage(latestLocalCostmap, localCostmapCanvasOff); }
function rebuildGlobalCostmapImage() { rebuildCostmapImage(latestGlobalCostmap, globalCostmapCanvasOff); }

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
        clearNavLoading();
        if (navStatus !== 'Navigating') {
            navStatus = 'Navigating';
            if (!missionGoal) {
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
    rosSubscribe(TOPICS.localCostmap, 'nav_msgs/OccupancyGrid', onLocalCostmapMsg, { queue_length: 1 });
    rosSubscribe(TOPICS.globalCostmap, 'nav_msgs/OccupancyGrid', onGlobalCostmapMsg, { queue_length: 1 });
    rosSubscribe(TOPICS.nav2Status, 'action_msgs/GoalStatusArray', onNav2GoalStatusMsg, { queue_length: 1 });
    rosSubscribe(TOPICS.btLog, 'nav2_msgs/BehaviorTreeLog', onBTLogMsg, { queue_length: 5 });
    rosSubscribe(TOPICS.rosout, 'rcl_interfaces/Log', onRosoutMsg, { queue_length: 10 });

    joyTopic = new ROSLIB.Topic({ ros, name: TOPICS.joy, messageType: 'geometry_msgs/Twist' });
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
//  SLAM map canvas & interaction
// =====================================================================
const mapWrap = document.getElementById('map-wrapper');
const mapCanvas = document.getElementById('map-canvas');
const view = { vx: 0, vy: 0, s: 30 };
let follow = true, viewFitted = false;
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

function togglePanelFullscreen(panel, button) {
    const isFullscreen = panel.classList.toggle('panel-fullscreen');
    button.textContent = isFullscreen ? '×' : '⛶';
    button.title = isFullscreen ? 'Exit fullscreen' : 'Fullscreen panel';
    button.setAttribute('aria-label', button.title);
    document.body.classList.toggle('panel-is-fullscreen', isFullscreen);
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
        panel.draggable = false;

        let tools = header.querySelector(':scope > .panel-tools');
        if (!tools) {
            tools = document.createElement('div');
            tools.className = 'panel-tools';
            header.appendChild(tools);
        }
        const dragHandle = document.createElement('button');
        dragHandle.type = 'button';
        dragHandle.className = 'panel-drag-handle';
        dragHandle.textContent = '☰';
        dragHandle.title = 'Press and drag to move panel';
        dragHandle.setAttribute('aria-label', dragHandle.title);
        dragHandle.draggable = true;
        dragHandle.addEventListener('pointerdown', (event) => {
            event.stopPropagation();
            panel.classList.add('panel-move-armed');
            dragHandle.classList.add('active');
        });
        dragHandle.addEventListener('click', (event) => event.stopPropagation());
        dragHandle.addEventListener('dragstart', (event) => {
            if (!panel.classList.contains('panel-move-armed')) {
                event.preventDefault();
                return;
            }
            panel.classList.add('panel-dragging');
            document.querySelector('.workspace').classList.add('is-reordering');
            event.dataTransfer.effectAllowed = 'move';
            event.dataTransfer.setData('text/plain', panel.dataset.panelId);
        });
        dragHandle.addEventListener('dragend', () => {
            panel.classList.remove('panel-dragging');
            panel.classList.remove('panel-move-armed');
            dragHandle.classList.remove('active');
            document.querySelector('.workspace').classList.remove('is-reordering');
            document.querySelectorAll('.panel-drop-target').forEach((item) => item.classList.remove('panel-drop-target'));
        });
        dragHandle.addEventListener('pointerup', () => {
            if (!panel.classList.contains('panel-dragging')) {
                panel.classList.remove('panel-move-armed');
                dragHandle.classList.remove('active');
            }
        });
        header.insertBefore(dragHandle, header.firstChild);

        const fullscreen = document.createElement('button');
        fullscreen.type = 'button';
        fullscreen.className = 'tool-btn panel-action-btn';
        fullscreen.textContent = '⛶';
        fullscreen.title = 'Fullscreen panel';
        fullscreen.setAttribute('aria-label', fullscreen.title);
        fullscreen.draggable = false;
        fullscreen.addEventListener('click', (event) => {
            event.stopPropagation();
            togglePanelFullscreen(panel, fullscreen);
        });
        tools.appendChild(fullscreen);

        panel.addEventListener('dragover', (event) => {
            const dragged = document.querySelector('.panel-dragging');
            if (!dragged || dragged === panel || dragged.parentElement !== panel.parentElement) return;
            event.preventDefault();
            panel.classList.add('panel-drop-target');
            const before = event.clientY < panel.getBoundingClientRect().top + panel.offsetHeight / 2;
            panel.parentElement.insertBefore(dragged, before ? panel : panel.nextSibling);
        });
        panel.addEventListener('dragleave', () => panel.classList.remove('panel-drop-target'));
        panel.addEventListener('drop', (event) => {
            event.preventDefault();
            panel.classList.remove('panel-drop-target');
        });
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

function drawAnimatedInterimPath(ctx, from, to, color) {
    const p1 = w2s(from.x, from.y), p2 = w2s(to.x, to.y);
    const dashLen = 10, gapLen = 7;
    const nowMs = performance.now();
    color = color || '#ffb020';

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.setLineDash([dashLen, gapLen]);
    ctx.lineDashOffset = -((nowMs / 40) % (dashLen + gapLen));
    ctx.beginPath();
    ctx.moveTo(p1[0], p1[1]);
    ctx.lineTo(p2[0], p2[1]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();

    const angle = Math.atan2(p2[1] - p1[1], p2[0] - p1[0]);
    const pulse = 1 + 0.15 * Math.sin(nowMs / 220);
    ctx.save();
    ctx.translate(p2[0], p2[1]);
    ctx.rotate(angle);
    ctx.scale(pulse, pulse);
    ctx.fillStyle = color;
    ctx.strokeStyle = theme['--panel'];
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(16, 0); ctx.lineTo(-8, -9); ctx.lineTo(-8, 9); ctx.closePath();
    ctx.fill(); ctx.stroke();
    ctx.restore();

    ctx.font = '700 11px Roboto, sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    ctx.fillStyle = color;
    ctx.fillText('PLANNING…', p2[0], p2[1] - 16);
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

    if (layerVis.plan && latestPlan && latestPlan.length > 0) {
        drawPathWithArrows(ctx, latestPlan, theme['--primary']);
        const finalPt = latestPlan[latestPlan.length - 1];
        drawArrow(ctx, finalPt.x, finalPt.y, finalPt.yaw, theme['--danger'], 'GOAL', 28);
    } else if (layerVis.plan && navStatus === 'Planning' && missionGoal && robot) {
        drawAnimatedInterimPath(ctx, robot, missionGoal);
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

        drawNav2StageTag(ctx, p[0], p[1], r);
        drawNav2ErrorBubble(ctx, p[0], p[1], r);
    }
}

// Small pill tag next to the location arrow with the live Nav2 stage
// (e.g. "Navigating", "Recovering: Spin", "Aborted"). Drawn upright,
// independent of the robot's heading rotation.
function drawNav2StageTag(ctx, x, y, r) {
    const label = nav2Stage || navStatus;
    if (!label || label === 'Idle') return;
    const tagCol = {
        Navigating: '#38bdf8', Planning: '#facc15', Recovering: '#fb923c',
        Reached: '#30a46c', Aborted: '#e5484d', Failed: '#e5484d', Canceled: '#94a3b8'
    };
    const key = Object.keys(tagCol).find((k) => label.startsWith(k)) || navStatus;
    const color = tagCol[key] || '#94a3b8';

    ctx.save();
    ctx.font = '700 10px Roboto, sans-serif';
    const textW = ctx.measureText(label).width;
    const padX = 6, padY = 3, boxW = textW + padX * 2, boxH = 15;
    const tx = x + r * 2 + 4, ty = y - boxH / 2;

    ctx.fillStyle = 'rgba(15,17,20,.88)';
    ctx.strokeStyle = color; ctx.lineWidth = 1.2;
    roundRectPath(ctx, tx, ty, boxW, boxH, 6);
    ctx.fill(); ctx.stroke();

    ctx.fillStyle = color;
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText(label, tx + padX, ty + boxH / 2 + 0.5);
    ctx.restore();
}

// Speech-bubble showing the exact terminal error/warning text from the
// last relevant Nav2 node (bt_navigator, controller_server, etc.), sourced
// from /rosout, anchored above the robot arrow while it's still fresh.
function drawNav2ErrorBubble(ctx, x, y, r) {
    if (!nav2LastError) return;
    const age = Date.now() - nav2LastError.ts;
    if (age > NAV2_ERROR_BUBBLE_MS) return;

    const maxCharsPerLine = 34;
    const raw = String(nav2LastError.text || '').trim();
    const lines = wrapText(raw, maxCharsPerLine).slice(0, 4);
    const header = '⚠ ' + nav2LastError.node;

    ctx.save();
    ctx.font = '700 10px Roboto, sans-serif';
    const headerW = ctx.measureText(header).width;
    ctx.font = '400 10px Roboto, sans-serif';
    const lineW = Math.max(...lines.map((l) => ctx.measureText(l).width), 0);
    const boxW = Math.min(260, Math.max(headerW, lineW) + 20);
    const lineH = 13;
    const boxH = 20 + lines.length * lineH;
    const bx = x - boxW / 2, by = y - r * 2 - boxH - 14;

    // Fade out over the last 2s of its lifetime.
    const alpha = age > NAV2_ERROR_BUBBLE_MS - 2000 ? Math.max(0, (NAV2_ERROR_BUBBLE_MS - age) / 2000) : 1;
    ctx.globalAlpha = alpha;

    ctx.fillStyle = 'rgba(40,14,16,.95)';
    ctx.strokeStyle = '#e5484d'; ctx.lineWidth = 1.3;
    roundRectPath(ctx, bx, by, boxW, boxH, 8);
    ctx.fill(); ctx.stroke();

    // Pointer tail toward the robot.
    ctx.beginPath();
    ctx.moveTo(x - 7, by + boxH);
    ctx.lineTo(x + 7, by + boxH);
    ctx.lineTo(x, by + boxH + 10);
    ctx.closePath();
    ctx.fillStyle = 'rgba(40,14,16,.95)';
    ctx.fill();
    ctx.strokeStyle = '#e5484d'; ctx.stroke();

    ctx.font = '700 10px Roboto, sans-serif';
    ctx.fillStyle = '#ff8686';
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(header, bx + 10, by + 8);

    ctx.font = '400 10px Roboto, sans-serif';
    ctx.fillStyle = '#f5d3d3';
    lines.forEach((l, i) => ctx.fillText(l, bx + 10, by + 8 + 16 + i * lineH));

    ctx.restore();
}

function roundRectPath(ctx, x, y, w, h, rad) {
    ctx.beginPath();
    ctx.moveTo(x + rad, y);
    ctx.arcTo(x + w, y, x + w, y + h, rad);
    ctx.arcTo(x + w, y + h, x, y + h, rad);
    ctx.arcTo(x, y + h, x, y, rad);
    ctx.arcTo(x, y, x + w, y, rad);
    ctx.closePath();
}

function wrapText(text, maxChars) {
    const words = text.split(/\s+/);
    const lines = [];
    let cur = '';
    words.forEach((w) => {
        if ((cur + ' ' + w).trim().length > maxChars) { if (cur) lines.push(cur); cur = w; }
        else cur = (cur + ' ' + w).trim();
    });
    if (cur) lines.push(cur);
    return lines;
}

function evtPos(evt) { const r = mapCanvas.getBoundingClientRect(); return [evt.clientX - r.left, evt.clientY - r.top]; }

let poseActionPopup = null;

function openPoseActionPopup(pose, evt, waypointIndex = -1) {
    const popup = document.getElementById('map-pose-popup');
    const poseText = document.getElementById('map-pose-popup-pose');
    const singleActions = document.getElementById('map-pose-single-actions');
    const waypointActions = document.getElementById('map-pose-waypoint-actions');
    if (!popup || !pose) return;

    poseActionPopup = { x: pose.x, y: pose.y, yaw: pose.yaw || 0, waypointIndex };
    const isWaypoint = waypointIndex >= 0;
    if (singleActions) singleActions.hidden = isWaypoint;
    if (waypointActions) waypointActions.hidden = !isWaypoint;
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
    activeGoal = { x: w.x, y: w.y, yaw: 0 };
    needsDraw = true;
});
mapWrap.addEventListener('pointermove', (evt) => {
    if (evt.target.closest('#map-pose-popup') || evt.target.closest('#map-legend')) return;
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
    if (localCostmapDirty && latestLocalCostmap) { rebuildLocalCostmapImage(); localCostmapDirty = false; needsDraw = true; }
    if (globalCostmapDirty && latestGlobalCostmap) { rebuildGlobalCostmapImage(); globalCostmapDirty = false; needsDraw = true; }
    if (navStatus === 'Planning' && missionGoal && (!latestPlan || latestPlan.length === 0)) needsDraw = true;
    if (nav2LastError && (Date.now() - nav2LastError.ts) < NAV2_ERROR_BUBBLE_MS) needsDraw = true;
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

let planBadgeEl = null;

function updateStatus() {
    const now = performance.now();
    if (cmdVelAt && now - CFG.cmdVelStaleMs > cmdVelAt && (currentCmdVel.linear || currentCmdVel.angular)) {
        currentCmdVel = { linear: 0, angular: 0 }; needsDraw = true;
    }

    document.getElementById('stat-speed').textContent = (odom ? odom.speed : 0).toFixed(2);

    distanceRemaining = calcDistanceRemaining();
    document.getElementById('stat-distance').textContent =
        (distanceRemaining !== null && (navStatus === 'Navigating' || navStatus === 'Planning')) ? distanceRemaining.toFixed(2) : '—';

    if (navStatus === 'Navigating' && !nav2StatusFresh()) {
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
    const col = { Reached: 'var(--success)', Aborted: 'var(--danger)', Failed: 'var(--danger)', Recovering: 'var(--warning)', Navigating: 'var(--primary)', Planning: 'var(--warning)' }[navStatus];
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
    nav2GoalStatusCode = null; nav2ActiveNode = null; nav2Stage = null; nav2LastError = null;
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
    nav2GoalStatusCode = null; nav2ActiveNode = null; nav2Stage = null; nav2LastError = null;
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
//  Save map
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

function extractMapNames(data) {
    const values = Array.isArray(data) ? data
        : data && (data.maps || data.map_names || data.names || data.items) || [];
    if (!Array.isArray(values)) return [];
    return values.map((item) => {
        if (typeof item === 'string') return item;
        if (item && typeof item === 'object') return item.name || item.map_name || item.filename || '';
        return '';
    }).map((name) => String(name).trim()).filter(Boolean)
        .map((name) => name.replace(/\.ya?ml$/i, '').replace(/\.zip$/i, ''))
        .filter((name, index, list) => list.indexOf(name) === index);
}

async function loadMapNames() {
    const select = document.getElementById('map-name-input');
    if (!select) return;
    const endpoints = ['/maps', '/map/list', '/maps/list'];
    for (const endpoint of endpoints) {
        try {
            const response = await fetch(REST_API_BASE + endpoint, { cache: 'no-store' });
            if (!response.ok) continue;
            const names = extractMapNames(await response.json());
            if (!names.length) continue;
            const current = select.value || 'small_warehouse';
            select.replaceChildren(...names.map((name) => {
                const option = document.createElement('option');
                option.value = name;
                option.textContent = name;
                return option;
            }));
            select.value = names.includes(current) ? current : names[0];
            return;
        } catch (_) {}
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
//  Joystick
// =====================================================================
const joyPad = document.getElementById('joy-pad'), joyKnob = document.getElementById('joy-knob');
const joyMaxVelInput = document.getElementById('joy-max-vel'), joyMaxAngInput = document.getElementById('joy-max-ang');
const joy = { x: 0, y: 0, active: false, timer: null };
let joyEnabled = false;

function clamp(val, lo, hi) { return Math.min(hi, Math.max(lo, val)); }

// Max linear/angular speed come from the user-editable fields; clamped to a
// sane non-negative range so a bad/blank input can't send an unbounded command.
function getJoyMaxVel() {
    const v = parseFloat(joyMaxVelInput && joyMaxVelInput.value);
    return clamp(Number.isFinite(v) ? v : 0, 0, 5);
}
function getJoyMaxAng() {
    const v = parseFloat(joyMaxAngInput && joyMaxAngInput.value);
    return clamp(Number.isFinite(v) ? v : 0, 0, 5);
}

function publishJoyRaw(turn, fwd) {
    if (!joyTopic || !rosConnected) return false;
    const maxVel = getJoyMaxVel(), maxAng = getJoyMaxAng();
    // turn/fwd are normalized joystick axes in [-1, 1]; scale by the user's
    // max speeds and clip in-browser before publishing to /cmd_vel_joy.
    const linX = clamp(fwd, -1, 1) * maxVel;
    const angZ = clamp(turn, -1, 1) * maxAng;
    joyTopic.publish(new ROSLIB.Message({
        linear: { x: linX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angZ }
    }));
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
    publishJoy(); setTimeout(publishJoy, 50); setTimeout(publishJoy, 100);
}
joyPad.addEventListener('pointerup', joyEnd);
joyPad.addEventListener('pointercancel', joyEnd);
joyPad.addEventListener('lostpointercapture', joyEnd);
window.addEventListener('blur', joyEnd);
document.addEventListener('visibilitychange', () => { if (document.hidden) joyEnd(); });

// =====================================================================
//  MAP VIEWER
// =====================================================================
const PGM_FREE = 254, PGM_UNKNOWN = 205, PGM_OCC = 0;
const OCC_THRESH = 65, FREE_THRESH = 25;

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
            gray[(m.h - 1 - j) * m.w + i] = g;
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
        pos++;
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

    const pxPerM = mv.s / mv.res;
    const nice = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100].find(n => n * pxPerM >= 70) || 100;
    const len = nice * pxPerM, bx = 14, by = H - 16;
    ctx.fillStyle = 'rgba(20,22,26,.8)'; ctx.fillRect(bx - 6, by - 20, len + 12, 28);
    ctx.strokeStyle = '#fff'; ctx.fillStyle = '#fff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by); ctx.lineTo(bx + len, by); ctx.lineTo(bx + len, by - 4); ctx.stroke();
    ctx.font = '600 11px Roboto, sans-serif'; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText(nice + ' m', bx, by - 6);
}

function openMapViewer() {
    mvEnsureInit();
    mv.root.hidden = false;
    mv.open = true;
    mv._fitted = false;
    $mv('mv-measure-btn').classList.toggle('on', mv.measure);
    mv.canvas.style.cursor = mv.measure ? 'crosshair' : 'grab';
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
function playListenSound() { tone('sine', 587.33, 880, 0.12, 0.2); }
function playProcessingSound() {
    tone('triangle', 500, 640, 0.08, 0.09);
    notifyHelioState('sound_play', 'thinking', '', 'thinking_blip');
}
function playErrorSound() {
    tone('sawtooth', 300, 140, 0.22, 0.4, 'exponentialRampToValueAtTime');
    notifyHelioState('sound_play', agentState, '', 'error_buzz');
}

const MicLevels = {
    stream: null, analyser: null, data: null, active: false,
    async start() {
        if (this.active) return;
        try {
            initAudio();
            this.stream = await navigator.mediaDevices.getUserMedia({ audio: true });
        } catch (_) { return; }
        const src = audioCtx.createMediaStreamSource(this.stream);
        this.analyser = audioCtx.createAnalyser();
        this.analyser.fftSize = 256;
        this.analyser.smoothingTimeConstant = 0.85;
        src.connect(this.analyser);
        this.data = new Uint8Array(this.analyser.frequencyBinCount);
        this.active = true;
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
        if (this.stream) { this.stream.getTracks().forEach((t) => t.stop()); this.stream = null; }
    }
};

const EyeMotion = {
    raf: null,
    gazeX: 0, gazeY: 0, targetX: 0, targetY: 0,
    nextGazeAt: 0, nextSmileAt: 0,
    start() {
        this.stop();
        const now = performance.now();
        this.gazeX = this.gazeY = this.targetX = this.targetY = 0;
        this.nextGazeAt = now + 1800;
        this.nextSmileAt = now + 5000 + Math.random() * 4000;
        this.loop();
    },
    stop() {
        if (this.raf) cancelAnimationFrame(this.raf);
        this.raf = null;
        if (robotFace) {
            robotFace.querySelectorAll('.eye').forEach((e) => { e.style.transform = ''; });
            robotFace.classList.remove('happy');
        }
    },
    loop() {
        if (!helioOpen || agentState !== 'listening') { this.raf = null; return; }
        const now = performance.now();
        if (now > this.nextGazeAt) {
            this.targetX = (Math.random() * 2 - 1) * 26;
            this.targetY = (Math.random() * 2 - 1) * 12;
            this.nextGazeAt = now + 3800 + Math.random() * 4200;
        }
        if (now > this.nextSmileAt && robotFace) {
            robotFace.classList.add('happy');
            setTimeout(() => { if (robotFace) robotFace.classList.remove('happy'); }, 1800 + Math.random() * 1400);
            this.nextSmileAt = now + 7000 + Math.random() * 7000;
        }
        this.gazeX += (this.targetX - this.gazeX) * 0.012;
        this.gazeY += (this.targetY - this.gazeY) * 0.012;
        const level = MicLevels.level();
        if (robotFace) {
            const scale = 1 + level * 0.14;
            robotFace.querySelectorAll('.eye').forEach((eye) => {
                eye.style.transform = `translate(${this.gazeX.toFixed(1)}px, ${this.gazeY.toFixed(1)}px) scaleY(${scale.toFixed(3)})`;
            });
        }
        this.raf = requestAnimationFrame(() => this.loop());
    }
};

const TalkAnimator = {
    timer: null,
    start() {
        this.stop();
        this.timer = setInterval(() => {
            if (!robotFace || agentState !== 'speaking') return;
            robotFace.querySelectorAll('.eye').forEach((eye, i) => {
                const s = 0.6 + Math.random() * 0.5;
                const lift = (Math.random() * 2 - 1) * 3;
                eye.style.transform = `scaleY(${s.toFixed(2)}) translateY(${lift.toFixed(1)}px)`;
            });
        }, 170);
    },
    stop() {
        if (this.timer) clearInterval(this.timer);
        this.timer = null;
        if (robotFace) robotFace.querySelectorAll('.eye').forEach((e) => { e.style.transform = ''; });
    }
};

let blinkTimer = null;
function scheduleBlink() {
    if (blinkTimer) clearTimeout(blinkTimer);
    blinkTimer = setTimeout(() => {
        if (helioOpen && robotFace && agentState !== 'thinking') {
            robotFace.classList.remove('blink'); void robotFace.offsetWidth; robotFace.classList.add('blink');
        }
        scheduleBlink();
    }, 3200 + Math.random() * 3200);
}

// =====================================================================
//  HELIO VOICE ASSISTANT
// =====================================================================
const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
const BASE_SPEECH_LANG = (navigator.language || '').toLowerCase().startsWith('en') ? navigator.language : 'en-US';
const SILENCE_COMMIT_MS = 1500;
const LOW_CONFIDENCE_GRACE_MS = 800;
const CONFIDENCE_THRESHOLD = 0.6;
const HELIO_TIMEOUT_MS = 20000;

const modal = document.getElementById('voice-modal');
const robotFace = document.getElementById('robot-face');
const robotScreen = document.getElementById('robot-screen');
const helioDots = document.getElementById('helio-dots');
const historyListEl = document.getElementById('history-list');

let recognition = null;
let recognitionRestartTimer = null;
let recognitionRestartDelay = 400;
let recognitionFlapCount = 0, recognitionFlapWindowStart = 0, recognitionCoolingDown = false;
const RECOGNITION_FLAP_LIMIT = 6, RECOGNITION_FLAP_WINDOW_MS = 10000, RECOGNITION_COOLDOWN_MS = 6000;
let silenceCommitTimer = null;
let lowConfidenceTimer = null;
let transcriptText = '';
let interimText = '';
let confidenceSum = 0;
let confidenceCount = 0;
let speechActive = false;
let helioOpen = false;
let agentState = 'idle';
let currentCallId = null;
let selectedLanguage = (navigator.language || '').toLowerCase().startsWith('hi') ? 'hi' : 'en';
let wakeLanguageOverride = null;
let thinkingTimer = null;
let commandSeq = 0;
let lastSpeechNetworkWarn = 0;
window.currentUtterance = null;

const speechLang = () => selectedLanguage === 'hi' ? 'hi-IN' : BASE_SPEECH_LANG;

function detectSpeechLanguage(text) {
    const value = String(text || '');
    if (/\p{Script=Devanagari}/u.test(value)) return 'hi';
    if (/[A-Za-z]/.test(value)) return 'en';
    return selectedLanguage;
}

function applyDetectedLanguage(text) {
    if (wakeLanguageOverride) {
        selectedLanguage = wakeLanguageOverride;
        if (recognition) recognition.lang = speechLang();
        return;
    }
    const detected = detectSpeechLanguage(text);
    if (detected === selectedLanguage) return;
    selectedLanguage = detected;
    if (recognition) recognition.lang = speechLang();
}

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

const HelioIdleController = {
    bubbleTimer: null,
    phrases: ['I am here when you need me.', 'Checking the rover state...', 'Ready for your next command.'],
    start() {
        this.stop();
        this.bubbleTimer = setInterval(() => this.thought(), 9000);
    },
    stop() {
        if (this.bubbleTimer) clearInterval(this.bubbleTimer);
        this.bubbleTimer = null;
        const bubble = document.getElementById('helio-idle-bubble');
        if (bubble) bubble.classList.remove('show');
    },
    thought() {
        if (!helioOpen || agentState !== 'listening' || speechActive) return;
        const bubble = document.getElementById('helio-idle-bubble');
        if (!bubble) return;
        bubble.textContent = this.phrases[Math.floor(Math.random() * this.phrases.length)];
        bubble.classList.add('show');
        setTimeout(() => bubble.classList.remove('show'), 3200);
    }
};

function setVoiceStatus() {}
function renderTranscript() {}

function setFace(state) {
    if (robotFace) robotFace.className = 'robot-face ' + state;
    if (helioDots) helioDots.classList.toggle('show', state === 'thinking');
    if (state === 'listening') { HelioIdleController.start(); MicLevels.start(); EyeMotion.start(); }
    else { HelioIdleController.stop(); MicLevels.stop(); EyeMotion.stop(); }
    if (state === 'speaking') TalkAnimator.start(); else TalkAnimator.stop();
}

function setTeleprompter() {}

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
    notifyHelioState('thinking_start', 'thinking', '', '');
    playProcessingSound();
    thinkingTimer = setInterval(() => playProcessingSound(), 900);
}
function stopThinkingIndicator() {
    if (thinkingTimer) clearInterval(thinkingTimer);
    thinkingTimer = null;
    notifyHelioState('thinking_stop', 'idle', '', '');
}

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
    recognitionStop();
    interimText = ''; transcriptText = '';
    confidenceSum = 0; confidenceCount = 0;
    
    notifyHelioState('listening_stop', 'thinking', '', '');
    notifyHelioState('input_received', 'thinking', text, '');
    
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
        notifyHelioState('listening_start', 'listening', '', '');
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        setFace('listening');
    };

    recognition.onresult = (event) => {
        if (!helioOpen || agentState !== 'listening') return;
        recognitionRestartDelay = 400;
        recognitionFlapCount = 0; recognitionFlapWindowStart = Date.now();
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
        notifyHelioState('listening_update', 'listening', pendingText(), '');
        
        applyDetectedLanguage([transcriptText, interimText].join(' '));
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
                notify('ERROR', 'Speech recognition needs an internet connection.');
            }
        }
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
    };

    recognition.onend = () => {
        speechActive = false;
        if (!helioOpen || agentState !== 'listening') return;

        const now = Date.now();
        if (now - recognitionFlapWindowStart > RECOGNITION_FLAP_WINDOW_MS) { recognitionFlapWindowStart = now; recognitionFlapCount = 0; }
        recognitionFlapCount++;
        if (recognitionFlapCount > RECOGNITION_FLAP_LIMIT) {
            if (!recognitionCoolingDown) {
                recognitionCoolingDown = true;
                setVoiceStatus('STABILIZING MICROPHONE...');
            }
            recognitionRestartTimer = setTimeout(() => {
                recognitionRestartTimer = null; recognitionCoolingDown = false;
                recognitionFlapCount = 0; recognitionFlapWindowStart = Date.now();
                scheduleRecognitionRestart();
            }, RECOGNITION_COOLDOWN_MS);
            return;
        }
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

function openVoiceModal() {
    if (!SpeechRecognitionImpl) {
        alert('Continuous speech recognition is not supported in this browser. Use Chrome or Edge.');
        return;
    }
    activateHelio();
}

function activateHelio() {
    stopWake();
    helioOpen = true;
    notifyHelioState('helio_on', 'waking', '', 'helio_wake');
    
    if (modal) { modal.classList.add('active'); modal.setAttribute('aria-hidden', 'false'); }
    const hist = document.getElementById('modal-history-column'); if (hist) hist.classList.remove('show');
    PeekController.stop();

    transcriptText = ''; interimText = ''; confidenceSum = 0; confidenceCount = 0;
    clearVoiceTimers();
    commandSeq += 1;
    const seq = commandSeq;
    currentCallId = currentCallId || String(Date.now());
    recognition.lang = speechLang();

    agentState = 'speaking';
    if (robotFace) robotFace.className = 'robot-face waking';
    speakText('What?', { pitch: 1.0, rate: 0.72, volume: 1 }).then(() => {
        if (!helioOpen || seq !== commandSeq) return;
        resumeListening();
        scheduleBlink();
    });
}

function closeVoiceModal() {
    helioOpen = false;
    agentState = 'idle';
    speechActive = false;
    notifyHelioState('helio_off', 'idle', '', '');
    
    commandSeq += 1;
    clearVoiceTimers();
    recognitionStop();
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    stopThinkingIndicator();
    MicLevels.stop();
    TalkAnimator.stop();
    EyeMotion.stop();
    if (blinkTimer) { clearTimeout(blinkTimer); blinkTimer = null; }
    if (modal) { modal.classList.remove('active'); modal.setAttribute('aria-hidden', 'true'); }
    transcriptText = ''; interimText = ''; confidenceSum = 0; confidenceCount = 0;
    PeekController.start();
    syncWake();
}

function cleanForSpeech(t) {
    return String(t || '')
        .replace(/```[\s\S]*?```/g, ' ')
        .replace(/!?\[([^\]]*)\]\([^)]*\)/g, '$1')
        .replace(/[*_`#>~]+/g, '')
        .replace(/\s+/g, ' ')
        .trim();
}

let cachedVoice = null, cachedVoiceLang = null;
function pickVoice(lang) {
    if (!('speechSynthesis' in window)) return null;
    const voices = window.speechSynthesis.getVoices() || [];
    if (!voices.length) return null;
    if (cachedVoice && cachedVoiceLang === lang && voices.includes(cachedVoice)) return cachedVoice;
    const base = lang.slice(0, 2);
    const wanted = voices.filter((v) => v.lang && v.lang.toLowerCase().startsWith(base));
    const pool = wanted.length ? wanted : voices;
    const preferred = pool.find((v) => /natural|enhanced|premium|neural/i.test(v.name)) || pool[0];
    cachedVoice = preferred; cachedVoiceLang = lang;
    return preferred;
}

function speakText(text, opts) {
    opts = opts || {};
    const spoken = cleanForSpeech(text);
    if (!('speechSynthesis' in window) || !spoken) return Promise.resolve();
    applyDetectedLanguage(spoken);
    try { window.speechSynthesis.cancel(); } catch (_) {}

    const chunks = opts.pitch != null
        ? [spoken]
        : spoken.split(/(?<=[.!?])\s+/).filter(Boolean);
    if (!chunks.length) chunks.push(spoken);

    const lang = speechLang();
    const voice = pickVoice(lang);

    return chunks.reduce((chain, chunk, i) => chain.then(() => new Promise((resolve) => {
        if (!helioOpen && !opts.forceIntro) { resolve(); return; }
        
        notifyHelioState('speaking_start', 'speaking', chunk, '');
        
        let finished = false, wd = null;
        const finish = () => { 
            if (finished) return; 
            finished = true; 
            clearTimeout(wd); 
            notifyHelioState('speaking_end', 'idle', chunk, '');
            resolve(); 
        };
        
        const u = new SpeechSynthesisUtterance(chunk);
        u.lang = lang;
        if (voice) u.voice = voice;
        const isQuestion = /\?\s*$/.test(chunk);
        const isExclaim = /!\s*$/.test(chunk);
        u.pitch = opts.pitch != null ? opts.pitch
            : (isExclaim ? 1.18 : isQuestion ? 1.12 : 1.0) + (Math.random() * 0.08 - 0.04);
        u.rate = opts.rate != null ? opts.rate : 1.0 + (Math.random() * 0.08 - 0.04);
        u.volume = opts.volume != null ? opts.volume : 1;
        u.onend = finish;
        u.onerror = finish;
        wd = setTimeout(finish, 2500 + chunk.length * 110);
        window.currentUtterance = u;
        window.speechSynthesis.speak(u);
    })), Promise.resolve());
}

async function handleCommand(text) {
    const seq = ++commandSeq;
    const alive = () => helioOpen && seq === commandSeq;

    agentState = 'thinking';
    setVoiceStatus();
    setFace('thinking');
    startThinkingIndicator();

    const reply = await queryHelio(text)
        .catch(() => ({ text: "Uh oh! I lost my intelligence, could you ask me later?", failed: true }));
    stopThinkingIndicator();
    if (!alive()) return;

    const answer = (reply && reply.text) || 'Done.';
    if (reply && reply.failed) playErrorSound();
    
    notifyHelioState('output_generated', 'speaking', answer, '');
    appendHistory('agent', answer);
    
    agentState = 'speaking';
    setVoiceStatus();
    setFace('speaking');
    await speakText(answer);
    if (!alive()) return;
    resumeListening();
}

window.toggleHistory = function () {
    const el = document.getElementById('modal-history-column'); if (el) el.classList.toggle('show');
};
window.closeHistory = function () {
    const el = document.getElementById('modal-history-column'); if (el) el.classList.remove('show');
};

// =====================================================================
//  HELIO BRAIN
// =====================================================================
let helioBackendDownUntil = 0;
let helioSocket = null;
let helioSocketConnectPromise = null;
let helioPendingRequest = null;

function closeHelioSocket(reason) {
    const socket = helioSocket;
    helioSocket = null;
    helioSocketConnectPromise = null;
    if (socket && socket.readyState < WebSocket.CLOSING) socket.close(1000, reason || 'closed');
}

function connectHelioSocket() {
    if (typeof WebSocket === 'undefined') return Promise.reject(new Error('WebSocket is unavailable.'));
    if (helioSocket && helioSocket.readyState === WebSocket.OPEN) return Promise.resolve(helioSocket);
    if (helioSocketConnectPromise) return helioSocketConnectPromise;

    helioSocketConnectPromise = new Promise((resolve, reject) => {
        let settled = false;
        const socket = new WebSocket(HELIO_WS_URL);
        helioSocket = socket;
        const timeout = setTimeout(() => {
            if (settled) return;
            settled = true;
            try { socket.close(); } catch (_) {}
            reject(new Error('Helio WebSocket connection timed out.'));
        }, HELIO_TIMEOUT_MS);

        socket.onopen = () => {
            if (settled) return;
            settled = true;
            clearTimeout(timeout);
            resolve(socket);
        };
        socket.onmessage = (event) => {
            let data;
            try { data = JSON.parse(event.data); } catch (_) { return; }
            if (data.conversation_id) currentCallId = data.conversation_id;
            if (data.type === 'final' && helioPendingRequest) {
                const request = helioPendingRequest;
                helioPendingRequest = null;
                clearTimeout(request.timer);
                request.resolve({ text: data.output || data.message || '', source: 'websocket' });
            } else if (data.type === 'error' && helioPendingRequest) {
                const request = helioPendingRequest;
                helioPendingRequest = null;
                clearTimeout(request.timer);
                request.reject(new Error(data.message || 'Helio agent error.'));
            }
        };
        socket.onerror = () => {
            if (!settled) {
                settled = true;
                clearTimeout(timeout);
                reject(new Error('Helio WebSocket connection failed.'));
            }
        };
        socket.onclose = () => {
            clearTimeout(timeout);
            if (helioSocket === socket) {
                helioSocket = null;
                helioSocketConnectPromise = null;
            }
            if (helioPendingRequest) {
                const request = helioPendingRequest;
                helioPendingRequest = null;
                clearTimeout(request.timer);
                request.reject(new Error('Helio WebSocket disconnected.'));
            }
        };
    }).finally(() => {
        helioSocketConnectPromise = null;
    });
    return helioSocketConnectPromise;
}

async function queryHelio(text) {
    if (Date.now() < helioBackendDownUntil) return localFallback(text);
    try {
        const socket = await connectHelioSocket();
        if (helioPendingRequest) throw new Error('Helio is already processing a request.');

        const result = new Promise((resolve, reject) => {
            const request = { resolve, reject, timer: null };
            request.timer = setTimeout(() => {
                if (helioPendingRequest === request) {
                    helioPendingRequest = null;
                    reject(new Error('Helio response timed out.'));
                }
            }, HELIO_TIMEOUT_MS);
            helioPendingRequest = request;
        });
        socket.send(JSON.stringify({ type: 'message', message: text }));
        return await result;
    } catch (error) {
        helioBackendDownUntil = Date.now() + 60000;
        closeHelioSocket('request failed');
        return localFallback(text);
    }
}

const UNRECOGNIZED_LOCAL_REPLY = "Sorry, I didn't understand that command. Try: go to x 2 y 3, follow waypoints, abort, save map, or status.";
async function localFallback(text) {
    const reply = await localAssistant(text);
    if (reply && reply.text === UNRECOGNIZED_LOCAL_REPLY) {
        return { text: 'Uh oh! I lost my intelligence, could you ask me later?', failed: true };
    }
    return reply;
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
    return { text: UNRECOGNIZED_LOCAL_REPLY };
}

// =====================================================================
//  Wake word
// =====================================================================
const WAKE_OPTIONS = { robot: ['robot'], jojo: ['jojo'] };
let wakeWord = 'robot';
let wakeEnabled = true;
let wakeBlocked = false;
let wakeRec = null, wakeRunning = false, wakeWanted = false, wakeRestartTimer = null, wakeRestartDelay = 800;
let wakeFlapCount = 0, wakeFlapWindowStart = 0, wakeCoolingDown = false;
const WAKE_FLAP_LIMIT = 6, WAKE_FLAP_WINDOW_MS = 10000, WAKE_COOLDOWN_MS = 8000;
try {
    const savedWord = localStorage.getItem('milusions-wake-word');
    const savedState = localStorage.getItem('milusions-wake');
    if (savedWord && WAKE_OPTIONS[savedWord]) {
        wakeWord = savedWord;
        wakeLanguageOverride = savedWord === 'jojo' ? 'hi' : 'en';
        selectedLanguage = wakeLanguageOverride;
    }
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
        button.textContent = key === 'jojo' ? 'Hey JoJo' : 'Hey Robot';
        button.onclick = () => {
            wakeWord = key;
            wakeLanguageOverride = key === 'jojo' ? 'hi' : 'en';
            selectedLanguage = wakeLanguageOverride;
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
    if (label) label.textContent = wakeWord === 'jojo' ? 'Hey JoJo' : 'Hey Robot';
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
        if (!wakeWanted || wakeBlocked || helioOpen) return;

        // The browser's continuous mode naturally ends every so often even
        // with nothing wrong; restarting too eagerly makes the mic indicator
        // rapidly flicker on/off. Track how often that happens and, if it's
        // flapping, back off for a stable cooldown instead of retrying instantly.
        const now = Date.now();
        if (now - wakeFlapWindowStart > WAKE_FLAP_WINDOW_MS) { wakeFlapWindowStart = now; wakeFlapCount = 0; }
        wakeFlapCount++;
        if (wakeFlapCount > WAKE_FLAP_LIMIT) {
            if (!wakeCoolingDown) {
                wakeCoolingDown = true;
                notify('WARNING', 'Wake mic was cycling too fast, stabilizing it - listening will resume shortly.');
            }
            wakeRestartTimer = setTimeout(() => {
                wakeRestartTimer = null; wakeCoolingDown = false; wakeFlapCount = 0; wakeFlapWindowStart = Date.now();
                if (wakeWanted) startWake();
            }, WAKE_COOLDOWN_MS);
            return;
        }
        wakeRestartTimer = setTimeout(() => { wakeRestartTimer = null; if (wakeWanted) startWake(); }, wakeRestartDelay);
    };
    wakeRec.onerror = (e) => {
        if (e.error === 'not-allowed' || e.error === 'service-not-allowed') {
            wakeBlocked = true;
            notify('WARNING', 'Microphone access is blocked, so the wake word is off.');
            updateWakeBtn();
        } else if (e.error === 'network') {
            wakeRestartDelay = 5000;
        }
    };
    wakeRec.onresult = (ev) => {
        wakeRestartDelay = 800;
        wakeFlapCount = 0; wakeFlapWindowStart = Date.now();
        for (let i = ev.resultIndex; i < ev.results.length; i++) {
            const heard = ev.results[i][0].transcript || '';
            const re = new RegExp('\\bhey\\s+' + wakeWord + '\\b', 'i');
            if (re.test(heard)) {
                wakeLanguageOverride = wakeWord === 'jojo' ? 'hi' : 'en';
                selectedLanguage = wakeLanguageOverride;
                if (recognition) recognition.lang = speechLang();
                if (SpeechRecognitionImpl) activateHelio();
                return;
            }
        }
    };
}

(function init() {
    let saved = null;
    try { saved = localStorage.getItem('milusions-theme'); } catch (e) {}
    applyTheme(saved !== 'dark');

    setupPanelUtilities();
    const followButton = document.getElementById('follow-btn');
    if (followButton) followButton.classList.toggle('on', follow);
    loadMapNames();

    updateWaypointsUI();
    setClickMode('single');
    initMapPanelExtras();
    renderMode();

    initROSBridge();

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
// =====================================================================
//  CONFIGURATION
// =====================================================================
const CFG = {
    chartWindowSec: 30,
    joyRateHz: 20
};

const MODE_LABELS = { auto_nav: 'Autonomous Navigation and Driving' };

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
let odom = null;
let robot = null;
let currentCmdVel = { linear: 0, angular: 0 };

let activeNavBtn = null;
let navInitTimeout = null;

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
        notify('ERROR', 'Mock watchdog timeout. Navigation server aborted.');
    }, 20000);
}

const wheelIds = ['fl', 'fr', 'bl', 'br'];
const series = {};
wheelIds.forEach(id => series[id] = { a: [], c: [] });

// =====================================================================
//  Mock API Hooks
// =====================================================================

function formatSigned(v, digits = 2) {
    const n = Number(v);
    if (!Number.isFinite(n)) return '—';
    if (Math.abs(n) < 0.0005) return '0.' + '0'.repeat(digits);
    return (n > 0 ? '+' : '') + n.toFixed(digits);
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
    ctx.strokeStyle = theme['--panel']; ctx.lineWidth = 1.5; stroke();
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

let lastStatus = 0;
function frame(ts) {
    if (mapImageDirty && latestMap) {
        rebuildMapImage(); mapImageDirty = false;
        if (!viewFitted) fitMapView();
        needsDraw = true;
    }
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
    updateWaypointsUI(); needsDraw = true;
}

function yawDegToQuaternion(yawDeg) {
    const r = (yawDeg * Math.PI) / 180;
    return { yaw_z: Math.sin(r / 2), yaw_w: Math.cos(r / 2) };
}

// MOCK API HOOKS
async function postJSON(url, body) {
    console.log('[Mock API Call] POST', url, body);
    return { ok: true };
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
    try {
        await postJSON('/navigate_to_pose', { x: t.x, y: t.y, yaw_deg: t.yawDeg });
        navStatus = 'Navigating';
        notify('INFO', 'Mock Goal sent to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) {
        clearNavLoading();
        notify('ERROR', 'Failed: ' + e.message);
    }
}
async function sendInitialPose() {
    const t = readTarget();
    try {
        await postJSON('/set_initial_pose', { x: t.x, y: t.y, yaw_deg: t.yawDeg });
        notify('INFO', 'Mock Initial pose set to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) { notify('ERROR', 'Failed: ' + e.message); }
}
async function sendWaypoints() {
    if (waypointsData.length === 0) return notify('WARNING', 'No waypoints added. Click the map to add some.');
    const btn = document.getElementById('follow-wp-btn');
    setNavLoading(btn);
    startNavWatchdog();
    const waypoints = waypointsData.map(wp => ({ x: wp.x, y: wp.y, yaw_deg: wp.yaw }));
    try {
        await postJSON('/follow_waypoints', { waypoints });
        navStatus = 'Navigating';
        notify('INFO', 'Mock Following ' + waypoints.length + ' waypoint(s).');
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
        notify('WARNING', 'Mock Mission abort requested.');
    }
    catch (e) { notify('ERROR', 'Failed: ' + e.message); }
}

async function saveCurrentMap() {
    const mapName = document.getElementById('map-name-input').value || 'map';
    notify('INFO', `Saving "${mapName}" map (PGM & YAML) and starting zip download...`);
    
    const dummyZipData = 'data:application/zip;base64,UEsFBgAAAAAAAAAAAAAAAAAAAAAAAA==';
    const newTab = window.open('', '_blank');
    if (newTab) {
        newTab.document.write(`
            <html><head><title>Downloading Map...</title></head>
            <body style="font-family: sans-serif; text-align: center; padding: 40px;">
                <h3>Downloading package: ${mapName}.zip</h3>
                <p>Includes PGM and YAML files.</p>
                <script>
                    const a = document.createElement('a');
                    a.href = '${dummyZipData}';
                    a.download = '${mapName}.zip';
                    document.body.appendChild(a);
                    a.click();
                    setTimeout(() => window.close(), 3500); // Closes tab after download initiates
                </script>
            </body></html>
        `);
    }
}

let currentMode = null, pending = null, modeSelectTouched = false;

async function fetchMode() {
    console.log('[Mock API Call] Fetching mode');
    return 'auto_nav';
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
}

function setCurrentMode(m) {
    currentMode = m;
    if (m && !modeSelectTouched && !pending) document.getElementById('sys-mode-select').value = m;
    renderMode();
}

function clearPending() {
    if (pending) clearTimeout(pending.timer);
    pending = null; renderMode();
}

async function applySystemMode() {
    if (pending) return;
    const mode = document.getElementById('sys-mode-select').value;
    const mapName = document.getElementById('map-name-input').value || 'small_warehouse';
    
    setCurrentMode(mode);
    notify('INFO', `Mock switched mode to ${MODE_LABELS[mode]} for map: ${mapName}`);
    await postJSON('/system/mode', { mode, map_name: mapName });
}

const joyPad = document.getElementById('joy-pad'), joyKnob = document.getElementById('joy-knob');
const joy = { x: 0, y: 0, active: false, timer: null };

// JOYSTICK ENABLE STATE
let joyEnabled = false;
window.toggleJoyEnable = function() {
    joyEnabled = !joyEnabled;
    const btn = document.getElementById('joy-enable-btn');
    btn.textContent = joyEnabled ? 'Disable' : 'Enable';
    btn.classList.toggle('btn-primary', joyEnabled);
    btn.classList.toggle('btn-secondary', !joyEnabled);
    notify('INFO', joyEnabled ? 'Joystick is now Enabled' : 'Joystick is now Disabled');
};

function publishJoy() {
    if (!joyEnabled) return;
    console.log(`[Mock Joy] Publishing x: ${joy.x.toFixed(2)}, y: ${joy.y.toFixed(2)}`);
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
const joyEnd = () => {
    if (!joy.active) return;
    joy.active = false; joyPad.classList.remove('active');
    clearInterval(joy.timer);
    joy.x = 0; joy.y = 0; joyShow();
    publishJoy(); setTimeout(publishJoy, 50); setTimeout(publishJoy, 100);
};
joyPad.addEventListener('pointerup', joyEnd);
joyPad.addEventListener('pointercancel', joyEnd);

(function init() {
    let saved = null;
    try { saved = localStorage.getItem('milusions-theme'); } catch (e) {}
    applyTheme(saved !== 'dark');
    
    // Setup Mock Mode on start
    fetchMode().then(m => {
        if (m) setCurrentMode(m);
    });
    requestAnimationFrame(frame);
})();

// ---- Web Audio API - Emo Sound FX ----
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

// =====================================================================
//  HELIO VOICE ASSISTANT
//  Continuous transcription with deliberate silence-based submission.
//  Speech-recognition segment boundaries NEVER submit a command.
// =====================================================================
const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
const SPEECH_LANG = (navigator.language || '').toLowerCase().startsWith('en') ? navigator.language : 'en-US';
const SILENCE_COMMIT_MS = 3600;       // long silence before auto-submit
const LOW_CONFIDENCE_GRACE_MS = 3200; // extra wait for uncertain recognition
const CONFIDENCE_THRESHOLD = 0.68;

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
let selectedLanguage = 'en';
let thinkingTimer = null;
let thinkingStageIndex = 0;
window.currentUtterance = null;

const THINKING_STAGES = ['Processing command', 'Working...', 'Preparing response'];

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
        this.timer = null;
        this.hideTimer = null;
        document.getElementById('helio-peek')?.classList.remove('peeking');
        document.getElementById('peek-bubble')?.classList.remove('show');
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

function setVoiceStatus(text, live = false) {
    if (!captionStatus) return;
    captionStatus.textContent = text;
    captionStatus.classList.toggle('live', live);
}

function renderTranscript(empty = 'Speak whenever you are ready...') {
    if (!captionText) return;
    const value = [transcriptText.trim(), interimText.trim()].filter(Boolean).join(' ');
    captionText.textContent = value || empty;
    captionText.scrollTop = captionText.scrollHeight;
}

function appendHistory(sender, text) {
    if (!historyListEl || !text) return;
    const item = document.createElement('div');
    item.className = `history-item ${sender}`;
    const label = document.createElement('div');
    label.className = 'history-label';
    label.textContent = sender === 'you' ? 'you:' : 'agent:';
    const content = document.createElement('div');
    content.textContent = text;
    item.append(label, content);
    historyListEl.appendChild(item);
    historyListEl.scrollTop = historyListEl.scrollHeight;
}

function clearVoiceTimers() {
    if (silenceCommitTimer) clearTimeout(silenceCommitTimer);
    if (lowConfidenceTimer) clearTimeout(lowConfidenceTimer);
    if (recognitionRestartTimer) clearTimeout(recognitionRestartTimer);
    silenceCommitTimer = null;
    lowConfidenceTimer = null;
    recognitionRestartTimer = null;
}

function setRobotMood(state, caption = '') {
    if (robotFace) robotFace.className = `robot-face ${state}`;
    if (caption !== null && teleprompter) {
        teleprompter.className = `teleprompter ${state === 'listening' ? 'user-speaking' : 'helio-speaking'}`;
        teleprompter.textContent = caption;
    }
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
    thinkingPanel?.classList.remove('active');
}

function recognitionStart() {
    if (!recognition || !helioOpen || agentState !== 'listening') return;
    try {
        recognition.start();
    } catch (_) {
        recognitionRestartTimer = setTimeout(() => recognitionStart(), 180);
    }
}

function scheduleRecognitionRestart() {
    if (!helioOpen || agentState !== 'listening') return;
    if (recognitionRestartTimer) clearTimeout(recognitionRestartTimer);
    recognitionRestartTimer = setTimeout(() => {
        recognitionRestartTimer = null;
        recognitionStart();
    }, 80);
}

function scheduleSilenceCommit() {
    if (silenceCommitTimer) clearTimeout(silenceCommitTimer);
    silenceCommitTimer = setTimeout(() => {
        silenceCommitTimer = null;
        if (!helioOpen || agentState !== 'listening' || !transcriptText.trim()) return;

        const confidence = confidenceCount ? confidenceSum / confidenceCount : 0;
        if (confidence >= CONFIDENCE_THRESHOLD) {
            commitTranscript();
        } else {
            setVoiceStatus('WAITING FOR CLEAR SPEECH', true);
            if (lowConfidenceTimer) clearTimeout(lowConfidenceTimer);
            lowConfidenceTimer = setTimeout(() => {
                lowConfidenceTimer = null;
                if (helioOpen && agentState === 'listening' && transcriptText.trim()) commitTranscript();
            }, LOW_CONFIDENCE_GRACE_MS);
        }
    }, SILENCE_COMMIT_MS);
}

function commitTranscript() {
    const text = transcriptText.replace(/\s+/g, ' ').trim();
    if (!text || agentState !== 'listening') return;

    clearVoiceTimers();
    interimText = '';
    transcriptText = '';
    confidenceSum = 0;
    confidenceCount = 0;
    renderTranscript();
    appendHistory('you', text);
    processVoiceCommand(text);
}

function bindRecognition() {
    if (!SpeechRecognitionImpl) return;
    recognition = new SpeechRecognitionImpl();
    recognition.continuous = true;
    recognition.interimResults = true;
    recognition.maxAlternatives = 3;
    recognition.lang = SPEECH_LANG;

    recognition.onstart = () => {
        if (!helioOpen) return;
        agentState = 'listening';
        speechActive = true;
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        setRobotMood('listening', '');
    };

    recognition.onresult = (event) => {
        if (!helioOpen || agentState !== 'listening') return;

        let currentInterim = '';
        for (let i = event.resultIndex; i < event.results.length; i += 1) {
            const result = event.results[i];
            const alt = result[0];
            const text = (alt?.transcript || '').trim();
            if (!text) continue;

            if (result.isFinal) {
                transcriptText = `${transcriptText} ${text}`.replace(/\s+/g, ' ').trim();
                const confidence = Number(alt?.confidence);
                if (Number.isFinite(confidence) && confidence > 0) {
                    confidenceSum += confidence;
                    confidenceCount += 1;
                }
            } else {
                currentInterim = `${currentInterim} ${text}`.trim();
            }
        }

        interimText = currentInterim;
        speechActive = true;
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        renderTranscript();

        // Final recognition results only update the transcript.
        // They do not end the listening session and do not submit anything.
        if (transcriptText.trim()) scheduleSilenceCommit();
    };

    recognition.onerror = (event) => {
        if (!helioOpen) return;
        if (event.error === 'not-allowed' || event.error === 'service-not-allowed' || event.error === 'audio-capture') {
            agentState = 'idle';
            speechActive = false;
            setVoiceStatus('MICROPHONE UNAVAILABLE');
            setRobotMood('listening', 'Microphone permission is required.');
            return;
        }
        // no-speech, aborted and transient browser errors are restart conditions.
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
    };

    recognition.onend = () => {
        speechActive = false;
        if (!helioOpen || agentState !== 'listening') return;
        // Browser recognition can end even with continuous=true. This is NOT a
        // semantic end-of-command. Restart it and keep the accumulated text.
        scheduleRecognitionRestart();
    };
}

function openVoiceModal() {
    if (!SpeechRecognitionImpl) {
        alert('Continuous speech recognition is not supported in this browser. Use Chrome or Edge.');
        return;
    }
    helioOpen = true;
    modal?.classList.add('active');
    modal?.setAttribute('aria-hidden', 'false');
    document.getElementById('modal-history-column')?.classList.remove('show');
    document.getElementById('prompt-drawer')?.classList.remove('show');
    PeekController.stop();

    transcriptText = '';
    interimText = '';
    confidenceSum = 0;
    confidenceCount = 0;
    clearVoiceTimers();
    currentCallId = currentCallId || String(Date.now());
    renderTranscript();
    setVoiceStatus('LISTENING CONTINUOUSLY', true);
    setRobotMood('listening', '');
    agentState = 'listening';

    try { recognition?.abort(); } catch (_) {}
    setTimeout(() => recognitionStart(), 80);
}

function closeVoiceModal() {
    helioOpen = false;
    agentState = 'idle';
    speechActive = false;
    clearVoiceTimers();
    try { recognition?.abort(); } catch (_) {}
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    stopThinkingIndicator();
    modal?.classList.remove('active');
    modal?.setAttribute('aria-hidden', 'true');
    currentCallId = null;
    transcriptText = '';
    interimText = '';
    confidenceSum = 0;
    confidenceCount = 0;
    renderTranscript();
    PeekController.start();
}

async function processVoiceCommand(text) {
    agentState = 'thinking';
    setVoiceStatus('SENDING TO HELIO', false);
    setRobotMood('thinking', '');
    startThinkingIndicator();
    playProcessingSound();

    // Offline placeholder. Replace only this function when the Helio backend
    // is connected again; the UI/voice pipeline remains unchanged.
    setTimeout(() => {
        if (!helioOpen) return;
        stopThinkingIndicator();
        const response = 'I heard you. Helio is ready for the next command.';
        appendHistory('agent', response);
        speakAndLoop(response);
    }, 900);

    void text;
}

function speakAndLoop(text) {
    if (!helioOpen) return;
    agentState = 'speaking';
    setVoiceStatus('HELIO SPEAKING', false);
    setRobotMood('speaking', text);

    if (!('speechSynthesis' in window)) {
        agentState = 'listening';
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        setRobotMood('listening', '');
        recognitionStart();
        return;
    }

    window.speechSynthesis.cancel();
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.lang = selectedLanguage === 'hi' ? 'hi-IN' : 'en-US';
    utterance.rate = 1.0;
    utterance.onend = () => {
        if (!helioOpen || window.currentUtterance !== utterance) return;
        agentState = 'listening';
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        setRobotMood('listening', '');
        recognitionStart();
    };
    utterance.onerror = utterance.onend;
    window.currentUtterance = utterance;
    window.speechSynthesis.speak(utterance);
}

function stopTalking() {
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    if (agentState === 'speaking') {
        agentState = 'listening';
        setVoiceStatus('LISTENING CONTINUOUSLY', true);
        setRobotMood('listening', '');
        recognitionStart();
    }
}

function sendManualText() {
    const input = document.getElementById('user-input');
    const text = (input?.value || '').trim();
    if (!text) return;
    if (!helioOpen) openVoiceModal();
    input.value = '';
    transcriptText = text;
    confidenceSum = 1;
    confidenceCount = 1;
    commitTranscript();
}

function setLang(lang) {
    selectedLanguage = lang === 'hi' ? 'hi' : 'en';
    document.getElementById('btn-en')?.classList.toggle('active', selectedLanguage === 'en');
    document.getElementById('btn-hi')?.classList.toggle('active', selectedLanguage === 'hi');
}


window.toggleHistory = function () {
    document.getElementById('modal-history-column')?.classList.toggle('show');
};
window.closeHistory = function () {
    document.getElementById('modal-history-column')?.classList.remove('show');
};
window.togglePrompt = function (event) {
    event?.stopPropagation();
    const drawer = document.getElementById('prompt-drawer');
    if (!drawer) return;
    drawer.classList.toggle('show');
    document.getElementById('prompt-toggle-btn')?.setAttribute('aria-expanded', drawer.classList.contains('show') ? 'true' : 'false');
    if (drawer.classList.contains('show')) setTimeout(() => document.getElementById('user-input')?.focus(), 80);
};


// Dashboard wake-word menu. The manual Ask Helio session is independent and
// remains continuous until the user closes it.
const WAKE_OPTIONS = {
    rover: ['rover'], helio: ['helio'], computer: ['computer'], car: ['car'], robot: ['robot']
};
let wakeWord = 'rover';
let wakeEnabled = true;
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
        button.onclick = () => { wakeWord = key; try { localStorage.setItem('milusions-wake-word', key); } catch (_) {} updateWakeBtn(); closeWakeMenu(); };
        menu.appendChild(button);
    });
    const hr = document.createElement('hr'); menu.appendChild(hr);
    const toggle = document.createElement('button');
    toggle.type = 'button'; toggle.className = `wm-toggle${wakeEnabled ? ' on' : ''}`; toggle.dataset.state = wakeEnabled ? 'ON' : 'OFF'; toggle.textContent = 'Listen for wake word';
    toggle.onclick = () => { wakeEnabled = !wakeEnabled; try { localStorage.setItem('milusions-wake', wakeEnabled ? 'on' : 'off'); } catch (_) {} updateWakeBtn(); };
    menu.appendChild(toggle);
}
function updateWakeBtn() {
    const btn = document.getElementById('wake-btn');
    const label = document.getElementById('wake-label');
    if (label) label.textContent = wakeWord;
    btn?.classList.toggle('on', wakeEnabled); btn?.classList.toggle('off', !wakeEnabled);
    renderWakeMenu();
}
function toggleWakeMenu(event) {
    event?.stopPropagation();
    const menu = document.getElementById('wake-menu');
    const btn = document.getElementById('wake-btn');
    if (!menu) return;
    menu.hidden = !menu.hidden;
    btn?.setAttribute('aria-expanded', menu.hidden ? 'false' : 'true');
}
function closeWakeMenu() {
    const menu = document.getElementById('wake-menu');
    const btn = document.getElementById('wake-btn');
    if (menu) menu.hidden = true;
    btn?.setAttribute('aria-expanded', 'false');
}
function toggleWake() { wakeEnabled = !wakeEnabled; try { localStorage.setItem('milusions-wake', wakeEnabled ? 'on' : 'off'); } catch (_) {} updateWakeBtn(); }
document.addEventListener('click', (event) => { if (!event.target.closest('.wake-menu-wrap')) closeWakeMenu(); });
document.addEventListener('keydown', (event) => { if (event.key === 'Escape') closeWakeMenu(); });

if (SpeechRecognitionImpl) bindRecognition();
updateWakeBtn();


PeekController.start();
document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') document.getElementById('prompt-drawer')?.classList.remove('show');
});

// =====================================================================
//  REALSENSE WEBRTC VIDEO
//  Two independent PeerConnections: RGB and Depth.
//  Media itself never travels through the rover control WebSocket.
// =====================================================================

const WEBRTC_CFG = {
    // Override with ?webrtc=http://ROVER_IP:8081 or window.WEBRTC_SIGNAL_URL.
    signalUrl: window.WEBRTC_SIGNAL_URL ||
        new URLSearchParams(window.location.search).get('webrtc') ||
        `${window.location.protocol}//${window.location.hostname}:8081`,
    reconnectMs: 1500
};

const webrtcStreams = {
    rgb: {
        video: document.getElementById('rgb-video'),
        empty: document.getElementById('rgb-video-empty'),
        state: document.getElementById('rgb-webrtc-state'),
        panel: document.getElementById('rgb-video')?.closest('.video-panel'),
        endpoint: '/offer/color', pc: null, retry: null
    },
    depth: {
        video: document.getElementById('depth-video'),
        empty: document.getElementById('depth-video-empty'),
        state: document.getElementById('depth-webrtc-state'),
        panel: document.getElementById('depth-video')?.closest('.video-panel'),
        endpoint: '/offer/depth', pc: null, retry: null
    }
};

function setWebRTCState(s, live, label) {
    if (!s.state) return;
    s.state.textContent = label || (live ? 'LIVE' : 'OFFLINE');
    s.state.classList.toggle('live', !!live);
    s.panel?.classList.toggle('live', !!live);
}

async function startWebRTCStream(kind) {
    const s = webrtcStreams[kind];
    if (!s || !s.video) return;
    if (s.retry) { clearTimeout(s.retry); s.retry = null; }
    if (s.pc) { try { s.pc.close(); } catch (_) {} s.pc = null; }
    setWebRTCState(s, false, 'CONNECTING');

    const pc = new RTCPeerConnection({ iceServers: [] });
    s.pc = pc;
    pc.addTransceiver('video', { direction: 'recvonly' });

    pc.ontrack = (event) => {
        const stream = event.streams?.[0] || new MediaStream([event.track]);
        s.video.srcObject = stream;
        s.video.play().catch(() => {});
        setWebRTCState(s, true, 'LIVE');
    };
    pc.onconnectionstatechange = () => {
        const state = pc.connectionState;
        if (state === 'connected') setWebRTCState(s, true, 'LIVE');
        if (state === 'failed' || state === 'disconnected' || state === 'closed') {
            setWebRTCState(s, false, state.toUpperCase());
            if (s.pc === pc) { try { pc.close(); } catch (_) {} s.pc = null; }
            s.retry = setTimeout(() => startWebRTCStream(kind), WEBRTC_CFG.reconnectMs);
        }
    };
    pc.oniceconnectionstatechange = () => {
        if (pc.iceConnectionState === 'failed') setWebRTCState(s, false, 'ICE FAILED');
    };

    try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        const response = await fetch(WEBRTC_CFG.signalUrl + s.endpoint, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type })
        });
        if (!response.ok) throw new Error(`WebRTC signaling HTTP ${response.status}`);
        await pc.setRemoteDescription(await response.json());
    } catch (err) {
        console.error(`[WebRTC:${kind}]`, err);
        setWebRTCState(s, false, 'RETRYING');
        try { pc.close(); } catch (_) {}
        if (s.pc === pc) s.pc = null;
        s.retry = setTimeout(() => startWebRTCStream(kind), WEBRTC_CFG.reconnectMs);
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

setTimeout(startRealSenseWebRTC, 0);

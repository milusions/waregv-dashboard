// Waypoint navigation add-on. Load AFTER app.js:  <script src="waypoints.js"></script>
// Uses Nav2 FollowWaypoints action (/follow_waypoints). Does not modify app.js/style.css/index.html.
(() => {
  const ACTION = '/follow_waypoints';
  const ACTION_TYPE = 'nav2_msgs/action/FollowWaypoints';
  const wps = [];            // {x, y, deg}
  let selecting = false, drag = null, goalId = null, actionClient = null;

  // ---- styles ----
  const st = document.createElement('style');
  st.textContent = `
  #wp-layer{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:5}
  #wp-layer.on{pointer-events:auto;cursor:crosshair}
  #wp-panel{position:absolute;left:10px;bottom:10px;z-index:6;background:rgba(255,255,255,.95);border:1px solid #cfd8e3;
    border-radius:8px;padding:8px;font:12px sans-serif;color:#1a2634;min-width:190px;max-width:260px;display:none}
  #wp-panel.show{display:block}
  #wp-panel h4{margin:0 0 6px;font-size:13px}
  #wp-list{max-height:140px;overflow:auto;margin:0 0 6px;padding:0;list-style:none}
  #wp-list li{display:flex;justify-content:space-between;align-items:center;padding:2px 0}
  #wp-list button{border:0;background:none;cursor:pointer;color:#c0392b;font-size:14px}
  #wp-panel .row{display:flex;gap:6px}
  #wp-panel .row button{flex:1;padding:5px;border:1px solid #0077ff;background:#fff;color:#0077ff;border-radius:6px;cursor:pointer}
  #wp-panel .row button.pri{background:#0077ff;color:#fff}
  #wp-panel .row button:disabled{opacity:.4;cursor:default}
  #wp-status{margin-top:6px;color:#556}`;
  document.head.appendChild(st);

  // ---- UI ----
  const stage = document.getElementById('stage');
  const layer = document.createElement('canvas');
  layer.id = 'wp-layer'; layer.width = canvas.width; layer.height = canvas.height;
  stage.appendChild(layer);
  const lctx = layer.getContext('2d');

  const panel = document.createElement('div');
  panel.id = 'wp-panel';
  panel.innerHTML = `<h4>Waypoints</h4>
    <div id="wp-help">Click map to add a waypoint (drag to set heading).</div>
    <ul id="wp-list"></ul>
    <div class="row"><button id="wp-go" class="pri">Execute</button><button id="wp-stop">Stop</button></div>
    <div class="row" style="margin-top:6px"><button id="wp-clear">Clear</button><button id="wp-done">Done</button></div>
    <div id="wp-status"></div>`;
  stage.appendChild(panel);
  const $ = id => document.getElementById(id);

  const btn = document.createElement('button');
  btn.id = 'btn-wp'; btn.className = 'tool'; btn.dataset.tip = 'Waypoint navigation'; btn.setAttribute('aria-label', 'Waypoint navigation');
  btn.innerHTML = '<svg viewBox="0 0 24 24"><circle cx="5" cy="18" r="2"/><circle cx="12" cy="7" r="2"/><circle cx="19" cy="16" r="2"/><path d="M6.5 16.5l4-7M14 8l4 6"/></svg>';
  document.getElementById('toolbar').appendChild(btn);

  const status = t => { $('wp-status').textContent = t; };

  function setSelecting(on) {
    selecting = on; drag = null;
    layer.classList.toggle('on', on);
    btn.classList.toggle('active', on);
    panel.classList.toggle('show', on || wps.length > 0);
    if (on && typeof setGoalMode === 'function') setGoalMode(false);
    redraw();
  }
  btn.onclick = () => setSelecting(!selecting);
  $('wp-done').onclick = () => setSelecting(false);
  $('wp-clear').onclick = () => { wps.length = 0; renderList(); status(''); redraw(); };

  function renderList() {
    const ul = $('wp-list'); ul.innerHTML = '';
    wps.forEach((w, i) => {
      const li = document.createElement('li');
      li.innerHTML = `<span>${i + 1}: ${w.x.toFixed(2)}, ${w.y.toFixed(2)}, ${w.deg}\u00B0</span>`;
      const b = document.createElement('button'); b.textContent = '\u00D7';
      b.onclick = () => { wps.splice(i, 1); renderList(); redraw(); };
      li.appendChild(b); ul.appendChild(li);
    });
    $('wp-go').disabled = !wps.length;
  }

  // ---- pointer input (only active while selecting) ----
  const pos = e => {
    const r = layer.getBoundingClientRect();
    return [(e.clientX - r.left) * layer.width / r.width, (e.clientY - r.top) * layer.height / r.height];
  };
  layer.addEventListener('pointerdown', e => {
    layer.setPointerCapture(e.pointerId);
    const [x, y] = pos(e); drag = { x0: x, y0: y, x1: x, y1: y };
  });
  layer.addEventListener('pointermove', e => { if (!drag) return; [drag.x1, drag.y1] = pos(e); redraw(); });
  layer.addEventListener('pointerup', () => {
    if (!drag) return;
    const d = drag; drag = null;
    const dx = d.x1 - d.x0, dy = d.y1 - d.y0;
    const deg = Math.hypot(dx, dy) > 12 ? Math.round(Math.atan2(-dy, dx) * 180 / Math.PI) : 0;
    wps.push({ x: (d.x0 - view.x) / view.s, y: -(d.y0 - view.y) / view.s, deg });
    renderList(); redraw();
  });
  layer.addEventListener('wheel', e => { // allow zoom while selecting
    e.preventDefault();
    const [px, py] = pos(e), k = e.deltaY < 0 ? 1.15 : 1 / 1.15;
    view.x = px - (px - view.x) * k; view.y = py - (py - view.y) * k; view.s *= k;
    dirty(); redraw();
  }, { passive: false });

  // ---- drawing (overlay canvas, follows app view) ----
  function redraw() {
    lctx.clearRect(0, 0, layer.width, layer.height);
    const pts = wps.map(w => [view.x + w.x * view.s, view.y - w.y * view.s, w.deg * Math.PI / 180]);
    lctx.strokeStyle = '#ff8800'; lctx.fillStyle = '#ff8800'; lctx.lineWidth = 2;
    if (pts.length > 1) { lctx.setLineDash([6, 4]); lctx.beginPath(); pts.forEach((p, i) => i ? lctx.lineTo(p[0], p[1]) : lctx.moveTo(p[0], p[1])); lctx.stroke(); lctx.setLineDash([]); }
    pts.forEach(([x, y, a], i) => {
      lctx.beginPath(); lctx.arc(x, y, 9, 0, 7); lctx.fill();
      lctx.beginPath(); lctx.moveTo(x, y); lctx.lineTo(x + Math.cos(-a) * 22, y + Math.sin(-a) * 22); lctx.stroke();
      lctx.fillStyle = '#fff'; lctx.font = 'bold 11px sans-serif'; lctx.textAlign = 'center'; lctx.textBaseline = 'middle';
      lctx.fillText(i + 1, x, y); lctx.fillStyle = '#ff8800';
    });
    if (drag) {
      lctx.strokeStyle = '#0077ff'; lctx.beginPath(); lctx.arc(drag.x0, drag.y0, 6, 0, 7); lctx.stroke();
      lctx.beginPath(); lctx.moveTo(drag.x0, drag.y0); lctx.lineTo(drag.x1, drag.y1); lctx.stroke();
    }
  }
  (function loop() { redraw(); setTimeout(() => requestAnimationFrame(loop), 100); })(); // keep in sync with pan/zoom

  // ---- action client ----
  function getClient() {
    if (actionClient) return actionClient;
    actionClient = new ROSLIB.Action({ ros, name: ACTION, actionType: ACTION_TYPE });
    return actionClient;
  }
  const poseMsg = w => {
    const yaw = w.deg * Math.PI / 180;
    return { header: { frame_id: 'map' },
      pose: { position: { x: w.x, y: w.y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } } };
  };

  $('wp-go').onclick = () => {
    if (!ros) { toast('Not connected.', { kind: 'error', label: 'Waypoints' }); return; }
    if (!wps.length) return;
    if (!confirm(`Execute ${wps.length} waypoint(s)?`)) return;
    const goal = { poses: wps.map(poseMsg) };
    goalId = getClient().sendGoal(
      goal,
      res => { // result
        const missed = (res && res.values ? res.values.missed_waypoints : res && res.missed_waypoints) || [];
        status(missed.length ? `Finished, missed: ${missed.join(', ')}` : 'All waypoints reached');
        addLog(missed.length ? `Waypoints finished, missed ${missed.join(', ')}` : 'All waypoints reached');
        goalId = null;
      },
      fb => { // feedback
        const f = fb && fb.values ? fb.values : fb;
        if (f && f.current_waypoint !== undefined) status(`Heading to waypoint ${f.current_waypoint + 1}/${wps.length}`);
      },
      err => { status('Failed: ' + err); addLog('Waypoint action failed: ' + err, 'ERROR'); goalId = null; }
    );
    status('Goal sent'); addLog(`Waypoint goal sent (${wps.length} poses)`);
    toast(`${wps.length} waypoints sent`, { kind: 'success', label: 'Waypoints' });
  };

  $('wp-stop').onclick = () => {
    if (goalId && actionClient) { actionClient.cancelGoal(goalId); status('Cancel requested'); addLog('Waypoint goal cancelled', 'WARN'); }
  };

  renderList();
})();
const CFG = {
    chartWindowSec: 30,
    joyRateHz: 20
};

const MODE_LABELS = { auto_nav: 'Autonomous Navigation and Driving' };

function notify(level, message) {
    level = (level || 'INFO').toUpperCase();
    const stack = document.getElementById('notify-stack');
    if (!stack) return;
    const el = document.createElement('div');
    el.className = 'notice ' + level.toLowerCase();
    el.innerHTML = '<div class="notice-hd"><span class="notice-lvl">' + level +
        '</span></div><div class="notice-msg"></div>';
    el.querySelector('.notice-msg').textContent = message;
    stack.appendChild(el);
    setTimeout(() => el.remove(), 5000);
}

function applyTheme(isLight) {
    document.body.classList.toggle('light-theme', isLight);
}
function toggleTheme() {
    applyTheme(!document.body.classList.contains('light-theme'));
}

const wheelIds = ['fl', 'fr', 'bl', 'br'];
const series = {};
wheelIds.forEach(id => series[id] = { a: [], c: [] });

const chartCanvases = {};
wheelIds.forEach(id => chartCanvases[id] = document.getElementById('chart-' + id));

function fitCanvas(canvas) {
    if (!canvas) return { ctx: null, w: 0, h: 0 };
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
    if (!w || !h || !ctx) return;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#8a919c';
    ctx.font = '10px Roboto, sans-serif';
    ctx.fillText('Wheel Data Stream', 10, 20);
}
setInterval(() => wheelIds.forEach(drawChart), 500);

let navStatus = 'Idle';

function updateStatus() {
    const statusEl = document.getElementById('stat-nav-status');
    if (statusEl) statusEl.textContent = navStatus;
}
setInterval(updateStatus, 200);

// =====================================================================
//  Voice & Assistant Logic (Happy Blue Mode, Idle Chats & User Attention)
// =====================================================================
const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
let recognition = null;
let modal = document.getElementById('voice-modal');
let robotFace = document.getElementById('robot-face');
let teleprompter = document.getElementById('teleprompter');
let historyListEl = document.getElementById('history-list');
let agentState = 'idle';
let fullTranscript = '';
let happyIdleTimer = null;
let sleepTimeoutTimer = null;

const SLEEP_TIMEOUT_MS = 30000; // 30 seconds of inactivity triggers sleep

const ROVER_CHATS_AND_JOKES = [
    "Scanning warehouse aisles... Everything looks smooth and clear!",
    "Did you know? Autonomous rovers use LiDAR and SLAM to map unknown environments seamlessly.",
    "Why did the autonomous robot cross the warehouse? To optimize the supply chain path!",
    "My obstacle avoidance sensors are fully engaged and ready for action.",
    "Joke time: Why do robots make great drivers? Because they never lose their train of thought—or their lane!",
    "Calculating optimal trajectories... Navigation efficiency is at 100%!",
    "Ready to roll out on your next waypoint command whenever you are!"
];

const thinkingPanel = document.getElementById('thinking-panel');
const thinkingStageText = document.getElementById('thinking-stage-text');

function startThinkingIndicator() {
    if (!thinkingPanel) return;
    thinkingPanel.classList.add('active');
    if (thinkingStageText) thinkingStageText.textContent = "Analyzing prompt...";
}

function stopThinkingIndicator() {
    if (thinkingPanel) thinkingPanel.classList.remove('active');
}

function appendHistory(sender, text) {
    if (!historyListEl) return;
    const item = document.createElement('div');
    item.className = `history-item ${sender}`;
    const label = document.createElement('div');
    label.className = 'history-label';
    label.textContent = sender === 'you' ? 'you:' : 'agent:';
    const content = document.createElement('div');
    content.textContent = text;
    item.appendChild(label);
    item.appendChild(content);
    historyListEl.appendChild(item);
    historyListEl.scrollTop = historyListEl.scrollHeight;
}

// Start Happy Blue Idle loop between active speaking/listening and sleep timeout
function startHappyIdleRoutine() {
    stopHappyIdleRoutine();
    
    // Switch to Happy Blue mode after initial greeting
    if (agentState === 'listening' && robotFace) {
        robotFace.className = 'robot-face happy-blue';
    }

    // Periodic random autonomous rover chats and jokes
    happyIdleTimer = setInterval(() => {
        if (agentState === 'listening' && modal.classList.contains('active')) {
            const randomMsg = ROVER_CHATS_AND_JOKES[Math.floor(Math.random() * ROVER_CHATS_AND_JOKES.length)];
            if (teleprompter) {
                teleprompter.className = 'teleprompter helio-speaking';
                teleprompter.textContent = randomMsg;
                teleprompter.scrollTop = teleprompter.scrollHeight;
            }
        }
    }, 7000);

    // Sleep timeout if user doesn't speak for long
    if (sleepTimeoutTimer) clearTimeout(sleepTimeoutTimer);
    sleepTimeoutTimer = setTimeout(() => {
        if (agentState === 'listening' && modal.classList.contains('active')) {
            goToSleep();
        }
    }, SLEEP_TIMEOUT_MS);
}

function stopHappyIdleRoutine() {
    if (happyIdleTimer) { clearInterval(happyIdleTimer); happyIdleTimer = null; }
    if (sleepTimeoutTimer) { clearTimeout(sleepTimeoutTimer); sleepTimeoutTimer = null; }
}

if (SpeechRecognitionImpl) {
    recognition = new SpeechRecognitionImpl();
    recognition.continuous = true;   
    recognition.interimResults = true;

    recognition.onstart = () => {
        agentState = 'listening';
        startHappyIdleRoutine();
    };

    recognition.onresult = (event) => {
        if (agentState !== 'listening') return;
        
        // User is speaking! Stop happy routine, switch to curious/attentive mode, and reset sleep timer.
        stopHappyIdleRoutine();
        if (robotFace) robotFace.className = 'robot-face curious';
        if (sleepTimeoutTimer) clearTimeout(sleepTimeoutTimer);

        let interim = '';
        let final = '';

        for (let i = event.resultIndex; i < event.results.length; ++i) {
            if (event.results[i].isFinal) {
                final += event.results[i][0].transcript;
            } else {
                interim += event.results[i][0].transcript;
            }
        }

        if (final) {
            fullTranscript += ' ' + final;
            processVoiceCommand(final.trim());
        }

        const displayString = (fullTranscript + ' ' + interim).trim();
        if (displayString && teleprompter) {
            teleprompter.className = 'teleprompter user-speaking';
            teleprompter.textContent = displayString;
            teleprompter.scrollTop = teleprompter.scrollHeight;
        }

        // Restart sleep timeout countdown if user stops speaking temporarily
        sleepTimeoutTimer = setTimeout(() => {
            if (agentState === 'listening' && modal.classList.contains('active')) {
                goToSleep();
            }
        }, SLEEP_TIMEOUT_MS);
    };

    recognition.onerror = (e) => {
        console.warn('Speech recognition error:', e.error);
    };

    recognition.onend = () => {
        if (agentState === 'listening' && modal.classList.contains('active')) {
            try { recognition.start(); } catch(err){}
        }
    };
}

function openVoiceModal() {
    if (!modal) return;
    modal.classList.add('active');
    agentState = 'listening';
    fullTranscript = '';
    if (robotFace) robotFace.className = 'robot-face happy-blue';
    if (teleprompter) teleprompter.textContent = 'Hello! Blue mode active. Listening for commands...';
    if (recognition) {
        try { recognition.start(); } catch(e){}
    }
    startHappyIdleRoutine();
}

function closeVoiceModal() {
    if (!modal) return;
    modal.classList.remove('active');
    agentState = 'idle';
    stopHappyIdleRoutine();
    if (recognition) {
        try { recognition.stop(); } catch(e){}
    }
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
}

function processVoiceCommand(text) {
    if (!text) return;
    agentState = 'thinking';
    stopHappyIdleRoutine();
    if (robotFace) robotFace.className = 'robot-face thinking';
    startThinkingIndicator();

    appendHistory('you', text);

    setTimeout(() => {
        stopThinkingIndicator();
        const responseText = "Processed command: " + text;
        appendHistory('agent', responseText);
        speakResponse(responseText);
    }, 1200);
}

function speakResponse(text) {
    agentState = 'speaking';
    if (robotFace) robotFace.className = 'robot-face speaking';
    if (teleprompter) {
        teleprompter.className = 'teleprompter helio-speaking';
        teleprompter.textContent = text;
        teleprompter.scrollTop = teleprompter.scrollHeight;
    }

    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
        const utterance = new SpeechSynthesisUtterance(text);
        utterance.onend = () => {
            if (modal && modal.classList.contains('active')) {
                agentState = 'listening';
                fullTranscript = '';
                if (robotFace) robotFace.className = 'robot-face happy-blue';
                if (teleprompter) teleprompter.textContent = 'Listening again...';
                try { recognition.start(); } catch(e){}
                startHappyIdleRoutine();
            }
        };
        window.speechSynthesis.speak(utterance);
    }
}

function goToSleep() {
    if (!modal.classList.contains('active') || agentState === 'asleep') return;
    agentState = 'asleep';
    stopHappyIdleRoutine();
    try { recognition.stop(); } catch (e) {}
    if (robotFace) robotFace.className = 'robot-face sleeping';
    if (teleprompter) {
        teleprompter.className = 'teleprompter helio-speaking';
        teleprompter.textContent = 'Zzz... Sleeping due to inactivity.';
    }
}

function stopTalking() {
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    stopThinkingIndicator();
    agentState = 'listening';
    fullTranscript = '';
    if (robotFace) robotFace.className = 'robot-face happy-blue';
    if (teleprompter) teleprompter.textContent = 'Interrupted. Back in happy mode...';
    try { recognition.start(); } catch(e){}
    startHappyIdleRoutine();
}

function sendManualText() {
    const input = document.getElementById('user-input');
    if (!input || !input.value.trim()) return;
    const txt = input.value.trim();
    input.value = '';
    if (!modal.classList.contains('active')) openVoiceModal();
    processVoiceCommand(txt);
}

// =====================================================================
//  RealSense WebRTC Integration (RGB & Depth Streams)
// =====================================================================
const WEBRTC_CFG = {
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

function startRealSenseWebRTC() {
    startWebRTCStream('rgb');
    startWebRTCStream('depth');
}

window.addEventListener('beforeunload', () => {
    stopWebRTCStream('rgb');
    stopWebRTCStream('depth');
});

setTimeout(startRealSenseWebRTC, 0);
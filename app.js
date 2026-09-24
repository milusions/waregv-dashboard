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

function formatSigned(v, digits = 2) {
    const n = Number(v);
    if (!Number.isFinite(n)) return '—';
    return (n > 0 ? '+' : '') + n.toFixed(digits);
}

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

// Map variables & handlers
let latestMap = null, latestPlan = [], navStatus = 'Idle', robot = null, odom = null;
let currentCmdVel = { linear: 0, angular: 0 };
const mapCanvas = document.getElementById('map-canvas');

function updateStatus() {
    const statusEl = document.getElementById('stat-nav-status');
    if (statusEl) statusEl.textContent = navStatus;
}
setInterval(updateStatus, 200);

// Voice & Assistant Logic with Real-Time Streaming Teleprompter & Auto-Scroll
const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
let recognition = null;
let modal = document.getElementById('voice-modal');
let robotFace = document.getElementById('robot-face');
let teleprompter = document.getElementById('teleprompter');
let historyListEl = document.getElementById('history-list');
let agentState = 'idle';
let thinkingTimer = null;
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

const SPEECH_LANG = 'en-US';
let lastText = '', committed = false;

if (SpeechRecognitionImpl) {
    recognition = new SpeechRecognitionImpl();
    recognition.continuous = true;       // Enables continuous transcription until you stop
    recognition.interimResults = true;   // Streams interim words instantly like Claude/Gemini

    recognition.onstart = () => {
        agentState = 'listening';
        committed = false;
        if (robotFace) robotFace.className = 'robot-face listening';
    };

    recognition.onresult = (event) => {
        if (agentState !== 'listening' || committed) return;
        let interimTranscript = '';
        let finalTranscript = '';

        for (let i = event.resultIndex; i < event.results.length; ++i) {
            if (event.results[i].isFinal) {
                finalTranscript += event.results[i][0].transcript;
            } else {
                interimTranscript += event.results[i][0].transcript;
            }
        }

        const currentStreamText = finalTranscript || interimTranscript;
        if (currentStreamText && teleprompter) {
            teleprompter.className = 'teleprompter user-speaking';
            teleprompter.textContent = currentStreamText;
            // Auto-scroll the teleprompter down as the text length grows
            teleprompter.scrollTop = teleprompter.scrollHeight;
            lastText = currentStreamText;
        }

        if (finalTranscript) {
            commitUtterance(finalTranscript);
        }
    };

    recognition.onerror = (e) => {
        console.warn('Speech recognition error:', e.error);
    };

    recognition.onend = () => {
        if (agentState === 'listening' && !committed && lastText) {
            commitUtterance(lastText);
        }
    };
}

function commitUtterance(text) {
    text = (text || '').trim();
    if (committed || !text) return;
    committed = true;
    try { recognition.stop(); } catch(e){}

    appendHistory('you', text);
    processVoiceCommand(text);
}

function openVoiceModal() {
    if (!modal) return;
    modal.classList.add('active');
    agentState = 'listening';
    if (robotFace) robotFace.className = 'robot-face listening';
    if (teleprompter) teleprompter.textContent = 'Listening... Speak your command.';
    if (recognition) {
        try { recognition.start(); } catch(e){}
    }
}

function closeVoiceModal() {
    if (!modal) return;
    modal.classList.remove('active');
    agentState = 'idle';
    if (recognition) {
        try { recognition.stop(); } catch(e){}
    }
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
}

function processVoiceCommand(text) {
    agentState = 'thinking';
    if (robotFace) robotFace.className = 'robot-face thinking';
    startThinkingIndicator();

    setTimeout(() => {
        stopThinkingIndicator();
        const responseText = "Processed: " + text;
        appendHistory('agent', responseText);
        speakResponse(responseText);
    }, 1500);
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
                if (robotFace) robotFace.className = 'robot-face listening';
                if (teleprompter) teleprompter.textContent = 'Listening again...';
                try { recognition.start(); } catch(e){}
            }
        };
        window.speechSynthesis.speak(utterance);
    }
}

function stopTalking() {
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    stopThinkingIndicator();
    agentState = 'listening';
    if (robotFace) robotFace.className = 'robot-face listening';
    if (teleprompter) teleprompter.textContent = 'Interrupted. Listening...';
    try { recognition.start(); } catch(e){}
}

function sendManualText() {
    const input = document.getElementById('user-input');
    if (!input || !input.value.trim()) return;
    const txt = input.value.trim();
    input.value = '';
    if (!modal.classList.contains('active')) openVoiceModal();
    appendHistory('you', txt);
    processVoiceCommand(txt);
}
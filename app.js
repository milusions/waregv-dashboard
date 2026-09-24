/* =====================================================================
   WAREGV DASHBOARD — HELIO + WEBRTC
   Clean client-only module layout.
   No rover REST/WS actions are performed here.
   ===================================================================== */

(() => {
    'use strict';

    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------
    const CONFIG = Object.freeze({
        webrtcSignalUrl:
            window.WEBRTC_SIGNAL_URL ||
            new URLSearchParams(window.location.search).get('webrtc') ||
            `${window.location.protocol}//${window.location.hostname}:8081`,
        webrtcReconnectMs: 1500,
        peekIntervalMs: 6000,
        peekInitialDelayMs: 1500,
    });

    // ------------------------------------------------------------------
    // Small DOM helpers
    // ------------------------------------------------------------------
    const $ = (selector, root = document) => root.querySelector(selector);
    const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];

    function setText(selector, value) {
        const el = $(selector);
        if (el) el.textContent = value;
    }

    // ------------------------------------------------------------------
    // Theme
    // ------------------------------------------------------------------
    function applyTheme(isLight) {
        document.body.classList.toggle('light-theme', !!isLight);
    }

    function toggleTheme() {
        applyTheme(!document.body.classList.contains('light-theme'));
    }

    window.toggleTheme = toggleTheme;

    // ------------------------------------------------------------------
    // Navigation-card UI helpers only.
    // These do not call rover APIs.
    // ------------------------------------------------------------------
    window.toggleNavCard = function toggleNavCard() {
        const card = $('#map-nav-card');
        const button = $('#nav-card-toggle');
        if (!card || !button) return;
        const collapsed = card.classList.toggle('collapsed');
        button.textContent = collapsed ? 'Show' : 'Hide';
    };

    window.toggleHistory = function toggleHistory() {
        $('#modal-history-column')?.classList.toggle('show');
    };

    window.closeHistory = function closeHistory() {
        $('#modal-history-column')?.classList.remove('show');
    };

    window.togglePrompt = function togglePrompt() {
        const drawer = $('#prompt-drawer');
        if (!drawer) return;
        drawer.classList.toggle('show');
        if (drawer.classList.contains('show')) {
            setTimeout(() => $('#user-input')?.focus(), 60);
        }
    };

    // ------------------------------------------------------------------
    // Helio peeking button
    // ------------------------------------------------------------------
    const PeekController = {
        messages: [
            'Ever wondered if you could control a rover hands free?',
            'Hey there, how are you?',
            'Controlling a rover, I see!',
            "Hey, what's this button?",
        ],
        timer: null,
        hideTimer: null,

        start() {
            this.stop();
            setTimeout(() => this.show(), CONFIG.peekInitialDelayMs);
            this.timer = setInterval(() => this.show(), CONFIG.peekIntervalMs);
        },

        stop() {
            if (this.timer) clearInterval(this.timer);
            if (this.hideTimer) clearTimeout(this.hideTimer);
            this.timer = null;
            this.hideTimer = null;
        },

        show() {
            const container = $('#helio-peek');
            const bubble = $('#peek-bubble');
            const modal = $('#voice-modal');
            if (!container || !bubble || modal?.classList.contains('active')) return;

            const message = this.messages[Math.floor(Math.random() * this.messages.length)];
            bubble.textContent = message;
            container.classList.add('peeking');
            bubble.classList.add('show');

            if (this.hideTimer) clearTimeout(this.hideTimer);
            this.hideTimer = setTimeout(() => {
                container.classList.remove('peeking');
                setTimeout(() => bubble.classList.remove('show'), 800);
            }, 3500);
        },
    };

    // ------------------------------------------------------------------
    // Session history
    // ------------------------------------------------------------------
    function appendHistory(sender, text) {
        const list = $('#history-list');
        if (!list || !text) return;

        const item = document.createElement('div');
        item.className = `history-item ${sender}`;

        const label = document.createElement('div');
        label.className = 'history-label';
        label.textContent = sender === 'you' ? 'you:' : 'agent:';

        const content = document.createElement('div');
        content.textContent = text;

        item.append(label, content);
        list.appendChild(item);
        list.scrollTop = list.scrollHeight;
    }

    // ------------------------------------------------------------------
    // Helio language selector — UI only for now.
    // ------------------------------------------------------------------
    let selectedLanguage = 'en';

    window.setLang = function setLang(language) {
        selectedLanguage = language === 'hi' ? 'hi' : 'en';
        $('#btn-en')?.classList.toggle('active', selectedLanguage === 'en');
        $('#btn-hi')?.classList.toggle('active', selectedLanguage === 'hi');
    };

    // ------------------------------------------------------------------
    // Voice Assistant
    //
    // Important behavior:
    //   • Speech recognition stays active until the user stops it.
    //   • Browser recognition ending is treated as a technical restart,
    //     not as the end of the user's message.
    //   • Final recognition results are TRANSCRIBED, never auto-submitted.
    //   • There is no inactivity timer and no automatic "sleep".
    //   • Submission happens only from an explicit Send action.
    // ------------------------------------------------------------------
    const VoiceAssistant = {
        recognition: null,
        supported: false,
        listening: false,
        restarting: false,
        finalText: '',
        interimText: '',
        modal: null,
        face: null,
        caption: null,
        status: null,
        hint: null,
        thinking: null,

        init() {
            this.modal = $('#voice-modal');
            this.face = $('#robot-face');
            this.caption = $('#caption-text');
            this.status = $('#caption-status');
            this.hint = $('#caption-hint');
            this.thinking = $('#thinking-panel');

            const Recognition = window.SpeechRecognition || window.webkitSpeechRecognition;
            if (!Recognition) {
                this.setStatus('Speech recognition unavailable');
                if (this.hint) this.hint.textContent = 'Use the text input button or a Chromium-based browser.';
                return;
            }

            this.supported = true;
            this.recognition = new Recognition();
            this.recognition.continuous = true;
            this.recognition.interimResults = true;
            this.recognition.maxAlternatives = 1;
            this.bindRecognitionEvents();
        },

        bindRecognitionEvents() {
            const r = this.recognition;

            r.onstart = () => {
                this.restarting = false;
                this.listening = true;
                this.setFace('listening');
                this.setStatus('Listening continuously', true);
                if (this.hint) this.hint.textContent = 'Keep talking — Helio will not auto-submit.';
            };

            r.onresult = (event) => {
                let newFinal = '';
                let newInterim = '';

                for (let i = event.resultIndex; i < event.results.length; i += 1) {
                    const result = event.results[i];
                    const transcript = result[0]?.transcript || '';
                    if (result.isFinal) newFinal += transcript;
                    else newInterim += transcript;
                }

                if (newFinal) {
                    this.finalText = `${this.finalText} ${newFinal}`.trim();
                }
                this.interimText = newInterim;
                this.renderTranscript();
            };

            r.onerror = (event) => {
                // no-speech and aborted are normal browser recognition events.
                // Never turn either one into a submission or a sleep state.
                if (event.error === 'not-allowed' || event.error === 'service-not-allowed') {
                    this.listening = false;
                    this.setStatus('Microphone permission required');
                    return;
                }
                this.setStatus('Reconnecting microphone…');
            };

            r.onend = () => {
                if (!this.listening || !this.modal?.classList.contains('active')) return;
                this.restartRecognition();
            };
        },

        restartRecognition() {
            if (!this.recognition || this.restarting || !this.listening) return;
            this.restarting = true;
            setTimeout(() => {
                if (!this.listening || !this.modal?.classList.contains('active')) {
                    this.restarting = false;
                    return;
                }
                try {
                    this.recognition.start();
                } catch (_) {
                    this.restarting = false;
                    setTimeout(() => this.restartRecognition(), 150);
                }
            }, 60);
        },

        startListening() {
            if (!this.supported) return;
            this.listening = true;
            this.setFace('listening');
            this.setStatus('Starting microphone…');
            try {
                this.recognition.start();
            } catch (_) {
                this.restartRecognition();
            }
            this.updateListenButton();
        },

        stopListening() {
            this.listening = false;
            this.restarting = false;
            if (this.recognition) {
                try { this.recognition.stop(); } catch (_) {}
            }
            this.setStatus('Listening stopped');
            this.setFace('listening');
            this.updateListenButton();
        },

        open() {
            if (!this.modal) return;
            this.modal.classList.add('active');
            PeekController.stop();
            this.finalText = '';
            this.interimText = '';
            this.renderTranscript('Speak whenever you are ready…');
            this.setFace('listening');
            this.startListening();
            this.updateListenButton();
        },

        close() {
            this.stopListening();
            if ('speechSynthesis' in window) window.speechSynthesis.cancel();
            this.modal?.classList.remove('active');
            this.setFace('listening');
            this.setStatus('Listening stopped');
            this.thinking?.classList.remove('active');
            PeekController.start();
        },

        renderTranscript(emptyText = '') {
            if (!this.caption) return;
            const committed = this.finalText.trim();
            const interim = this.interimText.trim();
            const value = [committed, interim].filter(Boolean).join(committed && interim ? ' ' : '');
            this.caption.textContent = value || emptyText;
            this.caption.scrollTop = this.caption.scrollHeight;
        },

        clearTranscript() {
            this.finalText = '';
            this.interimText = '';
            this.renderTranscript('Speak whenever you are ready…');
            this.setStatus(this.listening ? 'Listening continuously' : 'Listening stopped', this.listening);
        },

        submit(text) {
            const clean = (text || '').trim();
            if (!clean) return;

            appendHistory('you', clean);
            this.setStatus('Transcript ready', false);

            // Intentionally a placeholder. No REST/WS action is performed.
            this.onSubmit(clean, selectedLanguage);
        },

        onSubmit(text, language) {
            // Integration hook for the real Helio backend.
            // Keep this empty while the dashboard is in offline/mock mode.
            void text;
            void language;
        },

        sendTranscript() {
            this.submit(this.finalText);
        },

        finishInput() {
            this.stopListening();
            if (this.finalText.trim()) this.submit(this.finalText);
            this.renderTranscript(this.finalText || 'Listening stopped.');
        },

        setFace(state) {
            if (!this.face) return;
            this.face.className = `robot-face ${state}`;
        },

        setStatus(text, live = false) {
            if (!this.status) return;
            this.status.textContent = text;
            this.status.classList.toggle('live', !!live);
        },

        updateListenButton() {
            const button = $('#voice-listen-toggle');
            if (!button) return;
            button.textContent = this.listening ? 'Stop Listening' : 'Resume Listening';
        },

        interruptSpeech() {
            if ('speechSynthesis' in window) window.speechSynthesis.cancel();
            this.setFace(this.listening ? 'listening' : 'sleeping');
        },
    };

    window.openVoiceModal = () => VoiceAssistant.open();
    window.closeVoiceModal = () => VoiceAssistant.close();
    window.stopTalking = () => VoiceAssistant.interruptSpeech();
    window.clearVoiceTranscript = () => VoiceAssistant.clearTranscript();
    window.sendVoiceTranscript = () => VoiceAssistant.sendTranscript();
    window.finishVoiceInput = () => VoiceAssistant.finishInput();
    window.toggleVoiceListening = () => {
        if (VoiceAssistant.listening) VoiceAssistant.stopListening();
        else VoiceAssistant.startListening();
    };

    window.sendManualText = function sendManualText() {
        const input = $('#user-input');
        const text = input?.value?.trim();
        if (!text) return;
        VoiceAssistant.submit(text);
        if (input) input.value = '';
    };

    // ------------------------------------------------------------------
    // Lightweight wheel display — visual only, no telemetry connection.
    // ------------------------------------------------------------------
    function drawOfflineCharts() {
        $$('.chart-canvas').forEach((canvas) => {
            const rect = canvas.getBoundingClientRect();
            if (!rect.width || !rect.height) return;
            const dpr = window.devicePixelRatio || 1;
            canvas.width = Math.round(rect.width * dpr);
            canvas.height = Math.round(rect.height * dpr);
            const ctx = canvas.getContext('2d');
            if (!ctx) return;
            ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            ctx.clearRect(0, 0, rect.width, rect.height);
            ctx.fillStyle = '#8a919c';
            ctx.font = '10px Roboto, sans-serif';
            ctx.fillText('Wheel Data Stream — Offline', 10, 20);
        });
    }

    // ------------------------------------------------------------------
    // RealSense WebRTC — media only. No camera data is sent through WS.
    // ------------------------------------------------------------------
    const WebRTCViewer = {
        streams: {
            rgb: {
                video: $('#rgb-video'),
                state: $('#rgb-webrtc-state'),
                panel: $('#rgb-video')?.closest('.video-panel'),
                endpoint: '/offer/color',
                pc: null,
                retry: null,
            },
            depth: {
                video: $('#depth-video'),
                state: $('#depth-webrtc-state'),
                panel: $('#depth-video')?.closest('.video-panel'),
                endpoint: '/offer/depth',
                pc: null,
                retry: null,
            },
        },

        setState(stream, label, live = false) {
            stream.state?.classList.toggle('live', live);
            if (stream.state) stream.state.textContent = label;
            stream.panel?.classList.toggle('live', live);
        },

        schedule(kind) {
            const stream = this.streams[kind];
            if (!stream || stream.retry) return;
            stream.retry = setTimeout(() => {
                stream.retry = null;
                this.start(kind);
            }, CONFIG.webrtcReconnectMs);
        },

        async start(kind) {
            const stream = this.streams[kind];
            if (!stream?.video || !window.RTCPeerConnection) return;

            if (stream.retry) {
                clearTimeout(stream.retry);
                stream.retry = null;
            }
            if (stream.pc) {
                try { stream.pc.close(); } catch (_) {}
                stream.pc = null;
            }

            this.setState(stream, 'CONNECTING');
            const pc = new RTCPeerConnection({ iceServers: [] });
            stream.pc = pc;
            pc.addTransceiver('video', { direction: 'recvonly' });

            pc.ontrack = (event) => {
                const mediaStream = event.streams?.[0] || new MediaStream([event.track]);
                stream.video.srcObject = mediaStream;
                stream.video.play().catch(() => {});
                this.setState(stream, 'LIVE', true);
            };

            pc.onconnectionstatechange = () => {
                if (pc.connectionState === 'connected') {
                    this.setState(stream, 'LIVE', true);
                    return;
                }
                if (['failed', 'disconnected', 'closed'].includes(pc.connectionState)) {
                    this.setState(stream, pc.connectionState.toUpperCase());
                    if (stream.pc === pc) stream.pc = null;
                    try { pc.close(); } catch (_) {}
                    this.schedule(kind);
                }
            };

            try {
                const offer = await pc.createOffer();
                await pc.setLocalDescription(offer);
                const response = await fetch(CONFIG.webrtcSignalUrl + stream.endpoint, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        sdp: pc.localDescription.sdp,
                        type: pc.localDescription.type,
                    }),
                });
                if (!response.ok) throw new Error(`WebRTC signaling HTTP ${response.status}`);
                await pc.setRemoteDescription(await response.json());
            } catch (error) {
                console.warn(`[WebRTC:${kind}]`, error);
                try { pc.close(); } catch (_) {}
                if (stream.pc === pc) stream.pc = null;
                this.setState(stream, 'RETRYING');
                this.schedule(kind);
            }
        },

        startAll() {
            this.start('rgb');
            this.start('depth');
        },

        stopAll() {
            Object.values(this.streams).forEach((stream) => {
                if (stream.retry) clearTimeout(stream.retry);
                stream.retry = null;
                if (stream.pc) {
                    try { stream.pc.close(); } catch (_) {}
                    stream.pc = null;
                }
            });
        },
    };

    // ------------------------------------------------------------------
    // Boot
    // ------------------------------------------------------------------
    function boot() {
        applyTheme(true);
        VoiceAssistant.init();
        PeekController.start();
        drawOfflineCharts();
        WebRTCViewer.startAll();

        // Keep the dashboard visually stable when the window changes size.
        window.addEventListener('resize', drawOfflineCharts, { passive: true });
        window.addEventListener('beforeunload', () => {
            PeekController.stop();
            VoiceAssistant.stopListening();
            WebRTCViewer.stopAll();
        });
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', boot, { once: true });
    } else {
        boot();
    }
})();

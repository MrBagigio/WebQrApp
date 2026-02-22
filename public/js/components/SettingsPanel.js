export class SettingsPanel {
    constructor(engine) {
        this.engine = engine;
        this.visible = false;
        this.container = null;
        this.toggleBtn = null;
        
        // Cache for manual values when switching modes
        this.manualResponsivity = 0.5;
        
        this._init();
    }

    _init() {
        this._createMarkup();
        this._bindEvents();
        this._updateUIFromEngine();
    }

    _createMarkup() {
        // 1. Settings Button (Gear)
        this.toggleBtn = document.createElement('button');
        this.toggleBtn.id = 'settings-open-btn';
        this.toggleBtn.className = 'icon-btn';
        this.toggleBtn.innerHTML = `
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <circle cx="12" cy="12" r="3"></circle>
                <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"></path>
            </svg>
        `;
        
        // Append button to toolbar if exists, otherwise body
        const toolbar = document.querySelector('.ar-toolbar');
        if (toolbar) {
            toolbar.insertBefore(this.toggleBtn, toolbar.firstChild);
        } else {
            document.body.appendChild(this.toggleBtn);
        }

        // 2. Panel Container
        this.container = document.createElement('div');
        this.container.id = 'settings-panel';
        this.container.className = 'settings-panel hidden';
        this.container.innerHTML = `
            <div class="settings-header">
                <h3>Motore AR</h3>
                <button id="settings-close-btn" class="icon-btn small">✕</button>
            </div>
            
            <div class="settings-content">
                <!-- Adaptive Tuning -->
                <div class="settings-group">
                    <div class="settings-row">
                        <label for="stp-adaptive">Adattivo (Auto-Tune)</label>
                        <input type="checkbox" id="stp-adaptive" checked>
                    </div>
                    <p class="settings-desc">Regola automaticamente la reattività in base alla stabilità del tracciamento.</p>
                </div>

                <!-- Manual Responsivity (Visible only if Adaptive is OFF) -->
                <div class="settings-group" id="stp-manual-group">
                    <div class="settings-header-row">
                        <label for="stp-responsivity">Reattività</label>
                        <span id="stp-resp-val">0.5</span>
                    </div>
                    <input type="range" id="stp-responsivity" min="0.01" max="1.0" step="0.01" value="0.5">
                    <div class="scale-labels">
                        <span>STABILE</span>
                        <span>REATTIVO</span>
                    </div>
                </div>

                <!-- Debug Overlay -->
                <div class="settings-group">
                    <div class="settings-row">
                        <label for="stp-debug">Debug Overlay (HUD)</label>
                        <input type="checkbox" id="stp-debug">
                    </div>
                </div>

                <!-- Version Info -->
                <div class="settings-footer">
                    <span>Engine v2.1.0</span>
                    <button id="stp-reset-btn" class="text-btn">RESET DEFAULT</button>
                </div>
            </div>
        `;
        document.body.appendChild(this.container);
    }

    _bindEvents() {
        // Toggle Panel
        this.toggleBtn.addEventListener('click', () => this.toggle());
        this.container.querySelector('#settings-close-btn').addEventListener('click', () => this.close());

        // Adaptive Tune Checkbox
        const adaptiveCb = this.container.querySelector('#stp-adaptive');
        adaptiveCb.addEventListener('change', (e) => {
            const enabled = e.target.checked;
            this.engine.setAdaptiveTuningEnabled(enabled);
            this._updateManualGroupVisibility(enabled);
        });

        // Debug Overlay Checkbox
        const debugCb = this.container.querySelector('#stp-debug');
        debugCb.addEventListener('change', (e) => {
            this.engine.setDebugOverlayEnabled(e.target.checked);
        });

        // Manual Responsivity Slider
        const respSlider = this.container.querySelector('#stp-responsivity');
        const respVal = this.container.querySelector('#stp-resp-val');
        
        respSlider.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            this.manualResponsivity = val;
            respVal.textContent = val.toFixed(2);
            
            // If adaptive is off, apply immediately
            if (!this.engine.adaptiveTuningEnabled) {
                // We assume there's a method for this, or we fallback to setting specific params
                // Since engine might not have a direct "setResponsivity", we map it to alpha/tau
                // High responsivity = High alpha / Low Tau
                // Low responsivity = Low alpha / High Tau
                this._applyManualResponsivity(val);
            }
        });

        // Reset
        this.container.querySelector('#stp-reset-btn').addEventListener('click', () => {
            adaptiveCb.checked = true;
            debugCb.checked = false;
            respSlider.value = 0.5;
            
            this.engine.setAdaptiveTuningEnabled(true);
            this.engine.setDebugOverlayEnabled(false);
            this._updateManualGroupVisibility(true);
        });
    }

    _applyManualResponsivity(val) {
        // Map 0..1 to engine parameters
        // Val 1.0 (Reactive) -> Low smoothing
        // Val 0.0 (Stable) -> High smoothing
        
        // responsiveness ~ 1 - smoothing
        const smoothing = 1.0 - val;
        // Clamp for safety (engine usually expects 0.05 - 0.9)
        const safeSmoothing = Math.max(0.02, Math.min(0.95, smoothing));
        
        // Similar for rotationTimeConstant (lower is faster)
        // 0.03 (fast) ... 0.3 (slow)
        const rotTimeConstant = 0.3 - (val * 0.27); 

        if (typeof this.engine.setFilterParams === 'function') {
            this.engine.setFilterParams({
                positionSmoothing: safeSmoothing,
                rotationTimeConstant: rotTimeConstant
            });
        }
    }

    _updateManualGroupVisibility(adaptiveEnabled) {
        const group = this.container.querySelector('#stp-manual-group');
        group.style.opacity = adaptiveEnabled ? '0.5' : '1';
        group.style.pointerEvents = adaptiveEnabled ? 'none' : 'auto';
    }

    _updateUIFromEngine() {
        // Sync UI state with Engine state on init
        const adaptiveCb = this.container.querySelector('#stp-adaptive');
        const debugCb = this.container.querySelector('#stp-debug');
        
        adaptiveCb.checked = !!this.engine._adaptiveTuningEnabled; // accessing private for read
        debugCb.checked = !!this.engine._debugOverlayEnabled;
        
        this._updateManualGroupVisibility(adaptiveCb.checked);
    }

    toggle() {
        this.visible = !this.visible;
        this.container.classList.toggle('hidden', !this.visible);
        if (this.visible) this._updateUIFromEngine();
    }

    close() {
        this.visible = false;
        this.container.classList.add('hidden');
    }
}

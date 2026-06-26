// External quick-access toolbar.
//
// The sidebar (#panel) keeps every detailed control; this top-center bar surfaces the
// handful of things people toggle constantly so they don't have to open the panel. Each
// toolbar button DRIVES the underlying sidebar checkbox (dispatching its native `change`
// event so the existing app.js handlers run) and reflects its state — and toggling the
// sidebar checkbox updates the button too, so the two never drift apart. Loaded after
// app.js, so those handlers are already attached by the time we wire up.

// [toolbar button id, sidebar checkbox id]
const TOGGLES = [
	['tb-buildings', 'buildings-visible'],
	['tb-roads', 'road-visible'],
	['tb-signals', 'road-signals'],
	['tb-transit', 'transit-visible'],
	['tb-cameras', 'cameras-visible'],
	['tb-detections', 'road-cars'],
];

for (const [btnId, cbId] of TOGGLES) {
	const btn = document.getElementById(btnId);
	const cb = document.getElementById(cbId);
	if (!btn || !cb) continue;
	const reflect = () => btn.classList.toggle('on', cb.checked);
	reflect();
	btn.addEventListener('click', () => {
		cb.checked = !cb.checked;
		cb.dispatchEvent(new Event('change', { bubbles: true }));   // run the sidebar's handler
		reflect();
	});
	cb.addEventListener('change', reflect);   // stay in sync if toggled from the sidebar
}

// Sidebar show/hide — "keep the sidebar but get it out of the way".
const panelBtn = document.getElementById('tb-panel');
const panel = document.getElementById('panel');
if (panelBtn && panel) {
	let shown = true;
	const reflectPanel = () => { panel.style.display = shown ? '' : 'none'; panelBtn.classList.toggle('on', shown); };
	reflectPanel();
	panelBtn.addEventListener('click', () => { shown = !shown; reflectPanel(); });
}

// Reset view — reuse the sidebar's Reset button so there's a single code path.
const resetBtn = document.getElementById('tb-reset');
const resetSrc = document.getElementById('camera-reset');
if (resetBtn && resetSrc) resetBtn.addEventListener('click', () => resetSrc.click());

// TARS bench shell: loads every panel in /panels.json, gives them a shared
// event stream and API helper, and lays the page out. A panel file only calls Bench.register({...}).
//
//   Bench.register({
//     id: 'drive', title: 'Drive', order: 10,   // lower order = earlier in its area
//     area: 'side',                             // where it goes (below); default 'more'
//     help: 'one line saying what this is for', // shown under the title
//     view: 'Room',                             // area 'view' only: the label on the view switch
//     wide: false,                              // legacy, ignored by the layout
//     mount(el, Bench) { ... }                  // build the DOM inside el
//   });
//   Areas: 'header' (bits of the top bar), 'side' (always-visible left column: talk + drive),
//          'view' (the big centre view; several = a switch above it), or a drawer tab id from
//          TABS below ('robot', 'pi', 'sim', 'maps', 'voice', 'tests', 'debug'; an unknown id
//          makes a new tab, titled tabTitle or the id).
//   Bench.on('status', data => ...)   // server-sent event topic (also local Bench.emit topics)
//   Bench.emit('topic', data)         // local event to every Bench.on('topic') handler in this tab
//   await Bench.api('drive/cmd', {line: 'S'})   // POST /api/drive/cmd (GET if no body)
//   Bench.log('drive', 'text')        // local line in the log
//   Bench.openTab('debug') / Bench.closeDrawer() / Bench.showView('world2d')
//   Bench.setBadgeDetail('virtual SM11')        // text after SIM in the badge
//   Bench.mic.start() / await Bench.mic.stop() -> 16 kHz mono WAV ArrayBuffer (or null)
'use strict';
const Bench = (() => {
  const handlers = {};
  const panels = [];
  let es = null;
  let info = {panels: [], plugins: [], target: {}};

  // drawer tabs, in this order; help = the one-line explanation at the top of the drawer
  const TABS = [
    ['robot', 'Robot status', 'What the robot’s brain reports: go-to state and object memory, the safety guard, tars_motor’s raw STATUS, goto’s own map.'],
    ['pi', 'Pi & camera', 'Real-robot links: the phone camera through the Pi, the robot’s own voice input, Pi services and journals.'],
    ['sim', 'Sim room & car', 'SIMULATION controls: edit the room, inject sensor faults and noise, and poke the virtual car (faults, battery, motor guesses).'],
    ['maps', 'Map demos', 'Build a map, then go find things: explore live from nothing, or load a prescan (sim only).'],
    ['voice', 'Voice lab', 'Test the speech pipeline in depth: exactly what the classifier decides, what lines it sends, batch accuracy with noise.'],
    ['tests', 'Tests & recorder', 'Scripted bring-up checks (the PI_TEST.md steps) with pass/fail, and record/replay of everything.'],
    ['debug', 'Debug & log', 'Wheel-speed chart, a raw command console to tars_motor / goto / voice, and the full event log.'],
  ];
  const tabHelp = {};
  const tabOrder = [];

  function on(topic, fn) {
    (handlers[topic] = handlers[topic] || []).push(fn);
    if (es) es.addEventListener(topic, ev => fn(JSON.parse(ev.data)));
  }
  function emit(topic, data) {
    for (const fn of handlers[topic] || []) { try { fn(data); } catch (e) { console.error(e); } }
  }

  async function api(path, body) {
    const opt = body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: typeof body === 'string' ? body : JSON.stringify(body)};
    const r = await fetch('/api/' + path, opt);
    const ct = r.headers.get('Content-Type') || '';
    const data = ct.includes('json') ? await r.json() : await r.text();
    if (!r.ok) throw new Error((data && data.error) || r.status);
    return data;
  }

  const logLines = [];
  function log(src, text) {
    const t = new Date().toLocaleTimeString();
    const s = `${t} [${src}] ${text}`;
    logLines.unshift(s);
    if (logLines.length > 300) logLines.pop();
    const tk = document.getElementById('ticker-text');
    if (tk) tk.textContent = s;
    const box = document.getElementById('log');
    if (!box) return;
    const line = document.createElement('div');
    line.textContent = s;
    box.prepend(line);
    while (box.childElementCount > 300) box.lastChild.remove();
  }

  // ---------------------------------------------------------------- layout
  function ensureTab(id, title) {
    let body = document.querySelector(`#drawer-body [data-tab="${id}"]`);
    if (body) return body;
    const def = TABS.find(t => t[0] === id);
    title = (def && def[1]) || title || id;
    tabHelp[id] = {title, help: (def && def[2]) || ''};
    body = document.createElement('div');
    body.className = 'tab-body'; body.dataset.tab = id; body.hidden = true;
    document.getElementById('drawer-body').appendChild(body);
    const b = document.createElement('button');
    b.className = 'tab'; b.dataset.tab = id; b.textContent = title;
    b.title = tabHelp[id].help;
    b.onclick = () => (openTabId === id ? closeDrawer() : openTab(id));
    // keep the TABS order regardless of which panel registers first
    const rank = k => { const i = TABS.findIndex(t => t[0] === k); return i < 0 ? 99 : i; };
    const bar = document.getElementById('tabbar');
    const after = [...bar.querySelectorAll('button.tab')].find(x => rank(x.dataset.tab) > rank(id));
    bar.insertBefore(b, after || null);
    tabOrder.push(id);
    if (savedTab === id) openTab(id);
    return body;
  }

  let openTabId = null;
  const savedTab = (() => { try { return localStorage.getItem('bench.tab'); } catch (e) { return null; } })();
  function openTab(id) {
    if (!document.querySelector(`#drawer-body [data-tab="${id}"]`)) return;
    openTabId = id;
    for (const b of document.querySelectorAll('#drawer-body .tab-body')) b.hidden = b.dataset.tab !== id;
    for (const b of document.querySelectorAll('#tabbar button.tab')) b.classList.toggle('on', b.dataset.tab === id);
    document.getElementById('drawer-title').textContent = tabHelp[id].title;
    document.getElementById('drawer-help').textContent = tabHelp[id].help;
    document.getElementById('drawer').hidden = false;
    document.body.classList.add('drawer-open');
    try { localStorage.setItem('bench.tab', id); } catch (e) { /* private mode */ }
    relayout();
  }
  function closeDrawer() {
    openTabId = null;
    document.getElementById('drawer').hidden = true;
    document.body.classList.remove('drawer-open');
    for (const b of document.querySelectorAll('#tabbar button.tab')) b.classList.remove('on');
    try { localStorage.removeItem('bench.tab'); } catch (e) { /* private mode */ }
    relayout();
  }
  // canvases size themselves on window resize
  const relayout = () => requestAnimationFrame(() => window.dispatchEvent(new Event('resize')));

  let shownView = null;
  function showView(id, persist = true) {
    const v = document.querySelector(`#views [data-view="${id}"]`);
    if (!v) return;
    shownView = id;
    for (const x of document.querySelectorAll('#views > .view')) x.hidden = x.dataset.view !== id;
    for (const b of document.querySelectorAll('#viewbar button')) b.classList.toggle('on', b.dataset.view === id);
    if (persist) try { localStorage.setItem('bench.view', id); } catch (e) { /* private mode */ }
    emit('view', id);
    relayout();
  }

  function section(p) {
    const sec = document.createElement('section');
    sec.className = 'panel';
    sec.id = 'panel-' + p.id;
    sec.style.order = p.order ?? 50;
    const h = document.createElement('h2'); h.textContent = p.title || p.id;
    sec.appendChild(h);
    if (p.help) { const d = document.createElement('div'); d.className = 'panel-help'; d.textContent = p.help; sec.appendChild(d); }
    const body = document.createElement('div'); body.className = 'body';
    sec.appendChild(body);
    return [sec, body];
  }

  function register(p) {
    panels.push(p);
    const area = p.area || 'more';
    let mountEl;
    if (area === 'header') {
      mountEl = document.createElement('span');
      mountEl.id = 'panel-' + p.id;
      mountEl.className = 'hdr-panel';
      document.getElementById('topbar').insertBefore(mountEl, document.querySelector('#topbar .spacer'));
    } else if (area === 'view') {
      const v = document.createElement('div');
      v.className = 'view'; v.dataset.view = p.id; v.id = 'panel-' + p.id; v.style.order = p.order ?? 50;
      v.hidden = true;
      document.getElementById('views').appendChild(v);
      const b = document.createElement('button');
      b.dataset.view = p.id; b.textContent = p.view || p.title || p.id; b.style.order = p.order ?? 50;
      b.title = p.help || ''; b.setAttribute('role', 'tab');
      b.onclick = () => showView(p.id);
      document.getElementById('viewbar').appendChild(b);
      mountEl = v;
      if (!shownView) showView(p.id, false);
    } else {
      const [sec, body] = section(p);
      const parent = area === 'side' ? document.getElementById('side') : ensureTab(area, p.tabTitle);
      parent.appendChild(sec);
      mountEl = body;
    }
    try { p.mount(mountEl, Bench); }
    catch (e) { mountEl.textContent = 'panel failed: ' + e; console.error(e); }
  }

  // the shell's own log panel (Debug tab)
  function logPanel() {
    register({id: 'log', title: 'Log', order: 90, area: 'debug', help: 'Every message from every part, newest first.',
      mount(el) {
        const box = document.createElement('div'); box.id = 'log'; box.setAttribute('aria-label', 'log');
        el.appendChild(box);
        for (const s of logLines.slice().reverse()) { const d = document.createElement('div'); d.textContent = s; box.prepend(d); }
      }});
  }

  function connect() {
    es = new EventSource('/events');
    for (const [topic, fns] of Object.entries(handlers))
      for (const fn of fns) es.addEventListener(topic, ev => fn(JSON.parse(ev.data)));
    es.addEventListener('log', ev => { const d = JSON.parse(ev.data); log(d.src, d.text); });
    es.onopen = () => document.body.classList.remove('offline');
    es.onerror = () => document.body.classList.add('offline');
  }

  let badgeBase = '';
  function setBadgeDetail(detail) {
    const b = document.getElementById('badge');
    if (b) b.textContent = badgeBase + (detail ? ' · ' + detail : '');
  }

  async function boot() {
    info = await (await fetch('/panels.json')).json();
    const t = info.target || {};
    // SIM vs REAL ROBOT: always visible, so nobody mistakes the sim for the car (or the reverse)
    const badge = document.getElementById('badge');
    const real = t.name === 'pi' || t.name === 'usb';
    badgeBase = t.name === 'pi' ? 'REAL ROBOT (Pi)' : t.name === 'usb' ? 'REAL ROBOT (Uno on USB)' : 'SIM';
    badge.textContent = badgeBase;
    badge.className = 'badge ' + (real ? 'real' : 'sim');
    badge.title += `\ntarget: ${t.name}` + (t.name === 'pi' ? ` (relay ${t.relay})` : ` (motor ${t.motor})`) +
      (info.hidden && info.hidden.length ? `\nsim-only panels hidden: ${info.hidden.join(', ')}` : '');
    document.body.classList.toggle('real-robot', real);
    document.title = (real ? 'REAL ROBOT · ' : 'SIM · ') + 'TARS bench';
    for (const [id] of TABS) if (id === 'debug') ensureTab(id);
    logPanel();
    connect();
    for (const f of info.panels) {
      await new Promise(res => {
        const s = document.createElement('script');
        s.src = '/panels/' + f; s.onload = res;
        s.onerror = () => { log('shell', 'panel failed to load: ' + f); res(); };
        document.body.appendChild(s);
      });
    }
    // the saved view if it still exists, else the lowest-order one
    const views = [...document.querySelectorAll('#views > .view')];
    let want = null; try { want = localStorage.getItem('bench.view'); } catch (e) { /* private mode */ }
    if (views.length) showView(views.some(v => v.dataset.view === want) ? want
      : views.sort((a, b) => (+a.style.order) - (+b.style.order))[0].dataset.view, false);
    if (!views.length) {
      document.getElementById('views').innerHTML = '<div class="empty">No map view on this target. Open <b>More tools → Robot status</b> for goto’s map.</div>';
    }
    document.getElementById('viewbar').hidden = document.querySelectorAll('#viewbar button').length < 2;
  }

  // ---------------------------------------------------------------- mic (shared by every panel)
  // Records from the laptop mic, decimates to 16 kHz mono 16-bit and returns a WAV (the same
  // capture as mission control). One recording at a time; Bench.on('mic', {on}) follows it.
  const mic = (() => {
    let rec = null;
    async function start() {
      if (rec) return;
      rec = {chunks: [], stopping: false};
      emit('mic', {on: true});
      try {
        const stream = await navigator.mediaDevices.getUserMedia({audio: {channelCount: 1, echoCancellation: true, noiseSuppression: true}});
        const ctx = new AudioContext(), src = ctx.createMediaStreamSource(stream), node = ctx.createScriptProcessor(4096, 1, 1);
        node.onaudioprocess = ev => rec && rec.chunks.push(new Float32Array(ev.inputBuffer.getChannelData(0)));
        src.connect(node); node.connect(ctx.destination);
        Object.assign(rec, {stream, ctx, src, node, rate: ctx.sampleRate});
        if (rec.stopping) rec.resolveEarly && rec.resolveEarly();
      } catch (e) { rec = null; emit('mic', {on: false, error: 'no microphone: ' + e.message}); throw e; }
    }
    async function stop() {
      if (!rec) return null;
      if (!rec.ctx) {           // released before the mic opened: wait for it, then stop
        rec.stopping = true;
        await new Promise(res => { rec.resolveEarly = res; setTimeout(res, 3000); });
        if (!rec || !rec.ctx) { rec = null; emit('mic', {on: false}); return null; }
      }
      const r = rec; rec = null;
      emit('mic', {on: false});
      r.node.disconnect(); r.src.disconnect(); r.stream.getTracks().forEach(t => t.stop()); r.ctx.close();
      const n = r.chunks.reduce((a, c) => a + c.length, 0), all = new Float32Array(n);
      let o = 0; for (const c of r.chunks) { all.set(c, o); o += c.length; }
      const step = r.rate / 16000, out = new Int16Array(Math.floor(n / step));   // average-decimate to 16 kHz
      for (let i = 0; i < out.length; i++) {
        const a = Math.floor(i * step), b = Math.max(a + 1, Math.floor((i + 1) * step));
        let s = 0; for (let j = a; j < b; j++) s += all[j]; s /= (b - a); out[i] = Math.max(-1, Math.min(1, s)) * 32767;
      }
      if (out.length < 1600) { log('mic', 'clip under 0.1 s: ignored'); return null; }
      return wavOf(out);
    }
    function wavOf(pcm) {
      const b = new ArrayBuffer(44 + pcm.length * 2), v = new DataView(b);
      const w = (o, s) => [...s].forEach((c, i) => v.setUint8(o + i, c.charCodeAt(0)));
      w(0, 'RIFF'); v.setUint32(4, 36 + pcm.length * 2, true); w(8, 'WAVE'); w(12, 'fmt ');
      v.setUint32(16, 16, true); v.setUint16(20, 1, true); v.setUint16(22, 1, true); v.setUint32(24, 16000, true);
      v.setUint32(28, 32000, true); v.setUint16(32, 2, true); v.setUint16(34, 16, true); w(36, 'data');
      v.setUint32(40, pcm.length * 2, true);
      new Int16Array(b, 44).set(pcm);
      return b;
    }
    return {start, stop, get recording() { return !!rec; }};
  })();

  // ---------------------------------------------------------------- STOP + keys
  // STOP lives in the shell, not a panel, so it works even if every panel failed.
  // The robot plugin owns /api/robot/stop (S to tars_motor, then CANCEL to goto).
  function stop() {
    const b = document.getElementById('stop');
    b.classList.add('flash'); setTimeout(() => b.classList.remove('flash'), 400);
    api('robot/stop', {}).then(r => log('stop', JSON.stringify(r)))
      .catch(e => log('stop', 'STOP FAILED: ' + e.message));
  }
  const typing = t => { const tag = (t.tagName || '').toLowerCase(); return tag === 'input' || tag === 'textarea' || tag === 'select'; };
  window.addEventListener('keydown', ev => {
    if (ev.code === 'Escape' && !typing(ev.target)) {
      const help = document.getElementById('help');
      if (help && !help.hidden) { hideHelp(); return; }
      if (openTabId) closeDrawer();
      return;
    }
    if (ev.code !== 'Space' || ev.repeat) return;
    if (typing(ev.target) && ev.target.type !== 'range' && ev.target.type !== 'checkbox') return;
    ev.preventDefault();
    stop();
  });
  function showHelp() { document.getElementById('help').hidden = false; document.getElementById('help-close').focus(); }
  function hideHelp() {
    document.getElementById('help').hidden = true;
    try { localStorage.setItem('bench.helpSeen', '1'); } catch (e) { /* private mode */ }
  }
  window.addEventListener('DOMContentLoaded', () => {
    document.getElementById('stop').addEventListener('click', stop);
    document.getElementById('help-btn').addEventListener('click', showHelp);
    document.getElementById('help-close').addEventListener('click', hideHelp);
    document.getElementById('help').addEventListener('click', ev => { if (ev.target.id === 'help') hideHelp(); });
    document.getElementById('drawer-close').addEventListener('click', closeDrawer);
    document.getElementById('ticker').addEventListener('click', () => openTab('debug'));
    let seen = null; try { seen = localStorage.getItem('bench.helpSeen'); } catch (e) { /* private mode */ }
    if (!seen && !/[?&]nohelp\b/.test(location.search)) showHelp();
    boot();
  });
  return {on, emit, api, log, register, openTab, closeDrawer, showView, setBadgeDetail, mic,
    get info() { return info; }, get panels() { return panels; }};
})();

// TARS bench shell: loads every panel in /panels.json, gives them a shared
// event stream and API helper, and lays the page out as a navbar of pages (one job each).
// A panel file only calls Bench.register({...}).
//
//   Bench.register({
//     id: 'drive', title: 'Drive', order: 10,   // lower order = earlier on its page
//     area: 'drive',                            // which page (below); 'view' = the map; 'header' = top bar
//     help: 'one line saying what this is for', // behind the ⓘ next to the title (collapsed; remembered per panel)
//     view: 'Room',                             // area 'view' only: the label on the map's view switch
//     wide: false,                              // full-width pages: span every column
//     tall: false,                              // grid pages: span two rows (a long panel beside two short ones)
//     mount(el, Bench) { ... }                  // build the DOM inside el
//   });
//   Pages: PAGES below ('drive', 'room', 'memory', 'voice', 'car', 'tests', 'mission'). Old drawer tab ids
//   still work (LEGACY: side→drive, robot→memory, sim/maps→room, pi→mission, debug→tests); an unknown id
//   makes a new page titled tabTitle or the id. Pages with map: true show the map (area 'view') beside
//   their column; the others use the full width. Pages with sub: true show one panel at a time behind
//   sub-tabs (the panel titles), so a page is never one long scroll. Every page has a collapsible
//   "What's this page for?" box (ABOUT below), open on the first visit, then as the user left it.
//   Bench.on('status', data => ...)   // server-sent event topic (also local Bench.emit topics)
//   Bench.on('page', id => ...)       // local: the shown page changed (also fired once at boot)
//   Bench.emit('topic', data)         // local event to every Bench.on('topic') handler in this tab
//   await Bench.api('drive/cmd', {line: 'S'})   // POST /api/drive/cmd (GET if no body)
//   Bench.log('drive', 'text')        // local line in the log
//   Bench.openPage('room') / Bench.page / Bench.showView('world2d')   (openTab = openPage, legacy)
//   Bench.setBadgeDetail('virtual SM11')        // text after SIM in the badge
//   Bench.mic.start() / await Bench.mic.stop() -> 16 kHz mono WAV ArrayBuffer (or null)
'use strict';
const Bench = (() => {
  const handlers = {};
  const panels = [];
  let es = null;
  let info = {panels: [], plugins: [], target: {}};

  // pages, in navbar order: [id, title, help (navbar tooltip), map shown?, default map view, sub-tabs?]
  const PAGES = [
    ['drive', '🕹 Drive', 'Talk to TARS or drive by hand, and watch the map.', true, null],
    ['room', '🧱 Room', 'SIMULATION: edit the sim room (add, move, delete things), load or save rooms, break sensors on purpose, map demos.', true, 'world2d', true],
    ['memory', '🧠 Memory', 'What the robot’s go-to controller remembers and where. Forget one thing or everything; send GOTO / SCAN directly.', true, 'mc-map'],
    ['voice', '🎙 Voice lab', 'The speech pipeline in depth: what the classifier decides, which lines it sends, batch accuracy with noise.', false, null],
    ['car', '🚗 Car', 'The car: readings, the safety guard, tars_motor’s raw STATUS, and (sim) faults and motor guesses.', false, null],
    ['tests', '✅ Tests', 'Scripted bring-up checks with pass/fail, record/replay, the wheel chart, a raw command console and the full log.', false, null, true],
    ['mission', '📡 Mission', 'Real-robot mission control: the phone camera through the Pi, the robot’s own voice log, Pi services and journals.', false, null],
  ];
  // the "What's this page for?" box at the top of each page: what it's for, what you can do, one tip
  const ABOUT = {
    drive: ['Drive the car: tell TARS what to do in words, or drive it by hand. The map on the right shows where it is.',
      '<b>Talk to TARS</b>: type a command or hold <kbd>T</kbd> and speak, e.g. “go to the trash can”. Turn <b>Send to robot</b> off for a dry run.',
      '<b>Drive by hand</b>: hold <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> or press the pad. Exact distances and angles are under “Exact moves”.',
      'Tip: if Path says BLOCKED, <kbd>S</kbd> (reverse) still works. <kbd>Space</kbd>, <kbd>Esc</kbd> or STOP stops everything.'],
    room: ['Build the simulated room the car drives in. Sim only: nothing here touches a real robot.',
      'Add boxes, walls, people and objects with the tools above the map. <b>Things in the room</b> lists them all, with 🗑 to delete (or right-click on the map).',
      '<b>Faults</b> breaks sensors on purpose; <b>Map demos</b> runs explore-from-nothing and saved-map (prescan) demos.',
      'Tip: <b>🚗 Car to start</b> moves only the car back (your walls and goto’s map stay). <b>🗺 Reset map</b> puts the room back as loaded and clears goto’s map; the car stays put.'],
    memory: ['What the robot remembers seeing, and where. “Go to the trash can” drives back to these spots.',
      '<b>What the robot remembers</b>: Forget one object or all of them. Anything the camera still sees is learned again at once.',
      '<b>Go-to controller</b>: goto’s state now, direct GOTO / RECALL / SCAN / CANCEL, and its event log.',
      'Tip: the map on the right is the robot’s own map (what it believes), not the sim’s truth.'],
    voice: ['Test the speech pipeline: type or speak a phrase and see exactly what the classifier decides and which lines it sends.',
      'With <b>send to robot</b> on, the phrase also goes to the robot’s tars_voice, so the car acts on it. Off = dry run.',
      'Open <b>batch</b> to measure accuracy, confusion and latency over many phrases, with added noise.',
      'Tip: hold <kbd>T</kbd> anywhere on this page to talk.'],
    car: ['The car itself: the obstacle guard, the virtual car’s true state, and every field of tars_motor’s raw STATUS.',
      'Sim only: push, tip or brown out the car under <b>Faults</b>, and try motor guesses under <b>Profile</b> (nothing measured yet).',
      'Tip: if <b>Safety</b> says BLOCKED, the LiDAR guard is refusing forward drives. Clicking <b>Path</b> in the status strip opens this page.'],
    tests: ['Bring-up checks for when the real car arrives, and the tools to see why one failed.',
      '<b>Scenarios</b>: one click runs a scripted check and shows pass/fail with measured numbers. <b>Wheels</b>: commanded vs real wheel speed, plus a raw command console.',
      '<b>Recorder</b> saves everything and replays it without sending anything to the robot. <b>Log</b> has every message, newest first.',
      'Tip: STOP, <kbd>Space</kbd> or <kbd>Esc</kbd> aborts a running check.'],
    mission: ['Mission control for the real robot: the phone camera through the Pi, the robot’s own voice log, the phone link and Pi services.',
      'In SIM most of this stays empty: there is no robot camera and no Pi.',
      'Tip: hold <kbd>V</kbd> to talk straight to the robot’s tars_voice.'],
  };
  const LEGACY = {side: 'drive', robot: 'memory', sim: 'room', maps: 'room', pi: 'mission', debug: 'tests', more: 'tests'};
  const pageDef = {};
  const pageOrder = [];
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
  // Each page is a div in #pages; pages with map:true also show #center (the map) beside it.
  // Hidden pages stay in the DOM (display:none), so panels keep their state and keep listening.
  function ensurePage(id, title) {
    id = LEGACY[id] || id;
    let body = document.querySelector(`#pages > .page[data-page="${id}"]`);
    if (body) return body;
    const def = PAGES.find(t => t[0] === id);
    pageDef[id] = {title: (def && def[1]) || title || id, help: (def && def[2]) || '', map: !!(def && def[3]), view: def && def[4],
                   sub: !!(def && def[5])};
    body = document.createElement('div');
    body.className = 'page'; body.dataset.page = id; body.id = 'page-' + id; body.hidden = true;
    body.setAttribute('role', 'tabpanel');
    aboutBox(id);
    if (pageDef[id].sub) {
      const strip = document.createElement('div'); strip.className = 'subtabs'; strip.setAttribute('role', 'tablist');
      strip.setAttribute('aria-label', pageDef[id].title + ' sections');
      body.appendChild(strip); body.classList.add('has-sub');
    }
    const grid = document.createElement('div'); grid.className = 'page-grid';
    body.appendChild(grid);
    document.getElementById('pages').appendChild(body);
    const b = document.createElement('a');
    b.className = 'navtab'; b.dataset.page = id; b.href = '#' + id; b.setAttribute('role', 'tab');
    b.textContent = pageDef[id].title; b.title = pageDef[id].help;
    b.onclick = ev => { ev.preventDefault(); openPage(id); };
    // keep the PAGES order regardless of which panel registers first
    const rank = k => { const i = PAGES.findIndex(t => t[0] === k); return i < 0 ? 99 : i; };
    const bar = document.getElementById('navbar');
    const after = [...bar.querySelectorAll('.navtab')].find(x => rank(x.dataset.page) > rank(id));
    bar.insertBefore(b, after || null);
    pageOrder.splice(0, pageOrder.length, ...[...bar.querySelectorAll('.navtab')].map(x => x.dataset.page));
    return body;
  }
  const pageGrid = (id, title) => ensurePage(id, title).querySelector('.page-grid');

  // "What's this page for?": one <details> per page in #about (full width, under the navbar); only the
  // shown page's is visible. Open on the first visit; after that, as the user last left it (per page).
  function aboutBox(id) {
    const host = document.getElementById('about');
    if (!host || host.querySelector(`[data-page="${id}"]`)) return;
    const lines = ABOUT[id] || (pageDef[id].help ? [pageDef[id].help] : []);
    if (!lines.length) return;
    const d = document.createElement('details');
    d.className = 'about'; d.dataset.page = id; d.hidden = true;
    d.open = store.get('bench.about.' + id) !== 'closed';
    const sm = document.createElement('summary');
    sm.innerHTML = `<span class="about-q">What’s this page for?</span> <span class="about-short"></span>`;
    sm.querySelector('.about-short').textContent = pageDef[id].help;   // a one-line reminder while it's closed
    d.appendChild(sm);
    const ul = document.createElement('ul');
    for (const l of lines) { const li = document.createElement('li'); li.innerHTML = l; ul.appendChild(li); }   // our own text, not user input
    d.appendChild(ul);
    d.addEventListener('toggle', () => { store.set('bench.about.' + id, d.open ? 'open' : 'closed'); relayout(); });
    host.appendChild(d);
  }

  // sub-tabs: on a page with sub: true, one panel shows at a time; the strip lists the panel titles in order
  const subSel = {};
  function refreshSubtabs(id) {
    const pg = document.querySelector(`#pages > .page[data-page="${id}"]`);
    const strip = pg && pg.querySelector('.subtabs');
    if (!strip) return;
    const secs = [...pg.querySelectorAll('.page-grid > .panel')].sort((a, b) => (+a.style.order) - (+b.style.order));
    strip.textContent = '';
    for (const sec of secs) {
      const b = document.createElement('button');
      b.className = 'subtab'; b.dataset.panel = sec.dataset.panel; b.setAttribute('role', 'tab');
      b.textContent = sec.dataset.short || sec.querySelector('h2 .ttl').textContent;
      b.title = sec.dataset.help || '';
      b.onclick = () => openSub(id, sec.dataset.panel);
      strip.appendChild(b);
    }
    const want = [subSel[id], store.get('bench.sub.' + id), secs[0] && secs[0].dataset.panel]
      .find(p => p && secs.some(s => s.dataset.panel === p));
    if (want) openSub(id, want, false);
  }
  function openSub(page, panelId, persist = true) {
    page = LEGACY[page] || page;
    const pg = document.querySelector(`#pages > .page[data-page="${page}"]`);
    if (!pg || !pg.querySelector('.subtabs')) return false;
    if (persist) subSel[page] = panelId;   // an explicit choice sticks; the default follows the panel order
    for (const sec of pg.querySelectorAll('.page-grid > .panel')) sec.classList.toggle('sub-off', sec.dataset.panel !== panelId);
    for (const b of pg.querySelectorAll('.subtabs .subtab')) {
      const on = b.dataset.panel === panelId;
      b.classList.toggle('on', on); b.setAttribute('aria-selected', on ? 'true' : 'false');
    }
    if (persist) store.set('bench.sub.' + page, panelId);
    relayout();
    return true;
  }
  // show a panel wherever it is: its page, and its sub-tab if the page has them
  function showPanel(panelId) {
    const sec = document.getElementById('panel-' + panelId);
    const pg = sec && sec.closest('#pages > .page');
    if (!pg) return false;
    openPage(pg.dataset.page);
    openSub(pg.dataset.page, panelId);
    sec.scrollIntoView({block: 'nearest'});
    return true;
  }

  let curPage = null;
  const store = {get(k) { try { return localStorage.getItem(k); } catch (e) { return null; } },
                 set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* private mode */ } }};
  function openPage(id, {hash = true} = {}) {
    id = LEGACY[id] || id;
    const pg = document.querySelector(`#pages > .page[data-page="${id}"]`);
    if (!pg) return false;
    curPage = id;
    for (const x of document.querySelectorAll('#pages > .page')) x.hidden = x.dataset.page !== id;
    for (const x of document.querySelectorAll('#about > .about')) x.hidden = x.dataset.page !== id;
    for (const b of document.querySelectorAll('#navbar .navtab')) {
      const on = b.dataset.page === id;
      b.classList.toggle('on', on); b.setAttribute('aria-selected', on ? 'true' : 'false');
    }
    const def = pageDef[id] || {};
    document.body.dataset.page = id;
    document.body.classList.toggle('with-map', !!def.map);
    document.getElementById('center').hidden = !def.map;
    store.set('bench.page', id);
    if (hash && location.hash !== '#' + id) history.replaceState(null, '', '#' + id);
    // the map view this page wants: the one last chosen on this page, else the page's default
    if (def.map) {
      const has = v => v && document.querySelector(`#views [data-view="${v}"]`);
      const first = [...document.querySelectorAll('#views > .view')].sort((a, b) => (+a.style.order) - (+b.style.order))[0];
      const want = [store.get('bench.view.' + id), def.view, first && first.dataset.view].find(has);
      if (want && want !== shownView) showView(want, false);
    }
    emit('page', id);
    relayout();
    return true;
  }
  // canvases size themselves on window resize (also after a page or view switch: they were 0×0 while hidden)
  const relayout = () => requestAnimationFrame(() => window.dispatchEvent(new Event('resize')));

  let shownView = null;
  function showView(id, persist = true) {
    const v = document.querySelector(`#views [data-view="${id}"]`);
    if (!v) return;
    shownView = id;
    for (const x of document.querySelectorAll('#views > .view')) x.hidden = x.dataset.view !== id;
    for (const b of document.querySelectorAll('#viewbar button[data-view]')) b.classList.toggle('on', b.dataset.view === id);
    const vh = document.getElementById('view-help');
    if (vh) vh.textContent = (panels.find(p => p.id === id) || {}).help || '';
    if (persist) { store.set('bench.view', id); if (curPage) store.set('bench.view.' + curPage, id); }
    emit('view', id);
    relayout();
  }

  function section(p) {
    const sec = document.createElement('section');
    sec.className = 'panel' + (p.wide ? ' wide' : '');
    sec.id = 'panel-' + p.id;
    if (p.tall) sec.classList.add('tall');
    sec.style.order = p.order ?? 50;
    sec.dataset.panel = p.id;
    if (p.short) sec.dataset.short = p.short;
    const h = document.createElement('h2');
    const ttl = document.createElement('span'); ttl.className = 'ttl'; ttl.textContent = p.title || p.id;
    h.appendChild(ttl);
    sec.appendChild(h);
    if (p.help) {
      // ⓘ next to the title shows/hides a one-line description; closed by default, remembered per panel
      sec.dataset.help = p.help;
      const d = document.createElement('div'); d.className = 'panel-help'; d.textContent = p.help;
      const b = infoButton('bench.help.' + p.id, d, 'what this section does');
      h.appendChild(b);
      sec.appendChild(d);
    }
    const body = document.createElement('div'); body.className = 'body';
    sec.appendChild(body);
    return [sec, body];
  }

  function infoButton(key, target, what) {
    const b = document.createElement('button');
    b.type = 'button'; b.className = 'info-btn'; b.textContent = 'ⓘ';
    const set = open => {
      target.hidden = !open; b.classList.toggle('on', open);
      b.setAttribute('aria-expanded', open ? 'true' : 'false');
      b.title = (open ? 'Hide ' : 'Show ') + what;
    };
    set(store.get(key) === 'open');
    b.onclick = ev => { ev.stopPropagation(); const open = target.hidden; set(open); store.set(key, open ? 'open' : 'closed'); relayout(); };
    return b;
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
      pageGrid(area, p.tabTitle).appendChild(sec);
      mountEl = body;
      refreshSubtabs(LEGACY[area] || area);
    }
    try { p.mount(mountEl, Bench); }
    catch (e) { mountEl.textContent = 'panel failed: ' + e; console.error(e); }
  }

  // the shell's own log panel (Tests page)
  function logPanel() {
    register({id: 'log', title: 'Log', short: '📜 Log', order: 90, area: 'tests', wide: true, help: 'Every message from every part, newest first. The footer shows the latest line.',
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
    for (const [id] of PAGES) ensurePage(id);   // the navbar always lists the pages in PAGES order
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
    const views = [...document.querySelectorAll('#views > .view')];
    if (views.length) {   // the saved view if it still exists, else the lowest-order one
      const want = store.get('bench.view');
      showView(views.some(v => v.dataset.view === want) ? want
        : views.slice().sort((a, b) => (+a.style.order) - (+b.style.order))[0].dataset.view, false);
    } else {
      document.getElementById('views').innerHTML = '<div class="empty">No map view on this target.</div>';
    }
    document.getElementById('viewbar').hidden = document.querySelectorAll('#viewbar button').length < 2;
    if (views.length) {   // ⓘ at the end of the view switch: one line about the map view shown
      const vh = document.createElement('div'); vh.id = 'view-help'; vh.className = 'panel-help view-help';
      document.getElementById('views').before(vh);
      const ib = infoButton('bench.help.views', vh, 'what this map view shows');
      ib.style.order = 999;
      document.getElementById('viewbar').appendChild(ib);
      vh.textContent = (panels.find(p => p.id === shownView) || {}).help || '';
    }
    // a page with nothing on it (e.g. Room on the real robot: its panels are sim-only) leaves the navbar
    for (const pg of document.querySelectorAll('#pages > .page')) {
      const empty = !pg.querySelector('.page-grid > .panel');
      if (empty) { pg.remove(); document.querySelector(`#navbar .navtab[data-page="${pg.dataset.page}"]`)?.remove();
                   document.querySelector(`#about > .about[data-page="${pg.dataset.page}"]`)?.remove(); }
    }
    pageOrder.splice(0, pageOrder.length, ...[...document.querySelectorAll('#navbar .navtab')].map(x => x.dataset.page));
    // the page: the URL hash (#room), else the last one shown, else Drive
    const fromHash = location.hash.slice(1);
    if (!openPage(fromHash) && !openPage(store.get('bench.page'))) openPage(pageOrder[0] || 'drive');
    window.addEventListener('hashchange', () => { const h = location.hash.slice(1); if (h && h !== curPage) openPage(h, {hash: false}); });
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
    // Esc stops too, on every page and even in a text box (stop always wins), except when it has a
    // smaller job right there: close the help card, close a right-click menu, or deselect on the
    // Room page's editing map (focus on a canvas there), or cancel an inline table edit (data-esc="local").
    if (ev.code === 'Escape') {
      if (ev.repeat) return;
      const help = document.getElementById('help');
      if (help && !help.hidden) { hideHelp(); return; }
      if (document.querySelector('.ctxmenu')) return;
      if (curPage === 'room' && (ev.target.tagName || '').toLowerCase() === 'canvas') return;
      if (ev.target.dataset && ev.target.dataset.esc === 'local') return;   // an inline edit box: Esc cancels the edit
      stop();
      return;
    }
    // 1…7 switch pages (outside text boxes, no modifiers)
    if (/^Digit[1-9]$/.test(ev.code) && !typing(ev.target) && !ev.metaKey && !ev.ctrlKey && !ev.altKey && !ev.repeat) {
      const id = pageOrder[+ev.code.slice(5) - 1];
      if (id) { ev.preventDefault(); openPage(id); }
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
    document.getElementById('ticker').addEventListener('click', () => {
      showPanel('log');
    });
    let seen = null; try { seen = localStorage.getItem('bench.helpSeen'); } catch (e) { /* private mode */ }
    if (!seen && !/[?&]nohelp\b/.test(location.search)) showHelp();
    boot();
  });
  return {on, emit, api, log, register, openPage, openSub, showPanel, openTab: openPage, closeDrawer: () => openPage('drive'), showView, setBadgeDetail, mic,
    get info() { return info; }, get panels() { return panels; }, get page() { return curPage; }, PAGES};
})();

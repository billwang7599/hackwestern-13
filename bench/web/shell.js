// TARS bench shell: loads every panel in /panels.json, gives them a shared
// event stream and API helper. A panel file only calls Bench.register({...}).
//
//   Bench.register({
//     id: 'drive', title: 'Drive', order: 10,   // lower order = earlier
//     wide: false,                              // true = spans two columns
//     mount(el, Bench) { ... }                  // build the DOM inside el
//   });
//   Bench.on('status', data => ...)   // server-sent event topic
//   await Bench.api('drive/cmd', {line: 'S'})   // POST /api/drive/cmd (GET if no body)
//   Bench.log('drive', 'text')        // local line in the log strip
'use strict';
const Bench = (() => {
  const handlers = {};
  const panels = [];
  let es = null;
  let info = {panels: [], plugins: [], target: {}};

  function on(topic, fn) {
    (handlers[topic] = handlers[topic] || []).push(fn);
    if (es) es.addEventListener(topic, ev => fn(JSON.parse(ev.data)));
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

  function log(src, text) {
    const box = document.getElementById('log');
    if (!box) return;
    const line = document.createElement('div');
    const t = new Date().toLocaleTimeString();
    line.textContent = `${t} [${src}] ${text}`;
    box.prepend(line);
    while (box.childElementCount > 300) box.lastChild.remove();
  }

  function register(p) {
    panels.push(p);
    const grid = document.getElementById('panels');
    const sec = document.createElement('section');
    sec.className = 'panel' + (p.wide ? ' wide' : '');
    sec.id = 'panel-' + p.id;
    sec.style.order = p.order ?? 50;
    sec.innerHTML = `<h2>${p.title || p.id}</h2><div class="body"></div>`;
    grid.appendChild(sec);
    try { p.mount(sec.querySelector('.body'), Bench); }
    catch (e) { sec.querySelector('.body').textContent = 'panel failed: ' + e; console.error(e); }
  }

  function connect() {
    es = new EventSource('/events');
    for (const [topic, fns] of Object.entries(handlers))
      for (const fn of fns) es.addEventListener(topic, ev => fn(JSON.parse(ev.data)));
    es.addEventListener('log', ev => { const d = JSON.parse(ev.data); log(d.src, d.text); });
    es.onopen = () => document.body.classList.remove('offline');
    es.onerror = () => document.body.classList.add('offline');
  }

  async function boot() {
    info = await (await fetch('/panels.json')).json();
    const t = info.target || {};
    document.getElementById('target').textContent =
      `target: ${t.name}` + (t.name === 'pi' ? ` (${t.relay})` : ` (motor ${t.motor})`) +
      (info.hidden && info.hidden.length ? ` · sim-only panels hidden: ${info.hidden.join(', ')}` : '');
    // SIM vs REAL ROBOT: always visible, so nobody mistakes the sim for the car (or the reverse)
    const badge = document.getElementById('badge');
    if (badge) {
      const real = t.name === 'pi' || t.name === 'usb';
      badge.textContent = t.name === 'pi' ? 'REAL ROBOT (Pi)' : t.name === 'usb' ? 'REAL ROBOT (Uno on USB)' : 'SIM';
      badge.className = 'badge ' + (real ? 'real' : 'sim');
      document.body.classList.toggle('real-robot', real);
      document.title = (real ? 'REAL ROBOT · ' : 'SIM · ') + 'TARS bench';
    }
    connect();
    for (const f of info.panels) {
      await new Promise(res => {
        const s = document.createElement('script');
        s.src = '/panels/' + f; s.onload = res;
        s.onerror = () => { log('shell', 'panel failed to load: ' + f); res(); };
        document.body.appendChild(s);
      });
    }
  }

  // STOP lives in the shell, not a panel, so it works even if every panel failed.
  // The robot plugin owns /api/robot/stop (S to tars_motor, then CANCEL to goto).
  function stop() {
    api('robot/stop', {}).then(r => log('stop', JSON.stringify(r)))
      .catch(e => log('stop', 'STOP FAILED: ' + e.message));
  }
  window.addEventListener('keydown', ev => {
    if (ev.code !== 'Space' || ev.repeat) return;
    const tag = (ev.target.tagName || '').toLowerCase();
    if (tag === 'input' || tag === 'textarea' || tag === 'select') return;
    ev.preventDefault();
    stop();
  });
  window.addEventListener('DOMContentLoaded', () => {
    document.getElementById('stop').addEventListener('click', stop);
    boot();
  });
  return {on, api, log, register, get info() { return info; }, get panels() { return panels; }};
})();

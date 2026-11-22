// "Demo in 4 steps" (Drive page, top): pick a room → Car to start → "go to the trash can" → watch.
// Each step ticks itself off from live data (room loaded, car at its start, go-to started, arrived),
// so the order of the demo is obvious at a glance. Uses only existing routes:
// world/rooms, world/load, world/car_to_start, world/goto (sim) and voicelab/text (when tars_voice is up).
// On the real robot the two sim steps are left out.
'use strict';
(() => {
  const css = `
  .demo-steps { display:flex; flex-direction:column; gap:4px; }
  .demo-step { display:grid; grid-template-columns: 20px minmax(0, 1fr) auto; align-items:center; gap:8px; padding:4px 6px 4px 6px;
    border:1px solid var(--line2); border-radius:var(--r-md); background: var(--panel2); min-height:36px; }
  .demo-step .ds-n { width:20px; height:20px; border-radius:50%; display:inline-flex; align-items:center; justify-content:center;
    font: 700 11px var(--mono); background:#26323c; color:var(--text); }
  .demo-step .ds-h { font-size:13px; font-weight:600; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .demo-step.done { border-color:#24503b; } .demo-step.done .ds-n { background:var(--ok); color:#04121f; }
  .demo-step.done .ds-n::before { content:'✓'; } .demo-step.done .ds-n span { display:none; }
  .demo-step.now { border-color:var(--accent); box-shadow: inset 3px 0 0 var(--accent); } .demo-step.now .ds-n { background:var(--accent); color:#04121f; }
  .demo-step .ds-b { display:flex; gap:4px; align-items:center; justify-content:flex-end; min-width:0; }
  .demo-step .ds-b select { max-width:130px; }
  .demo-step .ds-b button { white-space:nowrap; padding:3px 9px; }
  .demo-step .ds-watch { grid-column: 2 / 4; font-size:12px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .demo-step .ds-watch.busy { color:#cfe6fb; }
  .demo-why { font-size:11px; color:var(--dim); height:15px; margin-top:3px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));

  Bench.register({
    id: 'demo', title: '▶ Demo in 4 steps', order: 0, area: 'drive',
    help: 'The whole demo, in order. Each step turns green by itself when it is done. Stop any time with Space, Esc or STOP.',
    mount(el) {
      const target = (Bench.info.target || {}).name, real = target === 'pi' || target === 'usb';
      const hasWorld = Bench.info.plugins.includes('world'), hasVoice = Bench.info.plugins.includes('voicelab');
      const sim = !real && hasWorld;
      const n = k => `<span class="ds-n"><span>${k}</span></span>`;
      el.innerHTML = `
        <div class="demo-steps">
          ${sim ? `<div class="demo-step" data-s="room">${n(1)}<span class="ds-h">Room</span>
            <span class="ds-b"><select class="ds-room" aria-label="room"><option>loading…</option></select>
            <button class="ds-load" title="Load this room: the car goes to its start and goto restarts">Load</button></span></div>
          <div class="demo-step" data-s="start">${n(2)}<span class="ds-h">Car at the start</span>
            <span class="ds-b"><button class="ds-start" title="Stop, then put only the car back at the room’s start (walls, goto’s map and memory stay)">🚗 Car to start</button></span></div>` : ''}
          <div class="demo-step" data-s="ask">${n(sim ? 3 : 1)}<span class="ds-h">Ask it</span>
            <span class="ds-b"><button class="ds-ask primary" title="Same as saying it. Or type / hold T in “Talk to QNXplorer” below.">🗣 go to the trash can</button></span></div>
          <div class="demo-step" data-s="watch">${n(sim ? 4 : 2)}<span class="ds-h">Watch the route on the map</span>
            <span></span><span class="ds-watch dim">the blue line is its plan</span></div>
        </div>
        <div class="demo-why"></div>`;
      const $ = s => el.querySelector(s);
      const step = s => el.querySelector(`.demo-step[data-s="${s}"]`);
      const why = $('.demo-why');
      const say = (t, cls = 'dim') => { why.textContent = t; why.className = 'demo-why ' + cls; };

      let W = null, G = null, links = {}, asked = false, rooms = [];
      Bench.on('world', w => { W = w; });
      Bench.on('goto', g => { G = (g && g.link && g.status) || null; });
      Bench.on('status', s => { links = (s && s.links) || {}; });

      if (sim) {
        const sel = $('.ds-room');
        (async () => {
          try { rooms = (await Bench.api('world/rooms')).rooms || []; } catch (e) { rooms = []; }
          sel.replaceChildren(...(rooms.length ? rooms.map(n => new Option(n, n)) : [new Option('no rooms (sim not connected)', '')]));
          if (W && W.room) sel.value = W.room.name;
        })();
        $('.ds-load').onclick = async () => {
          if (!sel.value) return;
          say(`loading ${sel.value}…`);
          try { const r = await Bench.api('world/load', {name: sel.value}); say(r.ok ? `loaded ${sel.value}` : 'load failed: ' + r.error, r.ok ? 'ok' : 'bad'); }
          catch (e) { say('load failed: ' + e.message, 'bad'); }
          asked = false; delete sel.dataset.touched;
        };
        $('.ds-start').onclick = async () => {
          try { const r = await Bench.api('world/car_to_start', {}); say(r.ok ? 'car is back at the start' : 'failed: ' + r.error, r.ok ? 'ok' : 'bad'); }
          catch (e) { say('car to start failed: ' + e.message, 'bad'); }
          asked = false;
        };
      }
      $('.ds-ask').onclick = async () => {
        const voiceUp = hasVoice && links.voice && links.voice.connected;
        asked = true;
        try {
          if (voiceUp) { await Bench.api('voicelab/text', {text: 'go to the trash can', mode: 'ptt'}); say('sent through voice (tars_voice heard it)'); }
          else if (hasWorld) { const r = await Bench.api('world/goto', {label: 'trash can'}); say(`voice is off, sent GOTO trash can directly: ${r.reply || r.error}`, r.ok ? 'dim' : 'bad'); }
          else { await Bench.api('mission/goto', {line: 'GOTO trash can'}); say('sent GOTO trash can'); }
        } catch (e) { say('couldn’t send: ' + e.message, 'bad'); asked = false; }
      };

      function paint() {
        const room = W && W.room, t = W && W.truth;
        const sel = $('.ds-room');
        if (sel && room && document.activeElement !== sel && [...sel.options].some(o => o.value === room.name) && !sel.dataset.touched) sel.value = room.name;
        const running = G && G.state && G.state !== 'idle';
        const res = String((G && G.result) || '');
        const done = {
          room: !!room,
          start: !!(room && t && room.start && Math.hypot(t.x - room.start.x, t.y - room.start.y) < 0.15) || (asked && (running || /arrived|failed/.test(res))),
          ask: !!(running || (asked && /arrived|failed/.test(res))),
          watch: /^arrived/.test(res) && asked,
        };
        let now = null;
        for (const s of ['room', 'start', 'ask', 'watch']) {
          const e = step(s); if (!e) continue;
          e.classList.toggle('done', !!done[s]);
          if (!done[s] && !now) now = s;
          e.classList.toggle('now', now === s);
        }
        // buttons that can't work say why instead of failing silently
        const b = $('.ds-start'), l = $('.ds-load'), a = $('.ds-ask');
        const wOk = !!(W && W.alive !== false);
        if (b) { b.disabled = !wOk; b.title = wOk ? 'Stop, then put only the car back at the room’s start (walls, goto’s map and memory stay)' : 'sim not connected'; }
        if (l) { l.disabled = !wOk; l.title = wOk ? 'Load this room: the car goes to its start and goto restarts' : 'sim not connected'; }
        const gOk = !!G;
        a.disabled = !gOk; a.title = gOk ? 'Same as saying it. Or type / hold T in “Talk to QNXplorer” below.' : 'go-to controller not connected';
        const wt = $('.ds-watch');
        let txt, cls = 'dim';
        if (!gOk) txt = 'go-to not connected yet';
        else if (running) { txt = `${G.state === 'follow_path' ? 'driving' : String(G.state).replace(/_/g, ' ')}${G.path && G.path !== '0/0' ? ' · leg ' + G.path : ''}${+G.replans ? ' · ' + G.replans + ' replans' : ''}`; cls = 'busy'; }
        else if (/^arrived/.test(res)) { txt = '✓ arrived'; cls = 'ok'; }
        else if (/^failed/.test(res)) { txt = '✗ ' + res.replace(/^failed:?/, 'failed ').replace(/_/g, ' '); cls = 'bad'; }
        else txt = 'the blue line is its plan';
        if (wt.textContent !== txt) wt.textContent = txt;
        wt.className = 'ds-watch ' + cls;
      }
      const sel = $('.ds-room');
      if (sel) sel.addEventListener('change', () => { sel.dataset.touched = '1'; });
      setInterval(paint, 300); paint();
    },
  });
})();

// Drive panel: hold W/A/S/D or arrows (server-side HoldDrive, D at 20 Hz, S on release;
// the server sends S if this tab stops refreshing held keys for 0.35 s), speed, MOVE/TURN,
// a live chart (commanded vs tars_motor output vs the virtual car's true wheel speeds), console.
'use strict';
(() => {
  const KEYMAP = {KeyW: 'w', KeyA: 'a', KeyS: 's', KeyD: 'd', ArrowUp: 'w', ArrowLeft: 'a', ArrowDown: 's', ArrowRight: 'd'};
  const css = `
  .drv-row { display:flex; gap:6px; align-items:center; flex-wrap:wrap; margin:4px 0; }
  .drv-keys { display:grid; grid-template-columns: repeat(3, 44px); gap:4px; touch-action:none; user-select:none; }
  .drv-keys span { border:1px solid var(--line); border-bottom-width:3px; border-radius:7px; text-align:center; padding:7px 0; font: 700 14px var(--mono); color: var(--text); background:#1d2a33; cursor:pointer; }
  .drv-keys span small { display:block; font: 10px system-ui, sans-serif; color: var(--dim); }
  .drv-keys span.on { background: var(--accent); color:#000; } .drv-keys span.on small { color:#04121f; }
  .drv-hint { font-size:12px; color: var(--dim); }
  .drv-console { height:130px; overflow:auto; font:12px/1.35 var(--mono); background:#0b1013; border-radius:6px; padding:4px 6px; }
  .drv-console .err { color: var(--bad); }
  .drv-legend span { margin-right:10px; font: 11px var(--mono); }
  .drv-disabled { opacity:.5; pointer-events:none; }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));

  // shared between the two panels below
  let addLine = () => {};
  const robotOk = () => Bench.info.plugins.includes('robot');

  // ---------------------------------------------------------------- Drive (left column)
  Bench.register({
    id: 'drive', title: '🕹 Drive by hand', order: 10, area: 'drive',
    help: 'Hold the keys (or press the pad). Releasing stops the wheels.',
    mount(el) {
      el.innerHTML = `
      <div class="drv-row" style="gap:12px;align-items:flex-start">
        <div class="drv-keys" title="Hold to drive; release to stop">
          <i></i><span data-k="w">W<small>fwd</small></span><i></i>
          <span data-k="a">A<small>left</small></span><span data-k="s">S<small>back</small></span><span data-k="d">D<small>right</small></span></div>
        <div style="flex:1;min-width:0">
          <div class="drv-hint msg">Hold <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> or arrows. Not while typing in a box.</div>
          <div class="drv-row">speed <input type="range" min="0.1" max="1" step="0.05" value="0.5" class="speed" style="flex:1;min-width:60px"> <b class="sp mono">0.50</b></div>
        </div>
      </div>
      <details style="margin-top:4px"><summary class="dim" style="cursor:pointer;font-size:12px">Exact moves (MOVE mm / TURN °)</summary>
      <div class="drv-row">
        <input class="mm" type="number" value="300" style="width:64px"> mm <button class="bmove" title="MOVE: drive this far forward (negative = back)">Move</button>
      </div>
      <div class="drv-row">
        <input class="deg" type="number" value="90" style="width:64px"> ° <button class="bturnl" title="TURN left">⟲ left</button><button class="bturn" title="TURN right">right ⟳</button>
        <label class="dim" title="slow turn at speed 0.40"><input type="checkbox" class="slow"> slow</label>
      </div></details>`;
      const $ = s => el.querySelector(s);
      const held = new Set();
      let speed = 0.5;
      if (!robotOk()) { el.classList.add('drv-disabled'); $('.msg').textContent = 'robot plugin not loaded: driving disabled'; }

      // ---- keys
      function paintKeys() { el.querySelectorAll('.drv-keys span').forEach(s => s.classList.toggle('on', held.has(s.dataset.k))); }
      async function push() {
        try { await Bench.api('robot/keys', {held: [...held], speed}); }
        catch (e) { $('.msg').textContent = 'keys: ' + e.message; }
      }
      const press = k => { if (!robotOk() || held.has(k)) return; held.add(k); paintKeys(); push(); };
      const lift = k => { if (!held.has(k)) return; held.delete(k); paintKeys(); push(); };
      const typing = t => ['input', 'textarea', 'select'].includes((t.tagName || '').toLowerCase()) && t.type !== 'range' && t.type !== 'checkbox';
      window.addEventListener('keydown', ev => {
        const k = KEYMAP[ev.code];
        if (!k || typing(ev.target) || !robotOk()) return;
        ev.preventDefault();
        press(k);
      });
      window.addEventListener('keyup', ev => { const k = KEYMAP[ev.code]; if (k) lift(k); });
      // on-screen pad: press and hold with the mouse / trackpad / touch
      el.querySelectorAll('.drv-keys span').forEach(s => {
        s.addEventListener('pointerdown', e => { e.preventDefault(); s.setPointerCapture(e.pointerId); press(s.dataset.k); });
        const up = () => lift(s.dataset.k);
        s.addEventListener('pointerup', up); s.addEventListener('pointercancel', up); s.addEventListener('lostpointercapture', up);
      });
      const release = () => { if (held.size) { held.clear(); paintKeys(); push(); } };
      window.addEventListener('blur', release);
      document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
      // heartbeat: the server stops the drive if this goes quiet for 0.35 s
      setInterval(() => { if (held.size) push(); }, 100);
      $('.speed').addEventListener('input', ev => {
        speed = +ev.target.value; $('.sp').textContent = speed.toFixed(2);
        if (held.size) push();
      });

      // say why the wheels don't turn when tars_motor refuses a held key (guard blocked, tipped…)
      const hint = $('.msg').innerHTML;
      Bench.on('status', st => {
        const f = (st && st.ok && st.fields) || {};
        const why = String(f.why || '');
        const refused = held.size && /guard|tip|bump/.test(why);
        $('.msg').classList.toggle('bad', !!refused);
        if (refused) $('.msg').textContent = `⚠ tars_motor refuses to drive: ${why.replace(/_/g, ' ')}` +
          (/guard/.test(why) ? ' (something is in front; the Path light is red). Move the car or the obstacle in the sim.' : '');
        else if (robotOk() && $('.msg').innerHTML !== hint && !/^keys:/.test($('.msg').textContent)) $('.msg').innerHTML = hint;
      });

      // ---- MOVE / TURN
      const slow = () => $('.slow').checked ? 0.40 : undefined;
      const mv = (kind, amount) => Bench.api('robot/move', {kind, amount, speed: slow()})
        .then(r => { addLine(`motor> ${kind} ${amount} -> ${r.reply}`, r.reply.startsWith('ERR')); if (r.reply.startsWith('ERR')) Bench.log('drive', `${kind} ${amount}: ${r.reply}`); })
        .catch(e => { addLine(kind + ' failed: ' + e.message, true); Bench.log('drive', kind + ' failed: ' + e.message); });
      $('.bmove').onclick = () => mv('MOVE', +$('.mm').value);
      $('.bturn').onclick = () => mv('TURN', Math.abs(+$('.deg').value));
      $('.bturnl').onclick = () => mv('TURN', -Math.abs(+$('.deg').value));
    }
  });

  // ---------------------------------------------------------------- wheel chart + console (Debug tab)
  Bench.register({
    id: 'drive-debug', title: 'Wheels chart + command console', order: 30, area: 'tests',
    help: 'Commanded wheel power vs what tars_motor outputs vs the virtual car’s real wheel speed; and send any raw line.',
    mount(el) {
      el.innerHTML = `
      <canvas class="chart" height="150"></canvas>
      <div class="drv-legend"><span style="color:#5aa9e6">— cmd L</span><span style="color:#9fd0f5">-- cmd R</span><span style="color:#4cc38a">— out L</span><span style="color:#a5e3c5">-- out R</span><span style="color:#e6b450">— car wl</span><span style="color:#f2d491">-- car wr</span><span class="dim carnote"></span></div>
      <div class="drv-row">
        <select class="dest" title="motor = tars_motor (5555), goto = go-to controller, voice = tars_voice"><option>motor</option><option>goto</option><option>voice</option></select>
        <input class="line mono" placeholder="STATUS / PING / LED ok / BEEP 2 / GOTO trash can ..." style="flex:1">
        <button class="bsend">send</button>
        <label class="dim"><input type="checkbox" class="showd"> show D</label>
      </div>
      <div class="drv-console"></div>`;
      const $ = s => el.querySelector(s);
      // ---- console
      const box = $('.drv-console');
      let dOk = 0;
      addLine = function (text, bad) {
        const d = document.createElement('div');
        d.textContent = new Date().toLocaleTimeString() + ' ' + text;
        if (bad) d.className = 'err';
        box.prepend(d);
        while (box.childElementCount > 200) box.lastChild.remove();
      };
      async function sendLine() {
        const line = $('.line').value.trim();
        if (!line) return;
        try { await Bench.api('robot/send', {dest: $('.dest').value, line}); }
        catch (e) { addLine('send failed: ' + e.message, true); }
        $('.line').select();
      }
      $('.bsend').onclick = sendLine;
      $('.line').addEventListener('keydown', ev => { if (ev.key === 'Enter') sendLine(); });
      Bench.on('cmd', c => {
        const isD = /^(D|DL) /.test(c.line) && c.src === 'drive';
        const bad = (c.reply || '').startsWith('ERR');
        if (isD && !bad && !$('.showd').checked) { dOk++; return; }
        addLine(`${c._replay ? '[replay] ' : ''}${c.dest}> ${c.line} -> ${c.reply}  (${c.src}, ${c.ms} ms)`, bad);
      });

      // ---- chart
      const N = 200; // ~20 s at 10 Hz
      const series = {cl: [], cr: [], ol: [], or: [], wl: [], wr: []};
      let car = null, carT = 0;
      Bench.on('car', d => {
        const t = d && (d.truth || d);
        if (t && 'wl' in t) { car = t; carT = performance.now(); }
      });
      Bench.on('status', s => {
        const f = s.fields || {}, dr = s.drive || {};
        const fresh = car && performance.now() - carT < 500;
        const scale = f.dc_mm_s > 0 ? 1000 / f.dc_mm_s : 1 / 0.15; // m/s -> fraction of D 1
        const push1 = (k, v) => { series[k].push(v); if (series[k].length > N) series[k].shift(); };
        push1('cl', dr.cmd_l ?? null); push1('cr', dr.cmd_r ?? null);
        push1('ol', s.ok ? f.out_l : null); push1('or', s.ok ? f.out_r : null);
        push1('wl', fresh ? car.wl * scale : null); push1('wr', fresh ? car.wr * scale : null);
        $('.carnote').textContent = fresh ? `car wheel m/s ÷ ${f.dc_mm_s > 0 ? 'dc_mm_s' : '0.15 (no dc_mm_s)'}` : 'no virtual car data';
        draw();
      });
      const cv = $('.chart');
      function draw() {
        const w = cv.width = cv.clientWidth * devicePixelRatio, h = cv.height = 150 * devicePixelRatio;
        const g = cv.getContext('2d');
        g.clearRect(0, 0, w, h);
        const y = v => h / 2 - v * (h / 2 - 6 * devicePixelRatio);
        g.strokeStyle = '#2a363f'; g.lineWidth = 1;
        for (const v of [-1, -0.5, 0, 0.5, 1]) { g.beginPath(); g.moveTo(0, y(v)); g.lineTo(w, y(v)); g.stroke(); }
        const lines = [['cl', '#5aa9e6', []], ['cr', '#9fd0f5', [5, 4]], ['ol', '#4cc38a', []], ['or', '#a5e3c5', [5, 4]],
                       ['wl', '#e6b450', []], ['wr', '#f2d491', [5, 4]]];
        for (const [k, col, dash] of lines) {
          const a = series[k];
          g.strokeStyle = col; g.lineWidth = 1.5 * devicePixelRatio; g.setLineDash(dash.map(x => x * devicePixelRatio));
          g.beginPath(); let pen = false;
          a.forEach((v, i) => {
            const X = w - (a.length - 1 - i) * (w / (N - 1));
            if (v === null || v === undefined) { pen = false; return; }
            const Y = y(Math.max(-1.1, Math.min(1.1, v)));
            pen ? g.lineTo(X, Y) : g.moveTo(X, Y); pen = true;
          });
          g.stroke();
        }
        g.setLineDash([]);
        g.fillStyle = '#8a9aa6'; g.font = `${10 * devicePixelRatio}px monospace`;
        g.fillText('+1', 2, y(1) + 10 * devicePixelRatio); g.fillText('-1', 2, y(-1) - 2);
      }
    }
  });
})();

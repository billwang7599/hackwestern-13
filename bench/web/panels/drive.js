// Drive panel: hold W/A/S/D or arrows (server-side HoldDrive, D at 20 Hz, S on release;
// the server sends S if this tab stops refreshing held keys for 0.35 s), speed, MOVE/TURN,
// a live chart (commanded vs tars_motor output vs the virtual car's true wheel speeds), console.
'use strict';
(() => {
  const KEYMAP = {KeyW: 'w', KeyA: 'a', KeyS: 's', KeyD: 'd', ArrowUp: 'w', ArrowLeft: 'a', ArrowDown: 's', ArrowRight: 'd'};
  const css = `
  .drv-row { display:flex; gap:6px; align-items:center; flex-wrap:wrap; margin:4px 0; }
  .drv-keys { display:grid; grid-template-columns: repeat(3, 34px); gap:3px; }
  .drv-keys span { border:1px solid var(--line); border-radius:5px; text-align:center; padding:4px 0; font: 600 12px var(--mono); color: var(--dim); }
  .drv-keys span.on { background: var(--accent); color:#000; }
  .drv-console { height:130px; overflow:auto; font:12px/1.35 var(--mono); background:#0b1013; border-radius:6px; padding:4px 6px; }
  .drv-console .err { color: var(--bad); }
  .drv-legend span { margin-right:10px; font: 11px var(--mono); }
  .drv-disabled { opacity:.5; pointer-events:none; }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));

  Bench.register({
    id: 'drive', title: 'Drive', order: 10, wide: true,
    mount(el) {
      el.innerHTML = `
      <div class="drv-row">
        <div class="drv-keys"><i></i><span data-k="w">W</span><i></i><span data-k="a">A</span><span data-k="s">S</span><span data-k="d">D</span></div>
        <div style="flex:1;min-width:200px">
          <div class="drv-row">speed <input type="range" min="0.1" max="1" step="0.05" value="0.5" class="speed" style="flex:1"> <b class="sp mono">0.50</b></div>
          <div class="drv-row mono dim msg">click the page, then hold W A S D / arrows. Space = STOP.</div>
        </div>
      </div>
      <div class="drv-row">
        MOVE <input class="mm" type="number" value="300" style="width:70px"> mm <button class="bmove">MOVE</button>
        &nbsp; TURN <input class="deg" type="number" value="90" style="width:60px"> ° <button class="bturn">right</button><button class="bturnl">left</button>
        <label class="dim"><input type="checkbox" class="slow"> slow (0.40)</label>
      </div>
      <canvas class="chart" height="150"></canvas>
      <div class="drv-legend"><span style="color:#5aa9e6">— cmd L</span><span style="color:#9fd0f5">-- cmd R</span><span style="color:#4cc38a">— out L</span><span style="color:#a5e3c5">-- out R</span><span style="color:#e6b450">— car wl</span><span style="color:#f2d491">-- car wr</span><span class="dim carnote"></span></div>
      <div class="drv-row">
        <select class="dest"><option>motor</option><option>goto</option><option>voice</option></select>
        <input class="line mono" placeholder="STATUS / PING / LED ok / BEEP 2 / GOTO trash can ..." style="flex:1">
        <button class="bsend">send</button>
        <label class="dim"><input type="checkbox" class="showd"> show D</label>
      </div>
      <div class="drv-console"></div>`;
      const $ = s => el.querySelector(s);
      const held = new Set();
      let shift = false;   // Shift+W = keyboard override towards an obstacle (robot plugin docstring)
      let speed = 0.5, robotOk = Bench.info.plugins.includes('robot');
      if (!robotOk) { el.classList.add('drv-disabled'); $('.msg').textContent = 'robot plugin not loaded: driving disabled'; }

      // ---- keys
      function paintKeys() { el.querySelectorAll('.drv-keys span').forEach(s => s.classList.toggle('on', held.has(s.dataset.k))); }
      async function push() {
        try { await Bench.api('robot/keys', {held: [...held], speed, shift}); }
        catch (e) { $('.msg').textContent = 'keys: ' + e.message; }
      }
      const typing = t => ['input', 'textarea', 'select'].includes((t.tagName || '').toLowerCase()) && t.type !== 'range' && t.type !== 'checkbox';
      window.addEventListener('keydown', ev => {
        if (ev.shiftKey !== shift) { shift = ev.shiftKey; if (held.size) push(); }
        const k = KEYMAP[ev.code];
        if (!k || typing(ev.target) || !robotOk) return;
        ev.preventDefault();
        if (held.has(k)) return;
        held.add(k); paintKeys(); push();
      });
      window.addEventListener('keyup', ev => {
        if (ev.shiftKey !== shift) { shift = ev.shiftKey; if (held.size) push(); }
        const k = KEYMAP[ev.code];
        if (!k || !held.has(k)) return;
        held.delete(k); paintKeys(); push();
      });
      // guard in plain words (status.guard.help from the robot plugin); '' = nothing to explain
      const hint = $('.msg').textContent;
      Bench.on('status', st => {
        const g = st && st.guard, m = $('.msg');
        if (!robotOk || !g) return;
        m.textContent = g.help || hint;
        m.style.color = g.help ? 'var(--warn, #e6b450)' : '';
      });
      const release = () => { shift = false; if (held.size) { held.clear(); paintKeys(); push(); } };
      window.addEventListener('blur', release);
      document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
      // heartbeat: the server stops the drive if this goes quiet for 0.35 s
      setInterval(() => { if (held.size) push(); }, 100);
      $('.speed').addEventListener('input', ev => {
        speed = +ev.target.value; $('.sp').textContent = speed.toFixed(2);
        if (held.size) push();
      });

      // ---- MOVE / TURN
      const slow = () => $('.slow').checked ? 0.40 : undefined;
      const mv = (kind, amount) => Bench.api('robot/move', {kind, amount, speed: slow()})
        .then(r => addLine(`motor> ${kind} ${amount} -> ${r.reply}`, r.reply.startsWith('ERR')))
        .catch(e => addLine(kind + ' failed: ' + e.message, true));
      $('.bmove').onclick = () => mv('MOVE', +$('.mm').value);
      $('.bturn').onclick = () => mv('TURN', Math.abs(+$('.deg').value));
      $('.bturnl').onclick = () => mv('TURN', -Math.abs(+$('.deg').value));

      // ---- console
      const box = $('.drv-console');
      let dOk = 0;
      function addLine(text, bad) {
        const d = document.createElement('div');
        d.textContent = new Date().toLocaleTimeString() + ' ' + text;
        if (bad) d.className = 'err';
        box.prepend(d);
        while (box.childElementCount > 200) box.lastChild.remove();
      }
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

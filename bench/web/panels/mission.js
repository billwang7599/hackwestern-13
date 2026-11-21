// Mission control panels (plugin `mission`): camera, safety, goto, object memory + phone, map,
// robot voice, Pi services + action journals + Andrew's room view. They work on every target:
// pi = what mission_relay streams (as iter2-autonomy/mission/laptop/mission.py shows it),
// sim/usb = the same goto/tars_voice data polled from the laptop (no camera).
// Rendering follows Andrew's iter2-autonomy/mission/laptop/index.html (inline JS there, so it
// can't be imported); data parsing is his mission.State, imported server side.
'use strict';
(() => {
  const css = `
  .mc-row { display:flex; gap:6px; align-items:center; flex-wrap:wrap; margin:4px 0; }
  .mc-t { width:100%; border-collapse:collapse; font: 12px/1.35 var(--mono); }
  .mc-t td { padding:1px 4px; vertical-align:top; } .mc-t td:first-child { color: var(--dim); white-space:nowrap; width:1%; }
  .mc-stale { opacity:.4; filter: grayscale(1); }
  .mc-log { height:170px; overflow:auto; font: 12px var(--mono); background:#0b1013; border-radius:6px; padding:4px 6px; }
  .mc-log div { white-space:pre-wrap; border-bottom:1px solid #141b20; }
  .mc-k-stop { color: var(--bad); font-weight:700; } .mc-k-goto { color: var(--accent); } .mc-k-say { color:#d2a8ff; } .mc-k-motor { color: var(--warn); }
  .mc-cam { position:relative; background:#000; border-radius:6px; overflow:hidden; aspect-ratio:4/3; }
  .mc-cam img, .mc-cam canvas { position:absolute; inset:0; width:100%; height:100%; object-fit:contain; }
  .mc-cam .none { position:absolute; inset:0; display:flex; align-items:center; justify-content:center; color: var(--dim); font-size:12px; text-align:center; padding:8px; }
  .mc-map { width:100%; aspect-ratio:1; background:#0b1013; border-radius:6px; display:block; }
  .mc-verdict { display:inline-block; padding:2px 10px; border-radius:6px; font-weight:800; font-family: var(--mono); }
  .mc-CLEAR { background:#12361e; color: var(--ok); } .mc-CAUTION { background:#3b2e0a; color: var(--warn); }
  .mc-BLOCKED { background:#3a1414; color: var(--bad); } .mc-UNKNOWN, .mc-STALE, .mc-OFF { background:#30363d; color: var(--dim); }
  .mc-pill { font: 600 11px var(--mono); padding:1px 6px; border-radius:4px; border:1px solid var(--line); margin-right:3px; }
  .mc-pill.up { color: var(--ok); border-color: var(--ok); } .mc-pill.down { color: var(--bad); border-color: var(--bad); }
  .mc-note { color: var(--dim); font-size:11px; }
  .mc-ptt.rec { background: var(--bad); color:#fff; }
  .mc-frame { width:100%; height:480px; border:0; border-radius:6px; background:#0b1013; }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));

  const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'})[c]);
  const fmtT = t => new Date(t * 1000).toLocaleTimeString();
  const ageTxt = a => a == null ? 'no data' : a < 1 ? '' : `${a.toFixed(1)} s old`;
  const rows = (el, pairs) => { el.innerHTML = pairs.map(([k, v]) => `<tr><td>${esc(k)}</td><td>${esc(v)}</td></tr>`).join(''); };
  const stale = (el, age, lim) => el.classList.toggle('mc-stale', age == null || age > lim);
  const api = (p, b) => Bench.api('mission/' + p, b).catch(e => { Bench.log('mission', `${p}: ${e.message}`); return {}; });

  // shared state, fed by the plugin's topics
  let S = {}, grid = null, phone = null, motor = null, gotoTopic = null;
  const renders = [];
  const render = () => renders.forEach(fn => { try { fn(); } catch (e) { console.error(e); } });
  Bench.on('mission', d => { S = d; render(); });
  Bench.on('mission_grid', g => { grid = g; });
  Bench.on('mission_phone', p => { phone = p.status; });
  Bench.on('status', s => { motor = s.ok ? s.fields : null; });
  Bench.on('goto', g => { gotoTopic = g; });

  // ------------------------------------------------------------------ camera
  Bench.register({
    id: 'mc-camera', title: 'Camera (phone, via the Pi)', order: 12,
    mount(el) {
      el.innerHTML = `<div class="mc-cam"><img alt=""><canvas></canvas><div class="none">no frames</div></div>
        <div class="mc-note"><span class="age"></span> · green = YOLOX on this frame (phone tee) · orange = goto's YOLO-World detections (<code>detect</code>'s own camera: lines up only if it's the same camera)</div>`;
      const img = el.querySelector('img'), cv = el.querySelector('canvas'), none = el.querySelector('.none');
      let shown = -1, loading = false;
      img.onload = img.onerror = () => { loading = false; };
      renders.push(() => {
        const c = S.cam || {};
        el.querySelector('.age').textContent = c.age != null ? `frame ${c.id} · ${c.age.toFixed(1)} s` : 'no frames';
        none.style.display = c.id ? 'none' : '';
        if (!c.id) none.textContent = S.target === 'pi' ? 'no frames from the relay CAM channel yet' : (S.camera || 'no camera');
        stale(el.querySelector('.mc-cam'), c.age, 1.5);
        if (c.id && c.id !== shown && !loading) { shown = c.id; loading = true; img.src = '/api/mission/cam.jpg?id=' + c.id; }
        drawBoxes();
      });
      function drawBoxes() {
        const x = cv.getContext('2d');
        cv.width = cv.clientWidth; cv.height = cv.clientHeight;
        x.clearRect(0, 0, cv.width, cv.height);
        const iw = img.naturalWidth || 4, ih = img.naturalHeight || 3;
        const s = Math.min(cv.width / iw, cv.height / ih), ox = (cv.width - iw * s) / 2, oy = (cv.height - ih * s) / 2;
        const box = (l, t, w, h, col, txt) => { x.strokeStyle = col; x.lineWidth = 2; x.strokeRect(ox + l * s, oy + t * s, w * s, h * s);
          x.fillStyle = col; x.font = '12px sans-serif'; x.fillText(txt, ox + l * s + 2, oy + t * s - 3); };
        const ai = (phone || {}).ai || {};
        if (ai.live && (S.phone_age ?? 99) < 2.5) for (const o of ai.objects || []) { const b = o.bbox || [];
          box(b[0] * iw, b[1] * ih, b[2] * iw, b[3] * ih, '#3fb950', `${o.label} ${(+o.confidence).toFixed(2)}`); }
        const D = S.dets, f = D && D.frame;
        if (f && f.w && D.age < 1.5) for (const d of f.dets || []) { const b = d.box, kx = iw / f.w, ky = ih / f.h;
          box(b[0] * kx, b[1] * ky, (b[2] - b[0]) * kx, (b[3] - b[1]) * ky, '#d29922', `${d.label} ${(+d.conf).toFixed(2)} (detect)`); }
      }
    }
  });

  // ------------------------------------------------------------------ safety
  Bench.register({
    id: 'mc-safety', title: 'Safety (guard)', order: 13,
    mount(el) {
      el.innerHTML = `<div class="mc-row"><span class="mc-verdict">?</span> <span class="why mc-note"></span></div>
        <table class="mc-t"></table>
        <div class="mc-row"><button class="reset" title="RESET to tars_motor: clears the tip latch (teleop's R key)">RESET tip</button><span class="rr mc-note"></span></div>`;
      const V = el.querySelector('.mc-verdict');
      el.querySelector('.reset').onclick = async () => { el.querySelector('.rr').textContent = (await api('reset', {})).reply || ''; };
      renders.push(() => {
        const M = motor || {}, saf = (phone || {}).safety || {};
        let v = 'UNKNOWN', why = '';
        if ((S.phone_age ?? 99) < 2.5 && saf.verdict) { v = saf.verdict; why = `phone/guard: ${saf.stop_source || ''} nearest ${saf.nearest_obstacle_m ?? '-'} m`; }
        else if (motor) {
          const reason = String(M.guard_reason ?? '');
          if (String(M.guard_enabled) === '0') { v = 'OFF'; why = 'tars_motor started without -g'; }
          else if (String(M.guard_permit) === '1') v = 'CLEAR';
          else if (reason.includes('blocked')) v = 'BLOCKED';
          else if (/stale|missing|invalid/.test(reason)) v = 'STALE';
          why = why || `tars_motor gate: ${reason || '?'} (${M.guard_age_ms ?? '?'} ms)`;
        } else v = 'STALE';
        V.textContent = v; V.className = 'mc-verdict mc-' + v; el.querySelector('.why').textContent = why;
        rows(el.querySelector('table'), [['gate permit', M.guard_permit], ['gate reason', M.guard_reason], ['gate age', (M.guard_age_ms ?? '?') + ' ms'],
          ['guard epoch', M.guard_epoch], ['tipped / tilt', `${M.tipped ?? '?'} / ${M.tilt ?? '?'}°`], ['stop reason', M.why],
          ['phone verdict', saf.verdict ?? '(no phone tee)'], ['inferred stop', saf.inferred_live ? `${saf.inferred_nearest_m} m` : '-']]);
      });
    }
  });

  // ------------------------------------------------------------------ goto
  Bench.register({
    id: 'mc-goto', title: 'Go-to', order: 14,
    mount(el) {
      el.innerHTML = `<table class="mc-t"></table>
        <div class="mc-row"><select class="obj"></select>
          <button data-g="GOTO">GOTO</button><button data-g="RECALL">RECALL</button><button data-g="FORGET">FORGET</button>
          <button class="fa">FORGET all</button></div>
        <div class="mc-row"><button class="scan">SCAN (look around)</button><button class="ms">MEMORY SAY</button><button class="cx">CANCEL</button>
          <span class="rep mc-note"></span></div>
        <div class="mc-log ev"></div>`;
      const $ = s => el.querySelector(s), sel = $('.obj');
      const g = async line => { const r = await api('goto', {line}); $('.rep').textContent = `${line} → ${r.reply ?? ''}`; };
      el.querySelectorAll('[data-g]').forEach(b => b.onclick = () => g(`${b.dataset.g} ${sel.value}`));
      $('.fa').onclick = () => g('FORGET all'); $('.scan').onclick = () => g('SCAN'); $('.cx').onclick = () => g('CANCEL');
      $('.ms').onclick = async () => { $('.rep').textContent = 'MEMORY SAY → ' + ((await api('memory_say', {})).reply ?? ''); };
      renders.push(() => {
        const G = (gotoTopic && gotoTopic.status) || {};
        stale($('table'), gotoTopic && gotoTopic.ok ? 0 : null, 1);
        rows($('table'), [['state', G.state], ['target', String(G.target ?? '').replace(/_/g, ' ')], ['result', G.result], ['mode / mem', `${G.mode ?? ''} / ${G.mem ?? ''}`],
          ['map / route', `${G.map ?? ''} ${G.path ?? ''} replans ${G.replans ?? 0}`], ['seen / bearing / h', `${G.seen ?? ''} / ${G.bearing ?? ''}° / ${G.h ?? ''}`],
          ['guard (goto)', G.guard], ['elapsed', (G.elapsed ?? '0') + ' s'], ['detector fps', G.fps]]);
        const list = S.objects_list || [];
        if (sel.options.length !== list.length) sel.innerHTML = list.map(o => `<option>${esc(o)}</option>`).join('');
        let evs = (S.events || []).map(e => ({...e}));
        if (S.target !== 'pi')   // pi: goto EVENTs arrive on WATCH (already in S.events); sim/usb: the robot plugin's WATCH
          evs = evs.concat(((gotoTopic || {}).events || []).map(e => ({t: e.t, kind: 'goto', text: e.text})));
        evs.sort((a, b) => b.t - a.t);
        $('.ev').innerHTML = evs.slice(0, 80).map(e => `<div class="mc-k-${esc(e.kind)}">${fmtT(e.t)} ${esc(e.text)}</div>`).join('');
      });
    }
  });

  // ------------------------------------------------------------------ memory + phone
  Bench.register({
    id: 'mc-memory', title: 'Object memory + phone', order: 15,
    mount(el) {
      el.innerHTML = `<div class="mc-note mage"></div><table class="mc-t mem"></table>
        <h2 style="margin-top:8px">phone <span class="page mc-note"></span></h2><table class="mc-t ph"></table>`;
      const $ = s => el.querySelector(s);
      renders.push(() => {
        const Mem = S.memory || {};
        stale($('.mem'), S.memory_age, 2.5); $('.mage').textContent = ageTxt(S.memory_age > 2.5 ? S.memory_age : 0);
        $('.mem').innerHTML = (Mem.objects || []).map(o => `<tr><td>${esc(o.name)}</td><td>${esc(o.status)} · ${esc(o.rel_dist)} mm @ ${esc(o.rel_bearing)}° · seen ${esc(o.seen)} · ${esc(o.age)} s ago${o.map ? ' · map ' + esc(o.map) : ''}</td></tr>`).join('')
          || '<tr><td>nothing remembered</td></tr>';
        const P = phone || {}, sur = P.surroundings || {}, ai = P.ai || {}, dep = P.depth || {};
        stale($('.ph'), S.phone_age, 2.5);
        $('.page').textContent = phone ? ageTxt(S.phone_age > 2.5 ? S.phone_age : 0) : (S.target === 'pi' ? '(phone tee off)' : '(no phone tee in sim/usb: see the iPhone 3D map panel)');
        rows($('.ph'), [['streaming', P.streaming], ['motion / camera', `${P.motion_hz ?? '-'} Hz / ${P.camera_fps ?? '-'} fps`], ['tracking', `${sur.tracking ?? '-'} floor ${sur.floor_ready ?? '-'}`],
          ['map', `${sur.voxels ?? '-'} voxels, ${sur.frames ?? '-'} frames, scanned ${sur.scan_degrees ?? '-'}°`], ['YOLOX', ai.live ? `${ai.inference_ms} ms, ${(ai.objects || []).length} objects` : (ai.error || 'off')],
          ['MiDaS', dep.live ? `${dep.inference_ms} ms` : (dep.error || 'off')], ['message', sur.message ?? '']]);
      });
    }
  });

  // ------------------------------------------------------------------ map (goto NAV GRID, else the phone's grid)
  Bench.register({
    id: 'mc-map', title: 'Map (goto)', order: 16,
    mount(el) {
      el.innerHTML = `<div class="mc-row"><button class="zi">+</button><button class="zo">−</button><span class="src mc-note"></span></div>
        <canvas class="mc-map" width="560" height="560"></canvas>
        <div class="mc-note">top-down, ARKit frame · grey free · orange occupied · purple inferred · blue = goto's route · ▲ robot · ● goto memory (filled), phone objects (ring)</div>`;
      const c = el.querySelector('canvas');
      let zoom = 60, cache = {key: null, g: null};
      el.querySelector('.zi').onclick = () => { zoom = Math.max(20, zoom / 1.5); draw(); };
      el.querySelector('.zo').onclick = () => { zoom = Math.min(400, zoom * 1.5); draw(); };
      function decode() {
        if (grid && grid.box && grid.box !== '-' && grid.rle) {
          const [x0, z0, w, h] = grid.box.split(',').map(Number), cells = new Uint8Array(w * h);
          let i = 0; for (const run of grid.rle.split(',')) { const [v, n] = run.split('x').map(Number); cells.fill(v, i, i + n); i += n; }
          return {x0, z0, w, h, cells, res: +grid.res || 0.1, ox: -10, oz: -10, src: 'goto'};
        }
        const sur = (phone || {}).surroundings;
        if (sur && sur.grid) { const n = sur.size || 200; return {x0: 0, z0: 0, w: n, h: n, cells: Uint8Array.from(sur.grid, ch => +ch),
          res: sur.resolution || 0.1, ox: (sur.origin || [-10, -10])[0], oz: (sur.origin || [-10, -10])[1], src: 'phone'}; }
        return null;
      }
      function draw() {
        const x = c.getContext('2d'), W = c.width, H = c.height;
        x.fillStyle = '#0b1013'; x.fillRect(0, 0, W, H);
        const key = (grid && grid.seq) + ':' + (phone ? (phone.surroundings || {}).frames : '');
        if (cache.key !== key) cache = {key, g: decode()};
        const g = cache.g;
        let px = 0, pz = 0, yaw = 0, psrc = 'none';
        const nav = S.nav || {};
        if (nav.pose && !String(nav.pose).startsWith('none')) { const p = String(nav.pose).split(','); psrc = p[0]; px = +p[1]; pz = +p[2]; yaw = +p[3] * Math.PI / 180; }
        else if (phone && phone.surroundings && phone.surroundings.pose) { const p = phone.surroundings.pose; psrc = 'phone'; px = p[0]; pz = p[2]; yaw = p[3]; }
        const sc = W / (zoom / 10), toX = wx => W / 2 + (wx - px) * sc, toY = wz => H / 2 + (wz - pz) * sc;
        el.querySelector('.src').textContent = `grid: ${g ? g.src : 'none'} · pose: ${psrc} · ${(W / sc).toFixed(1)} m wide`;
        stale(c, Math.min(S.grid_age ?? 99, S.phone_age ?? 99, S.nav_age ?? 99), 2.5);
        if (g) {
          const colors = [null, '#3a4048', '#d2762a', '#8957e5'];
          for (let r = 0; r < g.h; r++) for (let q = 0; q < g.w; q++) {
            const v = g.cells[r * g.w + q]; if (!v || !colors[v]) continue;
            x.fillStyle = colors[v]; x.fillRect(toX(g.ox + (g.x0 + q) * g.res), toY(g.oz + (g.z0 + r) * g.res), g.res * sc + 0.5, g.res * sc + 0.5);
          }
        }
        const route = (S.route || {}).points || [];
        const path = route.length ? route : (((phone || {}).surroundings || {}).path || []).map(p => [p[0], p[p.length - 1]]);
        if (path.length) { x.strokeStyle = '#58a6ff'; x.lineWidth = 3; x.beginPath(); x.moveTo(toX(px), toY(pz));
          for (const [wx, wz] of path) x.lineTo(toX(wx), toY(wz)); x.stroke(); }
        const dot = (wx, wz, col, txt, ring) => { x.beginPath(); x.arc(toX(wx), toY(wz), 6, 0, 7); x.fillStyle = col; x.strokeStyle = col; x.lineWidth = 2;
          ring ? x.stroke() : x.fill(); x.fillStyle = '#e6e9ee'; x.font = '12px sans-serif'; x.fillText(txt, toX(wx) + 8, toY(wz) + 4); };
        for (const o of (S.memory || {}).objects || []) if (o.map) { const [wx, wz] = String(o.map).split(',').map(Number);
          dot(wx, wz, o.status === 'fresh' ? '#3fb950' : o.status === 'stale' ? '#d29922' : '#6e7681', `${o.name} (${o.status})`); }
        for (const o of ((phone || {}).surroundings || {}).objects || []) dot(o.position[0], o.position[2], '#79c0ff', o.label, true);
        x.save(); x.translate(toX(px), toY(pz)); x.rotate(Math.PI - yaw); x.fillStyle = '#fff';
        x.beginPath(); x.moveTo(0, -12); x.lineTo(7, 8); x.lineTo(-7, 8); x.closePath(); x.fill(); x.restore();
      }
      renders.push(draw);
    }
  });

  // ------------------------------------------------------------------ robot voice (the robot's own classifier)
  Bench.register({
    id: 'mc-voice', title: 'Robot voice', order: 17,
    mount(el) {
      el.innerHTML = `<div class="mc-note mode"></div>
        <div class="mc-row"><button class="mc-ptt" title="hold (or hold V)">🎤 hold to talk</button>
          <input class="say" placeholder="type a command, e.g. tars go to the trash can" style="flex:1;min-width:160px"><button class="send">send</button></div>
        <div class="mc-log vl" style="height:240px"></div>
        <div class="mc-note">Goes to the robot's tars_voice (it classifies and acts). The Voice lab panel is the dry-run/batch tool; its mic key is T, this one's is V.</div>`;
      const $ = s => el.querySelector(s), ptt = $('.mc-ptt');
      const sendText = () => { const t = $('.say').value.trim(); if (t) api('text', {text: t}); $('.say').value = ''; };
      $('.send').onclick = sendText;
      $('.say').addEventListener('keydown', e => { if (e.key === 'Enter') sendText(); e.stopPropagation(); });
      renders.push(() => {
        $('.mode').textContent = S.laptop_stt ? 'speech-to-text: LAPTOP (sends text only)' : 'speech-to-text: on the robot (tars_voice)';
        $('.vl').innerHTML = (S.voice || []).slice().reverse().map(e => {
          if (e.type === 'heard') return `<div class="${e.action === 'stop' ? 'mc-k-stop' : ''}">${fmtT(e.t)} [${esc(e.src)}] “${esc(e.text)}” → <b>${esc(e.action)}</b> ${esc(e.intent)} ${(+e.prob).toFixed(2)} (${esc(e.why)})${e.lines && e.lines.length ? ': ' + esc(e.lines.join(' | ')) : ''}${e.stt_ms ? ` · whisper ${e.stt_ms} ms` : ''}</div>`;
          if (e.type === 'sent') return `<div class="mc-note">${fmtT(e.t)} → ${esc(e.to)} ${esc(e.line)} : ${esc(e.reply)}</div>`;
          if (e.type === 'say') return `<div class="mc-k-say">${fmtT(e.t)} TARS says: ${esc(e.text)}</div>`;
          if (e.type === 'memory') return `<div class="mc-k-say">${fmtT(e.t)} memory: ${esc(e.text)}</div>`;
          return `<div class="mc-k-goto">${fmtT(e.t)} laptop ${esc(e.src || '')}: ${esc(e.text)} → ${esc(e.reply)}</div>`;
        }).join('');
      });
      // push-to-talk: 16 kHz mono s16le (the same capture as mission control)
      let rec = null;
      async function start() {
        if (rec) return;
        rec = {chunks: [], stopping: false};
        ptt.classList.add('rec'); ptt.textContent = '🎤 listening… release to send';
        try {
          const stream = await navigator.mediaDevices.getUserMedia({audio: {channelCount: 1, echoCancellation: true, noiseSuppression: true}});
          const ctx = new AudioContext(), src = ctx.createMediaStreamSource(stream), node = ctx.createScriptProcessor(4096, 1, 1);
          node.onaudioprocess = ev => rec && rec.chunks.push(new Float32Array(ev.inputBuffer.getChannelData(0)));
          src.connect(node); node.connect(ctx.destination);
          Object.assign(rec, {stream, ctx, src, node, rate: ctx.sampleRate});
          if (rec.stopping) stop();
        } catch (e) { ptt.textContent = '🎤 no microphone: ' + e; rec = null; ptt.classList.remove('rec'); }
      }
      function stop() {
        if (!rec) return;
        if (!rec.ctx) { rec.stopping = true; return; }
        const r = rec; rec = null;
        ptt.classList.remove('rec'); ptt.textContent = '🎤 hold to talk';
        r.node.disconnect(); r.src.disconnect(); r.stream.getTracks().forEach(t => t.stop()); r.ctx.close();
        const n = r.chunks.reduce((a, c) => a + c.length, 0), all = new Float32Array(n);
        let o = 0; for (const c of r.chunks) { all.set(c, o); o += c.length; }
        const step = r.rate / 16000, out = new Int16Array(Math.floor(n / step));
        for (let i = 0; i < out.length; i++) { const a = Math.floor(i * step), b = Math.max(a + 1, Math.floor((i + 1) * step));
          let s = 0; for (let j = a; j < b; j++) s += all[j]; s /= (b - a); out[i] = Math.max(-1, Math.min(1, s)) * 32767; }
        if (out.length > 1600) fetch('/api/mission/audio', {method: 'POST', headers: {'Content-Type': 'application/octet-stream'}, body: out.buffer})
          .then(r => r.json()).then(d => Bench.log('voice', 'clip → ' + (d.reply || d.error))).catch(e => Bench.log('voice', String(e)));
      }
      ptt.onmousedown = start; ptt.onmouseup = stop; ptt.onmouseleave = stop;
      ptt.ontouchstart = e => { e.preventDefault(); start(); }; ptt.ontouchend = stop;
      const typing = e => ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName);
      window.addEventListener('keydown', e => { if ((e.key === 'v' || e.key === 'V') && !e.repeat && !typing(e) && !e.metaKey && !e.ctrlKey) start(); });
      window.addEventListener('keyup', e => { if ((e.key === 'v' || e.key === 'V') && !typing(e)) stop(); });
      window.addEventListener('blur', stop);
    }
  });

  // ------------------------------------------------------------------ Pi: services, journals, room view
  Bench.register({
    id: 'mc-pi', title: 'Pi: services · action journals · room view', order: 18, wide: true,
    mount(el) {
      el.innerHTML = `<div class="mc-row"><span class="ups"></span></div>
        <div class="mc-row"><label>ssh <input class="host" style="width:220px" placeholder="qnxuser@…"></label>
          <button class="jon">follow journals</button><button class="joff">stop</button><span class="jst mc-note"></span></div>
        <div class="mc-log jl" style="height:200px"></div>
        <div class="mc-row"><button class="ron">room view tunnel</button><button class="roff">close</button>
          <a class="rlink" target="_blank" rel="noopener"></a><button class="rshow" disabled>show here</button><span class="rst mc-note"></span></div>
        <div class="rbox"></div>
        <div class="mc-note">Andrew's tools over SSH (needs an ssh key for the Pi; nothing is asked for interactively):
          journals = <code>laptop/actions.command</code> (follow_actions.py on the Pi: tars-{input,motor,nav}-actions.log);
          room view = <code>laptop/room-view.command</code> (Pi 127.0.0.1:5580 → this Mac 127.0.0.1:5580).</div>`;
      const $ = s => el.querySelector(s);
      const host = () => $('.host').value.trim();
      $('.jon').onclick = () => api('journal', {on: true, host: host()});
      $('.joff').onclick = () => api('journal', {on: false});
      $('.ron').onclick = () => api('roomview', {on: true, host: host()});
      $('.roff').onclick = () => { api('roomview', {on: false}); $('.rbox').innerHTML = ''; };
      $('.rshow').onclick = () => { $('.rbox').innerHTML = $('.rbox').innerHTML ? '' : `<iframe class="mc-frame" src="${esc((S.roomview || {}).url)}"></iframe>`; };
      api('journal').then(j => { if (!$('.host').value) $('.host').value = j.host || ''; (j.lines || []).forEach(add); });
      const jl = $('.jl');
      function add(line) { const d = document.createElement('div'); d.textContent = line; jl.prepend(d); while (jl.childElementCount > 400) jl.lastChild.remove(); }
      Bench.on('journal', j => add(j.line));
      renders.push(() => {
        const L = S.links || {}, R = S.relay || {}, rs = (S.relay_age ?? 99) < 2.5;
        const pill = (n, ok, t) => `<span class="mc-pill ${ok ? 'up' : 'down'}" title="${esc(t || '')}">${esc(n)}</span>`;
        let h = 'links: ' + Object.entries(L).map(([k, v]) => pill(k, v.up, v.error)).join('');
        if (S.target === 'pi') h += ' · Pi services: ' + pill('tars_motor', rs && R.motor == '1') + pill('goto', rs && R.goto == '1') + pill('voice', rs && R.voice == '1') +
          pill('phone tee', rs && R.phone_tee == '1' && +R.phone_age < 3) + pill('camera', rs && +R.rgb_age >= 0 && +R.rgb_age < 2);
        $('.ups').innerHTML = h;
        const J = S.journal || {}, RV = S.roomview || {};
        $('.jst').textContent = J.running ? `following ${J.host}` : (J.error ? 'stopped: ' + J.error : 'off');
        $('.rst').textContent = RV.running ? `tunnel to ${RV.host} open` : (RV.error ? 'closed: ' + RV.error : 'closed');
        $('.rlink').textContent = RV.running ? RV.url : ''; $('.rlink').href = RV.url || '#';
        $('.rshow').disabled = !RV.running;
      });
    }
  });
})();

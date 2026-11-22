// "Talk to QNXplorer": the one obvious place to give the robot a command. Type, hold to talk, or click a
// suggestion. Goes through the voice lab plugin (the robot's real whisper + C classifier); with
// "Send to robot" on, the robot's tars_voice gets HEAR <text> / AUDIO and decides itself.
// History shows every voice result (from here, the Voice lab tab or its T key): heard -> intent -> sent.
'use strict';
(() => {
  const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
  const CHIPS = ['go to the trash can', 'turn left', 'turn right', 'go forward half a meter', 'turn around', 'look around', 'what do you remember', 'stop'];
  const NICE = {forward: 'drive forward', back: 'back up', left: 'turn left', right: 'turn right', turn_around: 'turn around',
    stop: 'STOP', goto: 'go to', spin: 'spin', status: 'status', scan: 'look around', forget: 'forget', memory: 'say what it remembers',
    recall: 'recall', unknown: 'not understood'};
  const css = `
  .cmd-in { display:flex; gap:6px; }
  .cmd-in input { flex:1; min-width:0; font-size:15px; padding:7px 9px; }
  .cmd-mic { width:100%; margin-top:6px; padding:9px; font-size:15px; font-weight:600; }
  .cmd-mic.rec { background: var(--bad); border-color:#ff9a9a; color:#fff; }
  .cmd-send { display:flex; align-items:center; gap:8px; margin:8px 0 4px; padding:6px 8px; border-radius:6px; border:1px solid var(--line); cursor:pointer; user-select:none; }
  .cmd-send.on { border-color: var(--ok); background:#11281c; }
  .cmd-send.off { border-color: var(--warn); background:#2a2410; }
  .cmd-send .sw { width:34px; height:18px; border-radius:9px; background:#4b5862; position:relative; flex:none; transition:background .15s; }
  .cmd-send .sw::after { content:''; position:absolute; top:2px; left:2px; width:14px; height:14px; border-radius:50%; background:#fff; transition:left .15s; }
  .cmd-send.on .sw { background: var(--ok); } .cmd-send.on .sw::after { left:18px; }
  .cmd-send .txt { font-size:13px; } .cmd-send .txt b { display:block; }
  .cmd-chips { display:flex; flex-wrap:wrap; gap:5px; margin:8px 0 2px; }
  .cmd-chips button { border-radius:999px; padding:3px 10px; font-size:13px; background:#1d2a33; }
  .cmd-chips button.stopchip { border-color:#6a2a2a; color:#ffb3b3; }
  .cmd-hist { margin-top:8px; display:flex; flex-direction:column; gap:5px; }
  .cmd-item { border-left:3px solid var(--line); padding:3px 8px; background:#131b20; border-radius:0 6px 6px 0; font-size:13px; }
  .cmd-item.command { border-left-color: var(--ok); } .cmd-item.stop { border-left-color: var(--bad); }
  .cmd-item.reject, .cmd-item.ignore { border-left-color: var(--warn); }
  .cmd-item .heard { font-weight:600; } .cmd-item .flow { color: var(--dim); font-size:12px; }
  .cmd-item .flow .arrow { margin:0 4px; }
  .cmd-empty { color: var(--dim); font-size:13px; padding:6px 0; }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));

  Bench.register({
    id: 'command', title: '🗣 Talk to QNXplorer', order: 1, area: 'drive',
    help: 'Say or type what the robot should do. No “TARS” wake word needed here.',
    mount(el) {
      el.innerHTML = `
        <div class="cmd-in"><input class="txt-in" type="text" placeholder="e.g. go to the trash can" aria-label="command">
          <button class="go primary" title="Send (Enter)">Send</button></div>
        <button class="cmd-mic" title="Hold this button (or hold the T key) and speak; release to send">🎤 Hold to talk <kbd>T</kbd></button>
        <div class="cmd-send off" role="switch" tabindex="0" aria-checked="false"><span class="sw"></span>
          <span class="txt"><b class="sw-title">Send to robot</b><span class="sw-note dim">checking…</span></span></div>
        <div class="dim" style="font-size:12px">Try one:</div>
        <div class="cmd-chips">${CHIPS.map(c => `<button class="${c === 'stop' ? 'stopchip' : ''}" data-c="${esc(c)}">${esc(c)}</button>`).join('')}</div>
        <div class="dim" style="font-size:12px;margin-top:8px">Recent <span class="dim" title="What it heard → what the classifier decided → what the robot did">ⓘ</span></div>
        <div class="cmd-hist"><div class="cmd-empty">Nothing yet. Click a suggestion above to try it.</div></div>`;
      const $ = s => el.querySelector(s);
      const sw = $('.cmd-send'), input = $('.txt-in'), micBtn = $('.cmd-mic'), hist = $('.cmd-hist');
      const target = (Bench.info.target || {}).name, real = target === 'pi' || target === 'usb' || target === 'robot';
      let info = {}, available = Bench.info.plugins.includes('voicelab');

      function paintSwitch() {
        const on = !!info.send;
        sw.className = 'cmd-send ' + (on ? 'on' : 'off');
        sw.setAttribute('aria-checked', on);
        $('.sw-title').textContent = on ? (real ? 'Sending to the REAL robot' : 'Send to robot: ON') : 'Dry run: nothing moves';
        $('.sw-note').textContent = !available ? 'voice lab plugin not loaded' : !info.robot_plugin ? 'robot link not loaded: dry run only'
          : target === 'pi' || target === 'robot' ? (on ? 'Sent as-is; the Pi transcribes and decides. Click to stop sending.'
                                  : 'Off: nothing is sent (no dry run here, the Pi decides). Click to send.')
          : on ? 'The robot hears it and acts. Click to switch to dry run.' : 'Only shows what it would do. Click to send for real.';
      }
      async function refresh() {
        if (!available) { paintSwitch(); return; }
        try { info = await Bench.api('voicelab/info'); } catch (e) { available = false; }
        paintSwitch();
      }
      async function setSend(on) {
        try { await Bench.api('voicelab/send', {on}); } catch (e) { Bench.log('command', e.message); }
        await refresh();
      }
      sw.onclick = () => {
        if (!info.robot_plugin) return;
        const on = !info.send;
        try { localStorage.setItem('bench.sendOff', on ? '' : '1'); } catch (e) { /* private mode */ }
        setSend(on);
      };
      sw.onkeydown = e => { if (e.key === 'Enter') sw.onclick(); };

      // SIM: send on by default (unless this browser turned it off). REAL: stays as the server has it.
      (async () => {
        await refresh();
        let off = null; try { off = localStorage.getItem('bench.sendOff'); } catch (e) { /* private mode */ }
        if (!real && info.robot_plugin && !info.send && !off) setSend(true);
      })();

      async function sendText(text) {
        text = (text || '').trim();
        if (!text) return;
        if (!available) { addItem({text, action: 'reject', intent: '-', sent: [], robot: {error: 'voice lab plugin not loaded'}}); return; }
        if (/^stop\b/i.test(text)) Bench.api('robot/stop', {why: 'command box: stop'}).catch(() => {});   // stop never waits on the classifier
        try { await Bench.api('voicelab/text', {text, mode: 'ptt'}); }   // the result arrives on topic `voice`
        catch (e) { addItem({text, action: 'reject', intent: '-', sent: [], robot: {error: e.message}}); }
      }
      $('.go').onclick = () => { sendText(input.value); input.select(); };
      input.addEventListener('keydown', e => { if (e.key === 'Enter') { sendText(input.value); input.select(); } });
      el.querySelectorAll('.cmd-chips button').forEach(b => b.onclick = () => { input.value = b.dataset.c; sendText(b.dataset.c); });

      // mic: hold the button (the T key is the Voice lab's, which uses the same Bench.mic)
      async function micDown(e) {
        e.preventDefault();
        if (!available) return;
        try { await Bench.mic.start(); } catch (err) { micBtn.textContent = '🎤 ' + err.message; }
      }
      async function micUp() {
        if (!Bench.mic.recording || !ownMic) return;
        ownMic = false;
        const wav = await Bench.mic.stop();
        if (!wav) return;
        micBtn.textContent = '… listening to the clip';
        try {
          const r = await fetch('/api/voicelab/audio?mode=ptt', {method: 'POST', headers: {'Content-Type': 'audio/wav'}, body: wav});
          const d = await r.json();
          if (!r.ok) throw new Error(d.error || r.status);
        } catch (err) { addItem({text: '(audio)', action: 'reject', intent: '-', sent: [], robot: {error: err.message}}); }
        paintMic(false);
      }
      let ownMic = false;
      micBtn.addEventListener('pointerdown', e => { ownMic = true; micDown(e); });
      micBtn.addEventListener('pointerup', micUp);
      micBtn.addEventListener('pointerleave', micUp);
      const paintMic = on => { micBtn.classList.toggle('rec', on); micBtn.innerHTML = on ? '🎤 Listening… release to send' : '🎤 Hold to talk <kbd>T</kbd>'; };
      Bench.on('mic', m => { paintMic(!!m.on); if (m.error) micBtn.textContent = '🎤 ' + m.error; });

      function addItem(r) {
        hist.querySelector('.cmd-empty')?.remove();
        const d = document.createElement('div');
        d.className = 'cmd-item ' + (r.action || '');
        const intent = r.action === 'command' || r.action === 'stop' ? `${NICE[r.intent] || r.intent}${r.prob ? ` <span class="dim">(${(+r.prob).toFixed(2)})</span>` : ''}`
          : r.action === 'reject' ? `<span class="warn">not sure</span>${r.intent && r.intent !== '-' ? ` <span class="dim">(${esc(NICE[r.intent] || r.intent)}? ${r.prob ? (+r.prob).toFixed(2) : ''})</span>` : ''}`
          : r.action === 'ignore' ? '<span class="warn">ignored</span>' : esc(r.action || '');
        const lines = (r.sent || []).map(s => s.line).filter(l => !/^SAY /.test(l));
        let out;
        // reject/ignore: the classifier chose to do nothing (at most an LED/beep), so never say "sent to robot"
        if (r.action === 'reject') out = '<span class="warn">robot did nothing (not sure)</span>';
        else if (r.action === 'ignore') out = '<span class="dim">robot did nothing (ignored)</span>';
        else if (r.robot && r.robot.error) out = `<span class="bad">${esc(r.robot.error)}</span>`;
        else if (r.robot) out = `<span class="ok">sent to robot</span>`;
        else out = lines.length ? `<span class="warn">dry run</span>, would send <span class="mono">${esc(lines.join(' · '))}</span>` : '<span class="dim">nothing to send</span>';
        d.innerHTML = `<div class="heard">${r.src === 'audio' ? '🎤 ' : ''}“${esc(r.text)}”</div>
          <div class="flow">${intent}<span class="arrow">→</span>${out}</div>`;
        d.title = (r.robot && r.robot.reply ? 'robot replied: ' + r.robot.reply + '\n' : '') + 'lines: ' + ((r.sent || []).map(s => s.line).join(' | ') || 'none');
        hist.prepend(d);
        while (hist.childElementCount > 5) hist.lastChild.remove();
      }
      Bench.on('voice', d => {
        if (d.kind === 'result') addItem(d);
        else if (d.kind === 'send') refresh();
      });
    },
  });
})();

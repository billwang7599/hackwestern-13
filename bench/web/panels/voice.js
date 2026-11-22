// Voice lab: text or laptop mic -> the robot's real whisper + C classifier on the laptop.
// Shows the decision, top-3, slots and the EXACT lines tars_voice sends (via a shadow
// tars_voice_host on fake listeners). Send switch (off = dry run) goes through the robot plugin.
'use strict';
(() => {
  const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
  const css = `
  .vl-row { display:flex; gap:6px; align-items:center; flex-wrap:wrap; margin:4px 0; }
  .vl-row input[type=text] { flex:1; min-width:180px; }
  .vl-card { border:1px solid var(--line); border-radius:6px; padding:6px 8px; margin:6px 0; }
  .vl-big { font-size:16px; font-weight:600; }
  .vl-bar { display:inline-block; height:8px; background:var(--accent); border-radius:2px; vertical-align:middle; }
  .vl-lines td:first-child { color:var(--dim); width:60px; }
  .vl-lines .stop td { color:var(--bad); font-weight:600; }
  #vl-ptt.rec { background:var(--bad); }
  .vl-hist div { font:12px/1.4 var(--mono); white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .vl-conf td, .vl-conf th { text-align:center; font:11px var(--mono); padding:1px 4px; }
  .vl-conf td.diag { color:var(--ok); } .vl-conf td.off { color:var(--warn); }
  details.vl summary { cursor:pointer; color:var(--dim); margin:6px 0; }`;

  Bench.register({
    id: 'voice', title: 'Voice lab', order: 40, wide: true, area: 'voice',
    help: 'Type or speak a phrase and see exactly what the classifier decides and every line the robot would send. Hold T anywhere on the page to talk.',
    mount(el) {
      const st = document.createElement('style'); st.textContent = css; document.head.appendChild(st);
      el.innerHTML = `
        <div id="vl-info" class="mono dim">loading…</div>
        <div class="vl-row">
          <input id="vl-text" type="text" placeholder="type what someone says, e.g. tars go to the trash can (Enter)">
          <select id="vl-mode" title="push-to-talk = the robot's HEAR/AUDIO net input (no wake word); robot mic = wake word required">
            <option value="ptt">push-to-talk (no wake word)</option><option value="mic">robot mic (needs “TARS”)</option></select>
          <button id="vl-go">classify</button>
          <button id="vl-ptt" title="hold, or hold T">🎤 hold to talk (T)</button>
        </div>
        <div class="vl-row">
          <label><input id="vl-send" type="checkbox"> send to robot</label>
          <span id="vl-sendnote" class="dim"></span>
        </div>
        <div id="vl-out"></div>
        <details class="vl"><summary>history</summary><div id="vl-hist" class="vl-hist"></div></details>
        <details class="vl" id="vl-batchbox"><summary>batch: accuracy, confusion, latency</summary>
          <div class="vl-row">
            <select id="vl-src"><option value="realistic">realistic.tsv (hand-written whisper text, labelled)</option>
              <option value="phrases">phrases.txt (training phrases, filled)</option><option value="wavdir">folder of WAVs</option></select>
            <input id="vl-path" type="text" placeholder="WAV folder (labels.tsv or intent__name.wav)" style="display:none">
          </div>
          <div class="vl-row">
            <label><input id="vl-audio" type="checkbox"> speak it (macOS say → whisper)</label>
            <input id="vl-voice" type="text" placeholder="say voice (blank = default)" style="max-width:160px">
            noise <select id="vl-noise"><option>none</option><option>white</option><option>babble</option></select>
            SNR <input id="vl-snr" type="number" value="15" style="width:56px"> dB
            <input id="vl-babble" type="text" placeholder="babble WAV (blank = say crowd)" style="max-width:180px">
          </div>
          <div class="vl-row">
            mode <select id="vl-bmode"><option value="">default (realistic: mic, else ptt)</option><option value="mic">robot mic</option><option value="ptt">push-to-talk</option></select>
            limit <input id="vl-limit" type="number" placeholder="all" style="width:64px">
            <button id="vl-run">run batch</button><button id="vl-cancel">cancel</button>
            <span id="vl-prog" class="dim"></span>
          </div>
          <div id="vl-batch"></div>
        </details>`;
      const $ = id => el.querySelector('#' + id);
      let info = {};

      async function refreshInfo() {
        try { info = await Bench.api('voicelab/info'); } catch (e) { $('vl-info').textContent = 'voice lab plugin not loaded: ' + e.message; return; }
        const w = info.whisper_model || {}, c = info.classifier || {};
        $('vl-info').innerHTML =
          `whisper <b>${esc(w.file)}</b> ${w.present ? 'sha ' + esc(w.sha256) : '<span class="bad">MISSING (get_model.sh)</span>'} · ` +
          `classifier intent_model.h sha ${esc(c.sha256)} thr ${c.threshold} stop ${c.stop_threshold}` +
          (c.intent_cli_stale ? ' <span class="bad">intent_cli older than the header: rebuild</span>' : '') +
          Object.entries(info.bins || {}).filter(([, ok]) => !ok).map(([k]) => ` <span class="bad">${k} missing (make -C iter2-autonomy/voice host)</span>`).join('') +
          `<br>shadow: ${esc(info.shadow)} · ${esc(info.reference)}`;
        $('vl-send').checked = !!info.send;
        $('vl-send').disabled = !info.robot_plugin;
        $('vl-sendnote').textContent = info.robot_plugin
          ? (info.send ? `ON → tars_voice on target ${info.target?.name} (the robot's classifier decides)` : 'off = dry run, nothing leaves the laptop')
          : 'robot plugin not loaded: dry run only';
      }

      function bars(top) {
        return (top || []).map(([k, p]) => `<div><span style="display:inline-block;width:90px">${esc(k)}</span>` +
          `<span class="vl-bar" style="width:${Math.round(p * 160)}px"></span> ${p.toFixed(3)}</div>`).join('');
      }

      function render(r) {
        const d = r.decision, x = r.detail || {};
        const cls = d.action === 'stop' ? 'bad' : d.action === 'command' ? 'ok' : d.action === 'reject' ? 'warn' : 'dim';
        const lines = (r.sent || []).map(s => `<tr class="${s.line === 'S' ? 'stop' : ''}"><td>${esc(s.to)}</td><td class="mono">${esc(s.line)}</td></tr>`).join('')
          || '<tr><td colspan=2 class="dim">nothing sent</td></tr>';
        const amt = x.amount ? `${x.amount.value} ${x.amount.unit || '(no unit)'} → MOVE ${x.amount.move_mm} mm / TURN ${x.amount.turn_deg}°` : 'none (defaults: 300 mm / 30°)';
        let robot = '';
        if (r.robot) robot = `<div class="vl-card"><b>sent to robot:</b> <span class="mono">${esc(r.robot.line || '')}</span> → ` +
          (r.robot.error ? `<span class="bad">${esc(r.robot.error)}</span>` : `<span class="mono">${esc(typeof r.robot.reply === 'string' ? r.robot.reply : JSON.stringify(r.robot.reply))}</span>`) +
          (r.robot.note ? `<div class="warn">${esc(r.robot.note)}</div>` : '') + '</div>';
        $('vl-out').innerHTML = `
          <div class="vl-card">
            <div>${r.src === 'audio' ? `🎤 ${r.audio_s}s → whisper heard:` : 'text:'} <span class="vl-big">“${esc(r.text)}”</span></div>
            <div class="vl-big ${cls}">${esc(d.action)} ${d.intent !== '-' ? esc(d.intent) : ''} ${d.prob ? d.prob.toFixed(2) : ''} <span class="dim">(${esc(d.why)})</span></div>
            <div class="dim">mode ${r.mode === 'mic' ? 'robot mic, wake word required' : 'push-to-talk, no wake word'} · wake word ${x.wake_word ? 'heard' : 'not heard'}` +
            `${x.safety_stop ? ' · <b class="bad">safety stop word</b>' : ''} · threshold ${x.threshold}</div>
            ${(r.notes || []).map(n => `<div class="${/MISMATCH|PARITY|STOP ORDER/.test(n) ? 'bad' : 'warn'}">${esc(n)}</div>`).join('')}
          </div>
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">
            <div class="vl-card"><b>lines the robot sends</b> <span class="dim">(${esc(r.sent_source)})</span>
              <table class="vl-lines">${lines}</table></div>
            <div class="vl-card"><b>top-3</b> <span class="dim">(intent.py, same weights)</span>${bars(x.top3)}
              <div>amount: ${esc(amt)}</div><div>object: ${esc(x.object || 'none')}</div>
              <div class="dim mono" title="tokens after the wake word">rest: ${esc((x.rest || []).join(' '))}</div></div>
          </div>
          <div class="dim">latency: ${r.stt_ms != null ? `whisper ${r.stt_ms} ms · ` : ''}classify ${r.classify_ms} ms <span title="${esc(r.classify_note)}">(ⓘ incl. process spawn)</span></div>
          ${robot}`;
        const h = document.createElement('div');
        h.innerHTML = `${new Date().toLocaleTimeString()} <span class="${cls}">${esc(d.action)} ${esc(d.intent)}</span> “${esc(r.text)}” → ${esc((r.sent || []).map(s => s.line).join(' | '))}`;
        $('vl-hist').prepend(h);
        while ($('vl-hist').childElementCount > 40) $('vl-hist').lastChild.remove();
      }

      async function classify() {
        const text = $('vl-text').value.trim();
        if (!text) return;
        try { render(await Bench.api('voicelab/text', {text, mode: $('vl-mode').value})); }
        catch (e) { $('vl-out').innerHTML = `<div class="bad">${esc(e.message)}</div>`; }
        refreshInfo();
      }
      $('vl-go').onclick = classify;
      $('vl-text').addEventListener('keydown', e => { if (e.key === 'Enter') classify(); });
      $('vl-send').onchange = async () => {
        try { await Bench.api('voicelab/send', {on: $('vl-send').checked}); } catch (e) { Bench.log('voice', e.message); }
        refreshInfo();
      };

      // ---- push-to-talk: the shell's shared capture (Bench.mic, 16 kHz mono WAV, as mission control)
      let mine = false;
      async function pttStart() {
        if (Bench.mic.recording) return;
        mine = true;
        try { await Bench.mic.start(); } catch (e) { mine = false; $('vl-ptt').textContent = '🎤 no microphone: ' + e.message; }
      }
      async function pttStop() {
        if (!mine) return;
        mine = false;
        const wav = await Bench.mic.stop();
        if (!wav) return;
        try {
          const resp = await fetch('/api/voicelab/audio?mode=' + $('vl-mode').value, {method: 'POST', headers: {'Content-Type': 'audio/wav'}, body: wav});
          const data = await resp.json();
          if (!resp.ok) throw new Error(data.error || resp.status);
          render(data);
        } catch (e) { $('vl-out').innerHTML = `<div class="bad">audio: ${esc(e.message)}</div>`; }
      }
      Bench.on('mic', m => {
        $('vl-ptt').classList.toggle('rec', !!m.on);
        $('vl-ptt').textContent = m.on ? '🎤 listening… release' : '🎤 hold to talk (T)';
      });
      $('vl-ptt').onmousedown = pttStart; $('vl-ptt').onmouseup = pttStop; $('vl-ptt').onmouseleave = pttStop;
      const typing = e => /^(input|textarea|select)$/i.test(e.target.tagName || '');
      window.addEventListener('keydown', e => { if ((e.key === 't' || e.key === 'T') && !e.repeat && !typing(e)) pttStart(); });
      window.addEventListener('keyup', e => { if ((e.key === 't' || e.key === 'T') && !typing(e)) pttStop(); });

      // ---- batch
      $('vl-src').onchange = () => { $('vl-path').style.display = $('vl-src').value === 'wavdir' ? '' : 'none'; };
      $('vl-run').onclick = async () => {
        const body = {source: $('vl-src').value, path: $('vl-path').value, audio: $('vl-audio').checked,
          voice: $('vl-voice').value, noise: $('vl-noise').value, snr: +$('vl-snr').value,
          babble_file: $('vl-babble').value, limit: +$('vl-limit').value || 0};
        if ($('vl-bmode').value) body.mode = $('vl-bmode').value;
        $('vl-prog').textContent = 'running…'; $('vl-batch').innerHTML = '';
        try {
          const r = await Bench.api('voicelab/batch', body);
          if (!r.started) renderBatch(r);
        } catch (e) { $('vl-prog').textContent = ''; $('vl-batch').innerHTML = `<div class="bad">${esc(e.message)}</div>`; }
      };
      $('vl-cancel').onclick = () => Bench.api('voicelab/batch_cancel', {});
      Bench.on('voice', d => {
        if (d.kind === 'batch' && d.running) $('vl-prog').textContent = `${d.done}/${d.total}…`;
        else if (d.kind === 'batch' && !d.running && (d.audio || d.error)) Bench.api('voicelab/batch_last').then(renderBatch);
        else if (d.kind === 'send') refreshInfo();
      });

      function renderBatch(r) {
        $('vl-prog').textContent = '';
        if (r.error) { $('vl-batch').innerHTML = `<div class="bad">${esc(r.error)}</div>`; return; }
        const labs = [...new Set([...Object.keys(r.confusion), ...Object.values(r.confusion).flatMap(o => Object.keys(o))])].sort();
        const conf = `<table class="vl-conf"><tr><th>want \\ got</th>${labs.map(l => `<th>${esc(l)}</th>`).join('')}</tr>` +
          Object.keys(r.confusion).sort().map(w => `<tr><th>${esc(w)}</th>${labs.map(g => {
            const n = r.confusion[w][g] || 0;
            const ok = g === w || (w === 'reject' && (g === 'ignore')) || (w === 'unknown' && g === 'reject');
            return `<td class="${n ? (ok ? 'diag' : 'off') : 'dim'}">${n || '·'}</td>`;
          }).join('')}</tr>`).join('') + '</table>';
        const L = r.latency_ms || {};
        const worst = (r.worst || []).map(x => `<tr><td class="${x.moved ? 'bad' : 'warn'}">${x.moved ? 'WRONG MOTION' : 'safe miss'}</td>` +
          `<td>${esc(x.text)}${x.heard !== x.text ? `<div class="dim">heard: ${esc(x.heard)}</div>` : ''}</td><td>${esc(x.want)} ${esc(x.cmd !== '-' ? x.cmd : '')}</td>` +
          `<td>${esc(x.got)} ${x.prob}</td><td class="mono">${esc(x.lines.join(' | '))}</td></tr>`).join('');
        $('vl-batch').innerHTML = `
          <div class="vl-card"><span class="vl-big">${r.correct}/${r.labelled} = ${r.accuracy != null ? (r.accuracy * 100).toFixed(1) + '%' : '-'}</span>
            · wrong motions <b class="${r.wrong_motion ? 'bad' : 'ok'}">${r.wrong_motion}</b> · stop recall ${esc(r.stop_recall)}
            <div class="dim">${esc(r.source)} · mode ${esc(r.mode)} · ${r.audio ? `spoken (${esc(r.voice)}) → whisper, noise ${esc(r.noise)}${r.snr != null ? ' @ ' + r.snr + ' dB SNR' : ''}` : 'text only (no whisper)'}
            · ${r.seconds}s · load ${r.load != null ? r.load.toFixed(2) : '?'}${r.cancelled ? ' · CANCELLED' : ''}</div>
            <div class="dim">latency: ${L.stt_median != null ? `whisper median ${L.stt_median} / p95 ${L.stt_p95} ms · ` : ''}classify median ${L.classify_median} / p95 ${L.classify_p95} ms (${esc(L.note)})</div></div>
          ${conf}
          <details class="vl" open><summary>worst misses (${(r.worst || []).length})</summary><table>${worst || '<tr><td class="ok">none</td></tr>'}</table></details>`;
      }

      refreshInfo();
    }
  });
})();

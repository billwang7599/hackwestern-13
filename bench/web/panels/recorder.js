// Recorder panel: everything the bench sees goes to bench/recordings/*.jsonl; replay re-shows
// a recording with its timing. REPLAY NEVER SENDS ANYTHING TO THE ROBOT (display only).
'use strict';
(() => {
  const css = `
  #replay-banner { position:fixed; top:0; left:0; right:0; z-index:50; background:#6b4a00; color:#fff;
    text-align:center; font: 600 13px system-ui; padding:4px; display:none; }
  body.replaying #replay-banner { display:block; }
  body.replaying main#panels { outline: 3px solid #b07a00; outline-offset:-3px; }
  .rec-list { max-height:160px; overflow:auto; font: 12px/1.6 var(--mono); }
  .rec-list div { display:flex; gap:6px; align-items:center; }
  .rec-list span { flex:1; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));
  const banner = Object.assign(document.createElement('div'), {id: 'replay-banner'});
  document.body.appendChild(banner);

  Bench.register({
    id: 'recorder', title: 'Recorder / replay', short: '⏺ Recorder', order: 40, area: 'tests',
    help: 'Everything is recorded from the start. Replay never sends anything to the robot.',
    mount(el) {
      el.innerHTML = `<div class="dim" style="font-size:12px">Every event (telemetry and every command sent) is saved as JSONL in bench/recordings/.
        Replay only redraws the panels: <b>nothing is sent to the robot</b>. STOP still stops the live robot.</div>
        <div style="margin:6px 0"><button class="tog"></button> <button class="new">new file</button> <span class="mono st"></span></div>
        <div style="margin:6px 0">replay speed <select class="spd"><option>0.5</option><option selected>1</option><option>2</option><option>5</option></select>
        <button class="stop">stop replay</button> <button class="ref">refresh</button></div>
        <div class="rec-list"></div>`;
      const $ = s => el.querySelector(s);
      let st = {};
      const kb = n => n > 1e6 ? (n / 1e6).toFixed(1) + ' MB' : (n / 1e3).toFixed(0) + ' kB';
      async function refresh() {
        try { st = await Bench.api('recorder/state'); } catch (e) { el.textContent = 'recorder plugin not loaded'; return; }
        $('.tog').textContent = st.recording ? 'pause recording' : 'record';
        $('.st').textContent = st.recording ? `● ${st.file} ${st.events} ev ${kb(st.bytes)}` : 'not recording';
        const L = $('.rec-list'); L.innerHTML = '';
        for (const r of st.recordings || []) {
          const d = document.createElement('div');
          d.innerHTML = `<span></span><i class="dim">${kb(r.bytes)}</i>`;
          d.firstChild.textContent = r.name;
          const b = Object.assign(document.createElement('button'), {textContent: 'replay'});
          b.onclick = () => Bench.api('recorder/replay', {name: r.name, speed: +$('.spd').value}).catch(e => Bench.log('replay', e.message));
          const a = Object.assign(document.createElement('a'), {textContent: 'file', href: '/api/recorder/file?name=' + encodeURIComponent(r.name), download: r.name});
          d.append(b, a); L.appendChild(d);
        }
      }
      $('.tog').onclick = () => Bench.api('recorder/record', {on: !st.recording}).then(refresh);
      $('.new').onclick = () => Bench.api('recorder/record', {new: true, on: true}).then(refresh);
      $('.stop').onclick = () => Bench.api('recorder/stop', {});
      $('.ref').onclick = refresh;
      Bench.on('replay', r => {
        document.body.classList.toggle('replaying', !!r.active);
        banner.textContent = r.active ? `REPLAY of ${r.name} (${r.pos}/${r.len} s, ×${r.speed}): display only, nothing is sent to the robot. Live status is paused.` : '';
      });
      refresh(); setInterval(() => { if (!document.hidden) refresh(); }, 5000);
    }
  });
})();

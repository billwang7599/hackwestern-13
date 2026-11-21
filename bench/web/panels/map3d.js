// iPhone LiDAR -> 3D map. The map itself is Andrew's room view (dashboard/room_view.html +
// room_controls.js), served unchanged by his serve_room() on the phone plugin's room port
// and shown in an iframe. This panel only picks the source and shows link state.
'use strict';
Bench.register({
  id: 'map3d', title: 'iPhone 3D map', order: 40, wide: true,
  mount(el, B) {
    el.innerHTML = `
      <div class="m3-bar">
        <label>source <select id="m3-src">
          <option value="off">off</option>
          <option value="sim">sim room (ray-cast)</option>
          <option value="replay">recording</option>
          <option value="live">live iPhone (USB, this Mac)</option>
          <option value="iproxy">live via my own iproxy</option>
        </select></label>
        <span id="m3-opts"></span>
        <button id="m3-apply">apply</button>
        <button id="m3-rec">● record</button>
        <button id="m3-clear" title="Andrew's map_clear">clear map</button>
        <label>target <select id="m3-target"><option value="">—</option></select></label>
        <a id="m3-open" target="_blank" rel="noopener">open full view ↗</a>
      </div>
      <div id="m3-state" class="mono m3-state">loading…</div>
      <iframe id="m3-frame" title="Room view (Andrew's renderer)"
              style="width:100%;height:560px;border:0;border-radius:6px;background:#0b1013"></iframe>
      <div class="mono m3-note">Sim and recordings are not the phone. Live = MotionUSB on an iPhone
        plugged into this Mac (usbmux); occupancy is Andrew's Python reference grid, not goto's native map.</div>`;
    const $ = id => el.querySelector('#' + id);
    const style = document.createElement('style');
    style.textContent = `.m3-bar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:6px}
      .m3-state{font-size:12px;margin:4px 0;white-space:pre-wrap}.m3-note{font-size:11px;opacity:.7;margin-top:4px}
      .m3-bar a{font-size:12px;margin-left:auto}.m3-bad{color:#ef5f5f}.m3-good{color:#4cc38a}`;
    el.appendChild(style);
    let frameUrl = '', recording = false, lastObjects = '';

    async function optsFor(kind) {
      const box = $('m3-opts');
      if (kind === 'replay') {
        let recs = [];
        try { recs = (await B.api('phone/recordings')).recordings; } catch (e) {}
        box.innerHTML = `<select id="m3-file">${recs.map(r => `<option>${r}</option>`).join('')}</select>
          speed <input id="m3-speed" type="number" value="1" min="0" step="0.5" style="width:4em">
          <label><input id="m3-loop" type="checkbox"> loop</label>`;
      } else if (kind === 'sim') {
        box.innerHTML = `pose <select id="m3-pose"><option value="auto">car truth, else script</option>
          <option value="car">car truth only</option><option value="script">scripted path</option></select>
          noise m <input id="m3-noise" type="number" value="0.01" step="0.005" style="width:5em">`;
      } else if (kind === 'iproxy') {
        box.innerHTML = `port <input id="m3-port" type="number" value="5595" style="width:6em">
          <span class="mono">(run: iproxy -l -s 127.0.0.1 5595:5561)</span>`;
      } else if (kind === 'live') {
        try {
          const d = await B.api('phone/devices');
          box.textContent = d.devices.length ? 'usbmux: ' + d.devices.map(x => `${x.connection} ${x.serial.slice(-6)}`).join(', ')
                                             : 'usbmux: no iPhone (' + (d.error || 'plug in, unlock, Trust') + ')';
        } catch (e) { box.textContent = 'usbmux check failed: ' + e.message; }
      } else box.textContent = '';
    }
    $('m3-src').onchange = () => optsFor($('m3-src').value);

    $('m3-apply').onclick = async () => {
      const kind = $('m3-src').value, body = {kind};
      if (kind === 'replay') Object.assign(body, {path: $('m3-file')?.value, speed: +$('m3-speed').value, loop: $('m3-loop').checked});
      if (kind === 'sim') Object.assign(body, {pose: $('m3-pose').value, range_noise: +$('m3-noise').value});
      if (kind === 'iproxy') body.port = +$('m3-port').value;
      try { show(await B.api('phone/source', body)); } catch (e) { B.log('phone', 'source failed: ' + e.message); }
    };
    $('m3-rec').onclick = async () => {
      const r = await B.api('phone/record', {on: !recording});
      if (r.saved) B.log('phone', `saved ${r.scenes} scenes to ${r.saved}`);
    };
    $('m3-clear').onclick = async () => B.log('phone', (await B.api('phone/command', {op: 'map_clear'})).message);
    $('m3-target').onchange = async () => {
      const id = $('m3-target').value;
      if (id) B.log('phone', 'path preview: ' + (await B.api('phone/command', {op: 'select_target', id})).message);
    };

    async function refreshTargets() {
      try {
        const room = await B.api('phone/map.json');
        const objs = (room.objects || []).filter(o => o.id !== 'truth');
        const key = objs.map(o => o.id + o.label).join();
        if (key === lastObjects) return;
        lastObjects = key;
        const cur = $('m3-target').value;
        $('m3-target').innerHTML = '<option value="">—</option>' +
          objs.map(o => `<option value="${o.id}">${o.label} #${o.id}</option>`).join('');
        $('m3-target').value = cur;
      } catch (e) {}
    }

    function show(s) {
      if (s.room_url && s.room_url !== frameUrl) {
        frameUrl = s.room_url; $('m3-frame').src = frameUrl; $('m3-open').href = frameUrl;
      }
      if (document.activeElement !== $('m3-src') && $('m3-src').value !== s.source && !show.touched) {
        $('m3-src').value = s.source; optsFor(s.source);
      }
      recording = !!s.recording;
      $('m3-rec').textContent = recording ? `■ stop (${s.recording.scenes})` : '● record';
      const live = /^Receiving/.test(s.link);
      const room = s.source === 'sim' ? ` · room: ${s.room_source}` +
        (s.room_counts ? ` (${s.room_counts.walls} walls, ${s.room_counts.boxes + s.room_counts.cylinders} objects)` : '') : '';
      const motion = s.motion ? ` · motion ${s.motion.hz} Hz gaps ${s.motion.gaps} (${s.motion.status})` : '';
      const overlays = [s.truth_overlay && 'car truth', s.route_overlay && 'goto route'].filter(Boolean).join(', ');
      $('m3-state').innerHTML = '';
      const line = document.createElement('span');
      line.className = s.error || s.room_error ? 'm3-bad' : live ? 'm3-good' : '';
      line.textContent = `${s.source}: ${s.link}${s.source_state ? ' · ' + s.source_state : ''} · ${s.scenes_per_s} scenes/s` +
        ` · frames ${s.frames} · voxels ${s.voxels} · free ${s.free_cells} · occupied ${s.occupied_cells}` +
        ` · tracking ${s.tracking}${s.floor_ready ? '' : ' (no floor)'}${room}${motion}` +
        (overlays ? ` · overlay: ${overlays}` : '') + (s.error ? ` · ERROR ${s.error}` : '') +
        (s.room_error ? ` · ${s.room_error}` : '');
      $('m3-state').appendChild(line);
    }
    $('m3-src').addEventListener('mousedown', () => { show.touched = true; });
    B.on('map3d', s => { show(s); });
    B.api('phone/status').then(show).catch(e => { $('m3-state').textContent = 'phone plugin not loaded: ' + e.message; });
    setInterval(refreshTargets, 2000);
  },
});

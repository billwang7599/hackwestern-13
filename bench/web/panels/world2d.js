// world2d: top-down view of the SIMULATED room, and a course editor.
// Every edit goes through the world sim's control port (POST /api/world/ctl), which
// re-casts the guard rays, detections, the phone's map feed and the car's collision walls.
'use strict';
(() => {
  const COLORS = {
    'trash can': '#4cc38a', chair: '#c79a5a', person: '#e6b450', door: '#6f7f8c', box: '#a77ad6',
    table: '#8f7a5a', backpack: '#5aa9e6', 'water bottle': '#59c2d6', laptop: '#9aa6b2', cup: '#d68a59', '': '#7d8a96',
  };
  const FAULTS = [
    ['person', 'person steps in front', {dist: 0.45}],
    ['guard_silent', 'guard silent'], ['guard_invalid', 'guard invalid pkts'], ['guard_stale', 'guard stale stamps'],
    ['pose_stale', 'phone feed stops'], ['pose_lost', 'tracking lost'], ['pose_frozen', 'pose frozen'],
    ['reloc', 'reloc jump (once)'], ['false_det', 'false bin det (5 s)', {s: 5}], ['cam_off', 'camera off'],
  ];
  const SLIDERS = [
    ['pose_noise_m', 'pose noise m', 0, 0.1, 0.005], ['pose_noise_deg', 'yaw noise °', 0, 5, 0.1],
    ['pose_latency_ms', 'pose latency ms', 0, 500, 10], ['dropout_per_min', 'dropouts /min', 0, 10, 0.5],
    ['miss_rate', 'YOLO miss', 0, 1, 0.01], ['fp_rate', 'YOLO false +', 0, 1, 0.01],
    ['cam_latency_ms', 'cam latency ms', 0, 500, 10], ['det_jitter_px', 'box jitter px', 0, 20, 0.5],
  ];
  const TOOLS = [['select', 'Select'], ['box', 'Box'], ['wall', 'Wall'], ['person', 'Person'], ['object', 'Object']];

  function css(el, s) { Object.assign(el.style, s); return el; }
  function h(tag, attrs = {}, ...kids) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k === 'style') css(e, v); else if (k.startsWith('on')) e.addEventListener(k.slice(2), v); else e.setAttribute(k, v);
    }
    for (const k of kids) e.append(k);
    return e;
  }

  Bench.register({
    id: 'world2d', title: 'Sim world (2D)', view: '🗺 Room (sim)', order: 5, wide: true, area: 'view',
    help: 'Top-down view of the simulated room: the car, its route, what the camera sees and the go-to target.',
    mount(el, B) {
      // the SIM badge is the shell's header badge (always visible); this adds the car kind to it
      const badge = {set textContent(t) { B.setBadgeDetail(t.replace(/^SIM · /, '')); }};

      let W = null;          // latest world event
      let motorPose = null;  // tars_motor dead-reckoned pose (world frame)
      let gotoEv = [];
      let tool = 'select', objLabel = 'trash can';
      let sel = null;        // {kind: 'obj'|'wall', id}
      let drag = null;
      let ghost = null;      // shape being drawn
      const trail = [];
      let view = {s: 80, ox: 0, oy: 0};
      let dirty = true;
      let liveMem = null;    // goto's memory in room metres (from the gotomem panel), null = not polled
      let memDrag = null;    // {key, x, y} while dragging a memory ring
      B.on('gotomem', e => { liveMem = e; dirty = true; });

      // ---------- layout: the big view (room bar, edit tools, canvas) here; faults, noise, sensors in the Sim tab
      const banner = h('div', {class: 'mono'});   // goto + guard line, shown in the Sim tab (the header has the plain-words version)
      css(banner, {fontSize: '13px', display: 'flex', gap: '10px', alignItems: 'baseline', flexWrap: 'wrap', marginBottom: '6px'});
      const roomSel = h('select', {title: 'Pick a room layout'});
      const saveName = h('input', {placeholder: 'save as…', size: 14});
      // editing lives on the Room page: this button just goes there (or back to Drive)
      const editBtn = h('button', {title: 'Edit the room: add, move and delete boxes, walls, people and objects (the Room page)',
        onclick: () => (B.openPage ? B.openPage(editing ? 'drive' : 'room') : setEdit(!editing))}, '✏ Edit room');
      const legendBtn = h('button', {class: 'ghost', title: 'What the shapes on the map mean', onclick: () => { legend.hidden = !legend.hidden; }}, 'Legend');
      const bar = h('div', {}, h('span', {class: 'dim'}, 'Room'), roomSel,
        h('button', {onclick: () => act('load', {name: roomSel.value}), title: 'Load the picked room (car back to its start, goto restarted)'}, 'Load'),
        carStartBtn(), resetMapBtn(),
        editBtn, h('span', {style: {flex: 1}}), legendBtn);
      css(bar, {display: 'flex', gap: '6px', alignItems: 'center', flexWrap: 'wrap', marginBottom: '6px'});
      const toolBar = h('div');
      css(toolBar, {display: 'none', gap: '4px', alignItems: 'center', flexWrap: 'wrap', marginBottom: '6px',
        padding: '4px 6px', border: '1px dashed var(--accent)', borderRadius: '6px'});
      const toolBtns = {};
      toolBar.append(h('span', {class: 'dim', style: {fontSize: '12px'}}, 'Add:'));
      for (const [id, name] of TOOLS) {
        toolBtns[id] = h('button', {onclick: () => setTool(id)}, name);
        toolBar.append(toolBtns[id]);
      }
      const labelSel = h('select', {onchange: () => { objLabel = labelSel.value; setTool('object'); }, title: 'Which object “Object” places'});
      toolBar.append(labelSel, saveName, h('button', {onclick: save, title: 'Save this layout as a new room'}, 'Save room'));
      const selInfo = h('span', {class: 'mono dim'});
      toolBar.append(h('span', {style: {width: '12px'}}), selInfo,
        h('span', {class: 'dim', style: {fontSize: '11px', width: '100%'}}, 'Select: drag to move, drag a wall end to stretch, [ ] rotate, Del or right-click → Delete removes, ⌘Z undoes it, Esc deselects. Box: drag out. Wall: click-drag.'));
      let editing = false;
      function setEdit(on) {
        editing = on; toolBar.style.display = on ? 'flex' : 'none';
        editBtn.classList.toggle('primary', on);
        editBtn.textContent = on ? '✓ Done editing' : '✏ Edit room';
        if (!on) { setTool('select'); sel = null; }
        dirty = true; window.dispatchEvent(new Event('resize'));
      }
      B.on('page', p => { if ((p === 'room') !== editing) setEdit(p === 'room'); });
      const canvas = h('canvas', {tabindex: 0, title: 'Click here, then hold W A S D to drive'});
      css(canvas, {position: 'absolute', inset: 0, height: '100%', cursor: 'default', outline: 'none'});
      const seeing = h('div', {class: 'mono'});
      css(seeing, {position: 'absolute', bottom: '8px', right: '8px', background: 'rgba(11,16,19,.85)', padding: '4px 8px',
        borderRadius: '6px', fontSize: '12px', pointerEvents: 'none', maxWidth: '60%'});
      const legend = h('div', {class: 'dim', hidden: ''},
        h('div', {}, h('b', {}, 'white car'), ' = where the car really is · ', h('b', {style: {color: '#e6b450'}}, '◇'), ' = where the phone thinks it is (what goto is told) · ',
          h('b', {style: {color: '#a77ad6'}}, '○'), ' = wheel-count estimate (tars_motor)'),
        h('div', {}, h('b', {style: {color: '#5aa9e6'}}, 'blue line'), ' = planned route · faint line = path driven · light-blue cone = camera view · ',
          h('b', {style: {color: '#4cc38a'}}, 'green line'), ' = the camera sees that object · thin rays = the LiDAR obstacle check (red = blocked) · ',
          h('b', {style: {color: '#ffffff'}}, 'pulsing ring'), ' = the go-to target'));
      css(legend, {position: 'absolute', left: '8px', bottom: '28px', right: '8px', background: 'rgba(11,16,19,.92)', padding: '6px 8px',
        borderRadius: '6px', fontSize: '12px', border: '1px solid var(--line)'});
      const wrap = h('div', {}, canvas, seeing, legend);
      css(wrap, {position: 'relative', flex: '1 1 0', minHeight: '200px'});
      el.append(bar, toolBar, wrap);

      // Sim tab: goto/guard line, sensor readout, faults, sensor noise
      const side = h('div');
      css(side, {display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(240px, 1fr))', gap: '10px', marginTop: '6px'});
      const faultBox = h('div'), sliderBox = h('div');
      side.append(faultBox, sliderBox);
      const sensors = h('div', {class: 'mono'});
      css(sensors, {fontSize: '12px', marginTop: '6px', whiteSpace: 'pre-wrap'});
      B.register({id: 'world-tools', title: 'Faults and sensor noise (sim)', short: '⚠ Faults & noise', order: 10, area: 'room',
        help: 'Break things on purpose to see how the robot copes. Faults toggle on click; sliders change the sim sensors live.',
        mount(el2) { el2.append(banner, sensors, side); }});
      // Room page: every object and wall as a table, with delete buttons and undo
      const objTbody = h('tbody'), wallTbody = h('tbody');
      const undoBar = h('div', {class: 'undo-bar', hidden: ''});
      const delSelBtn = h('button', {class: 'danger', disabled: '', title: 'Delete the object or wall selected on the map or in the table (Del)', onclick: () => removeSel()}, '🗑 Delete selected');
      const wallsBox = h('details', {}, h('summary', {class: 'dim', style: {cursor: 'pointer', fontSize: '12px', margin: '8px 0 4px'}}, 'Walls'),
        h('div', {class: 'tbl-scroll', style: {maxHeight: '200px'}}, h('table', {class: 'dt', 'data-k': 'walls'},
          h('thead', {}, h('tr', {}, h('th', {}, 'Wall'), h('th', {}, 'From (x, y)'), h('th', {}, 'To (x, y)'), h('th', {}, 'Length'), h('th', {}, ''))), wallTbody)));
      B.register({id: 'room-objects', title: 'Things in the room', short: '📦 Things', order: 1, area: 'room',
        help: 'Every object in the sim room. Click a row to select it on the map; 🗑 deletes it (Undo brings it back). You can also right-click it on the map → Delete.',
        mount(el2) {
          el2.append(h('div', {class: 'toolbar'}, delSelBtn,
              carStartBtn(), resetMapBtn(),
              h('span', {class: 'dim', style: {fontSize: '12px'}}, 'Add things with the tools above the map.')),
            undoBar,
            h('div', {class: 'tbl-scroll'}, h('table', {class: 'dt nowrap', 'data-k': 'objects'},
              h('thead', {}, h('tr', {}, h('th', {}, 'Name'), h('th', {}, 'Type'), h('th', {title: 'metres, x forward / y left from the room origin'}, 'x, y m'), h('th', {}, ''))), objTbody)),
            wallsBox);
        }});
      let tableKey = '';
      const f2 = v => (+v).toFixed(2);
      function kindOf(o) { return o.kind === 'person' || o.label === 'person' ? 'person' : o.label ? (o.solid === false ? o.label + ' (walk-through)' : o.label) : 'box (obstacle)'; }
      function renderTables() {
        const r = W && W.room;
        delSelBtn.disabled = !sel;
        const key = r ? JSON.stringify([r.objects.map(o => [o.id, o.label, f2(o.x), f2(o.y), o.r, o.w, o.d]), r.walls.map(w => w.map(f2)), sel && sel.id]) : '';
        if (key === tableKey) return;
        tableKey = key;
        if (!r) { objTbody.replaceChildren(h('tr', {class: 'empty-row'}, h('td', {colspan: 4}, 'waiting for the sim room…'))); wallTbody.replaceChildren(); return; }
        objTbody.replaceChildren(...(r.objects.length ? r.objects.map(o => {
          const tr = h('tr', {'data-id': o.id, class: sel && sel.id === o.id ? 'sel' : '', style: {cursor: 'pointer'},
              onclick: () => { sel = {kind: 'obj', id: o.id}; dirty = true; renderTables(); }},
            h('td', {title: o.r !== undefined ? `⌀ ${f2(2 * o.r)} m` : `${f2(o.w)} × ${f2(o.d)} m`}, o.id), h('td', {}, kindOf(o)), h('td', {class: 'num'}, `${f2(o.x)}, ${f2(o.y)}`),
            h('td', {class: 'act'},
              ...(o.label && o.label !== 'person' ? [h('button', {title: `Send GOTO ${o.label}`, onclick: ev => { ev.stopPropagation(); sendGoto(o.label); }}, '➜')] : []),
              ' ', h('button', {class: 'danger', 'data-del': o.id, title: `Delete ${o.id}`, onclick: ev => { ev.stopPropagation(); removeId(o.id); }}, '🗑')));
          return tr;
        }) : [h('tr', {class: 'empty-row'}, h('td', {colspan: 4}, 'the room is empty: add things with the tools above the map'))]));
        wallTbody.replaceChildren(...r.walls.map((w, i) => h('tr', {'data-id': 'w' + i, class: sel && sel.id === 'w' + i ? 'sel' : '', style: {cursor: 'pointer'},
            onclick: () => { sel = {kind: 'wall', id: 'w' + i}; dirty = true; renderTables(); }},
          h('td', {}, 'w' + i), h('td', {class: 'num'}, `${f2(w[0])}, ${f2(w[1])}`), h('td', {class: 'num'}, `${f2(w[2])}, ${f2(w[3])}`),
          h('td', {class: 'num'}, f2(Math.hypot(w[2] - w[0], w[3] - w[1])) + ' m'),
          h('td', {class: 'act'}, h('button', {class: 'danger', 'data-del': 'w' + i, title: `Delete wall w${i}`, onclick: ev => { ev.stopPropagation(); removeId('w' + i); }}, '🗑')))));
        wallsBox.querySelector('summary').textContent = `Walls (${r.walls.length})`;
      }
      // removal with a one-step undo (the removed object/wall is re-added as it was)
      let lastRemoved = null;
      async function removeId(id) {
        const r = W && W.room; if (!r) return;
        let saved = null;
        if (/^w\d+$/.test(id)) { const w = r.walls[+id.slice(1)]; if (w) saved = {kind: 'wall', seg: w.slice(), name: 'wall ' + id}; }
        else { const o = r.objects.find(o => o.id === id); if (o) saved = {kind: 'obj', obj: JSON.parse(JSON.stringify(o)), name: o.id}; }
        const res = await ctl({op: 'remove', id});
        if (sel && sel.id === id) sel = null;
        dirty = true;
        if (!res.ok || !saved) return;
        lastRemoved = saved;
        B.log('world', `deleted ${saved.name}`);
        undoBar.replaceChildren(h('span', {}, `Deleted ${saved.name}.`), h('button', {onclick: undo}, '↶ Undo'),
          h('span', {class: 'dim'}, '(⌘Z)'), h('span', {style: {flex: 1}}), h('button', {class: 'ghost', onclick: () => { undoBar.hidden = true; }}, '✕'));
        undoBar.hidden = false;
      }
      async function undo() {
        const u = lastRemoved; if (!u) return;
        lastRemoved = null; undoBar.hidden = true;
        const res = u.kind === 'wall' ? await ctl({op: 'add_wall', seg: u.seg}) : await ctl({op: 'add', obj: u.obj});
        B.log('world', res.ok ? `restored ${u.name}` : `undo failed: ${res.error}`);
      }
      window.addEventListener('keydown', ev => {
        if (ev.key !== 'z' || !(ev.metaKey || ev.ctrlKey) || ev.shiftKey || !editing || !lastRemoved) return;
        const tag = (ev.target.tagName || '').toLowerCase();
        if (tag === 'input' || tag === 'textarea' || tag === 'select') return;
        ev.preventDefault(); undo();
      });
      // right-click on the map: Delete / GOTO for what's under the pointer
      let menu = null;
      const closeMenu = () => { if (menu) { menu.remove(); menu = null; } };
      window.addEventListener('mousedown', ev => { if (menu && !menu.contains(ev.target)) closeMenu(); });
      window.addEventListener('keydown', ev => { if (ev.key === 'Escape') closeMenu(); });
      canvas.addEventListener('contextmenu', ev => {
        if (!W || !W.room) return;
        ev.preventDefault(); closeMenu();
        const [x, y] = ev2w(ev);
        const o = hitObj(x, y), hw = o ? null : hitWall(x, y);
        const items = [];
        if (o) {
          sel = {kind: 'obj', id: o.id};
          items.push(h('div', {class: 'dim'}, `${o.id} (${kindOf(o)})`),
            h('button', {class: 'danger', 'data-menu': 'delete', onclick: () => { closeMenu(); removeId(o.id); }}, `🗑 Delete ${o.id}`));
          if (o.label && o.label !== 'person') items.push(h('button', {onclick: () => { closeMenu(); sendGoto(o.label); }}, `➜ GOTO ${o.label}`));
        } else if (hw) {
          sel = {kind: 'wall', id: 'w' + hw.i};
          items.push(h('div', {class: 'dim'}, `wall w${hw.i}`),
            h('button', {class: 'danger', 'data-menu': 'delete', onclick: () => { closeMenu(); removeId('w' + hw.i); }}, `🗑 Delete wall w${hw.i}`));
        } else items.push(h('div', {class: 'dim'}, 'nothing here: right-click an object or wall'));
        if (!editing && B.openPage) items.push(h('button', {onclick: () => { closeMenu(); B.openPage('room'); }}, '✏ Edit room…'));
        dirty = true;
        menu = h('div', {class: 'ctxmenu', role: 'menu'}, ...items);
        menu.style.left = Math.min(ev.clientX, innerWidth - 200) + 'px'; menu.style.top = Math.min(ev.clientY, innerHeight - 120) + 'px';
        document.body.appendChild(menu);
      });

      faultBox.append(h('div', {class: 'dim'}, 'Faults (click to toggle; red = on)'));
      const faultBtns = {};
      for (const [name, text, extra] of FAULTS) {
        const b = h('button', {onclick: () => {
          const on = !(W && W.faults && name in W.faults);
          ctl({op: 'fault', name, on: name === 'reloc' ? true : on, ...(extra || {})});
        }}, text);
        css(b, {margin: '2px'});
        faultBtns[name] = b;
        faultBox.append(b);
      }
      faultBox.append(h('button', {class: 'stop', style: {margin: '2px'}, onclick: () => ctl({op: 'fault', name: 'clear'})}, 'clear faults'));
      sliderBox.append(h('div', {class: 'dim'}, 'Sensor noise and errors (live)'));
      const sliders = {};
      for (const [k, text, lo, hi, step] of SLIDERS) {
        const v = h('span', {class: 'mono'});
        const s = h('input', {type: 'range', min: lo, max: hi, step});
        s.addEventListener('input', () => { v.textContent = s.value; });
        s.addEventListener('change', () => ctl({op: 'set', params: {[k]: parseFloat(s.value)}}));
        sliders[k] = [s, v];
        const row = h('label', {}, h('span', {style: {display: 'inline-block', width: '120px'}}, text), s, ' ', v);
        css(row, {display: 'block', fontSize: '12px'});
        sliderBox.append(row);
      }

      // ---------- server calls
      async function ctl(msg) {
        try {
          const r = await B.api('world/ctl', msg);
          if (!r.ok) B.log('world', 'ERR ' + r.error);
          return r;
        } catch (e) { B.log('world', 'world control failed: ' + e.message); return {ok: false}; }
      }
      // two separate resets: the car's position, and the room + goto's map
      // function declarations (hoisted): the room bar and the Things panel call these before this line runs
      function carStartBtn() {
        return h('button', {onclick: () => act('car_to_start', {}),
          title: 'Stop, then put the car back at the room\'s start. Keeps everything else: things and walls you added, ' +
                 'and goto\'s map and memory (goto is not restarted).'}, '🚗 Car to start');
      }
      function resetMapBtn() { return h('button', {class: 'danger', onclick: () => {
          if (confirm('Reset map?\n\nThe room goes back to how it was loaded or saved: walls and things you added are removed. ' +
                      'goto restarts with an empty map and the room\'s memory. The car stays where it is.')) act('reset_map', {});
        },
        title: 'Put the room back as loaded/saved (removes walls and things you added) and clear goto\'s map and memory ' +
               '(goto restarts). The car stays where it is.'}, '🗺 Reset map'); }
      async function act(what, body) {
        try {
          const r = await B.api('world/' + what, body);
          B.log('world', `${what}: ${r.ok ? 'ok' : r.error}${r.note ? ' (' + r.note + ')' : ''}`);
          trail.length = 0; sel = null; refreshRooms();
        } catch (e) { B.log('world', `${what} failed: ${e.message}`); }
      }
      async function save() {
        const name = saveName.value.trim();
        if (!name) { B.log('world', 'type a name to save as'); return; }
        let r = await B.api('world/save', {name}).catch(e => ({ok: false, error: e.message, exists: /exists/.test(e.message)}));
        if (!r.ok && r.exists && confirm(`Room "${name}" exists. Overwrite?`)) r = await B.api('world/save', {name, overwrite: true}).catch(e => ({ok: false, error: e.message}));
        B.log('world', r.ok ? `saved ${r.path}` : 'save: ' + r.error);
        refreshRooms();
      }
      async function refreshRooms() {
        try {
          const r = await B.api('world/rooms');
          const cur = W && W.room && W.room.name;
          roomSel.replaceChildren(...r.rooms.map(n => h('option', {value: n}, n)));
          if (cur && r.rooms.includes(cur)) roomSel.value = cur;
        } catch (e) { /* bench without world plugin */ }
      }
      refreshRooms();

      function setTool(t) {
        tool = t;
        for (const [id, b] of Object.entries(toolBtns)) b.style.borderColor = id === t ? 'var(--accent)' : '';
        canvas.style.cursor = t === 'select' ? 'default' : 'crosshair';
      }
      setTool('select');

      // ---------- view transform (world y = left = screen up)
      function fit() {
        const r = W && W.room;
        const dpr = window.devicePixelRatio || 1;
        const cw = canvas.clientWidth, ch = canvas.clientHeight;
        if (canvas.width !== cw * dpr || canvas.height !== ch * dpr) { canvas.width = cw * dpr; canvas.height = ch * dpr; }
        if (!r) return;
        let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
        for (const w of r.walls) { x0 = Math.min(x0, w[0], w[2]); x1 = Math.max(x1, w[0], w[2]); y0 = Math.min(y0, w[1], w[3]); y1 = Math.max(y1, w[1], w[3]); }
        if (x0 > x1) { x0 = -2; x1 = 2; y0 = -2; y1 = 2; }
        const pad = 0.4;
        const s = Math.min(cw / (x1 - x0 + 2 * pad), ch / (y1 - y0 + 2 * pad));
        view = {s, ox: (cw - s * (x1 + x0)) / 2, oy: (ch + s * (y1 + y0)) / 2};
      }
      const S = (x, y) => [view.ox + x * view.s, view.oy - y * view.s];
      const Wd = (px, py) => [(px - view.ox) / view.s, (view.oy - py) / view.s];

      // ---------- data
      B.on('world', d => {
        W = d;
        if (d.truth) {
          const last = trail[trail.length - 1];
          if (!last || Math.hypot(last[0] - d.truth.x, last[1] - d.truth.y) > 0.01) trail.push([d.truth.x, d.truth.y]);
          if (trail.length > 1500) trail.shift();
        }
        if (d.room && labelSel.options.length === 0) {
          labelSel.replaceChildren(...(d.room.labels || []).map(n => h('option', {value: n}, n)));
          labelSel.value = objLabel;
        }
        if (d.room && d.room.params) for (const [k, [s, v]] of Object.entries(sliders)) {
          if (document.activeElement !== s && k in d.room.params) { s.value = d.room.params[k]; v.textContent = d.room.params[k]; }
        }
        if (d.room && roomSel.value !== d.room.name && document.activeElement !== roomSel) {
          if ([...roomSel.options].some(o => o.value === d.room.name)) roomSel.value = d.room.name;
        }
        for (const [name, b] of Object.entries(faultBtns)) {
          const on = d.faults && name in d.faults;
          b.style.background = on ? '#5a1f1f' : ''; b.style.borderColor = on ? 'var(--bad)' : '';
        }
        dirty = true;
      });
      B.on('status', d => {
        const f = d && d.fields;
        let p = f && f.pose;
        if (typeof p === 'string') p = p.split(',').map(Number);
        if (Array.isArray(p) && p.length >= 3 && W && W.room) {
          const st = W.room.start, f0 = p[0] / 1000, l0 = -p[1] / 1000, th = -p[2] * Math.PI / 180;
          const c = Math.cos(st.th), s = Math.sin(st.th);
          motorPose = [st.x + c * f0 - s * l0, st.y + s * f0 + c * l0, st.th + th];
        } else motorPose = null;
        dirty = true;
      });
      B.on('goto', d => { gotoEv = (d && d.events) || []; dirty = true; });

      // ---------- banner + sensor text
      function updateText() {
        if (!W) { banner.textContent = 'waiting for the world sim… (start it with bench/run_sim.py)'; seeing.textContent = 'waiting for the sim room…'; return; }
        const dets = (W.dets || []);
        seeing.replaceChildren(h('span', {class: 'dim'}, 'Camera sees: '),
          dets.length ? h('span', {class: dets.some(d => d.fp) ? 'warn' : 'ok'}, dets.map(d => `${d.fp ? 'FALSE ' : ''}${d.label} ${d.conf ?? ''}`).join(', '))
                      : h('span', {class: 'dim'}, 'nothing right now'));
        const g = W.goto;
        const parts = [];
        const car = W.car_kind === 'virtual' ? 'virtual SM11' : (W.truth && W.truth.stub ? 'STUB CAR (no physics)' : (W.car_kind || 'no car'));
        badge.textContent = `SIM · ${car}`;
        if (!W.alive) parts.push(h('span', {class: 'bad'}, 'world sim not responding'));
        if (g && g.state && g.state !== 'idle') {
          parts.push(h('span', {}, `GOTO ${g.target || '?'}`), h('span', {class: 'warn'}, `→ ${String(g.state).toUpperCase()}`));
          if (g.path && g.path !== '0/0') parts.push(h('span', {class: 'dim'}, `leg ${g.path}`));
          if (g.replans && g.replans !== '0' && g.replans !== 0) parts.push(h('span', {class: 'dim'}, `replans ${g.replans}`));
        } else if (g) {
          const res = g.result && g.result !== '-' ? g.result : 'idle';
          parts.push(h('span', {class: /arrived/.test(res) ? 'ok' : (/fail|cancel/.test(res) ? 'bad' : 'dim')},
            g.target && g.target !== '-' ? `GOTO ${g.target}: ${res}` : 'goto idle'));
        } else parts.push(h('span', {class: 'dim'}, W.robot ? 'goto: no status yet' : 'robot plugin not loaded: view only'));
        const ev = gotoEv.length ? gotoEv[gotoEv.length - 1] : null;
        if (ev) parts.push(h('span', {class: 'dim', style: {fontSize: '13px'}}, 'last: ' + (typeof ev === 'string' ? ev : (ev.text || JSON.stringify(ev)))));
        const gv = W.guard || {};
        parts.push(h('span', {class: gv.permit ? 'ok' : 'bad', style: {marginLeft: 'auto'}}, `guard ${gv.verdict || '?'}`));
        banner.replaceChildren(...parts);
        const t = W.truth, p = W.phone;
        const fl = Object.keys(W.faults || {});
        sensors.textContent =
          `room ${W.room ? W.room.name : '?'}  ·  car ${t ? `${t.x.toFixed(2)}, ${t.y.toFixed(2)} m  ${(t.th * 180 / Math.PI).toFixed(0)}°  v ${(+t.v || 0).toFixed(2)} m/s${t.contact ? '  CONTACT' : ''}` : 'no truth'}` +
          `  ·  phone ${p ? (p.tracking ? 'tracking' : 'LIMITED') + ` ${p.points} pts` : 'silent'}` +
          `  ·  guard ${gv.verdict || '?'} (${gv.sent || '-'}${gv.nearest ? `, nearest ${gv.nearest} m` : ''})` +
          `  ·  YOLO ${(W.dets || []).map(d => d.label + (d.fp ? '?' : '') + ' ' + (d.conf || '')).join(', ') || 'nothing'}` +
          (fl.length ? `  ·  FAULTS ${fl.join(', ')}` : '');
        if (sel) {
          const o = sel.kind === 'obj' && W.room && W.room.objects.find(o => o.id === sel.id);
          selInfo.replaceChildren(sel.kind === 'wall' ? `wall ${sel.id}` : (o ? `${o.id} (${o.label || 'obstacle'})` : ''), ' ',
            ...(o && o.label ? [h('button', {onclick: () => sendGoto(o.label)}, `GOTO ${o.label}`)] : []),
            ...(o ? [h('button', {onclick: () => rotate(-15)}, '⟲'), h('button', {onclick: () => rotate(15)}, '⟳')] : []),
            h('button', {onclick: removeSel}, 'Delete'));
        } else selInfo.textContent = tool === 'object' ? `click to place: ${objLabel}` : '';
      }

      async function sendGoto(label) {
        try {
          const r = await B.api('world/goto', {label});
          B.log('world', `GOTO ${label}: ${r.reply || r.error}`);
        } catch (e) { B.log('world', `GOTO ${label} failed: ${e.message}`); }
      }

      // ---------- drawing
      function draw() {
        fit();
        const ctx = canvas.getContext('2d');
        const dpr = window.devicePixelRatio || 1;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
        if (!W || !W.room) return;
        const r = W.room, s = view.s;
        // grid every 0.5 m
        ctx.strokeStyle = '#16212a'; ctx.lineWidth = 1;
        for (let gx = Math.floor(Wd(0, 0)[0] * 2) / 2; gx < Wd(canvas.clientWidth, 0)[0]; gx += 0.5) {
          const [px] = S(gx, 0); ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, canvas.clientHeight); ctx.stroke();
        }
        for (let gy = Math.floor(Wd(0, canvas.clientHeight)[1] * 2) / 2; gy < Wd(0, 0)[1]; gy += 0.5) {
          const [, py] = S(0, gy); ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(canvas.clientWidth, py); ctx.stroke();
        }
        // layers other panels add (window.World2DLayers: fn(ctx, S, scale, W)), e.g. goto's map (maps panel)
        for (const fn of window.World2DLayers || []) { try { fn(ctx, S, s, W); } catch (e) { /* a layer never breaks the view */ } }
        // camera cone (truth)
        const t = W.truth;
        if (t) {
          const hf = (W.hfov_deg || 60) * Math.PI / 360, L = 3;
          ctx.fillStyle = 'rgba(90,169,230,.08)';
          ctx.beginPath(); ctx.moveTo(...S(t.x, t.y));
          ctx.lineTo(...S(t.x + L * Math.cos(t.th + hf), t.y + L * Math.sin(t.th + hf)));
          ctx.lineTo(...S(t.x + L * Math.cos(t.th - hf), t.y + L * Math.sin(t.th - hf)));
          ctx.closePath(); ctx.fill();
        }
        // guard rays + stop corridor
        const g = W.guard || {};
        if (g.rays && g.pose) {
          const [gx, gy, gth] = g.pose;
          const col = g.verdict === 'BLOCKED' ? 'rgba(239,95,95,.55)' : g.verdict === 'CLEAR' ? 'rgba(76,195,138,.25)' : 'rgba(230,180,80,.35)';
          ctx.strokeStyle = col; ctx.lineWidth = 1;
          for (const [a, d] of g.rays) {
            ctx.beginPath(); ctx.moveTo(...S(gx, gy)); ctx.lineTo(...S(gx + d * Math.cos(gth + a), gy + d * Math.sin(gth + a))); ctx.stroke();
          }
          const hw = (r.params && r.params.guard_half_w) || 0.45, sm = (r.params && r.params.guard_stop_m) || 0.6;
          const c = Math.cos(gth), sn = Math.sin(gth);
          const pts = [[0, -hw], [sm, -hw], [sm, hw], [0, hw]].map(([f, l]) => S(gx + c * f - sn * l, gy + sn * f + c * l));
          ctx.strokeStyle = g.verdict === 'BLOCKED' ? '#ef5f5f' : 'rgba(138,154,166,.5)'; ctx.setLineDash([4, 4]);
          ctx.beginPath(); ctx.moveTo(...pts[0]); for (const p of pts.slice(1)) ctx.lineTo(...p); ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
        }
        // walls
        r.walls.forEach((w, i) => {
          const isSel = sel && sel.kind === 'wall' && sel.id === 'w' + i;
          ctx.strokeStyle = isSel ? '#5aa9e6' : '#c8d3db'; ctx.lineWidth = isSel ? 5 : 4; ctx.lineCap = 'round';
          ctx.beginPath(); ctx.moveTo(...S(w[0], w[1])); ctx.lineTo(...S(w[2], w[3])); ctx.stroke();
        });
        // goto's live memory (Memory edit panel, GET /api/world/memory): a dashed ring per entry, draggable there
        const MEMCOL = {fresh: '#4cc38a', stale: '#e6b450', gone: '#7d8a96'};
        for (const m of liveMem || []) {
          const [px, py] = memDrag && memDrag.key === m.key ? S(memDrag.x, memDrag.y) : S(m.x, m.y);
          const col = MEMCOL[m.state] || MEMCOL.fresh;
          ctx.strokeStyle = col; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.arc(px, py, 0.16 * s, 0, 7); ctx.stroke(); ctx.setLineDash([]);
          label(ctx, px, py + 0.16 * s + 12, `remembers ${m.key}${m.state !== 'fresh' ? ' (' + m.state + ')' : ''}`, col, 11);
        }
        // remembered-elsewhere ghosts (room memory with its own x, y: the moved-bin demo), until goto's live memory arrives
        for (const m of liveMem ? [] : r.memory || []) {
          if (m.x === undefined) continue;
          const [px, py] = S(m.x, m.y);
          ctx.strokeStyle = '#4cc38a'; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.arc(px, py, 0.16 * s, 0, 7); ctx.stroke(); ctx.setLineDash([]);
          label(ctx, px, py - 0.16 * s - 4, `goto remembers ${m.id} here`, '#4cc38a', 11);
        }
        // objects
        for (const o of r.objects) {
          const isSel = sel && sel.kind === 'obj' && sel.id === o.id;
          const col = COLORS[o.label] || COLORS[''];
          ctx.fillStyle = col + (o.solid === false ? '22' : '66'); ctx.strokeStyle = isSel ? '#ffffff' : col; ctx.lineWidth = isSel ? 2.5 : 1.5;
          if (o.solid === false) ctx.setLineDash([4, 3]);
          ctx.beginPath();
          if (o.r !== undefined) { const [px, py] = S(o.x, o.y); ctx.arc(px, py, o.r * s, 0, 7); }
          else corners(o).forEach((p, k) => (k ? ctx.lineTo(...S(...p)) : ctx.moveTo(...S(...p))));
          ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.setLineDash([]);
          const [px, py] = S(o.x, o.y);
          label(ctx, px, py - (o.r !== undefined ? o.r : Math.max(o.w, o.d) / 2) * s - 5, o.label || o.id, '#dbe4ea', 12);
        }
        // the go-to target: a pulsing ring on every object with that label
        const gt = W.goto && W.goto.state && W.goto.state !== 'idle' && W.goto.target ? String(W.goto.target).replace(/_/g, ' ') : '';
        if (gt) {
          const pulse = 0.5 + 0.5 * Math.sin(performance.now() / 250);
          for (const o of r.objects) {
            if ((o.label || '') !== gt) continue;
            const [px, py] = S(o.x, o.y), rr = (o.r !== undefined ? o.r : Math.max(o.w, o.d) / 2) * s + 8 + 4 * pulse;
            ctx.strokeStyle = `rgba(255,255,255,${0.5 + 0.5 * pulse})`; ctx.lineWidth = 2.5;
            ctx.beginPath(); ctx.arc(px, py, rr, 0, 7); ctx.stroke();
            label(ctx, px, py + rr + 14, 'go-to target', '#ffffff', 11);
          }
        }
        // route
        if (W.route && W.route.length) {
          ctx.strokeStyle = '#5aa9e6'; ctx.lineWidth = 2.5;
          ctx.beginPath();
          if (t) ctx.moveTo(...S(t.x, t.y));
          W.route.forEach((p, k) => (k || t ? ctx.lineTo(...S(p[0], p[1])) : ctx.moveTo(...S(p[0], p[1]))));
          ctx.stroke();
          W.route.forEach((p, k) => {
            const [px, py] = S(p[0], p[1]);
            ctx.fillStyle = k === W.route_i ? '#ffffff' : '#5aa9e6'; ctx.beginPath(); ctx.arc(px, py, k === W.route_i ? 5 : 3, 0, 7); ctx.fill();
          });
        }
        // trail
        if (trail.length > 1) {
          ctx.strokeStyle = 'rgba(219,228,234,.35)'; ctx.lineWidth = 1.5; ctx.beginPath();
          trail.forEach((p, k) => (k ? ctx.lineTo(...S(...p)) : ctx.moveTo(...S(...p)))); ctx.stroke();
        }
        // detections: lines from the camera to what YOLO reports
        if (t) for (const d of W.dets || []) {
          ctx.strokeStyle = d.fp ? '#ef5f5f' : '#4cc38a'; ctx.lineWidth = 1; ctx.setLineDash(d.fp ? [2, 3] : []);
          const tx = d.fp ? t.x + 2 * Math.cos(t.th + d.bearing) : d.x, ty = d.fp ? t.y + 2 * Math.sin(t.th + d.bearing) : d.y;
          ctx.beginPath(); ctx.moveTo(...S(t.x, t.y)); ctx.lineTo(...S(tx, ty)); ctx.stroke(); ctx.setLineDash([]);
          const [px, py] = S(tx, ty);
          label(ctx, px + 8, py + 14, `${d.fp ? 'FALSE ' : ''}${d.label} ${d.conf}`, d.fp ? '#ef5f5f' : '#4cc38a', 11);
        }
        // poses: tars_motor dead reckoning (circle), phone (diamond outline), truth car (body + heading)
        if (motorPose) {
          const [px, py] = S(motorPose[0], motorPose[1]);
          ctx.strokeStyle = '#a77ad6'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(px, py, 0.12 * s, 0, 7); ctx.stroke();
          ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(...S(motorPose[0] + 0.2 * Math.cos(motorPose[2]), motorPose[1] + 0.2 * Math.sin(motorPose[2]))); ctx.stroke();
        }
        if (W.phone) {
          const p = W.phone;
          ctx.strokeStyle = p.tracking ? '#e6b450' : '#ef5f5f'; ctx.lineWidth = 1.5;
          car(ctx, p.x, p.y, p.th, false);
        }
        if (t) { ctx.fillStyle = t.contact ? '#ef5f5f' : '#dbe4ea'; ctx.strokeStyle = '#0f1418'; car(ctx, t.x, t.y, t.th, true); }
        // shape being drawn
        if (ghost) {
          ctx.strokeStyle = '#5aa9e6'; ctx.setLineDash([5, 4]); ctx.lineWidth = 2; ctx.beginPath();
          if (ghost.kind === 'wall') { ctx.moveTo(...S(ghost.a[0], ghost.a[1])); ctx.lineTo(...S(ghost.b[0], ghost.b[1])); }
          else { const [ax, ay] = S(ghost.a[0], ghost.a[1]), [bx, by] = S(ghost.b[0], ghost.b[1]); ctx.rect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay)); }
          ctx.stroke(); ctx.setLineDash([]);
        }
        // scale bar
        ctx.fillStyle = '#8a9aa6'; ctx.fillRect(12, canvas.clientHeight - 14, s, 3);
        label(ctx, 12 + s / 2, canvas.clientHeight - 20, '1 m', '#8a9aa6', 11);
      }
      function corners(o) {
        const c = Math.cos(o.yaw || 0), sn = Math.sin(o.yaw || 0), hx = o.w / 2, hy = o.d / 2;
        return [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]].map(([ax, ay]) => [o.x + c * ax - sn * ay, o.y + sn * ax + c * ay]);
      }
      function car(ctx, x, y, th, filled) {
        const c = Math.cos(th), sn = Math.sin(th);
        const P = (f, l) => S(x + c * f - sn * l, y + sn * f + c * l);
        ctx.beginPath();          // SM11-ish body 0.26 x 0.16 m with a pointed nose
        ctx.moveTo(...P(0.17, 0)); ctx.lineTo(...P(0.09, 0.08)); ctx.lineTo(...P(-0.13, 0.08)); ctx.lineTo(...P(-0.13, -0.08)); ctx.lineTo(...P(0.09, -0.08));
        ctx.closePath();
        if (filled) { ctx.lineWidth = 2; ctx.fill(); ctx.stroke(); ctx.strokeStyle = '#ef5f5f'; ctx.beginPath(); ctx.moveTo(...P(0, 0)); ctx.lineTo(...P(0.3, 0)); ctx.stroke(); }
        else ctx.stroke();
      }
      function label(ctx, x, y, text, col, size) {
        ctx.font = `600 ${size}px system-ui, sans-serif`; ctx.textAlign = 'center';
        ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(11,16,19,.85)'; ctx.strokeText(text, x, y);
        ctx.fillStyle = col; ctx.fillText(text, x, y);
      }

      // ---------- editing
      function hitObj(wx, wy) {
        const objs = (W && W.room && W.room.objects) || [];
        for (let i = objs.length - 1; i >= 0; i--) {
          const o = objs[i];
          const dx = wx - o.x, dy = wy - o.y, c = Math.cos(-(o.yaw || 0)), sn = Math.sin(-(o.yaw || 0));
          const lx = c * dx - sn * dy, ly = sn * dx + c * dy, m = 6 / view.s;
          if (o.r !== undefined ? Math.hypot(dx, dy) < o.r + m : Math.abs(lx) < o.w / 2 + m && Math.abs(ly) < o.d / 2 + m) return o;
        }
        return null;
      }
      function hitWall(wx, wy) {
        const walls = (W && W.room && W.room.walls) || [];
        let best = null, bd = 8 / view.s;
        walls.forEach((w, i) => {
          const ex = w[2] - w[0], ey = w[3] - w[1], L = ex * ex + ey * ey;
          const u = L ? Math.max(0, Math.min(1, ((wx - w[0]) * ex + (wy - w[1]) * ey) / L)) : 0;
          const d = Math.hypot(wx - w[0] - u * ex, wy - w[1] - u * ey);
          if (d < bd) { bd = d; best = {i, end: u < 0.08 && Math.hypot(ex, ey) * view.s > 30 ? 0 : (u > 0.92 && Math.hypot(ex, ey) * view.s > 30 ? 1 : -1)}; }
        });
        return best;
      }
      const ev2w = ev => { const b = canvas.getBoundingClientRect(); return Wd(ev.clientX - b.left, ev.clientY - b.top); };
      let lastSend = 0;
      canvas.addEventListener('mousedown', ev => {
        canvas.focus();
        if (ev.button !== 0) return;   // right-click = the Delete/GOTO menu
        const [x, y] = ev2w(ev);
        if (!W || !W.room) return;
        if (window.World2DPick) { const f = window.World2DPick; window.World2DPick = null; f(x, y); return; }  // one click for another panel
        if (!editing) return;   // view mode: clicking only focuses the map (for WASD)
        if (tool === 'select' && liveMem && window.GotoMemEditable && window.GotoMemEditable()) {
          const m = liveMem.find(m => Math.hypot(m.x - x, m.y - y) < 0.2);
          if (m) { memDrag = {key: m.key, x: m.x, y: m.y, dx: m.x - x, dy: m.y - y}; dirty = true; return; }
        }
        if (tool === 'select') {
          const o = hitObj(x, y);
          if (o) { sel = {kind: 'obj', id: o.id}; drag = {kind: 'obj', id: o.id, dx: o.x - x, dy: o.y - y}; }
          else {
            const hw = hitWall(x, y);
            if (hw) { sel = {kind: 'wall', id: 'w' + hw.i}; drag = {kind: 'wall', i: hw.i, end: hw.end, x0: x, y0: y, seg: W.room.walls[hw.i].slice()}; }
            else sel = null;
          }
        } else if (tool === 'box' || tool === 'wall') ghost = {kind: tool, a: [x, y], b: [x, y]};
        else if (tool === 'person') ctl({op: 'add', obj: {kind: 'person', label: 'person', x, y}});
        else if (tool === 'object') ctl({op: 'add', obj: {label: objLabel, x, y, ...defaultSize(objLabel)}});
        dirty = true;
      });
      window.addEventListener('mousemove', ev => {
        if (memDrag) { const [x, y] = ev2w(ev); memDrag.x = x + memDrag.dx; memDrag.y = y + memDrag.dy; memDrag.moved = true; dirty = true; return; }
        if (!drag && !ghost) return;
        const [x, y] = ev2w(ev);
        if (ghost) { ghost.b = [x, y]; dirty = true; return; }
        if (drag.kind === 'obj') {
          const o = W.room.objects.find(o => o.id === drag.id);
          if (!o) return;
          o.x = x + drag.dx; o.y = y + drag.dy;        // local preview; the world sim gets it at ~10 Hz
          if (performance.now() - lastSend > 100) { lastSend = performance.now(); ctl({op: 'update', id: o.id, x: o.x, y: o.y}); }
        } else {
          const s = drag.seg.slice(), dx = x - drag.x0, dy = y - drag.y0;
          if (drag.end !== 1) { s[0] += dx; s[1] += dy; }
          if (drag.end !== 0) { s[2] += dx; s[3] += dy; }
          W.room.walls[drag.i] = s;
          if (performance.now() - lastSend > 100) { lastSend = performance.now(); ctl({op: 'update', id: 'w' + drag.i, seg: s}); }
        }
        drag.moved = true; dirty = true;
      });
      window.addEventListener('mouseup', () => {
        if (memDrag) {
          const m = memDrag; memDrag = null; dirty = true;
          if (m.moved && window.GotoMemMoved) window.GotoMemMoved(m.key, Math.round(m.x * 1000) / 1000, Math.round(m.y * 1000) / 1000);
          return;
        }
        if (ghost) {
          const g = ghost; ghost = null;
          const len = Math.hypot(g.b[0] - g.a[0], g.b[1] - g.a[1]);
          if (g.kind === 'wall' && len > 0.05) ctl({op: 'add_wall', seg: [...g.a, ...g.b]});
          if (g.kind === 'box') {
            const w = Math.abs(g.b[0] - g.a[0]), d = Math.abs(g.b[1] - g.a[1]);
            if (w > 0.04 && d > 0.04) ctl({op: 'add', obj: {label: '', x: (g.a[0] + g.b[0]) / 2, y: (g.a[1] + g.b[1]) / 2, w, d, h: 0.5}});
            else ctl({op: 'add', obj: {label: '', x: g.a[0], y: g.a[1], w: 0.4, d: 0.4, h: 0.5}});
          }
        } else if (drag && drag.moved) {
          if (drag.kind === 'obj') { const o = W.room.objects.find(o => o.id === drag.id); if (o) ctl({op: 'update', id: o.id, x: o.x, y: o.y}); }
          else ctl({op: 'update', id: 'w' + drag.i, seg: W.room.walls[drag.i]});
        }
        drag = null; dirty = true;
      });
      canvas.addEventListener('keydown', ev => {
        if (!editing) return;
        if (ev.key === 'Delete' || ev.key === 'Backspace') { ev.preventDefault(); removeSel(); }
        else if (ev.key === '[') rotate(-15);
        else if (ev.key === ']') rotate(15);
        else if (ev.key === 'Escape') { sel = null; ghost = null; setTool('select'); dirty = true; }
      });
      function rotate(deg) {
        if (!sel || sel.kind !== 'obj') return;
        const o = W.room.objects.find(o => o.id === sel.id);
        if (o) ctl({op: 'update', id: o.id, yaw: (o.yaw || 0) + deg * Math.PI / 180});
      }
      function removeSel() { if (sel) removeId(sel.id); }
      function defaultSize(label) {
        return ({'trash can': {r: 0.15, h: 0.4}, chair: {w: 0.45, d: 0.45, h: 0.85}, person: {r: 0.2, h: 1.7},
          backpack: {w: 0.32, d: 0.22, h: 0.45}, 'water bottle': {r: 0.04, h: 0.25}, table: {w: 1.2, d: 0.7, h: 0.75},
          laptop: {w: 0.32, d: 0.22, h: 0.02}, cup: {r: 0.05, h: 0.1}, box: {w: 0.4, d: 0.3, h: 0.3},
          door: {w: 0.04, d: 0.8, h: 2.0, solid: false}})[label] || {w: 0.3, d: 0.3, h: 0.5};
      }

      // ---------- frame loop
      let lastText = 0;
      function frame(ts) {
        if (dirty || (W && W.goto && W.goto.state && W.goto.state !== 'idle')) { draw(); dirty = false; }
        if (ts - lastText > 200) { updateText(); renderTables(); lastText = ts; }
        requestAnimationFrame(frame);
      }
      window.addEventListener('resize', () => { dirty = true; });
      window.World2DRedraw = () => { dirty = true; };
      requestAnimationFrame(frame);
    },
  });
})();

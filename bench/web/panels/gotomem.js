// gotomem: edit goto's object memory to set up test cases (SIM). Room page, sub-tab "🧠 Memory edit".
// Live table of goto MEMORY in room metres (GET /api/world/memory, ~1 s), Forget / Forget all (goto FORGET),
// add an entry by clicking the map, "remember real position" per room object, edit x/y/state, drag the
// ghost on the map (world2d draws the entries), and copy the live memory into the room's "memory".
// Add / edit / set: the plugin sends S + CANCEL first, then restarts goto with the edited memory.
'use strict';
(() => {
  const STATES = ['fresh', 'stale', 'gone'];
  function h(tag, attrs = {}, ...kids) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k.startsWith('on')) e.addEventListener(k.slice(2), v); else if (k === 'value') e.value = v; else e.setAttribute(k, v);
    }
    for (const k of kids) e.append(k);
    return e;
  }

  Bench.register({
    id: 'gotomem', title: 'Edit what goto remembers (SIM)', short: '🧠 Memory edit', order: 15, wide: true, area: 'room',
    help: 'Set up a test through memory: make goto remember a thing somewhere (behind a wall, where nothing is, ' +
      'or stale/gone), then say “tars go to the trash can”. Add/edit restarts goto (S + CANCEL first; its map is ' +
      'cleared, the car stays put). Forget does not restart. Drag a dashed ring on the map to move an entry.',
    mount(el, B) {
      let entries = [], room = null, busy = false, editing = null;   // editing = key of a row whose inputs have focus
      const say = (t, cls = '') => { msg.textContent = t; msg.className = 'mono ' + cls; };
      const msg = h('div', {class: 'mono'});
      const label = h('select', {title: 'What goto should remember'});
      const state = h('select', {title: 'fresh = seen recently; stale = old sighting (goto checks it but trusts it less); gone = looked there, not there'});
      for (const s of STATES) state.append(h('option', {value: s}, s));
      const addBtn = h('button', {class: 'primary', title: 'Then click the map where goto should think it is',
        onclick: () => {
          if (!label.value) return say('pick a label first', 'bad');
          window.World2DPick = (x, y) => edit({op: 'add', label: label.value, x, y, state: state.value}, `add ${label.value}`);
          say(`click the map where goto should remember the ${label.value}… (the next map click is used)`, 'warn');
        }}, '＋ Add on map');
      const forgetAll = h('button', {title: 'goto FORGET (everything); no restart',
        onclick: () => { if (confirm('Forget everything goto remembers?')) edit({op: 'forget'}, 'forget all'); }}, 'Forget all');
      const toRoom = h('button', {title: 'The live memory becomes this room’s memory: Reset map / reload starts with it, and Save room keeps it',
        onclick: () => edit({op: 'to_room'}, 'memory → room')}, 'Use as room memory');
      const toFile = h('button', {title: 'Use as room memory AND overwrite the room file under its current name',
        onclick: () => { if (confirm(`Overwrite room file ${(room && room.name) || ''}.json with this memory?`)) edit({op: 'to_room', write: true}, 'memory → room file'); }},
        '💾 Save memory to room');
      const tbody = h('tbody');
      const table = h('table', {class: 'dt nowrap'},
        h('thead', {}, h('tr', {}, ...['thing', 'state', 'x m', 'y m', 'age s', 'conf', ''].map(t => h('th', {}, t)))), tbody);
      const objBody = h('tbody');
      const objTable = h('table', {class: 'dt nowrap'}, h('thead', {}, h('tr', {}, h('th', {}, 'room object'), h('th', {}, 'x, y'), h('th', {}, ''))), objBody);

      el.append(
        h('div', {style: 'display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-bottom:6px'},
          h('span', {class: 'dim'}, 'goto remembers a'), label, h('span', {class: 'dim'}, 'state'), state, addBtn,
          h('span', {style: 'flex:1'}), forgetAll, toRoom, toFile),
        msg,
        h('div', {class: 'tbl-scroll', style: 'margin-top:6px'}, table),
        h('div', {class: 'dim', style: 'margin:10px 0 4px'}, 'Remember where a room object really is:'),
        h('div', {class: 'tbl-scroll'}, objTable));

      async function edit(body, what) {
        if (busy) return say('still working on the last edit…', 'warn');
        busy = true; say(`${what}…`);
        try {
          const r = await B.api('world/memory', body);
          if (r.entries) { entries = r.entries; render(); B.emit('gotomem', entries); }
          say(r.ok ? `${what}: OK${r.goto_restart ? ' (S + CANCEL, goto restarted)' : ''}${r.reply ? ' — ' + r.reply : ''}`
            : `${what}: ${r.error || JSON.stringify(r)}`, r.ok ? 'ok' : 'bad');
          B.log('memory', `${what} -> ${r.ok ? 'OK' : (r.error || 'failed')}`);
        } catch (e) { say(`${what}: ${e}`, 'bad'); }
        busy = false;
      }
      window.GotoMemMoved = (key, x, y) => edit({op: 'update', key, x, y}, `move ${key}`);
      window.GotoMemEditable = () => el.offsetParent !== null;   // world2d drags ghosts only while this is shown

      function num(v, d = 2) { return v == null ? '–' : Number(v).toFixed(d); }
      function render() {
        if (editing) return;   // don't wipe an input the user is typing in
        tbody.replaceChildren();
        if (!entries.length) tbody.append(h('tr', {class: 'empty-row'}, h('td', {colspan: 7}, 'goto remembers nothing')));
        for (const e of entries) {
          const st = h('select', {}, ...STATES.map(s => h('option', {value: s}, s)));
          st.value = e.state;
          st.onchange = () => edit({op: 'update', key: e.key, state: st.value}, `${e.key} → ${st.value}`);
          const xi = h('input', {size: 5, value: num(e.x)}), yi = h('input', {size: 5, value: num(e.y)});
          for (const i of [xi, yi]) {
            i.onfocus = () => { editing = e.key; };
            i.onblur = () => setTimeout(() => { if (document.activeElement !== xi && document.activeElement !== yi) editing = null; }, 0);
            i.onkeydown = ev => {
              if (ev.key === 'Enter') {
                const x = parseFloat(xi.value), y = parseFloat(yi.value);
                editing = null; i.blur();
                if (isFinite(x) && isFinite(y)) edit({op: 'update', key: e.key, x, y}, `move ${e.key}`);
              } else if (ev.key === 'Escape') { editing = null; i.blur(); render(); }
            };
          }
          tbody.append(h('tr', {title: `frame: ${e.frame} (map = goto's ARKit position; car = from its memory-frame pose)`},
            h('td', {}, e.key), h('td', {}, st), h('td', {class: 'num'}, xi), h('td', {class: 'num'}, yi),
            h('td', {class: 'num'}, num(e.age, 0)), h('td', {class: 'num'}, num(e.conf)),
            h('td', {class: 'act'}, h('button', {title: `goto FORGET ${e.key}`, onclick: () => edit({op: 'forget', key: e.key}, `forget ${e.key}`)}, 'Forget'))));
        }
      }
      function renderObjects() {
        objBody.replaceChildren();
        const objs = ((room && room.objects) || []).filter(o => o.label);
        if (!objs.length) objBody.append(h('tr', {class: 'empty-row'}, h('td', {colspan: 3}, 'no labelled objects')));
        for (const o of objs) objBody.append(h('tr', {},
          h('td', {}, `${o.label} (${o.id})`), h('td', {class: 'num'}, `${num(o.x)}, ${num(o.y)}`),
          h('td', {class: 'act'}, h('button', {onclick: () => edit({op: 'add', object: o.id, state: state.value}, `remember ${o.id}`)}, 'Remember real position'))));
      }

      let roomKey = '';
      B.on('world', d => {
        const r = d && d.room;
        if (!r) return;
        room = r;
        const k = JSON.stringify([r.labels, r.objects.map(o => [o.id, o.label, o.x, o.y])]);
        if (k === roomKey) return;
        roomKey = k;
        const cur = label.value;
        label.replaceChildren(...(r.labels || []).map(l => h('option', {value: l}, l)));
        if (cur && (r.labels || []).includes(cur)) label.value = cur;
        renderObjects();
      });

      async function poll() {
        if (!document.hidden && !busy) {
          try {
            const r = await B.api('world/memory');
            if (r.ok) { entries = r.entries; render(); B.emit('gotomem', entries); if (/^goto MEMORY|^robot/.test(msg.textContent)) say(''); }
            else if (!msg.textContent) say(r.error || 'no memory from goto', 'dim');
          } catch (e) { /* bench restarting */ }
        }
        setTimeout(poll, 1000);
      }
      poll();
    },
  });
})();

// maps: the two map demo modes (SIMULATION). Server side: bench/plugins/maps.py, bench/maps/README.md.
//  A. Explore live: goto starts knowing nothing, scans / explores, its map fills from what the sim
//     LiDAR sees (drawn under the 2D view), then GOTO.
//  B. Prescan: a saved map becomes the sim room + goto's prior map + memory + the 3D backdrop; drop
//     the car anywhere, then GOTO. Simulation only: the robot can't load a map (yet).
'use strict';
(() => {
  function h(tag, attrs = {}, ...kids) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k === 'style') Object.assign(e.style, v); else if (k.startsWith('on')) e.addEventListener(k.slice(2), v); else e.setAttribute(k, v);
    }
    for (const k of kids) e.append(k);
    return e;
  }

  Bench.register({
    id: 'maps', title: 'Map demos (SIM)', short: '🗺 Map demos', order: 20, wide: true, area: 'room',
    help: 'A: start knowing nothing, explore, then find the target. B: load a saved map, drop the car anywhere, GOTO. Watch the map view while it runs.',
    mount(el, B) {
      let grid = null, cells = null, mode = {}, showGrid = true, world = null;
      const row = (...k) => h('div', {style: {display: 'flex', gap: '6px', alignItems: 'center', flexWrap: 'wrap', margin: '4px 0'}}, ...k);
      const say = (t, cls) => { status.className = 'mono ' + (cls || ''); status.textContent = t; };
      async function api(what, body) {
        try {
          const r = await B.api('maps/' + what, body);
          B.log('maps', `${what}: ${r.ok === false ? 'ERR ' + (r.error || '') : 'ok'}`);
          return r;
        } catch (e) { B.log('maps', `${what} failed: ${e.message}`); say(`${what}: ${e.message}`, 'bad'); return {ok: false, error: e.message}; }
      }

      // ---------- A
      const target = h('select');
      const thenGo = h('input', {type: 'checkbox', checked: ''});
      const hops = h('input', {type: 'number', min: 1, max: 20, value: 6, style: {width: '48px'}});
      const aBox = h('div', {class: 'card', style: {padding: '6px 8px', border: '1px solid var(--line)', borderRadius: '6px'}},
        h('div', {style: {fontWeight: 600}}, 'A · Explore live: the car starts knowing nothing'),
        row(h('button', {onclick: async () => { say('fresh start…'); const r = await api('fresh', {}); say(r.ok ? 'goto restarted: empty map, no memory. Now Scan or Explore.' : 'fresh: ' + (r.error || JSON.stringify(r)), r.ok ? 'ok' : 'bad'); }, title: 'Reload this room with no memory, fresh goto (empty map), clear the 3D map'}, '1. Forget everything'),
          h('button', {onclick: async () => { const r = await api('scan', {}); say(r.reply || r.error); }, title: 'goto SCAN: spin in place and look'}, '2. Scan (spin)'),
          h('button', {onclick: async () => { const r = await api('explore', {target: target.value, max_hops: +hops.value, then_goto: thenGo.checked}); if (!r.ok) say(r.error, 'bad'); }, title: 'SCAN, drive to the nearest unmapped edge, SCAN… until goto remembers the target'}, '3. Explore & find'),
          'target', target, 'hops ≤', hops, h('label', {}, thenGo, ' then GOTO'),
          h('button', {class: 'stop', onclick: () => api('explore_stop', {})}, 'stop explore')),
        h('div', {class: 'dim', style: {fontSize: '12px'}}, 'goto’s own map (NAV GRID) is drawn in the 2D view: teal = floor it has seen, orange = obstacles it has seen, unshaded = unknown. The iPhone 3D map fills the same way (press Surface there for the splats).'));

      // ---------- B
      const saveName = h('input', {placeholder: 'prescan name', size: 14});
      const scanSel = h('select');
      const dropTh = h('input', {type: 'number', value: 0, step: 15, style: {width: '56px'}, title: 'heading for the drop, degrees CCW (0 = +x, right in the 2D view)'});
      const recSel = h('select');
      const recKind = h('select', {}, h('option', {value: 'recording'}, 'phone3d recording'), h('option', {value: 'mapper'}, 'mapper save (path)'));
      const recPath = h('input', {placeholder: 'or a path (repo-relative)', size: 22});
      const recName = h('input', {placeholder: 'name', size: 10});
      const bBox = h('div', {style: {padding: '6px 8px', border: '1px solid #6b5520', borderRadius: '6px', marginTop: '6px'}},
        h('div', {style: {fontWeight: 600}}, 'B · Prescan ', h('span', {style: {background: '#e6b450', color: '#1a1300', padding: '1px 6px', borderRadius: '4px', fontSize: '12px'}}, 'SIMULATION ONLY')),
        row(saveName, h('button', {onclick: async () => {
          const name = saveName.value.trim(); if (!name) { say('type a name', 'bad'); return; }
          let r = await api('save_prescan', {name});
          if (!r.ok && r.exists && confirm(`Prescan "${name}" exists. Overwrite?`)) r = await api('save_prescan', {name, overwrite: true});
          say(r.ok ? `saved ${r.path}: ${r.free} free / ${r.occ} occupied cells, objects ${r.objects.join(', ') || 'none'}, ${r.surfaces} splats` : 'save: ' + r.error, r.ok ? 'ok' : 'bad');
          refresh();
        }, title: 'goto’s map + what it remembers + the 3D splats → bench/world/prescans/<name>.json'}, 'Save goto’s map as a prescan')),
        row(scanSel, h('button', {onclick: async () => {
          if (!scanSel.value) return;
          say('loading prescan… (room, goto restart, seeding its map)');
          const r = await api('load_prescan', {name: scanSel.value});
          say(r.ok ? `loaded: ${r.room.walls} walls from the scan, ${r.room.objects} objects; goto map seeded ${JSON.stringify((r.seed || {}).got)}. Drop the car, then GOTO.`
            : 'load: ' + (r.error || JSON.stringify(r.seed || r)), r.ok ? 'ok' : 'bad');
        }}, 'Load prescan'),
          h('button', {onclick: () => { say('click in the 2D view where the car goes…'); window.World2DPick = drop; }, title: 'next click in the 2D view puts the car there (goto restarted, map re-seeded)'}, 'Drop car (click in 2D)'), 'heading°', dropTh,
          h('button', {onclick: async () => { const r = await B.api('world/goto', {label: target.value}).catch(e => ({error: e.message})); say(`GOTO ${target.value}: ${r.reply || r.error}`); }}, 'GOTO target')),
        row('import a real scan:', recKind, recSel, recPath, recName, h('button', {onclick: async () => {
          const path = recPath.value.trim() || recSel.value, name = recName.value.trim();
          if (!path || !name) { say('pick a recording (or a path) and a name', 'bad'); return; }
          const r = await api('import', {kind: recKind.value, path, name});
          say(r.ok ? `imported → ${r.path}: ${r.free} free / ${r.occ} occ, objects ${r.objects.join(', ') || 'none'}, ${r.surfaces} splats. ${r.about}` : 'import: ' + r.error, r.ok ? 'ok' : 'bad');
          refresh();
        }}, 'Import')),
        h('div', {class: 'dim', style: {fontSize: '12px'}}, 'The sim world frame is the scan’s ARKit frame (x = forward at the scan start). Walls come from the scan’s occupied cells, objects from what it saw. On the real robot this needs goto to load a map and the phone to relocalise in it: not built.'));

      const status = h('div', {class: 'mono', style: {fontSize: '12px', marginTop: '6px', whiteSpace: 'pre-wrap'}});
      const exLine = h('div', {class: 'mono', style: {fontSize: '12px'}});
      const gridToggle = h('label', {style: {fontSize: '12px'}}, h('input', {type: 'checkbox', checked: '', onchange: e => { showGrid = e.target.checked; redraw(); }}), ' show goto’s map in the 2D view');
      el.append(aBox, bBox, row(gridToggle), exLine, status);

      async function drop(x, y) {
        const th = (+dropTh.value || 0) * Math.PI / 180;
        say(`dropping the car at ${x.toFixed(2)}, ${y.toFixed(2)}…`);
        const r = await api('drop', {x, y, th});
        say(r.ok ? `car at ${x.toFixed(2)}, ${y.toFixed(2)}; goto restarted${r.seed ? ', map re-seeded ' + JSON.stringify(r.seed.got) : ''}` : 'drop: ' + (r.error || JSON.stringify(r)), r.ok ? 'ok' : 'bad');
      }
      async function refresh() {
        try {
          const r = await B.api('maps/info');
          const cur = scanSel.value;
          scanSel.replaceChildren(...r.prescans.map(n => h('option', {value: n}, n)));
          if (r.prescans.includes(cur)) scanSel.value = cur;
          recSel.replaceChildren(h('option', {value: ''}, '(recording)'), ...r.recordings.map(n => h('option', {value: n}, n.split('/').pop())));
          if (!r.supervisor) say('Not started by bench/run_sim.py: Forget/Load/Drop need it (they restart goto).', 'warn');
        } catch (e) { say('maps plugin not loaded', 'bad'); }
      }
      refresh();

      // ---------- data
      const decode = g => {
        if (!g || !g.box || g.box === '-' || !g.rle || g.rle === '-') return [];
        const [x0, z0, w] = g.box.split(',').map(Number);
        const out = []; let k = 0;
        for (const run of g.rle.split(',')) {
          const [v, n] = run.split('x').map(Number);
          if (v) for (let i = 0; i < n; i++) out.push([x0 + (k + i) % w, z0 + Math.floor((k + i) / w), v]);
          k += n;
        }
        return out;
      };
      B.on('navgrid', g => { grid = g; cells = decode(g); redraw(); });
      B.on('world', w => { world = w; });
      B.on('maps', m => {
        mode = m || {};
        const ex = mode.explore;
        exLine.textContent = (mode.mode === 'prescan' ? `PRESCAN ${mode.prescan} (simulation only)` : mode.mode === 'explore' ? 'EXPLORE mode' : '') +
          (ex ? `  ·  explore ${ex.running ? 'running' : 'done'}: ${ex.phase}, ${ex.scans} scans, ${ex.hops} hops${ex.found ? `, found ${ex.target} at map ${ex.found.x_ark.toFixed(2)},${ex.found.z_ark.toFixed(2)}` : ''}${ex.result ? '  →  ' + ex.result : ''}` : '');
        redraw();
      });
      B.on('world', w => {
        if (w && w.room && w.room.labels && target.options.length === 0) {
          target.replaceChildren(...w.room.labels.map(n => h('option', {value: n}, n)));
          target.value = w.room.labels.includes('trash can') ? 'trash can' : w.room.labels[0];
        }
      });
      const redraw = () => { if (window.World2DRedraw) window.World2DRedraw(); };

      // ---------- the 2D layer: goto's map, the explorer's hop, the mode
      (window.World2DLayers = window.World2DLayers || []).push((ctx, S, s, W) => {
        const st = (W && W.room && W.room.start) || {x: 0, y: 0, th: 0};
        const c = Math.cos(st.th), sn = Math.sin(st.th);
        const toW = (xa, za) => [st.x + c * za - sn * xa, st.y + sn * za + c * xa];     // ARKit -> world
        if (showGrid && grid && cells) {
          const res = grid.res || 0.1, ox = (grid.origin || [-10, -10])[0], oz = (grid.origin || [-10, -10])[1];
          for (const [ix, iz, v] of cells) {
            const xa = ox + ix * res, za = oz + iz * res;
            ctx.fillStyle = v === 2 ? 'rgba(239,140,60,.55)' : 'rgba(76,195,170,.16)';
            ctx.beginPath();
            [[0, 0], [res, 0], [res, res], [0, res]].forEach(([a, b], k) => { const p = S(...toW(xa + a, za + b)); k ? ctx.lineTo(...p) : ctx.moveTo(...p); });
            ctx.closePath(); ctx.fill();
          }
          ctx.font = '600 12px system-ui, sans-serif'; ctx.textAlign = 'left'; ctx.fillStyle = '#4cc3aa';
          ctx.fillText(`goto's map: ${grid.free || 0} free · ${grid.occ || 0} obstacle cells`, 10, 18);
        }
        if (mode.mode === 'prescan') {
          ctx.font = '700 13px system-ui, sans-serif'; ctx.textAlign = 'left'; ctx.fillStyle = '#e6b450';
          ctx.fillText(`PRESCAN ${mode.prescan} · SIMULATION ONLY`, 10, 36);
        }
        const ex = mode.explore;
        if (ex && ex.running && ex.path && ex.path.length > 1) {
          ctx.strokeStyle = '#d65ad6'; ctx.lineWidth = 2; ctx.setLineDash([6, 4]); ctx.beginPath();
          ex.path.forEach(([xa, za], k) => { const p = S(...toW(xa, za)); k ? ctx.lineTo(...p) : ctx.moveTo(...p); });
          ctx.stroke(); ctx.setLineDash([]);
          if (ex.goal) { const [gx, gy] = S(...toW(ex.goal[0], ex.goal[1])); ctx.fillStyle = '#d65ad6'; ctx.beginPath(); ctx.arc(gx, gy, 5, 0, 7); ctx.fill(); }
        }
      });
    },
  });
})();

// Top bar: connection lights and what the robot is doing, in plain words.
// Reads topics `status` (tars_motor STATUS + robot links), `goto` (goto STATUS), `car` and `world`.
// It only fills #lights and #activity in the shell's header; STOP stays in the shell.
'use strict';
(() => {
  const T = s => String(s ?? '').replace(/_/g, ' ').trim();
  const has = v => v !== undefined && v !== null && v !== '' && v !== '-';
  const REASONS = {not_found: 'not found after looking all around', blocked: 'the way is blocked', timeout: 'took too long',
    no_route: 'no route there', lost: 'lost sight of it', guard: 'obstacle sensor stopped it', motor: 'motor link problem',
    unknown_object: 'it doesn’t know that object'};

  // goto STATUS -> [text, class]
  function phrase(G) {
    const t = has(G.target) ? T(G.target) : 'it';
    const searched = +G.searched || 0;
    const swept = searched ? ` · ${searched}° searched` : '';
    const leg = has(G.path) && G.path !== '0/0' ? ` · leg ${G.path}` : '';
    if (G.mode === 'scan' && G.state !== 'idle') return [`Scanning the room…${swept}`, 'busy'];
    switch (G.state) {
      case 'look': return [`Looking for the ${t}…${swept}`, 'busy'];
      case 'search_turn': return [`Turning to look for the ${t}…${swept}`, 'busy'];
      case 'align_turn': return [`Spotted the ${t}, turning to face it`, 'busy'];
      case 'approach': return [`Driving up to the ${t}`, 'busy'];
      case 'mem_turn': case 'mem_move': return [`Heading to where it last saw the ${t}`, 'busy'];
      case 'recall': return [`Remembering where the ${t} was`, 'busy'];
      case 'plan': return [`Planning a route to the ${t}`, 'busy'];
      case 'follow_path': return [`Driving to the ${t}${leg}`, 'busy'];
      case 'occluded': return [`The ${t} is hidden behind something`, 'busy'];
      case 'check_others': return [`The ${t} wasn’t there: checking other sightings`, 'busy'];
      case 'detour': return [`Going around an obstacle to see the ${t}`, 'busy'];
      case 'next_candidate': return [`Trying another ${t}`, 'busy'];
    }
    const res = String(G.result ?? '');
    if (/^arrived/.test(res)) return [`✓ Arrived at the ${t}`, 'ok'];
    if (/^failed/.test(res)) {
      const why = res.split(':')[1] || '';
      return [`✗ Couldn’t reach the ${t}: ${REASONS[why] || T(why) || 'failed'}`, 'bad'];
    }
    if (/cancel/.test(res)) return [`Stopped (go-to cancelled)`, ''];
    if (has(G.state) && G.state !== 'idle') return [`${T(G.state)} ${t}`, 'busy'];
    return ['Ready. Try “go to the trash can”', ''];
  }

  Bench.register({
    id: 'topbar', area: 'header', order: 0,
    mount() {
      const lights = document.getElementById('lights'), act = document.getElementById('activity');
      const real = ['pi', 'usb'].includes((Bench.info.target || {}).name);
      const L = {};
      const DEFS = [
        ['motor', 'Motors', 'tars_motor: the only program that moves the wheels. Green = STATUS arriving.'],
        ['goto', 'Go-to', 'goto: the controller that finds and drives to objects.'],
        ['voice', 'Voice', 'tars_voice: the robot’s speech + intent classifier.'],
        ['car', real ? 'Car' : 'Car (sim)', real ? 'The Uno and motors, as tars_motor sees them.' : 'The virtual SM11 car: real Uno firmware on a physics model.'],
        ['guard', 'Path', 'The phone LiDAR obstacle check in front of the car. Red = blocked, the car won’t drive forward.'],
      ];
      for (const [k, name, tip] of DEFS) {
        const s = document.createElement('span');
        s.className = 'light off'; s.textContent = name; s.dataset.tip = tip; s.title = tip + '\n(no data yet)';
        lights.appendChild(s); L[k] = {el: s, name, tip};
      }
      const set = (k, cls, detail) => {
        const l = L[k]; l.el.className = 'light ' + cls; l.el.title = l.tip + (detail ? '\n' + detail : '');
      };

      let st = null, stT = 0, go = null, goT = 0, carT = 0, carAlive = false, world = null;
      Bench.on('status', s => { st = s; stT = performance.now(); });
      Bench.on('goto', g => { go = g; goT = performance.now(); });
      Bench.on('car', d => { const t = d && (d.truth || d); carT = performance.now(); carAlive = !!(t && t.alive !== false && !t.stub); });
      Bench.on('world', w => { world = w; });

      function tick() {
        const now = performance.now();
        const links = (st && st.links) || {};
        const f = (st && st.ok && st.fields) || {};
        const fresh = st && st.ok && now - stT < 1500;
        set('motor', fresh ? 'ok' : (links.motor && !links.motor.connected ? 'bad' : 'warn'),
          fresh ? 'live' : (links.motor && links.motor.error) || 'no STATUS in the last 1.5 s');
        const gfresh = go && go.link && now - goT < 3000;
        set('goto', gfresh ? (go.ok ? 'ok' : 'warn') : 'bad', gfresh ? 'live' : 'not connected');
        if (links.voice) set('voice', links.voice.connected ? 'ok' : 'bad', links.voice.connected ? 'connected' : (links.voice.error || 'not connected'));
        else set('voice', 'off', 'not used on this target');
        if (!real) set('car', now - carT < 2000 ? (carAlive ? 'ok' : 'warn') : 'off', now - carT < 2000 ? (carAlive ? 'virtual car running' : 'stub car (no physics) or car not answering') : 'no car data');
        else set('car', fresh && f.ready !== 0 && f.ready !== '0' ? 'ok' : 'warn', fresh ? `ready=${f.ready ?? '?'} backend=${f.backend ?? '?'}` : 'no STATUS');
        const gv = (world && world.guard && world.guard.verdict) || (go && go.status && go.status.guard) || '';
        const v = String(gv).toUpperCase();
        set('guard', /BLOCK/.test(v) ? 'bad' : /CLEAR/.test(v) ? 'ok' : v ? 'warn' : 'off', v ? `verdict ${v}` : 'no obstacle data');
        L.guard.el.textContent = /BLOCK/.test(v) ? 'Path blocked' : 'Path';
        guardNote(/BLOCK/.test(v), f);

        // activity, in priority order: safety stops, go-to, hand driving
        let text, cls = '';
        const G = (go && go.status) || null;
        // estop=1 why=watchdog is tars_motor's normal resting state (no client driving), not an alarm
        const alarmWhy = /tip|bump|tilt|fall|lift/.test(String(f.why || ''));
        if (fresh && (+f.tipped === 1 || (+f.estop === 1 && alarmWhy))) { text = `⚠ Safety stop: ${+f.tipped === 1 ? 'tipped over' : T(f.why)}`; cls = 'bad'; }
        else if (G && (G.state !== 'idle' || has(G.result))) {
          [text, cls] = phrase(G);
          if (G.state !== 'idle' && /BLOCK/.test(v)) text += ' · path blocked';
          if (G.state === 'idle' && fresh && (Math.abs(+f.out_l || 0) > 0.01 || Math.abs(+f.out_r || 0) > 0.01)) { text = 'Driving by hand'; cls = 'busy'; }
        } else if (fresh && (Math.abs(+f.out_l || 0) > 0.01 || Math.abs(+f.out_r || 0) > 0.01)) { text = 'Driving by hand'; cls = 'busy'; }
        else if (!gfresh && !fresh) { text = 'Not connected to the robot stack yet…'; cls = 'bad'; }
        else [text, cls] = phrase(G || {state: 'idle'});
        act.textContent = text; act.className = 'activity ' + cls;
        act.title = G ? `goto state=${G.state} target=${G.target} result=${G.result} searched=${G.searched ?? '-'}° elapsed=${G.elapsed ?? '-'} s` : 'goto: no status';
      }
      // plain words under the top bar while the guard blocks, so "why can't I move?" answers itself.
      // If the robot plugin supplies its own wording (status topic `guard_text`), that wins.
      const note = document.getElementById('guardnote');
      L.guard.el.style.cursor = 'pointer';
      L.guard.el.addEventListener('click', () => Bench.openPage && Bench.openPage('car'));
      let noteKey = '';
      function guardNote(blocked, f) {
        const own = st && (st.guard_text || (st.guard && st.guard.text));
        const reason = String(f.guard_reason || '');
        const show = !!(blocked || (own && /block/i.test(reason)));
        const k = show ? (own || '') + '|' + reason : '';
        if (!note || k === noteKey) return;
        noteKey = k; note.hidden = !show;
        if (!show) return;
        note.replaceChildren();
        if (own) { note.textContent = '⚠ ' + own; return; }
        note.append('⚠ Path blocked: no driving forward. ');
        const d = document.createElement('span'); d.className = 'dim';
        d.textContent = 'Back up (S), turn away, or move the obstacle (Room page). Details: Car → Safety.';
        note.append(d);
      }
      setInterval(tick, 250); tick();
    },
  });
})();

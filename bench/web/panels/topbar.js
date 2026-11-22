// Status strip under the top bar: goto's state as one word (Idle / Planning / Searching / Driving / Arrived /
// Failed / Stopped), the target, what the robot is doing in plain words, the path guard, sim speed and the links.
// Reads topics `status` (tars_motor STATUS + robot links), `goto` (goto STATUS), `car` and `world`.
// Every cell is always there with a fixed width or ellipsis, so the strip never changes height and
// nothing on the page moves. STOP stays in the shell (shell.js).
'use strict';
(() => {
  const T = s => String(s ?? '').replace(/_/g, ' ').trim();
  const has = v => v !== undefined && v !== null && v !== '' && v !== '-';
  // goto `result=failed:<why>` -> plain words (the code stays in brackets, so the log and the strip match)
  const REASONS = {not_found: 'not found after looking all around', blocked: 'the path sensor kept blocking it', timeout: 'took too long',
    no_route: 'no route there', unreachable: 'no way through to it', lost: 'lost sight of it', guard: 'the path sensor stopped it',
    motor: 'motor link problem', unknown_object: 'it doesn’t know that object', cancelled: 'cancelled'};
  // goto state -> the one-word phase on the chip
  const PHASE = {plan: 'PLANNING', recall: 'PLANNING',
    look: 'SEARCHING', search_turn: 'SEARCHING', align_turn: 'SEARCHING', occluded: 'SEARCHING', check_others: 'SEARCHING',
    next_candidate: 'SEARCHING', scan: 'SEARCHING',
    approach: 'DRIVING', mem_turn: 'DRIVING', mem_move: 'DRIVING', follow_path: 'DRIVING', detour: 'DRIVING'};

  // goto STATUS -> [sentence, class]
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
      return [`✗ Couldn’t reach the ${t}: ${REASONS[why] || T(why) || 'failed'}${why ? ` (${T(why)})` : ''}`, 'bad'];
    }
    if (/cancel/.test(res)) return [`Stopped (go-to cancelled)`, ''];
    if (has(G.state) && G.state !== 'idle') return [`${T(G.state)} ${t}`, 'busy'];
    return ['Ready. Say or type “go to the trash can”', ''];
  }
  // goto STATUS -> [chip word, chip class]
  function phase(G) {
    if (!G) return ['IDLE', ''];
    if (G.state !== 'idle') return [PHASE[G.state] || (G.mode === 'scan' ? 'SEARCHING' : T(G.state).toUpperCase() || 'BUSY'), 'busy'];
    const res = String(G.result ?? '');
    if (/^arrived/.test(res)) return ['ARRIVED', 'ok'];
    if (/^failed/.test(res)) return ['FAILED', 'bad'];
    if (/cancel/.test(res)) return ['STOPPED', 'dim'];
    return ['IDLE', ''];
  }

  Bench.register({
    id: 'topbar', area: 'header', order: 0,
    mount() {
      const $ = id => document.getElementById(id);
      const lights = $('lights'), act = $('activity'), chip = $('sb-state'), tgt = $('sb-target');
      const gchip = $('sb-guard'), gcell = $('sb-guard-cell'), speedCell = $('sb-speed'), speedV = $('sb-speed-v');
      const real = ['pi', 'usb'].includes((Bench.info.target || {}).name);
      const L = {};
      const DEFS = [
        ['motor', 'Motors', 'tars_motor: the only program that moves the wheels. Green = STATUS arriving.'],
        ['goto', 'Go-to', 'goto: the controller that finds and drives to objects.'],
        ['voice', 'Voice', 'tars_voice: the robot’s speech + intent classifier.'],
        ['car', 'Car', real ? 'The Uno and motors, as tars_motor sees them.' : 'The virtual SM11 car: real Uno firmware on a physics model.'],
      ];
      for (const [k, name, tip] of DEFS) {
        const s = document.createElement('span');
        s.className = 'light off'; s.textContent = name; s.dataset.tip = tip; s.title = tip + '\n(no data yet)';
        lights.appendChild(s); L[k] = {el: s, name, tip};
      }
      const set = (k, cls, detail) => {
        const l = L[k]; l.el.className = 'light ' + cls; l.el.title = l.tip + '\n' + (detail || '');
      };
      gcell.addEventListener('click', () => Bench.openPage && Bench.openPage('car'));

      // sim only: demo speed (run_sim --speed; the car, world and goto all run N x the wall clock)
      let speedSel = null, speedBusy = false;
      if (real) { speedCell.title = 'Real robot: always real time'; speedV.textContent = '1×'; }
      else {
        speedSel = document.createElement('select');
        speedSel.id = 'simspeed'; speedSel.className = 'simspeed';
        const tip = 'Sim speed (demo): the virtual car, the world sim and goto all run this many times faster than real time, ' +
          'so a MOVE still lands where it does at 1×. Voice stays real time. Changing it stops the car and restarts goto ' +
          '(memory kept, map cleared). Use 1× for testing timing-sensitive things.';
        speedSel.dataset.tip = tip; speedSel.title = tip;
        for (const v of [1, 2, 4]) speedSel.append(new Option(`${v}×`, String(v)));
        speedSel.disabled = true;
        speedSel.addEventListener('change', async () => {
          const v = +speedSel.value;
          speedBusy = true; speedSel.disabled = true;
          try {
            const r = await Bench.api('world/speed', {speed: v});
            Bench.log('world', `sim speed ${v}×: ${r && r.ok ? 'ok (goto restarted, memory kept)' : 'failed'}`);
          } catch (e) {
            Bench.log('world', `sim speed not changed: ${e.message}`);
          } finally { speedBusy = false; }
        });
        speedV.replaceWith(speedSel);
      }

      let st = null, stT = 0, go = null, goT = 0, carT = 0, carAlive = false, world = null, worldT = 0;
      Bench.on('status', s => { st = s; stT = performance.now(); });
      Bench.on('goto', g => { go = g; goT = performance.now(); });
      Bench.on('car', d => { const t = d && (d.truth || d); carT = performance.now(); carAlive = !!(t && t.alive !== false && !t.stub); });
      Bench.on('world', w => { world = w; worldT = performance.now(); });

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
        else set('voice', 'off', 'not used on this target (or the sim was started with --no-voice)');
        if (!real) set('car', now - carT < 2000 ? (carAlive ? 'ok' : 'warn') : 'off', now - carT < 2000 ? (carAlive ? 'virtual car running' : 'stub car (no physics) or car not answering') : 'no car data');
        else set('car', fresh && f.ready !== 0 && f.ready !== '0' ? 'ok' : 'warn', fresh ? `ready=${f.ready ?? '?'} backend=${f.backend ?? '?'}` : 'no STATUS');

        // path guard chip
        const gv = (world && world.guard && world.guard.verdict) || (go && go.status && go.status.guard) || '';
        const v = String(gv).toUpperCase();
        const blocked = /BLOCK/.test(v);
        gchip.textContent = blocked ? 'BLOCKED' : /CLEAR/.test(v) ? 'CLEAR' : v && v !== '-' ? v : 'no data';
        gchip.className = 'chip ' + (blocked ? 'bad' : /CLEAR/.test(v) ? 'ok' : v && v !== '-' ? 'warn' : 'dim');
        guardNote(blocked, f);

        // sim speed
        if (speedSel && !speedBusy) {
          const wfresh = world && now - worldT < 3000;
          const sp = (world && +world.speed) || 1;
          if (![...speedSel.options].some(o => +o.value === sp)) speedSel.append(new Option(`${sp}×`, String(sp)));
          if (document.activeElement !== speedSel) speedSel.value = String(sp);
          speedSel.disabled = !(wfresh && world.supervisor);
          speedSel.title = speedSel.disabled
            ? (!wfresh ? 'Speed: sim not connected yet' : 'Speed can only change when the sim was started with bench/run_sim.py')
            : speedSel.dataset.tip;
          speedSel.classList.toggle('fast', sp !== 1);
        }

        // phase chip + sentence, in priority order: offline, safety stops, go-to, hand driving
        const G = (go && go.status) || null;
        let text, cls = '', word, wcls;
        // estop=1 why=watchdog is tars_motor's normal resting state (no client driving), not an alarm
        const alarmWhy = /tip|bump|tilt|fall|lift/.test(String(f.why || ''));
        const hand = fresh && (Math.abs(+f.out_l || 0) > 0.01 || Math.abs(+f.out_r || 0) > 0.01);
        [word, wcls] = phase(G);
        if (!gfresh && !fresh) { text = 'Not connected to the robot stack yet…'; cls = 'bad'; word = 'OFFLINE'; wcls = 'bad'; }
        else if (fresh && (+f.tipped === 1 || (+f.estop === 1 && alarmWhy))) {
          text = `⚠ Safety stop: ${+f.tipped === 1 ? 'tipped over' : T(f.why)}`; cls = 'bad'; word = 'SAFETY STOP'; wcls = 'bad';
        } else if (G && (G.state !== 'idle' || has(G.result))) {
          [text, cls] = phrase(G);
          if (G.state !== 'idle' && blocked) text += ' · path blocked';
          if (G.state === 'idle' && hand) { text = 'Driving by hand'; cls = 'busy'; word = 'MANUAL'; wcls = 'busy'; }
        } else if (hand) { text = 'Driving by hand'; cls = 'busy'; word = 'MANUAL'; wcls = 'busy'; }
        else [text, cls] = phrase(G || {state: 'idle'});
        if (act.textContent !== text) act.textContent = text;
        act.className = 'activity ' + cls;
        act.title = G ? `goto state=${G.state} target=${G.target} result=${G.result} path=${G.path ?? '-'} replans=${G.replans ?? '-'} elapsed=${G.elapsed ?? '-'} s` : 'goto: no status';
        if (chip.textContent !== word) chip.textContent = word;
        chip.className = 'chip ' + wcls;
        const target = G && has(G.target) ? T(G.target) : '—';
        if (tgt.textContent !== target) tgt.textContent = target;
      }
      // plain words while the guard blocks, so "why can't I move?" answers itself. A toast that floats over
      // the page (no layout shift). If the robot plugin supplies its own wording (`guard_text`), that wins.
      const note = $('guardnote');
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

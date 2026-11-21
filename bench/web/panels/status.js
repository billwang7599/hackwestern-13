// Status panel: every tars_motor STATUS field, grouped; changed values flash; stale is loud.
// Also the robot links and goto's STATUS + last EVENTs.
'use strict';
(() => {
  const css = `
  .st-head { display:flex; gap:8px; align-items:center; flex-wrap:wrap; margin-bottom:6px; }
  .st-badge { font: 600 11px var(--mono); padding:2px 6px; border-radius:4px; border:1px solid var(--line); }
  .st-badge.ok { color: var(--ok); border-color: var(--ok); } .st-badge.bad { color: var(--bad); border-color: var(--bad); }
  .st-badge.warn { color: var(--warn); border-color: var(--warn); }
  .st-grid { display:grid; grid-template-columns: repeat(auto-fill, minmax(170px, 1fr)); gap:6px; }
  .st-grp { border:1px solid var(--line); border-radius:6px; padding:4px 6px; }
  .st-grp h3 { margin:0 0 2px; font-size:11px; color: var(--dim); text-transform:uppercase; letter-spacing:.05em; }
  .st-kv { display:flex; justify-content:space-between; gap:6px; font: 12px/1.35 var(--mono); }
  .st-kv b { font-weight:500; text-align:right; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; transition: background .8s; }
  .st-kv b.chg { background: #3a5a2a; transition: none; }
  .st-kv b.alarm { color: var(--bad); }
  .st-stale .st-grid { opacity:.45; }
  .st-goto { font: 12px/1.35 var(--mono); margin-top:6px; }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));

  const GROUPS = [
    ['drive', /^(out_l|out_r|ms_since_cmd|dist_m|hold_ms|wheels|dc_mm_s|dc_deg_s|dc_deg_s_slow|wheel_d|wheel_t)$/],
    ['move', /^move/],
    ['guard', /^guard/],
    ['safety', /^(estop|why|tipped|tilt|bump_ms|bumps|stops|stop_ms)$/],
    ['led / buzzer / buttons', /^(led|btn_teach|btn_forget|beep)/],
    ['clients / mode / vision', /^(clients|ready|backend|mode_req|mode_seq|vis.*)$/],
    ['pose', /^pose/],
    ['arms / gesture', /^(arm|gesture)/],
  ];
  const ALARM = {estop: v => v === 1, tipped: v => v === 1, why: v => v !== 'none', guard_permit: v => v === 0,
                 move_why: v => v !== '-' && v !== 'done'};

  Bench.register({
    id: 'status', title: 'tars_motor STATUS (raw)', order: 30, area: 'car', wide: true,
    help: 'Every field the motor server reports, grouped. Changes flash; red = an alarm (estop, tipped, blocked).',
    mount(el) {
      el.innerHTML = `<div class="st-head"><span class="st-badge age">no data</span><span class="links"></span><span class="st-badge rp" style="display:none">REPLAY</span></div>
        <div class="st-grid"></div><div class="st-goto"></div>`;
      const grid = el.querySelector('.st-grid'), age = el.querySelector('.age');
      const cells = {}, last = {};
      let lastT = 0, lastOk = false;
      function cell(k) {
        if (cells[k]) return cells[k];
        let gi = GROUPS.findIndex(([, re]) => re.test(k));
        const gname = gi < 0 ? 'other' : GROUPS[gi][0];
        let g = grid.querySelector(`[data-g="${gname}"]`);
        if (!g) {
          g = document.createElement('div'); g.className = 'st-grp'; g.dataset.g = gname;
          g.style.order = gi < 0 ? 99 : gi;
          g.innerHTML = `<h3>${gname}</h3>`; grid.appendChild(g);
        }
        const row = document.createElement('div'); row.className = 'st-kv';
        row.innerHTML = `<span></span><b></b>`; row.firstChild.textContent = k;
        g.appendChild(row);
        return (cells[k] = row.lastChild);
      }
      Bench.on('status', s => {
        lastT = performance.now(); lastOk = !!s.ok;
        el.querySelector('.rp').style.display = s._replay ? '' : 'none';
        const L = s.links || {};
        el.querySelector('.links').innerHTML = Object.entries(L).map(([k, v]) =>
          `<span class="st-badge ${v.connected ? 'ok' : 'bad'}" title="${(v.error || '').replace(/"/g, "'")}">${k}</span>`).join(' ');
        if (!s.ok) { if (s.raw) age.title = s.raw; return; }
        for (const [k, v] of Object.entries(s.fields || {})) {
          if (k === 'pose_x_mm' || k === 'pose_y_mm' || k === 'pose_hdg_deg') continue;
          const b = cell(k);
          const txt = String(v);
          if (last[k] !== txt) {
            b.textContent = txt; b.title = txt;
            if (k in last) { b.classList.add('chg'); setTimeout(() => b.classList.remove('chg'), 60); }
            last[k] = txt;
          }
          b.classList.toggle('alarm', !!(ALARM[k] && ALARM[k](v)));
        }
      });
      setInterval(() => {
        const a = (performance.now() - lastT) / 1000;
        const stale = !lastT || a > 1 || !lastOk;
        el.classList.toggle('st-stale', stale);
        age.className = 'st-badge ' + (stale ? 'bad' : 'ok');
        age.textContent = !lastT ? 'no data' : !lastOk ? 'STALE: no STATUS from tars_motor' : stale ? `STALE ${a.toFixed(1)} s` : 'live';
      }, 250);
      Bench.on('goto', g => {
        const st = g.status || {};
        const ev = (g.events || []).slice(-5).reverse().map(e => `<div class="dim">${new Date(e.t * 1000).toLocaleTimeString()} ${e.text.replace(/</g, '&lt;')}</div>`).join('');
        el.querySelector('.st-goto').innerHTML =
          `<div><span class="st-badge ${g.link ? (g.ok ? 'ok' : 'warn') : 'bad'}">goto</span> ` +
          (g.status ? `state=<b>${st.state}</b> target=${st.target} result=${st.result} seen=${st.seen} mem=${st.mem} map=${st.map} path=${st.path} guard=${st.guard}` : 'no STATUS') +
          `</div>${ev}`;
      });
    }
  });
})();

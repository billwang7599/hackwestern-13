// Virtual SM11 panel: live truth (pwm, wheel speeds, battery, contact), profile
// sliders and fault buttons. Talks to plugins/car.py (/api/car/...), topic `car`.
'use strict';
Bench.register({
  id: 'car', title: 'Virtual car (SM11)', order: 20, area: 'car',
  help: 'The simulated car’s true state, car faults (push, tip, brownout…) and its motor guesses (none measured yet).',
  mount(el, B) {
    const SLIDERS = [
      // key, label, min, max, step
      ['gain_l', 'gain left', 0.5, 1.5, 0.01],
      ['gain_r', 'gain right', 0.5, 1.5, 0.01],
      ['v_keep', 'deadband: keep rolling (V)', 0.5, 3.0, 0.05],
      ['v_start', 'deadband: start from rest (V)', 0.5, 4.0, 0.05],
      ['batt_v', 'battery (V)', 5.0, 8.6, 0.05],
      ['tau_s', 'time constant (s)', 0.01, 0.5, 0.01],
      ['slip', 'slip', 0, 0.5, 0.01],
      ['drift_deg_per_m', 'drift (deg/m, + right)', -30, 30, 0.5],
      ['track_eff_m', 'effective track (m)', 0.08, 0.4, 0.005],
      ['scrub_v', 'spin scrub (V)', 0, 1.5, 0.05],
    ];
    const FAULTS = [
      ['push', {fault: 'push', vy: 0.3}], ['bump', {fault: 'bump'}], ['tip', {fault: 'tip'}],
      ['lift L', {fault: 'lift_left'}], ['lift R', {fault: 'lift_right'}],
      ['brownout', {fault: 'brownout', v: 4.0, ms: 400}], ['unplug USB', {fault: 'unplug', ms: 1500}],
    ];
    el.innerHTML = `
      <div class="mono" data-k="head">no car yet: <code>python3 bench/car/launch.py</code></div>
      <table class="dt" style="margin:6px 0"><thead><tr><th>Reading</th><th>Value</th></tr></thead><tbody data-k="tbl"></tbody></table>
      <div data-k="bars" style="display:grid;grid-template-columns:auto 1fr auto;gap:2px 6px;align-items:center;font-size:12px"></div>
      <h3 style="font-size:12px;color:var(--dim);margin:10px 0 4px">Faults</h3>
      <div data-k="faults" style="display:flex;flex-wrap:wrap;gap:4px"></div>
      <h3 style="font-size:12px;color:var(--dim);margin:10px 0 4px">Profile (live, not saved; <code>measured: false</code> guesses in profiles/sm11.json)</h3>
      <div data-k="sliders" style="display:grid;grid-template-columns:1fr 140px 52px;gap:2px 6px;align-items:center;font-size:12px"></div>`;
    const $ = k => el.querySelector(`[data-k="${k}"]`);

    const send = (path, body) => B.api(path, body)
      .then(r => { if (r && r.error) B.log('car', r.error); return r; })
      .catch(e => B.log('car', `${path}: ${e.message}`));

    for (const [label, body] of FAULTS) {
      const b = document.createElement('button');
      b.textContent = label;
      b.onclick = () => send('car/fault', body);
      $('faults').appendChild(b);
    }
    const clear = document.createElement('button');
    clear.textContent = 'clear faults'; clear.className = 'ok';
    clear.onclick = () => send('car/fault', {fault: 'clear'});
    $('faults').appendChild(clear);
    for (const [label, btn] of [['KEY tap (TEACH)', 'tap'], ['KEY hold 2.5 s (FORGET)', 'hold']]) {
      const b = document.createElement('button');
      b.textContent = label;
      b.onclick = () => send('car/button', {button: btn});
      $('faults').appendChild(b);
    }
    const rst = document.createElement('button');
    rst.textContent = 'Uno reset'; rst.onclick = () => send('car/reset', {});
    $('faults').appendChild(rst);

    const inputs = {};
    let dragging = null;
    for (const [key, label, min, max, step] of SLIDERS) {
      const l = document.createElement('label'); l.textContent = label; l.title = key;
      const r = document.createElement('input');
      Object.assign(r, {type: 'range', min, max, step});
      const v = document.createElement('span'); v.className = 'mono';
      let timer = null;
      r.oninput = () => {
        dragging = key; v.textContent = r.value;
        clearTimeout(timer);
        timer = setTimeout(() => send('car/set', {[key]: parseFloat(r.value)}), 120);
      };
      r.onchange = () => { dragging = null; };
      inputs[key] = {r, v};
      $('sliders').append(l, r, v);
    }

    const bar = (name) => {
      const n = document.createElement('span'); n.textContent = name;
      const track = document.createElement('div');
      track.style.cssText = 'height:8px;background:#0b1013;border-radius:4px;position:relative';
      const fill = document.createElement('div');
      fill.style.cssText = 'position:absolute;top:0;bottom:0;left:50%;background:var(--accent);border-radius:4px';
      track.appendChild(fill);
      const val = document.createElement('span'); val.className = 'mono';
      $('bars').append(n, track, val);
      return (x, full, text) => {          // x signed, full = value at the end of the bar
        const f = Math.max(-1, Math.min(1, x / full));
        fill.style.left = (f >= 0 ? 50 : 50 + 50 * f) + '%';
        fill.style.width = Math.abs(50 * f) + '%';
        val.textContent = text;
      };
    };
    const bPwmL = bar('pwm L'), bPwmR = bar('pwm R'), bWl = bar('wheel L'), bWr = bar('wheel R');

    const fmt = (x, d = 2) => (x === undefined || x === null) ? '–' : Number(x).toFixed(d);
    const row = (k, v, cls = '') => `<tr><td class="k">${k}</td><td class="${cls}">${v}</td></tr>`;

    B.on('car', d => {
      const t = d.truth;
      if (!d.alive || !t) {
        $('head').innerHTML = `<span class="bad">no virtual car</span> on udp ${d.ctl_port}` +
          (d.age_ms != null ? ` (last truth ${d.age_ms} ms ago)` : '') +
          `: <code>python3 bench/car/launch.py</code>`;
        return;
      }
      const f = t.faults || {};
      const on = Object.keys(f).filter(k => f[k]);
      $('head').innerHTML =
        `<span class="${t.fw === 'run' ? 'ok' : 'warn'}">firmware ${t.fw}</span> · boots ${t.boots} · ` +
        `USB ${t.plugged ? (t.host_open ? '<span class="ok">open</span>' : '<span class="warn">no host</span>') : '<span class="bad">unplugged</span>'} ` +
        `<span class="dim">${t.pty || ''}</span>` +
        (on.length ? ` · <span class="bad">faults: ${on.join(', ')}</span>` : '') +
        (t.stalls ? ` · <span class="warn">${t.stalls} loop stalls (max ${t.stall_max_ms} ms)</span>` : '');
      $('tbl').innerHTML =
        row('pose x y th', `${fmt(t.x)} m  ${fmt(t.y)} m  ${fmt(t.th * 180 / Math.PI, 1)}°`) +
        row('v / w', `${fmt(t.v * 1000, 0)} mm/s  ${fmt(t.w * 180 / Math.PI, 1)}°/s`) +
        row('battery', `${fmt(t.batt_v)} V  ${fmt(t.batt_a)} A`, t.batt_v < 6.5 ? 'warn' : '') +
        row('contact', t.contact ? 'WALL' : 'no', t.contact ? 'bad' : '') +
        row('roll', `${fmt(t.roll, 0)}°`) +
        row('LED / buzzer / KEY', `${t.led ? '●' : '○'}  ${t.buzz ? '♪' : '·'}  ${t.btn ? 'down' : 'up'}`) +
        row('imu (g)', `${fmt(t.imu[0])} ${fmt(t.imu[1])} ${fmt(t.imu[2])}`) +
        row('odometer', `${fmt(t.odo, 3)} m`);
      const mode = m => ['coast', 'brake', 'drive'][m] || '?';
      bPwmL(t.pwm_l, 255, `${t.pwm_l} ${mode(t.mode_l)}`);
      bPwmR(t.pwm_r, 255, `${t.pwm_r} ${mode(t.mode_r)}`);
      bWl(t.wl, 0.4, `${fmt(t.wl * 1000, 0)} mm/s`);
      bWr(t.wr, 0.4, `${fmt(t.wr * 1000, 0)} mm/s`);
      if (d.params) for (const k in inputs) {
        if (k === dragging || d.params[k] === undefined) continue;
        inputs[k].r.value = d.params[k];
        inputs[k].v.textContent = d.params[k];
      }
    });
  },
});

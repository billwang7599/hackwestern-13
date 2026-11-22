// Scenarios panel: run scripted checks (bench/plugins/scenarios.py), answer prompts, see numbers.
'use strict';
(() => {
  const css = `
  .sc-list td:first-child { font: 13px system-ui, sans-serif; }
  .sc-res { font: 11px var(--mono); white-space:nowrap; }
  .sc-run { margin-top:8px; border-top:1px solid var(--line); padding-top:6px; }
  .sc-steps { max-height:180px; overflow:auto; font: 12px/1.35 var(--mono); }
  .sc-steps .pass { color: var(--ok); } .sc-steps .fail { color: var(--bad); } .sc-steps .skip, .sc-steps .ask { color: var(--warn); }
  .sc-steps .cmd { color: var(--dim); }
  .sc-prompt { background:#2a2410; border:1px solid var(--warn); border-radius:6px; padding:6px; margin:6px 0; }
  .sc-nums td { font: 12px var(--mono); }`;
  document.head.appendChild(Object.assign(document.createElement('style'), {textContent: css}));
  const COL = {pass: 'ok', fail: 'bad', error: 'bad', aborted: 'bad', skip: 'warn', running: 'dim'};

  Bench.register({
    id: 'scenarios', title: 'Scenarios (bring-up checks)', short: '✅ Scenarios', order: 10, area: 'tests',
    help: 'One-click scripted checks with pass/fail and measured numbers. STOP aborts a run.',
    async mount(el) {
      el.innerHTML = `<div class="dim where" style="font-size:12px;margin-bottom:4px"></div><div class="tbl-scroll" style="max-height:max(420px, calc(100vh - 380px))"><table class="dt sc-list"><thead><tr><th>Check</th><th>Needs</th><th>PI_TEST</th><th>Last result</th><th></th></tr></thead><tbody></tbody></table></div>
        <div class="sc-run" style="display:none"><div><b class="name"></b> <span class="state"></span> <span class="dim wh"></span>
        <button class="cancel" style="float:right">cancel</button></div>
        <div class="sc-prompt" style="display:none"></div>
        <table class="dt sc-nums"></table><div class="sc-steps"></div></div>`;
      const $ = s => el.querySelector(s);
      let info;
      try { info = await Bench.api('scenarios/list'); }
      catch (e) { el.textContent = 'scenarios plugin not loaded'; return; }
      $('.where').textContent = info.robot ? `STOP / Space aborts a run. car plugin: ${info.car ? 'yes' : 'no'}, world: ${info.world ? 'yes' : 'no'}`
                                           : 'robot plugin not loaded: nothing can run';
      const results = {};
      const list = $('.sc-list tbody');
      for (const s of info.scenarios) {
        const tr = document.createElement('tr');
        tr.innerHTML = `<td></td><td class="dim"></td><td class="dim"></td><td><span class="sc-res dim" data-r="${s.name}">not run</span></td><td class="act"></td>`;
        tr.children[0].textContent = s.title; tr.children[0].title = s.name;
        tr.children[1].textContent = s.needs; tr.children[2].textContent = s.pi_test || '';
        const b = document.createElement('button'); b.textContent = '▶ run'; b.disabled = !info.robot;
        b.onclick = () => Bench.api('scenarios/run', {name: s.name}).catch(e => Bench.log('scenario', e.message));
        tr.children[4].appendChild(b);
        list.append(tr);
      }
      $('.cancel').onclick = () => Bench.api('scenarios/cancel', {});
      Bench.on('scenario', s => {
        $('.sc-run').style.display = '';
        $('.name').textContent = s.name;
        $('.state').textContent = s.state.toUpperCase(); $('.state').className = COL[s.state] || '';
        $('.wh').textContent = `on ${s.where} (backend ${s.backend})${s._replay ? ' [replay]' : ''}  ${s.elapsed}s`;
        $('.cancel').style.display = s.state === 'running' ? '' : 'none';
        const nums = Object.entries(s.numbers || {});
        $('.sc-nums').innerHTML = nums.length ? '<thead><tr><th>Measured</th><th>Value</th><th>Unit</th></tr></thead>' +
          nums.map(([k, v]) => `<tr><td class="k">${k}</td><td class="num">${v.v}</td><td class="dim">${v.unit || ''}</td></tr>`).join('') : '';
        const st = $('.sc-steps'); st.innerHTML = '';
        for (const x of s.steps || []) {
          const d = document.createElement('div'); d.className = x.kind;
          d.textContent = `${String(x.t).padStart(6)}  ${x.kind === 'pass' ? '✓' : x.kind === 'fail' ? '✗' : x.kind === 'skip' ? 'SKIP' : x.kind === 'ask' ? '?' : '·'} ${x.text}`;
          st.appendChild(d);
        }
        st.scrollTop = st.scrollHeight;
        const p = $('.sc-prompt');
        if (s.prompt && s.state === 'running' && !s._replay) {
          p.style.display = '';
          p.innerHTML = '<div class="q" style="margin-bottom:4px"></div>';
          p.querySelector('.q').textContent = s.prompt.q;
          const ans = v => Bench.api('scenarios/answer', {value: v});
          if (s.prompt.kind === 'number') {
            const i = Object.assign(document.createElement('input'), {type: 'number', step: 'any', style: 'width:100px'});
            const b = Object.assign(document.createElement('button'), {textContent: 'enter'});
            b.onclick = () => ans(i.value); i.onkeydown = e => { if (e.key === 'Enter') ans(i.value); };
            p.append(i, b); setTimeout(() => i.focus(), 0);
          } else if (s.prompt.kind === 'yesno') {
            for (const [t, v] of [['yes', 'yes'], ['no', 'no']]) {
              const b = Object.assign(document.createElement('button'), {textContent: t}); b.onclick = () => ans(v); p.append(b, ' ');
            }
          } else {
            const b = Object.assign(document.createElement('button'), {textContent: 'OK'}); b.onclick = () => ans('ok'); p.append(b);
          }
        } else p.style.display = 'none';
        if (s.state !== 'running') {
          const r = el.querySelector(`[data-r="${s.name}"]`);
          if (r) { r.textContent = s.state.toUpperCase() + ' (' + s.where + ')'; r.className = 'sc-res ' + (COL[s.state] || ''); }
        }
      });
    }
  });
})();

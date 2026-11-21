'use strict';

function renderTelemetry(room) {
    const live = room.publisher_live && room.motion_live;
    const rate = Number.isFinite(room.motion_hz) ? ' · ' + room.motion_hz.toFixed(1) + ' Hz input' : '';
    const hasSample = Array.isArray(room.accel_m_s2) || Array.isArray(room.gyro_rad_s);
    el('imu-state').textContent = live && hasSample ? 'Live' + rate : hasSample ? 'Last sample' : 'Waiting for sample';
    for (const [prefix, values] of [['a', room.accel_m_s2], ['g', room.gyro_rad_s]]) {
        ['x', 'y', 'z'].forEach((axis, i) => {
            el(prefix + axis).textContent = Number.isFinite(values?.[i]) ? values[i].toFixed(2) : '—';
        });
    }

    const rows = document.createDocumentFragment();
    for (const obj of room.objects || []) {
        const row = document.createElement('tr');
        const seen = room.publisher_live && room.tracking_live && obj.currently_seen;
        const state = seen ? 'In view' : obj.restored && obj.age_s == null ? 'Saved' : 'Remembered';
        const xyz = [0, 1, 2].map(i => Number.isFinite(obj.position?.[i]) ? obj.position[i].toFixed(2) : '—');
        const confidence = Number.isFinite(obj.confidence) ? Math.round(100 * obj.confidence) + '%' : '—';
        const name = obj.label + (obj.id != null ? ' · ' + obj.id : '');
        for (const text of [name, ...xyz, confidence, state]) {
            const cell = document.createElement('td');
            cell.textContent = text;
            row.append(cell);
        }
        rows.append(row);
    }
    if (!rows.childNodes.length) {
        const row = document.createElement('tr'), cell = document.createElement('td');
        cell.colSpan = 6;
        cell.textContent = 'No mapped objects';
        row.append(cell);
        rows.append(row);
    }
    el('objects').replaceChildren(rows);
}

// Pan is measured in meters along the view plane. Zoom about the gesture centroid.
function bindMap(canvas, pan, getSpan, setSpan, minSpan, maxSpan, draw, orbit) {
    const pointers = new Map();
    let frame = 0, safariScale = null;
    const repaint = () => {
        if (!frame) frame = requestAnimationFrame(() => { frame = 0; draw(); });
    };
    const pixelsPerMeter = () => Math.min(canvas.clientWidth, canvas.clientHeight) / getSpan();
    function move(dx, dy) {
        const scale = pixelsPerMeter();
        pan.x += dx / scale;
        pan.y += dy / scale;
        repaint();
    }
    function zoomBy(factor, anchor) {
        if (!(factor > 0) || !Number.isFinite(factor)) return;
        const oldScale = pixelsPerMeter();
        setSpan(Math.max(minSpan, Math.min(maxSpan, getSpan() / factor)));
        if (anchor) {
            const rect = canvas.getBoundingClientRect();
            const change = 1 / pixelsPerMeter() - 1 / oldScale;
            pan.x += (anchor.x - rect.left - rect.width / 2) * change;
            pan.y += (anchor.y - rect.top - rect.height / 2) * change;
        }
        repaint();
    }
    const point = e => ({x:e.clientX, y:e.clientY});
    function pair() {
        const [a,b] = pointers.values();
        return {x:(a.x+b.x)/2, y:(a.y+b.y)/2, dist:Math.hypot(a.x-b.x,a.y-b.y)};
    }
    canvas.addEventListener('pointerdown', e => {
        if (e.button !== 0) return;
        pointers.set(e.pointerId, point(e));
        canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener('pointermove', e => {
        const previous = pointers.get(e.pointerId);
        if (!previous) return;
        const before = pointers.size === 2 ? pair() : null;
        pointers.set(e.pointerId, point(e));
        if (before) {
            const after = pair();
            move(after.x-before.x, after.y-before.y);
            if (before.dist > 0) zoomBy(after.dist/before.dist, after);
        } else if (pointers.size === 1) {
            const dx = e.clientX-previous.x, dy = e.clientY-previous.y;
            if (orbit) { orbit(dx,dy); repaint(); }
            else move(dx,dy);
        }
    });
    for (const event of ['pointerup','pointercancel','lostpointercapture']) {
        canvas.addEventListener(event, e => pointers.delete(e.pointerId));
    }
    canvas.addEventListener('wheel', e => {
        e.preventDefault();
        if (safariScale !== null || pointers.size >= 2) return;
        const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? canvas.clientHeight : 1;
        if (e.ctrlKey) zoomBy(Math.exp(-e.deltaY * unit * .01), point(e));
        else move(-e.deltaX * unit, -e.deltaY * unit);
    }, {passive:false});
    // Safari reports Mac trackpad pinch as GestureEvents.
    canvas.addEventListener('gesturestart', e => {
        e.preventDefault();
        safariScale = e.scale || 1;
    }, {passive:false});
    canvas.addEventListener('gesturechange', e => {
        e.preventDefault();
        if (safariScale !== null && pointers.size < 2 && e.scale > 0) {
            const anchor = Number.isFinite(e.clientX) && Number.isFinite(e.clientY) ? point(e) : null;
            zoomBy(e.scale / safariScale, anchor);
        }
        safariScale = e.scale;
    }, {passive:false});
    canvas.addEventListener('gestureend', e => {
        e.preventDefault();
        safariScale = null;
    }, {passive:false});
    return zoomBy;
}

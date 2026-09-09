/* ══════════════════════════════════════════════════════════════════════════
   widgets.js — the instrument primitives.

   Every widget here follows one rule: it must be readable from the position
   of its pointer alone, before any digit is read. The digits are for the log
   and for the screenshot; the geometry is for the operator.
   ══════════════════════════════════════════════════════════════════════════ */
(function (W) {
  'use strict';

  var SVGNS = 'http://www.w3.org/2000/svg';

  function el(tag, cls, txt) {
    var n = document.createElement(tag);
    if (cls) n.className = cls;
    if (txt != null) n.textContent = txt;
    return n;
  }

  /* Format a number for an instrument face.
     null / undefined / NaN render as an em-dash, NEVER as 0. A zero that means
     "no data" is how you drive into something you were told was 3 m away. */
  function fmt(v, dp) {
    if (v === null || v === undefined || v === '' || (typeof v === 'number' && !isFinite(v))) {
      return (typeof v === 'number' && v === Infinity) ? '∞' : '—';
    }
    var n = Number(v);
    if (!isFinite(n)) return '—';
    return n.toFixed(dp === undefined ? 2 : dp);
  }

  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  /* ─────────────────────────── sparkline ─────────────────────────── */
  /* A short history strip. Not a chart — no axes, no gridlines. Its only job
     is to answer "is this settling or running away?" in peripheral vision. */
  function Spark(w, h, colour) {
    var svg = document.createElementNS(SVGNS, 'svg');
    svg.setAttribute('width', w); svg.setAttribute('height', h);
    svg.style.display = 'block';
    var line = document.createElementNS(SVGNS, 'polyline');
    line.setAttribute('fill', 'none');
    line.setAttribute('stroke', colour || 'var(--t2)');
    line.setAttribute('stroke-width', '1.1');
    var dot = document.createElementNS(SVGNS, 'circle');
    dot.setAttribute('r', '1.6'); dot.setAttribute('fill', colour || 'var(--t2)');
    svg.appendChild(line); svg.appendChild(dot);

    return {
      node: svg,
      draw: function (vals) {
        var v = vals.filter(function (x) { return typeof x === 'number' && isFinite(x); });
        if (v.length < 2) { line.setAttribute('points', ''); dot.setAttribute('r', '0'); return; }
        var mn = Math.min.apply(null, v), mx = Math.max.apply(null, v);
        var rng = (mx - mn) || 1, pts = [], lastY = 0;
        for (var i = 0; i < v.length; i++) {
          var x = i * w / (v.length - 1);
          var y = h - (v[i] - mn) / rng * (h - 3) - 1.5;
          pts.push(x.toFixed(1) + ',' + y.toFixed(1));
          lastY = y;
        }
        line.setAttribute('points', pts.join(' '));
        dot.setAttribute('r', '1.6');
        dot.setAttribute('cx', (w - 0.8).toFixed(1));
        dot.setAttribute('cy', lastY.toFixed(1));
      }
    };
  }

  /* ──────────────────── moving analog indicator ──────────────────── */
  /* opts: {label, unit, dp, min, max, band:[lo,hi], ticks:[..], invert,
            limit: fn(value) -> ''|'caution'|'alarm'} */
  function MAI(opts) {
    var root  = el('div', 'mai');
    var top   = el('div', 'maitop');
    var lab   = el('span', 'lbl', opts.label);
    var val   = el('span', 'num v');
    var unit  = el('span', 'u', ' ' + (opts.unit || ''));
    val.textContent = '—'; val.appendChild(unit);
    top.appendChild(lab); top.appendChild(val);

    var row = el('div', 'row');
    var bar = el('div', 'bar');
    if (opts.band && opts.band[1] > opts.band[0]) {
      var b = el('div', 'band');
      b.style.left = opts.band[0] + '%';
      b.style.width = (opts.band[1] - opts.band[0]) + '%';
      bar.appendChild(b);
    }
    (opts.ticks || []).forEach(function (p) {
      var t = el('div', 'tick'); t.style.left = p + '%'; bar.appendChild(t);
    });
    var ptr = el('div', 'ptr'); ptr.style.left = '50%'; ptr.style.opacity = '.25';
    bar.appendChild(ptr);

    var sparkBox = el('div', 'spark');
    var sp = Spark(64, 17, 'var(--t3)');
    sparkBox.appendChild(sp.node);
    row.appendChild(bar); row.appendChild(sparkBox);
    root.appendChild(top); root.appendChild(row);

    var hist = [], HIST = 40;

    return {
      node: root,
      set: function (v) {
        var ok = (typeof v === 'number' && isFinite(v));
        val.firstChild.nodeValue = fmt(v, opts.dp);
        if (!ok) { ptr.style.opacity = '.25'; return; }
        ptr.style.opacity = '1';
        var pct = clamp((v - opts.min) / ((opts.max - opts.min) || 1), 0, 1) * 100;
        ptr.style.left = (opts.invert ? 100 - pct : pct) + '%';

        var lim = opts.limit ? opts.limit(v) : '';
        var col = lim === 'alarm' ? 'var(--alarm)' : (lim === 'caution' ? 'var(--caution)' : 'var(--t1)');
        val.style.color = col; ptr.style.background = col;
        sp.node.querySelector('polyline').setAttribute('stroke', lim ? col : 'var(--t3)');
        sp.node.querySelector('circle').setAttribute('fill', lim ? col : 'var(--t3)');

        hist.push(v); if (hist.length > HIST) hist.shift();
        sp.draw(hist);
      }
    };
  }

  /* ───────────────────────────── tape ───────────────────────────── */
  /* Vertical scale with the value's own limits painted on its edge, so
     "too fast" is a PLACE, not a comparison you have to perform. Carries two
     markers on purpose: white = actual, magenta = commanded. */
  function Tape(host, opts) {
    var inner = host.querySelector('.inner');
    var read  = host.querySelector('.read .num');
    inner.textContent = '';

    (opts.bands || []).forEach(function (b) {
      var d = el('div', 'bandmark');
      d.style.top = b[0] + '%'; d.style.height = (b[1] - b[0]) + '%';
      d.style.background = b[2];
      inner.appendChild(d);
    });
    (opts.ticks || []).forEach(function (t) {
      var g = el('div', 'gridline'); g.style.top = t[0] + '%'; inner.appendChild(g);
      var l = el('div', 'gridlbl', t[1]); l.style.top = t[0] + '%'; inner.appendChild(l);
    });
    var act = el('div', 'act'); act.style.top = '50%'; act.style.opacity = '.25';
    var cmd = el('div', 'cmd'); cmd.style.top = '50%'; cmd.style.opacity = '.25';
    inner.appendChild(act); inner.appendChild(cmd);

    /* top of the tape is opts.max, bottom is opts.min */
    function pct(v) { return clamp(1 - (v - opts.min) / ((opts.max - opts.min) || 1), 0, 1) * 100; }

    return {
      set: function (actual, commanded) {
        var okA = typeof actual === 'number' && isFinite(actual);
        var okC = typeof commanded === 'number' && isFinite(commanded);
        act.style.opacity = okA ? '1' : '.25';
        cmd.style.opacity = okC ? '1' : '.25';
        if (okA) act.style.top = pct(actual) + '%';
        if (okC) cmd.style.top = pct(commanded) + '%';
        /* show the actual when it exists; fall back to the commanded value
           rather than a dash, but keep it in the commanded colour so the
           reader is never in doubt about which number they are looking at */
        var shown = okA ? actual : commanded;
        read.textContent = fmt(shown, opts.dp);
        read.style.color = okA ? '' : (okC ? 'var(--cmd)' : '');
      }
    };
  }

  /* ───────────────────────────── chips ───────────────────────────── */
  function Chip(name) {
    var n = el('div', 'chip');
    var t = document.createTextNode(name + ' ');
    var b = el('b', null, '—');
    var u = el('span', 'u');
    n.appendChild(t); n.appendChild(b); n.appendChild(u);
    return {
      node: n,
      set: function (value, unit, level) {
        b.textContent = value == null ? '—' : value;
        u.textContent = unit || '';
        n.className = 'chip' + (level ? ' is-' + level : '');
      }
    };
  }

  /* ───────────────────── state lanes (last 60 s) ───────────────────── */
  /* A discrete-signal strip chart. Answers "how long has it been like this?"
     — the question a single indicator light can never answer. */
  function Lanes(host, names) {
    var CELLS = 60, lanes = {};
    names.forEach(function (nm) {
      var row = el('div', 'lane');
      row.appendChild(el('span', 'lbl lbl-d', nm));
      var strip = el('div', 'strip'), cells = [];
      for (var i = 0; i < CELLS; i++) {
        var c = el('i'); c.style.flex = '1'; strip.appendChild(c); cells.push(c);
      }
      row.appendChild(strip); host.appendChild(row);
      lanes[nm] = cells;
    });
    return {
      /* values: {NAME: ''|'go'|'caut'|'alarm'} — shifts one cell left per call */
      push: function (values) {
        Object.keys(lanes).forEach(function (nm) {
          var cells = lanes[nm];
          for (var i = 0; i < cells.length - 1; i++) cells[i].className = cells[i + 1].className;
          cells[cells.length - 1].className = values[nm] || '';
        });
      }
    };
  }

  /* ───────────────────────── event log ───────────────────────── */
  function EventLog(host, cap) {
    var rows = [];
    var COL = { CAUTION: 'var(--caution)', ALARM: 'var(--alarm)', REC: 'var(--go)',
                ESTOP: 'var(--alarm)', MODE: 'var(--t3)', NAV: 'var(--t3)',
                PREARM: 'var(--t3)', LINK: 'var(--caution)' };
    return {
      add: function (met, level, msg) {
        var r = el('div', 'ev');
        r.appendChild(el('span', 't num', met));
        var lv = el('span', 'lbl lv', level);
        lv.style.color = COL[level] || 'var(--t3)';
        r.appendChild(lv);
        r.appendChild(el('span', 'm', msg));
        host.insertBefore(r, host.firstChild);
        rows.push(r);
        while (rows.length > (cap || 60)) { host.removeChild(rows.shift()); }
      }
    };
  }

  W.UI = { el: el, fmt: fmt, clamp: clamp, Spark: Spark, MAI: MAI, Tape: Tape,
           Chip: Chip, Lanes: Lanes, EventLog: EventLog, SVGNS: SVGNS };
})(window);

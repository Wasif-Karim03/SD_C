/* ══════════════════════════════════════════════════════════════════════════
   radar.js — DRIVE scene. Top-down, TRACK-UP (the car's nose is always up).

   Track-up rather than north-up on purpose: the operator is steering, not
   navigating. A turn should rotate the world, not the vehicle symbol — that
   is the mapping the hand already has.

   Frame: the server hands us points as [forward_m, lateral_m] already
   re-referenced to the nose using LIDAR_FORWARD_DEG. Screen x grows to the
   right of the car, screen y grows toward the top (forward).
   ══════════════════════════════════════════════════════════════════════════ */
(function (W) {
  'use strict';

  function css(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }

  function Radar(canvas) {
    var ctx = canvas.getContext('2d');
    var dpr = 1, w = 0, h = 0;
    var scale = 46;                 /* px per metre — set by fit() */
    var RANGE_M = 4.5;              /* what the ring set spans */

    /* lateral sign. The scanner's rotation direction decides whether +lateral
       is the car's left or its right; it is a one-bit fact about the hardware
       and it is wrong to guess it silently. Flip with ?mirror=1 and note it. */
    var mirror = /[?&]mirror=1/.test(location.search) ? -1 : 1;

    function fit() {
      dpr = Math.min(window.devicePixelRatio || 1, 2);
      var r = canvas.parentNode.getBoundingClientRect();
      w = Math.max(1, Math.round(r.width)); h = Math.max(1, Math.round(r.height));
      canvas.width = w * dpr; canvas.height = h * dpr;
      canvas.style.width = w + 'px'; canvas.style.height = h + 'px';
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      /* fit RANGE_M into the shorter half-dimension, leaving room for the rose */
      scale = Math.min(w, h) * 0.5 * 0.86 / RANGE_M;
    }

    /* the car sits below centre so more of the road ahead is visible —
       the same reason a driver's eye height is not the middle of the windscreen */
    function cx() { return w * 0.5; }
    function cy() { return h * 0.62; }
    function X(fwd, lat) { return cx() + mirror * lat * scale; }
    function Y(fwd, lat) { return cy() - fwd * scale; }

    /* Both grounds have to work. On a light ground the same low-alpha washes
       vanish and the same greys go muddy, so the scene asks which ground it is
       on rather than assuming the dark one. */
    function isDay() { return document.documentElement.dataset.hmi === 'day'; }
    function hair(a) { return isDay() ? 'rgba(40,48,54,' + a + ')' : 'rgba(150,160,170,' + a + ')'; }

    var C = {};
    function readColours() {
      C = { t1: css('--t1'), t2: css('--t2'), t3: css('--t3'), t4: css('--t4'),
            pts: css('--pts'), cmd: css('--cmd'), ref: css('--ref'),
            caution: css('--caution'), alarm: css('--alarm'), well: css('--well') };
    }

    function ring() {
      ctx.save();
      /* range rings, every metre, dashed so they never read as an obstacle */
      ctx.setLineDash([1, 5]); ctx.lineWidth = 1;
      ctx.strokeStyle = hair(.20);
      ctx.fillStyle = C.t4;
      ctx.font = '8.5px "IBM Plex Mono", monospace';
      for (var r = 1; r <= Math.ceil(RANGE_M); r++) {
        ctx.beginPath(); ctx.arc(cx(), cy(), r * scale, 0, Math.PI * 2); ctx.stroke();
        ctx.fillText(r + ' m', cx() + 5, cy() - r * scale + 10);
      }
      ctx.setLineDash([]);
      ctx.restore();
    }

    function rose() {
      var R = Math.min(w, h) * 0.5 * 0.96, RI = R - 8;
      ctx.save();
      ctx.font = '8.5px "IBM Plex Mono", monospace';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (var d = 0; d < 360; d += 10) {
        var maj = (d % 30) === 0;
        var a = (d - 90) * Math.PI / 180;
        var r1 = maj ? R : R - 3;
        ctx.beginPath();
        ctx.strokeStyle = maj ? hair(.36) : hair(.18);
        ctx.lineWidth = 1;
        ctx.moveTo(cx() + Math.cos(a) * RI, cy() + Math.sin(a) * RI);
        ctx.lineTo(cx() + Math.cos(a) * r1, cy() + Math.sin(a) * r1);
        ctx.stroke();
        if (maj) {
          ctx.fillStyle = C.t4;
          var lr = R + 11;
          ctx.fillText(('00' + d).slice(-3),
                       cx() + Math.cos(a) * lr, cy() + Math.sin(a) * lr);
        }
      }
      ctx.beginPath(); ctx.arc(cx(), cy(), RI, 0, Math.PI * 2);
      ctx.strokeStyle = hair(.14); ctx.stroke();
      ctx.restore();
    }

    /* The costmap, drawn the way the planner builds it: an inflation skirt
       around every lethal return. Five decreasing-alpha passes approximate the
       exponential decay. Magenta core = lethal (the planner will never route
       through it); the cyan halo is the inscribed radius. */
    /* widths are in METRES, not pixels: the inflation skirt is a physical
       distance around an obstacle, so it has to grow and shrink with the
       zoom exactly the way the obstacle does. */
    var INFL = [[0.38, 'rgba(150,32,54,.030)'],
                [0.24, 'rgba(180,70,40,.038)'],
                [0.12, 'rgba(205,140,38,.046)']];

    function costmap(pts) {
      if (!pts.length) return;
      ctx.save(); ctx.lineCap = 'round'; ctx.lineJoin = 'round';
      var boost = isDay() ? 2.4 : 1.0;      /* washes need more weight on light */
      for (var k = 0; k < INFL.length; k++) {
        ctx.strokeStyle = INFL[k][1].replace(/,(\.\d+)\)$/, function (_, a) {
          return ',' + Math.min(0.5, parseFloat(a) * boost).toFixed(3) + ')';
        });
        ctx.lineWidth = Math.max(2, INFL[k][0] * scale);
        ctx.beginPath();
        for (var i = 0; i < pts.length; i++) {
          var x = X(pts[i][0], pts[i][1]), y = Y(pts[i][0], pts[i][1]);
          if (i === 0 || !near(pts[i], pts[i - 1])) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
      }
      ctx.restore();
    }
    /* two consecutive returns belong to the same surface only if they are
       close in range — otherwise the polyline leaps across a doorway and
       invents a wall that is not there */
    function near(a, b) {
      var dx = a[0] - b[0], dy = a[1] - b[1];
      return (dx * dx + dy * dy) < 0.09;      /* 30 cm */
    }

    function points(pts) {
      ctx.save();
      ctx.fillStyle = C.pts;
      for (var i = 0; i < pts.length; i++) {
        var d = Math.hypot(pts[i][0], pts[i][1]);
        ctx.globalAlpha = Math.max(0.28, 0.92 - d * 0.09);
        var x = X(pts[i][0], pts[i][1]), y = Y(pts[i][0], pts[i][1]);
        ctx.beginPath(); ctx.arc(x, y, 1.35, 0, Math.PI * 2); ctx.fill();
      }
      ctx.restore();
    }

    /* The reaction cone — the wedge the LiDAR navigator actually watches for
       "nearest ahead". Drawing it stops the operator wondering why the car
       ignored something 80 degrees off the nose. */
    function cone(halfDeg, reactM) {
      ctx.save();
      var a0 = (-90 - halfDeg) * Math.PI / 180, a1 = (-90 + halfDeg) * Math.PI / 180;
      ctx.beginPath(); ctx.moveTo(cx(), cy());
      ctx.arc(cx(), cy(), reactM * scale, a0, a1);
      ctx.closePath();
      ctx.fillStyle = 'rgba(63,182,200,.035)'; ctx.fill();
      ctx.setLineDash([2, 5]); ctx.strokeStyle = 'rgba(63,182,200,.20)';
      ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);
      ctx.restore();
    }

    function poly(pathM, style, width, dash) {
      if (!pathM || pathM.length < 2) return;
      ctx.save();
      ctx.beginPath();
      for (var i = 0; i < pathM.length; i++) {
        var x = X(pathM[i][0], pathM[i][1]), y = Y(pathM[i][0], pathM[i][1]);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      }
      ctx.strokeStyle = style; ctx.lineWidth = width || 1.4;
      if (dash) ctx.setLineDash(dash);
      ctx.stroke();
      ctx.restore();
    }

    /* The predicted arc: where the wheels are pointed RIGHT NOW, integrated
       forward at constant curvature. R = L / tan(delta), the bicycle model.
       This is a prediction from two calibration constants, one of which
       (MAX_STEER_ANGLE_RAD) is still an estimate — which is why the DIAGNOSE
       screen marks it, and why this arc is drawn thin. */
    function arcFromSteer(steerNorm, wheelbase, maxSteer, horizonM) {
      var d = steerNorm * maxSteer;
      var out = [], n = 26;
      if (Math.abs(d) < 1e-3) {
        for (var i = 0; i <= n; i++) out.push([horizonM * i / n, 0]);
        return out;
      }
      var R = wheelbase / Math.tan(d);
      var th = horizonM / R;
      for (var j = 0; j <= n; j++) {
        var t = th * j / n;
        out.push([R * Math.sin(t), R * (1 - Math.cos(t))]);
      }
      return out;
    }

    function car() {
      ctx.save();
      ctx.translate(cx(), cy());
      var L = 0.33 * scale, Wd = 0.20 * scale;   /* Slash 4x4, roughly */
      /* headlight wedge — the camera's horizontal field, so the operator can
         see what the detector could possibly have seen */
      ctx.beginPath();
      ctx.moveTo(0, -L * 0.5);
      ctx.lineTo(-2.6 * scale * Math.tan(0.48), -2.6 * scale);
      ctx.lineTo(2.6 * scale * Math.tan(0.48), -2.6 * scale);
      ctx.closePath();
      ctx.fillStyle = isDay() ? 'rgba(0,0,0,.035)' : 'rgba(255,255,255,.028)'; ctx.fill();
      ctx.setLineDash([2, 4]); ctx.strokeStyle = hair(.14);
      ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);

      ctx.fillStyle = 'rgba(231,234,237,.13)';
      ctx.strokeStyle = C.t1; ctx.lineWidth = 1.3;
      ctx.beginPath(); ctx.rect(-Wd / 2, -L / 2, Wd, L); ctx.fill(); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, -L / 2); ctx.lineTo(0, -L / 2 - 9); ctx.stroke();
      ctx.restore();
    }

    function vignette() {
      var g = ctx.createRadialGradient(w / 2, h / 2, Math.min(w, h) * 0.30,
                                       w / 2, h / 2, Math.max(w, h) * 0.62);
      g.addColorStop(0, 'rgba(0,0,0,0)');
      g.addColorStop(1, document.documentElement.dataset.hmi === 'day'
                        ? 'rgba(0,0,0,.12)' : 'rgba(0,0,0,.40)');
      ctx.fillStyle = g; ctx.fillRect(0, 0, w, h);
      /* corner brackets — a frame that says "this is an instrument, and this
         is its edge", so a clipped scene is never mistaken for empty space */
      ctx.strokeStyle = hair(.26); ctx.lineWidth = 1;
      [[10, 22, 10, 10, 22, 10], [w - 22, 10, w - 10, 10, w - 10, 22],
       [10, h - 22, 10, h - 10, 22, h - 10], [w - 22, h - 10, w - 10, h - 10, w - 10, h - 22]]
        .forEach(function (p) {
          ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(p[2], p[3]); ctx.lineTo(p[4], p[5]); ctx.stroke();
        });
    }

    return {
      fit: fit,
      scaleFor: function () { return scale; },
      /* d: {pts, trail, route, goal, lookahead, steer, cal, react, coneDeg} */
      draw: function (d) {
        if (!w || !h) fit();
        readColours();
        ctx.clearRect(0, 0, w, h);
        ring(); rose();
        cone(d.coneDeg || 22, d.react || 1.3);
        costmap(d.pts || []);
        points(d.pts || []);

        /* where we HAVE been — faint, dotted, no colour: it is history */
        poly(d.trail, isDay() ? 'rgba(30,34,36,.30)' : 'rgba(231,234,237,.26)', 1.2, [1, 3]);
        /* the planner's route — a reference, so cyan, dashed */
        poly(d.route, C.ref, 1.4, [6, 4]);
        /* what the wheels will actually do — white, solid, the actual */
        poly(arcFromSteer(d.steer || 0, d.cal.wheelbase, d.cal.maxSteer, 2.2),
             C.t1, 2.0);

        if (d.lookahead) {
          var lx = X(d.lookahead[0], d.lookahead[1]), ly = Y(d.lookahead[0], d.lookahead[1]);
          ctx.save();
          ctx.strokeStyle = C.cmd; ctx.lineWidth = 1.6;
          ctx.beginPath(); ctx.arc(lx, ly, 6, 0, Math.PI * 2); ctx.stroke();
          ctx.fillStyle = C.cmd;
          ctx.beginPath(); ctx.arc(lx, ly, 1.8, 0, Math.PI * 2); ctx.fill();
          ctx.setLineDash([2, 2]); ctx.strokeStyle = 'rgba(199,74,199,.45)'; ctx.lineWidth = 1;
          ctx.beginPath(); ctx.moveTo(lx, ly); ctx.lineTo(lx + 14, ly - 16); ctx.stroke();
          ctx.setLineDash([]);
          ctx.font = '9px "IBM Plex Sans", sans-serif';
          ctx.fillText('LOOKAHEAD ' + (d.cal.lookahead || 0).toFixed(2) + ' m', lx + 18, ly - 19);
          ctx.restore();
        }
        if (d.goal) {
          var gx = X(d.goal[0], d.goal[1]), gy = Y(d.goal[0], d.goal[1]);
          ctx.save();
          ctx.strokeStyle = C.ref; ctx.lineWidth = 1.3;
          ctx.beginPath(); ctx.arc(gx, gy, 8, 0, Math.PI * 2); ctx.stroke();
          ctx.beginPath(); ctx.arc(gx, gy, 2, 0, Math.PI * 2); ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(gx - 13, gy); ctx.lineTo(gx + 13, gy);
          ctx.moveTo(gx, gy - 13); ctx.lineTo(gx, gy + 13); ctx.stroke();
          ctx.fillStyle = C.ref; ctx.font = '9px "IBM Plex Sans", sans-serif';
          ctx.fillText('GOAL · ' + (d.goalDist == null ? '—' : d.goalDist.toFixed(2) + ' m'),
                       gx + 15, gy - 11);
          ctx.restore();
        }
        car();
        vignette();
      },
      /* screen px -> metres in the car frame, for click-to-set-goal */
      unproject: function (px, py) {
        return [(cy() - py) / scale, mirror * (px - cx()) / scale];
      }
    };
  }

  W.Radar = Radar;
})(window);

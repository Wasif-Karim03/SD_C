/* ══════════════════════════════════════════════════════════════════════════
   vision.js — the chase-camera scene.

   HONESTY NOTE, and it is the most important comment in this file:
   the RPLIDAR C1 sees ONE horizontal plane. What this view draws is that
   plane extruded upward to a nominal height so that walls read as walls.
   It is NOT a 3D reconstruction, and nothing in it knows how tall anything
   actually is. A kerb and a wall look the same here. The screen says so, in
   the corner, permanently — a view that flatters the sensor is how an
   operator ends up trusting a gap the car cannot fit through.

   World frame: X right, Y forward, Z up. Metres.
   ══════════════════════════════════════════════════════════════════════════ */
(function (W) {
  'use strict';

  var EYE    = [0, -2.30, 1.32];      /* chase camera: behind and above */
  var TARGET = [0,  3.20, 0.15];
  var FOV    = 62 * Math.PI / 180;
  var EXTRUDE = 0.75;                  /* nominal wall height, m — see note above */
  var NEAR = 0.35;

  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }
  function norm(a) { var m = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / m, a[1] / m, a[2] / m]; }

  function css(n) { return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }
  function isDay() { return document.documentElement.dataset.hmi === 'day'; }

  function Vision(canvas) {
    var ctx = canvas.getContext('2d');
    var w = 0, h = 0, dpr = 1, focal = 0;
    var f = norm(sub(TARGET, EYE));
    var r = norm(cross(f, [0, 0, 1]));
    var u = cross(r, f);
    var mirror = /[?&]mirror=1/.test(location.search) ? -1 : 1;

    function fit() {
      dpr = Math.min(window.devicePixelRatio || 1, 2);
      var b = canvas.parentNode.getBoundingClientRect();
      w = Math.max(1, Math.round(b.width)); h = Math.max(1, Math.round(b.height));
      canvas.width = w * dpr; canvas.height = h * dpr;
      canvas.style.width = w + 'px'; canvas.style.height = h + 'px';
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      focal = (w / 2) / Math.tan(FOV / 2);
    }

    /* returns [sx, sy, depth] or null when behind the near plane */
    function P(x, y, z) {
      var v = sub([x, y, z], EYE);
      var zc = dot(v, f);
      if (zc < NEAR) return null;
      return [w / 2 + focal * dot(v, r) / zc, h / 2 - focal * dot(v, u) / zc, zc];
    }
    /* a scan point [forward, lateral] -> world */
    function wp(p, z) { return P(mirror * p[1], p[0], z || 0); }

    function sky() {
      var g = ctx.createLinearGradient(0, 0, 0, h);
      g.addColorStop(0, css('--scene-sky-a'));
      g.addColorStop(0.52, css('--well'));
      g.addColorStop(1, css('--scene-sky-b'));
      ctx.fillStyle = g; ctx.fillRect(0, 0, w, h);
    }

    /* Ground grid, 0.5 m. Its convergence is the only depth cue that does not
       depend on the sensor being right — it is the ruler the rest is read against. */
    function grid() {
      ctx.save();
      ctx.strokeStyle = isDay() ? 'rgba(40,48,54,.14)' : 'rgba(150,160,170,.09)'; ctx.lineWidth = 1;
      var i, a, b;
      for (i = -12; i <= 12; i++) {
        a = P(i * 0.5, 0.2, 0); b = P(i * 0.5, 9.0, 0);
        if (a && b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
      }
      for (i = 0; i <= 18; i++) {
        a = P(-6, 0.2 + i * 0.5, 0); b = P(6, 0.2 + i * 0.5, 0);
        if (a && b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
      }
      ctx.restore();
    }

    function samesurface(a, b) {
      var dx = a[0] - b[0], dy = a[1] - b[1];
      return (dx * dx + dy * dy) < 0.09;
    }

    /* Extruded scan. Far-to-near painter's order, alpha falling with depth so
       the near geometry — the geometry that can actually hit you — is the
       geometry that reads strongest. */
    /* Extruded scan, drawn as CONTIGUOUS RUNS rather than one quad per pair
       of returns. Per-pair quads put a stroke between every adjacent sample,
       which turns a smooth wall into a picket fence — 500 vertical lines that
       look like structure and are not. A run is one polygon with one outline:
       the surface the scanner actually traced. */
    function runs(pts) {
      var out = [], cur = [];
      for (var i = 0; i < pts.length; i++) {
        if (cur.length && !samesurface(pts[i], cur[cur.length - 1])) {
          if (cur.length > 1) out.push(cur);
          cur = [];
        }
        cur.push(pts[i]);
      }
      if (cur.length > 1) out.push(cur);
      return out;
    }

    function walls(pts, obst) {
      var segs = runs(pts), shapes = [], i, k;
      for (i = 0; i < segs.length; i++) {
        var run = segs[i], bot = [], top = [], depth = 0, isObst = false, n = 0;
        for (k = 0; k < run.length; k++) {
          var b = wp(run[k], 0), t = wp(run[k], EXTRUDE);
          if (!b || !t) continue;
          bot.push(b); top.push(t); depth += b[2]; n++;
          if (obst && Math.abs(run[k][0] - obst.fwd) < obst.d / 2 + 0.18 &&
                      Math.abs(run[k][1] - obst.lat) < obst.w / 2 + 0.10) isObst = true;
        }
        if (bot.length < 2) continue;
        shapes.push({ bot: bot, top: top, d: depth / n, obst: isObst });
      }
      shapes.sort(function (a, b) { return b.d - a.d; });   /* painter's order */

      for (i = 0; i < shapes.length; i++) {
        var sh = shapes[i];
        ctx.beginPath();
        ctx.moveTo(sh.bot[0][0], sh.bot[0][1]);
        for (k = 1; k < sh.bot.length; k++) ctx.lineTo(sh.bot[k][0], sh.bot[k][1]);
        for (k = sh.top.length - 1; k >= 0; k--) ctx.lineTo(sh.top[k][0], sh.top[k][1]);
        ctx.closePath();
        if (sh.obst) {
          ctx.fillStyle = 'rgba(199,74,199,.16)';
          ctx.strokeStyle = css('--cmd'); ctx.lineWidth = 1.3;
        } else {
          var a = Math.max(0.05, Math.min(0.30, 0.40 - sh.d * 0.032));
          var base = isDay() ? '58,78,92' : '120,150,168';
          ctx.fillStyle = 'rgba(' + base + ',' + (isDay() ? a * 1.6 : a).toFixed(3) + ')';
          ctx.strokeStyle = 'rgba(' + base + ',' + Math.min(0.60, a + 0.24).toFixed(3) + ')';
          ctx.lineWidth = 1;
        }
        ctx.fill(); ctx.stroke();
        /* the top edge, brighter: it is the silhouette the eye reads a wall by */
        ctx.beginPath();
        ctx.moveTo(sh.top[0][0], sh.top[0][1]);
        for (k = 1; k < sh.top.length; k++) ctx.lineTo(sh.top[k][0], sh.top[k][1]);
        ctx.strokeStyle = sh.obst ? css('--cmd')
                        : (isDay() ? 'rgba(40,56,68,.55)' : 'rgba(170,196,210,.42)');
        ctx.lineWidth = 1.1; ctx.stroke();
      }
      return shapes.length;
    }

    /* individual returns as short vertical ticks at the base — the raw
       measurement, kept visible under the interpretation drawn on top of it */
    function ticks(pts) {
      ctx.save(); ctx.strokeStyle = css('--pts'); ctx.lineWidth = 1.1;
      for (var i = 0; i < pts.length; i += 2) {
        /* only nearby returns get a tick. Far ones would be a millimetre of
           hatching along a wall base — texture that carries no information. */
        if (Math.hypot(pts[i][0], pts[i][1]) > 3.0) continue;
        var a = wp(pts[i], 0), b = wp(pts[i], 0.12);
        if (!a || !b) continue;
        ctx.globalAlpha = Math.max(0.16, 0.72 - a[2] * 0.06);
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      }
      ctx.restore();
    }

    /* A path drawn as a ribbon on the ground rather than a line, because a
       line has no width and the car does. The ribbon is the car's track. */
    function ribbon(path, colour, fillTop, halfW) {
      if (!path || path.length < 2) return;
      var L = [], R = [], i;
      for (i = 0; i < path.length; i++) {
        var p = path[i];
        var nx, ny;
        var q = path[Math.min(i + 1, path.length - 1)];
        var o = path[Math.max(i - 1, 0)];
        var tx = q[0] - o[0], ty = q[1] - o[1];
        var m = Math.hypot(tx, ty) || 1;
        nx = -ty / m; ny = tx / m;                     /* normal in (fwd,lat) */
        L.push([p[0] + nx * halfW, p[1] + ny * halfW]);
        R.push([p[0] - nx * halfW, p[1] - ny * halfW]);
      }
      var poly = [], ok = true;
      for (i = 0; i < L.length; i++) { var a = wp(L[i], 0.012); if (!a) { ok = false; break; } poly.push(a); }
      for (i = R.length - 1; i >= 0; i--) { var b = wp(R[i], 0.012); if (!b) { ok = false; break; } poly.push(b); }
      if (!ok || poly.length < 4) return;
      ctx.beginPath();
      ctx.moveTo(poly[0][0], poly[0][1]);
      for (i = 1; i < poly.length; i++) ctx.lineTo(poly[i][0], poly[i][1]);
      ctx.closePath();
      var g = ctx.createLinearGradient(0, h, 0, h * 0.35);
      g.addColorStop(0, fillTop[0]); g.addColorStop(1, fillTop[1]);
      ctx.fillStyle = g; ctx.fill();
      ctx.strokeStyle = colour; ctx.lineWidth = 1.2; ctx.stroke();
    }

    /* the wireframe box the planner is actually inflating around */
    function obstacleBox(c) {
      if (!c) return;
      var x0 = c.lat - c.w / 2, x1 = c.lat + c.w / 2;
      var y0 = c.fwd - c.d / 2, y1 = c.fwd + c.d / 2, z1 = 0.55;
      var v = [[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0],
               [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]
              .map(function (p) { return P(mirror * p[0], p[1], p[2]); });
      if (v.some(function (p) { return !p; })) return;
      var E = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
      ctx.save(); ctx.strokeStyle = css('--cmd'); ctx.lineWidth = 1.2; ctx.globalAlpha = .95;
      E.forEach(function (e) {
        ctx.beginPath(); ctx.moveTo(v[e[0]][0], v[e[0]][1]);
        ctx.lineTo(v[e[1]][0], v[e[1]][1]); ctx.stroke();
      });
      ctx.restore();
      var lab = P(mirror * c.lat, c.fwd - c.d / 2, z1 + 0.30);
      if (lab) {
        ctx.save();
        ctx.fillStyle = css('--cmd'); ctx.font = '9.5px "IBM Plex Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(c.fwd.toFixed(2) + ' m · ' + (c.w * 100).toFixed(0) + ' cm wide', lab[0], lab[1]);
        ctx.restore();
      }
    }

    function vignette() {
      var g = ctx.createRadialGradient(w / 2, h * 0.46, Math.min(w, h) * 0.35,
                                       w / 2, h * 0.46, Math.max(w, h) * 0.66);
      g.addColorStop(0, 'rgba(0,0,0,0)');
      g.addColorStop(1, document.documentElement.dataset.hmi === 'day'
                        ? 'rgba(0,0,0,.14)' : 'rgba(0,0,0,.62)');
      ctx.fillStyle = g; ctx.fillRect(0, 0, w, h);
    }

    /* ── obstacle finder ──────────────────────────────────────────────
       Cluster returns inside the forward corridor and take the nearest
       cluster. This is deliberately the SAME corridor the LiDAR navigator
       uses, so what is boxed here is what the car will actually react to.  */
    function findObstacle(pts, corridorHalfW, maxFwd) {
      var inC = [];
      for (var i = 0; i < pts.length; i++) {
        var fwd = pts[i][0], lat = pts[i][1];
        if (fwd > 0.15 && fwd < maxFwd && Math.abs(lat) < corridorHalfW) inC.push([fwd, lat, i]);
      }
      if (!inC.length) return null;
      inC.sort(function (a, b) { return a[0] - b[0]; });
      var seed = inC[0], grp = [seed];
      for (var k = 1; k < inC.length; k++) {
        if (Math.abs(inC[k][0] - seed[0]) < 0.45) grp.push(inC[k]);
      }
      var lats = grp.map(function (g) { return g[1]; });
      var fwds = grp.map(function (g) { return g[0]; });
      var idx  = grp.map(function (g) { return g[2]; });
      return {
        fwd: fwds.reduce(function (a, b) { return a + b; }, 0) / fwds.length,
        lat: (Math.min.apply(null, lats) + Math.max.apply(null, lats)) / 2,
        w: Math.max(0.12, Math.max.apply(null, lats) - Math.min.apply(null, lats)),
        d: Math.max(0.10, Math.max.apply(null, fwds) - Math.min.apply(null, fwds)),
        near: Math.min.apply(null, fwds),
        idx: [Math.min.apply(null, idx), Math.max.apply(null, idx)]
      };
    }

    function arc(steerNorm, wheelbase, maxSteer, horizonM, n) {
      var d = steerNorm * maxSteer, out = [];
      n = n || 24;
      if (Math.abs(d) < 1e-3) {
        for (var i = 0; i <= n; i++) out.push([horizonM * i / n, 0]);
        return out;
      }
      var R = wheelbase / Math.tan(d), th = horizonM / R;
      for (var j = 0; j <= n; j++) {
        var t = th * j / n;
        out.push([R * Math.sin(t), R * (1 - Math.cos(t))]);
      }
      return out;
    }

    return {
      fit: fit,
      extrude: EXTRUDE,
      findObstacle: findObstacle,
      arc: arc,
      /* d: {pts, planned, steer, cal, react, corridor, speed} */
      draw: function (d) {
        if (!w || !h) fit();
        ctx.clearRect(0, 0, w, h);
        sky(); grid();

        var pts = d.pts || [];
        var obst = findObstacle(pts, d.corridor || 0.32, d.react || 3.0);

        /* PLANNED — cyan. What the planner commanded. */
        if (d.planned && d.planned.length > 1) {
          ribbon(d.planned, css('--ref'),
                 ['rgba(63,182,200,.55)', 'rgba(63,182,200,.06)'], 0.13);
        }
        /* PREDICTED — white. Where the wheels are pointed right now.
           Horizon scales with speed: at 2 s of travel, so the ribbon always
           shows the distance that matters at THIS speed, not a fixed length. */
        var horizon = Math.max(1.2, Math.min(4.0, (d.speed || 0.4) * 2.0 + 0.9));
        var pred = arc(d.steer || 0, d.cal.wheelbase, d.cal.maxSteer, horizon);
        ribbon(pred, isDay() ? 'rgba(22,25,27,.80)' : 'rgba(231,234,237,.75)',
               isDay() ? ['rgba(22,25,27,.26)', 'rgba(22,25,27,.03)']
                       : ['rgba(231,234,237,.30)', 'rgba(231,234,237,.04)'], 0.11);

        walls(pts, obst);
        ticks(pts);
        obstacleBox(obst);
        vignette();

        /* divergence at the 2 s horizon: how far the actual arc ends from the
           commanded route's matching point. This is the tracking error, made
           visible without spending a second chart on it. */
        var div = null;
        if (d.planned && d.planned.length > 1) {
          var end = pred[pred.length - 1];
          var best = Infinity;
          for (var i = 0; i < d.planned.length; i++) {
            var p = d.planned[i];
            var s = Math.hypot(p[0] - end[0], p[1] - end[1]);
            if (s < best) best = s;
          }
          div = best;
        }
        var R = Math.abs((d.steer || 0) * d.cal.maxSteer) < 1e-3
                ? Infinity : d.cal.wheelbase / Math.tan(Math.abs(d.steer * d.cal.maxSteer));
        return { divergence: div, radius: R, obstacle: obst };
      }
    };
  }

  W.Vision = Vision;
})(window);

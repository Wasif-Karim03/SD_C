/* ══════════════════════════════════════════════════════════════════════════
   position.js — where the car is, and how much that claim is worth.

   Two facts shape this whole surface:

   1. The SE100 is a non-RTK receiver. Its horizontal error is metres, not
      centimetres. A dot on a map at that accuracy is a CLAIM, so it is drawn
      with its accuracy circle around it, always. A bare dot would imply a
      precision this hardware does not have.

   2. Indoors there is no degraded fix. There is NO fix. The panel does not
      fade GNSS out gracefully, because the sensor does not: it drops to
      SLAM and says which one it is using.

   The basemap is optional by design. If Leaflet and tiles are reachable the
   real map is shown; if not, the same geometry is drawn on a local ENU grid.
   The car's position is never less legible for want of a network.
   ══════════════════════════════════════════════════════════════════════════ */
(function (W) {
  'use strict';

  function css(n) { return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }

  /* local tangent plane — good to millimetres over the tens of metres a
     1/10-scale car covers, and it needs no projection library */
  function enu(lat, lon, lat0, lon0) {
    var R = 6378137.0, d = Math.PI / 180;
    return [(lon - lon0) * d * R * Math.cos(lat0 * d), (lat - lat0) * d * R];
  }

  function Position(hostFallback, hostLeaflet, srcEl) {
    var cv = hostFallback, ctx = cv.getContext('2d');
    var w = 0, h = 0, dpr = 1;
    var origin = null;              /* first accepted fix — the local frame */
    var track = [];                 /* [e, n] metres */
    var lmap = null, lmarker = null, laccu = null, ltrack = null, ltiles = null;
    var tilesLive = false;
    var mppx = 0.35;                /* metres per pixel in the fallback view */

    function fit() {
      dpr = Math.min(window.devicePixelRatio || 1, 2);
      var b = cv.parentNode.getBoundingClientRect();
      w = Math.max(1, Math.round(b.width)); h = Math.max(1, Math.round(b.height));
      cv.width = w * dpr; cv.height = h * dpr;
      cv.style.width = w + 'px'; cv.style.height = h + 'px';
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      if (lmap) lmap.invalidateSize();
    }

    /* Try the real basemap. OpenStreetMap rather than Google on purpose:
       no API key, no billing account, no per-load quota, and the licence
       permits exactly this use. If the tiles never arrive we fall back
       silently — a map that half-loads is worse than one that is drawn. */
    function tryLeaflet(lat, lon, onStatus) {
      if (lmap || typeof W.L === 'undefined') return;
      try {
        lmap = W.L.map(hostLeaflet, {
          zoomControl: false, attributionControl: true,
          preferCanvas: true, keyboard: false
        }).setView([lat, lon], 19);
        ltiles = W.L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
          maxZoom: 19, attribution: '&copy; OpenStreetMap'
        }).addTo(lmap);
        ltiles.on('tileload', function () {
          if (!tilesLive) { tilesLive = true; cv.style.display = 'none'; onStatus('basemap: OpenStreetMap · z19'); }
        });
        ltiles.on('tileerror', function () {
          if (!tilesLive) onStatus('basemap: offline — local grid');
        });
        ltrack = W.L.polyline([], { color: css('--t1'), weight: 1.6, opacity: .8 }).addTo(lmap);
        laccu  = W.L.circle([lat, lon], { radius: 5, color: css('--ref'), weight: 1,
                                          fillColor: css('--ref'), fillOpacity: .07 }).addTo(lmap);
        lmarker = W.L.circleMarker([lat, lon], { radius: 4, color: css('--t1'), weight: 1.6,
                                                 fillColor: css('--t1'), fillOpacity: 1 }).addTo(lmap);
        setTimeout(function () { if (!tilesLive) onStatus('basemap: offline — local grid'); }, 4000);
      } catch (e) { lmap = null; }
    }

    function gridDraw(fix, headingDeg, waypoints) {
      ctx.clearRect(0, 0, w, h);
      var cx = w / 2, cy = h / 2;

      /* metre grid — the fallback's honesty: it is a ruler, not a place.
         No invented streets, no fake buildings. */
      ctx.save();
      ctx.strokeStyle = 'rgba(150,160,170,.07)'; ctx.lineWidth = 1;
      var step = 5 / mppx, i;                       /* 5 m */
      for (i = -Math.ceil(w / 2 / step); i <= Math.ceil(w / 2 / step); i++) {
        ctx.beginPath(); ctx.moveTo(cx + i * step, 0); ctx.lineTo(cx + i * step, h); ctx.stroke();
      }
      for (i = -Math.ceil(h / 2 / step); i <= Math.ceil(h / 2 / step); i++) {
        ctx.beginPath(); ctx.moveTo(0, cy + i * step); ctx.lineTo(w, cy + i * step); ctx.stroke();
      }
      ctx.restore();

      if (!fix) {
        ctx.save();
        ctx.fillStyle = css('--t4');
        ctx.font = '600 11px "IBM Plex Sans", sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('NO GNSS FIX', cx, cy - 8);
        ctx.font = '10px "IBM Plex Sans", sans-serif';
        ctx.fillText('nothing is being plotted — this is not a stale position', cx, cy + 10);
        ctx.restore();
        scalebar();
        return;
      }

      var last = track.length ? track[track.length - 1] : [0, 0];
      function X(e) { return cx + (e - last[0]) / mppx; }
      function Y(n) { return cy - (n - last[1]) / mppx; }

      /* the track we have actually driven */
      if (track.length > 1) {
        ctx.save();
        ctx.strokeStyle = 'rgba(231,234,237,.55)'; ctx.lineWidth = 1.4;
        ctx.beginPath();
        for (i = 0; i < track.length; i++) {
          var x = X(track[i][0]), y = Y(track[i][1]);
          i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
        }
        ctx.stroke();
        ctx.restore();
      }

      /* waypoints and the leg we are commanded to fly */
      if (waypoints && waypoints.length) {
        ctx.save();
        ctx.strokeStyle = css('--cmd'); ctx.lineWidth = 1.3; ctx.setLineDash([5, 4]);
        ctx.beginPath(); ctx.moveTo(X(last[0]), Y(last[1]));
        waypoints.forEach(function (wpt) { ctx.lineTo(X(wpt[0]), Y(wpt[1])); });
        ctx.stroke(); ctx.setLineDash([]);
        waypoints.forEach(function (wpt, k) {
          ctx.beginPath(); ctx.arc(X(wpt[0]), Y(wpt[1]), 4, 0, Math.PI * 2);
          ctx.strokeStyle = css('--cmd'); ctx.stroke();
          ctx.fillStyle = css('--cmd'); ctx.font = '8.5px "IBM Plex Mono", monospace';
          ctx.fillText(String(k + 1), X(wpt[0]) + 7, Y(wpt[1]) - 5);
        });
        ctx.restore();
      }

      /* accuracy circle — drawn BEFORE the dot so the dot sits inside its
         own uncertainty rather than on top of a decoration */
      var acc = (fix.hdop != null ? fix.hdop * 2.5 : 5.0);
      ctx.save();
      ctx.beginPath(); ctx.arc(X(last[0]), Y(last[1]), Math.max(3, acc / mppx), 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(63,182,200,.07)'; ctx.fill();
      ctx.strokeStyle = 'rgba(63,182,200,.45)'; ctx.lineWidth = 1; ctx.stroke();
      ctx.restore();

      /* heading cone — from course-over-ground, which only exists while
         moving. Standing still it is not drawn, because it is not known. */
      if (headingDeg != null) {
        var a = (headingDeg - 90) * Math.PI / 180, spread = 0.30;
        ctx.save();
        ctx.beginPath(); ctx.moveTo(X(last[0]), Y(last[1]));
        ctx.arc(X(last[0]), Y(last[1]), 46, a - spread, a + spread);
        ctx.closePath();
        ctx.fillStyle = 'rgba(231,234,237,.10)'; ctx.fill();
        ctx.strokeStyle = 'rgba(231,234,237,.30)'; ctx.lineWidth = 1; ctx.stroke();
        ctx.restore();
      }

      ctx.save();
      ctx.beginPath(); ctx.arc(X(last[0]), Y(last[1]), 4, 0, Math.PI * 2);
      ctx.fillStyle = css('--t1'); ctx.fill();
      ctx.restore();
      scalebar();
    }

    function scalebar() {
      var px = 10 / mppx;                      /* 10 m */
      ctx.save();
      ctx.strokeStyle = css('--t3'); ctx.lineWidth = 1;
      var x0 = 16, y0 = h - 22;
      ctx.beginPath();
      ctx.moveTo(x0, y0 - 4); ctx.lineTo(x0, y0); ctx.lineTo(x0 + px, y0); ctx.lineTo(x0 + px, y0 - 4);
      ctx.stroke();
      ctx.fillStyle = css('--t3');
      ctx.font = '600 9.5px "IBM Plex Sans", sans-serif';
      ctx.fillText('10 m', x0 + px + 7, y0 + 1);
      ctx.restore();
    }

    /* ── satellite sky view ────────────────────────────────────────────
       C/N0 bars, not a count. Four satellites at 18 dB-Hz and four at 42
       are the same number and completely different situations.            */
    function sats(host, list) {
      host.textContent = '';
      if (!list || !list.length) {
        var n = document.createElement('span');
        n.className = 'lbl lbl-d'; n.textContent = 'NO SATELLITE DETAIL';
        host.appendChild(n); return;
      }
      list.forEach(function (s) {
        var b = document.createElement('div');
        var cn = Math.max(0, Math.min(50, s.cn || 0));
        b.style.width = '7px';
        b.style.height = Math.max(2, cn / 50 * 34) + 'px';
        /* below ~30 dB-Hz a satellite is in the solution but barely carrying
           it — that is a caution, not a failure, and it is coloured as one */
        b.style.background = s.used ? (cn < 30 ? css('--caution') : css('--t2')) : css('--fill-idle');
        b.title = (s.prn || '?') + ' · ' + cn + ' dB-Hz' + (s.used ? ' · used' : ' · tracked');
        host.appendChild(b);
      });
    }

    function sources(rows) {
      srcEl.textContent = '';
      rows.forEach(function (r) {
        var tr = document.createElement('tr');
        if (r.state === 'REJECTED' || r.state === 'ABSENT') tr.className = 'rej';
        [r.name, r.state, r.sigma].forEach(function (v, i) {
          var td = document.createElement('td');
          if (i === 2) td.className = 'n';
          td.textContent = v == null ? '—' : v;
          tr.appendChild(td);
        });
        srcEl.appendChild(tr);
      });
    }

    return {
      fit: fit,
      sats: sats,
      sources: sources,
      hasBasemap: function () { return tilesLive; },
      /* gps: {fix, lat, lon, sats, hdop, alt, course, speed} */
      update: function (gps, waypoints, onStatus) {
        var ok = gps && gps.fix && gps.lat != null && gps.lon != null;
        if (ok) {
          if (!origin) { origin = [gps.lat, gps.lon]; tryLeaflet(gps.lat, gps.lon, onStatus); }
          var p = enu(gps.lat, gps.lon, origin[0], origin[1]);
          var last = track[track.length - 1];
          if (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > 0.25) track.push(p);
          if (track.length > 4000) track.shift();
          if (lmap && tilesLive) {
            lmarker.setLatLng([gps.lat, gps.lon]);
            laccu.setLatLng([gps.lat, gps.lon]).setRadius(gps.hdop != null ? gps.hdop * 2.5 : 5);
            ltrack.addLatLng([gps.lat, gps.lon]);
            lmap.panTo([gps.lat, gps.lon], { animate: false });
          }
        }
        if (!tilesLive) gridDraw(ok ? gps : null, ok ? gps.course : null, waypoints);
      }
    };
  }

  W.Position = Position;
})(window);

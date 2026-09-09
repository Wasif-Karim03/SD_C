/* ══════════════════════════════════════════════════════════════════════════
   core.js — the state client and the control surface.

   Three rules this file exists to enforce:

   1. NEVER SHOW A STALE NUMBER AS A LIVE ONE. If the link drops, values are
      struck through and the banner says why. A frozen dashboard that still
      looks alive is the failure mode that hurts people.

   2. THE DEADMAN IS REAL. The server stops the motor if it has not heard a
      throttle command in 0.5 s. So the browser reposts while a key is held,
      and STOPS reposting the moment the tab is hidden, the window loses
      focus, or the key is released. Driving a car from a background tab is
      not a feature.

   3. E-STOP IS A LATCH, AND CLEARING IT IS A SEPARATE DECISION. Arming does
      not clear it. GO does not clear it. Only the slide does, and clearing
      leaves the car disarmed.
   ══════════════════════════════════════════════════════════════════════════ */
(function (W, D) {
  'use strict';

  var UI = W.UI;
  var $ = function (id) { return D.getElementById(id); };

  /* ─────────────────────────── link state ─────────────────────────── */

  var S = {};                     /* last /state payload */
  var SCAN = [];                  /* last /scan points */
  var lastOK = 0;                 /* monotonic-ish ms of last good /state */
  var lastLatency = null;
  var linkUp = false;
  var t0 = Date.now();            /* MET origin, replaced by server session start */

  function live() { return linkUp && (Date.now() - lastOK) < 1500; }

  function markStale(on) {
    D.querySelectorAll('.num').forEach(function (n) { n.classList.toggle('stale', on); });
    $('linkdown').classList.toggle('on', on);
  }

  /* ─────────────────────────── commands ─────────────────────────── */

  var pending = 0;
  function post(qs) {
    if (pending > 6) return Promise.resolve();     /* never queue commands */
    pending++;
    return fetch('/?' + qs, { method: 'POST' })
      .catch(function () {})
      .then(function () { pending--; });
  }

  /* ─────────────────────── header + chips ─────────────────────── */

  var chips = {};
  ['LIDAR', 'CAM-F', 'CAM-R', 'VESC', 'STEER', 'LOC', 'REC'].forEach(function (n) {
    chips[n] = UI.Chip(n);
    $('chips').appendChild(chips[n].node);
  });

  function met() {
    var s = (Date.now() - t0) / 1000;
    var m = Math.floor(s / 60), r = s - m * 60;
    return ('0' + m).slice(-2) + ':' + (r < 10 ? '0' : '') + r.toFixed(1);
  }

  /* ─────────────────────────── MAIs ─────────────────────────── */

  var mais = {};
  function addMAI(key, opts) {
    mais[key] = UI.MAI(opts);
    $('mais').appendChild(mais[key].node);
  }
  addMAI('hdg',  { label: 'HEADING ERROR', unit: '°', dp: 1, min: -30, max: 30,
                   band: [42, 58], ticks: [20, 80],
                   limit: function (v) { return Math.abs(v) > 20 ? 'caution' : ''; } });
  addMAI('xtrk', { label: 'CROSS-TRACK', unit: 'm', dp: 2, min: -0.6, max: 0.6,
                   band: [44, 56], ticks: [24, 76],
                   limit: function (v) { return Math.abs(v) > 0.35 ? 'caution' : ''; } });
  addMAI('near', { label: 'NEAREST AHEAD', unit: 'm', dp: 2, min: 0, max: 4,
                   band: [33, 100], ticks: [18, 33],
                   limit: function (v) { return v < 0.7 ? 'alarm' : (v < 1.3 ? 'caution' : ''); } });
  addMAI('duty', { label: 'DUTY (COMMANDED)', unit: '%', dp: 1, min: -20, max: 20,
                   band: [35, 65], ticks: [15, 85] });
  addMAI('batt', { label: 'BATTERY', unit: 'V', dp: 2, min: 12.0, max: 17.0,
                   band: [40, 100], ticks: [14, 26],
                   /* 4S LiPo: 3.5 V/cell is the "land now" line, 3.3 is damage */
                   limit: function (v) { return v < 13.2 ? 'alarm' : (v < 14.0 ? 'caution' : ''); } });
  addMAI('mos',  { label: 'VESC MOSFET', unit: '°C', dp: 1, min: 20, max: 90,
                   band: [0, 71], ticks: [71],
                   limit: function (v) { return v > 80 ? 'alarm' : (v > 70 ? 'caution' : ''); } });
  addMAI('amps', { label: 'MOTOR CURRENT', unit: 'A', dp: 1, min: -40, max: 40,
                   band: [25, 75], ticks: [12, 88] });

  /* Steering authority. Two things can steer this car — pure pursuit, which
     is following the plan, and the LiDAR veto, which is avoiding whatever is
     actually in front of it. When they disagree the operator must be able to
     see WHICH ONE has the wheel, because "the car turned and I don't know why"
     is the mode confusion that Sarter & Woods spent a career documenting. */
  var authority = (function () {
    var root = UI.el('div', 'mai');
    var top  = UI.el('div', 'maitop');
    top.appendChild(UI.el('span', 'lbl', 'STEERING AUTHORITY'));
    var v = UI.el('span', 'num'); v.style.fontSize = '12px'; v.textContent = '—';
    top.appendChild(v);
    var bar = UI.el('div', 'bar'); bar.style.display = 'flex'; bar.style.padding = '0';
    var a = UI.el('div'); a.style.background = 'rgba(199,74,199,.5)';
    var b = UI.el('div'); b.style.background = 'rgba(228,178,58,.5)';
    bar.appendChild(a); bar.appendChild(b);
    var foot = UI.el('div'); foot.style.cssText = 'display:flex;justify-content:space-between;margin-top:4px';
    var la = UI.el('span', 'lbl'); la.style.color = 'var(--cmd)';
    var lb = UI.el('span', 'lbl'); lb.style.color = 'var(--caution)';
    foot.appendChild(la); foot.appendChild(lb);
    root.appendChild(top); root.appendChild(bar); root.appendChild(foot);
    $('mais').appendChild(root);
    return function (pursuit, veto) {
      if (pursuit == null && veto == null) {
        v.textContent = 'MANUAL'; a.style.width = '0'; b.style.width = '0';
        la.textContent = 'OPERATOR'; lb.textContent = ''; return;
      }
      var pa = Math.abs(pursuit || 0), va = Math.abs(veto || 0), tot = pa + va;
      var share = tot < 1e-6 ? 50 : Math.round(pa / tot * 100);
      a.style.width = share + '%'; b.style.width = (100 - share) + '%';
      v.textContent = 'BLEND ' + share + ' / ' + (100 - share);
      v.style.color = share < 55 ? 'var(--caution)' : 'var(--t1)';
      la.textContent = 'PURE PURSUIT ' + (pursuit >= 0 ? '+' : '') + (pursuit || 0).toFixed(2);
      lb.textContent = 'LIDAR VETO ' + (veto >= 0 ? '+' : '') + (veto || 0).toFixed(2);
    };
  })();

  addMAI('icp', { label: 'ICP FAIL STREAK', unit: '/ 5', dp: 0, min: 0, max: 5,
                  band: [0, 20], ticks: [80],
                  limit: function (v) { return v >= 4 ? 'alarm' : (v >= 2 ? 'caution' : ''); } });

  /* ─────────────────────────── tapes ─────────────────────────── */

  var tapeSpeed = UI.Tape($('tape-speed'), {
    min: 0, max: 1.6, dp: 2,
    ticks: [[3, '1.5'], [28, '1.0'], [53, '0.5'], [78, '0']],
    /* muted fills, not signal colours: a band is a REGION, and if regions
       are as loud as alarms then nothing on the screen is loud. */
    bands: [[0, 10, 'var(--fill-alarm)'], [10, 28, 'var(--fill-caut)'],
            [28, 80, 'var(--fill-go)'], [80, 100, 'var(--fill-idle)']]
  });
  var tapeSteer = UI.Tape($('tape-steer'), {
    min: -1, max: 1, dp: 2,
    ticks: [[3, '+1'], [28, '+.5'], [50, '0'], [72, '-.5'], [94, '-1']],
    bands: [[0, 14, 'var(--fill-caut)'], [14, 86, 'var(--fill-go)'], [86, 100, 'var(--fill-caut)']]
  });

  /* ─────────────────────────── renderers ─────────────────────────── */

  var radar    = W.Radar($('radar'));
  var vision   = W.Vision($('vision'));
  var position = W.Position($('mapfall'), $('leaflet'), $('srclist'));
  var lanes    = UI.Lanes($('lanebody'), ['MODE', 'ARMED', 'BLOCKED', 'LOC OK']);
  var log      = UI.EventLog($('eventlist'), 80);

  /* calibration. Defaults match config.py; the server overrides them, and
     DIAGNOSE marks the two that are still estimates rather than measurements. */
  var CAL = { wheelbase: 0.25, maxSteer: 0.45, lookahead: 0.55, react: 1.3,
              forwardDeg: 354.6, maxDuty: 0.20, mpt: 0.003424,
              est: ['WHEELBASE_M', 'MAX_STEER_ANGLE_RAD'] };

  /* ─────────────────────── bypass sequence ─────────────────────── */
  /* Six states, derived from real signals — not a decoration. Each one is a
     thing the stack actually does, so a car that hesitates can be diagnosed
     by reading which step it is stuck on. */
  var SEQ = [
    ['DETECT',   'return inside the forward corridor'],
    ['RANGE',    'nearest range resolved and held'],
    ['INFLATE',  'costmap grown by the car half-width'],
    ['RE-PLAN',  'A* run against the inflated mask'],
    ['COMMIT',   'pure pursuit takes the new route'],
    ['CLEAR',    'obstacle behind the rear axle']
  ];
  SEQ.forEach(function (s, i) {
    var d = UI.el('div', 'seqstep');
    d.appendChild(UI.el('span', 'n', String(i + 1)));
    d.appendChild(UI.el('span', 'nm', s[0]));
    d.appendChild(UI.el('span', 'd', s[1]));
    $('seq').appendChild(d);
  });
  function seqStage(near, hasPath, following, obst) {
    if (!obst) return following ? 5 : -1;
    if (near == null) return 0;
    if (!hasPath) return 1;
    if (!following) return 3;
    return 4;
  }

  /* ─────────────────────── alarms ─────────────────────── */
  /* An alarm here means: something changed, and an operator has to decide.
     Anything that does not meet that test is a value on a face, not an
     alarm — the single biggest cause of alarm floods in real control rooms. */
  function alarms() {
    var out = [], t = S.tele || {}, h = S.health || {}, d = S.drive || {};
    if (d.estop) out.push(['ALARM', 'E-STOP LATCHED — clear with the slide, then arm again']);
    if (t.fault && t.fault !== 'FAULT_CODE_NONE' && t.fault !== 'NONE')
      out.push(['ALARM', 'VESC fault: ' + t.fault]);
    if (S.scan_age != null && S.scan_age > 0.6)
      out.push(['ALARM', 'LiDAR scan ' + S.scan_age.toFixed(1) + ' s old — auto-drive refuses to move']);
    else if (!h.lidar) out.push(['ALARM', 'no LiDAR — obstacle stopping is not running']);
    if (t.v_in != null && t.v_in < 13.2) out.push(['ALARM', 'pack ' + t.v_in.toFixed(1) + ' V — stop now']);
    else if (t.v_in != null && t.v_in < 14.0) out.push(['CAUTION', 'pack ' + t.v_in.toFixed(1) + ' V — one run left at most']);
    if (t.temp_mos != null && t.temp_mos > 70) out.push(['CAUTION', 'VESC MOSFET ' + t.temp_mos.toFixed(0) + ' °C']);
    if (!h.cam) out.push(['CAUTION', 'front camera not opened']);
    if (S.rear_on === false && S.mode === 'perception')
      out.push(['CAUTION', 'rear camera released to hold the 8 GB budget — rear detection unavailable']);
    if (!h.steer) out.push(['CAUTION', 'steering board not found — steering commands go nowhere']);
    if (S.mode === 'navigate' && h.loc === false)
      out.push(['CAUTION', 'localizer unhealthy — pose is not trustworthy']);
    return out;
  }

  var lastStripKey = '';
  function drawStrip() {
    var a = alarms();
    var el = $('strip');
    if (!a.length) { el.hidden = true; lastStripKey = ''; return; }
    var top = a[0];
    el.hidden = false;
    el.classList.toggle('is-alarm', top[0] === 'ALARM');
    $('strip-sev').textContent = top[0];
    $('strip-msg').textContent = top[1];
    $('strip-count').textContent = a.length + ' ACTIVE';
    var key = top[0] + top[1];
    if (key !== lastStripKey) {
      lastStripKey = key;
      $('strip-t').textContent = 'T+' + met();
      log.add('T+' + met(), top[0], top[1]);
    }
  }

  /* ─────────────────────── tuning readout ─────────────────────── */

  function drawTune() {
    var rows = [['LOOKAHEAD', CAL.lookahead, 'm'], ['STEER GAIN', CAL.steerGain, ''],
                ['GOAL TOL', CAL.goalTol, 'm'], ['REACT', CAL.react, 'm'],
                ['DUTY CAP', CAL.maxDuty, ''], ['FWD OFFSET', CAL.forwardDeg, '°']];
    $('tune').textContent = '';
    rows.forEach(function (r) {
      var d = UI.el('div', 'kv');
      d.appendChild(UI.el('span', 'lbl lbl-d', r[0]));
      var v = UI.el('span', 'num');
      v.style.fontSize = '10.5px'; v.style.color = 'var(--t2)';
      v.textContent = r[1] == null ? '—' : Number(r[1]).toFixed(r[0] === 'FWD OFFSET' ? 1 : (r[0] === 'DUTY CAP' ? 3 : 2));
      var u = UI.el('span', 'u', ' ' + r[2]); v.appendChild(u);
      d.appendChild(v); $('tune').appendChild(d);
    });
  }

  /* ─────────────────────── diagnose tables ─────────────────────── */

  function row(host, k, v, cls) {
    var tr = D.createElement('tr');
    var a = D.createElement('td'); a.textContent = k;
    var b = D.createElement('td'); b.className = 'n'; b.textContent = v;
    if (cls) b.style.color = cls;
    tr.appendChild(a); tr.appendChild(b); host.appendChild(tr);
  }
  function drawDiagnose() {
    var h = S.health || {}, t = S.tele || {};
    var H = $('d-health'); H.textContent = '';
    [['LiDAR', h.lidar], ['VESC', h.vesc], ['Steering', h.steer], ['Front camera', h.cam],
     ['Detector', h.detector], ['Localizer', h.loc], ['GNSS', S.gps && S.gps.fix]]
      .forEach(function (r) {
        row(H, r[0], r[1] ? 'RUNNING' : 'ABSENT',
            r[1] ? 'var(--go)' : 'var(--t4)');
      });

    var T = $('d-tele'); T.textContent = '';
    [['Pack voltage', t.v_in, 'V', 2], ['MOSFET', t.temp_mos, '°C', 1],
     ['Motor temp', t.temp_motor, '°C', 1], ['Motor current', t.motor_current, 'A', 1],
     ['ERPM', t.erpm, '', 0], ['Tachometer', t.tach, '', 0],
     ['Speed (odometry)', t.speed, 'm/s', 2]]
      .forEach(function (r) { row(T, r[0], UI.fmt(r[1], r[3]) + ' ' + r[2]); });
    row(T, 'Fault', t.fault || '—',
        (t.fault && t.fault.indexOf('NONE') < 0) ? 'var(--alarm)' : null);

    var C = $('d-cal'); C.textContent = '';
    [['METERS_PER_TACH', CAL.mpt, 6], ['LIDAR_FORWARD_DEG', CAL.forwardDeg, 1],
     ['MAX_DUTY', CAL.maxDuty, 3], ['WHEELBASE_M', CAL.wheelbase, 3],
     ['MAX_STEER_ANGLE_RAD', CAL.maxSteer, 3], ['LOOKAHEAD_M', CAL.lookahead, 2]]
      .forEach(function (r) {
        var est = CAL.est.indexOf(r[0]) >= 0;
        row(C, r[0] + (est ? '  EST' : ''), UI.fmt(r[1], r[2]), est ? 'var(--caution)' : null);
      });

    var Z = $('d-sess'); Z.textContent = '';
    var rec = S.rec || {};
    row(Z, 'Mode', (S.mode || '—').toUpperCase());
    row(Z, 'Environment', (S.env || '—').toUpperCase());
    row(Z, 'Control loop', UI.fmt(S.loop_ms, 1) + ' ms');
    row(Z, 'Recording', rec.on ? 'RUNNING' : 'off', rec.on ? 'var(--go)' : null);
    row(Z, 'Session dir', rec.dir || '—');
    row(Z, 'Rows written', rec.rows == null ? '—' : rec.rows);
    row(Z, 'Git', (S.session && S.session.sha) || '—');
  }

  /* ─────────────────────── the frame ─────────────────────── */

  var lastMode = null, lastArmed = null, lastEstop = null, lastRec = null;

  function frame() {
    var stale = !live();
    markStale(stale);

    var d = S.drive || {}, t = S.tele || {}, h = S.health || {}, fol = S.follow || {};

    /* header */
    var auto = fol.on ? 'AUTO' : 'MANUAL';
    $('h-autonomy').textContent = auto;
    $('h-autonomy').style.color = fol.on ? 'var(--go)' : 'var(--t2)';
    $('h-state').textContent =
      d.estop ? 'E-STOP LATCHED' : (d.armed ? (fol.on ? 'ARMED · FOLLOWING' : 'ARMED') : 'DISARMED');
    $('h-state').style.color = d.estop ? 'var(--alarm)' : 'var(--t1)';
    $('h-met').textContent = met();
    $('h-lat').textContent = 'CMD ' + (lastLatency == null ? '—' : Math.round(lastLatency)) +
                             ' ms · LOOP ' + UI.fmt(S.loop_ms, 0) + ' ms';
    var bars = $('linkbars').children, q = lastLatency == null ? 0 : (lastLatency < 40 ? 5 : lastLatency < 90 ? 4 : lastLatency < 180 ? 3 : lastLatency < 400 ? 2 : 1);
    for (var i = 0; i < bars.length; i++) {
      bars[i].style.height = (4 + i * 1.75) + 'px';
      bars[i].classList.toggle('on', i < q && !stale);
    }
    $('estop').classList.toggle('latched', !!d.estop);
    $('estop').firstChild.nodeValue = d.estop ? '■ LATCHED' : '■ STOP';
    $('slide-label').textContent = d.estop ? 'SLIDE TO CLEAR' : 'SLIDE TO STOP';

    /* chips */
    chips['LIDAR'].set(h.lidar ? (S.scan_age != null ? (1 / Math.max(S.scan_age, .01)).toFixed(1) : 'OK') : 'DOWN',
                       h.lidar ? 'Hz' : '', h.lidar ? '' : 'alarm');
    chips['CAM-F'].set(h.cam ? 'LIVE' : 'OFF', '', h.cam ? '' : 'caution');
    chips['CAM-R'].set(S.rear_on ? 'LIVE' : 'OFF', '', S.rear_on ? '' : 'caution');
    chips['VESC'].set(t.v_in != null ? t.v_in.toFixed(1) : '—', 'V',
                      t.v_in == null ? 'caution' : (t.v_in < 13.2 ? 'alarm' : (t.v_in < 14.0 ? 'caution' : '')));
    chips['STEER'].set(h.steer ? 'OK' : 'DOWN', '', h.steer ? '' : 'caution');
    chips['LOC'].set(S.mode === 'navigate' ? (h.loc ? 'OK' : 'LOST') : 'N/A', '',
                     S.mode === 'navigate' && !h.loc ? 'caution' : '');
    var rec = S.rec || {};
    chips['REC'].set(rec.on ? (rec.rows != null ? rec.rows : 'ON') : 'OFF',
                     rec.on && rec.rows != null ? 'rows' : '', rec.on ? 'go' : '');

    /* MAIs */
    mais.hdg.set(fol.heading_err);
    mais.xtrk.set(fol.cross_track);
    mais.near.set(S.near);
    mais.duty.set(d.duty);
    mais.batt.set(t.v_in);
    mais.mos.set(t.temp_mos);
    mais.amps.set(t.motor_current);
    mais.icp.set(fol.icp_fail == null ? (h.loc ? 0 : null) : fol.icp_fail);
    authority(fol.on ? fol.steer_pursuit : null, fol.on ? fol.steer_veto : null);

    /* tapes: white is the measured value, magenta is what we asked for */
    var cmdSpeed = d.duty != null ? Math.abs(d.duty) / 100 / CAL.maxDuty * 1.6 : null;
    tapeSpeed.set(t.speed, cmdSpeed);
    /* The steering servo is open loop — there is no encoder on it, so there
       is no measured steering angle to draw. Echoing the command back in the
       "actual" colour would invent a measurement this car cannot make, so the
       actual marker stays empty and only the commanded wedge moves. */
    tapeSteer.set(null, S.ctrl ? S.ctrl.steer : null);

    /* guidance line */
    $('g-line').innerHTML = fol.on
      ? 'AUTO <span class="sep">▸</span> FollowPath <span class="sep">▸</span> ' +
        '<span style="color:var(--caution)">' + (fol.note || 'running') + '</span>'
      : 'MANUAL <span class="sep">▸</span> operator' +
        (d.armed ? '' : ' <span class="sep">▸</span> <span style="color:var(--t3)">disarmed</span>');

    /* scene readouts */
    $('r-returns').textContent = 'LIDAR · ' + SCAN.length + ' RETURNS · SELF-MASK ON';
    $('r-sub').textContent = 'TRACK-UP · FWD ' + CAL.forwardDeg.toFixed(1) + '° · COSTMAP + INFLATION';
    $('r-age').textContent = UI.fmt(S.scan_age, 2);
    $('r-age').style.color = (S.scan_age != null && S.scan_age > 0.6) ? 'var(--alarm)' : 'var(--t1)';
    $('r-loop').textContent = S.loop_ms ? (1000 / S.loop_ms).toFixed(1) : '—';
    $('r-near').textContent = UI.fmt(S.near, 2);
    $('r-near').style.color = S.near == null ? 'var(--t1)'
      : (S.near < 0.7 ? 'var(--alarm)' : (S.near < 1.3 ? 'var(--caution)' : 'var(--t1)'));
    /* time-to-contact, the number that actually decides whether to brake */
    if (S.near != null && t.speed > 0.05) {
      $('r-ttc').textContent = '— ' + (S.near / t.speed).toFixed(1) + ' s to contact at current speed';
    } else { $('r-ttc').textContent = ''; }

    /* renderers */
    var steerNow = S.ctrl ? S.ctrl.steer : 0;
    var common = { pts: SCAN, steer: steerNow, cal: CAL, react: CAL.react,
                   speed: t.speed || 0, planned: S.path_local || null };
    if (cur === 'drive') {
      radar.draw({ pts: SCAN, trail: S.trail_local, route: S.path_local, goal: S.goal_local,
                   goalDist: S.goal_dist, lookahead: S.lookahead_local,
                   steer: steerNow, cal: CAL, react: CAL.react, coneDeg: 22 });
    } else if (cur === 'vision') {
      var r = vision.draw(common);
      $('v-div').textContent = r.divergence == null ? '—' : r.divergence.toFixed(2);
      $('v-rad').textContent = isFinite(r.radius) ? r.radius.toFixed(2) : '∞';
      $('v-ext').textContent = vision.extrude.toFixed(2);
      var stage = seqStage(S.near, !!(S.path_local && S.path_local.length), fol.on, r.obstacle);
      var steps = $('seq').children;
      for (var k = 0; k < steps.length; k++) {
        steps[k].className = 'seqstep' + (k < stage ? ' done' : (k === stage ? ' act' : ''));
      }
    } else if (cur === 'position') {
      position.update(S.gps, S.waypoints_enu, function (msg) { $('p-src').textContent = msg; });
      var g = S.gps || {};
      $('p-fix').textContent = g.fix
        ? 'GNSS · ' + (g.mode || 'FIX') + ' · ' + (g.sats || 0) + ' SATS · HDOP ' + UI.fmt(g.hdop, 1)
        : 'GNSS · NO FIX';
      $('p-fix').style.color = g.fix ? 'var(--t3)' : 'var(--caution)';
      $('p-ll').textContent = g.fix
        ? g.lat.toFixed(6) + (g.lat >= 0 ? ' N ' : ' S ') + Math.abs(g.lon).toFixed(6) + (g.lon >= 0 ? ' E' : ' W') +
          ' · ALT ' + UI.fmt(g.alt, 1) + ' m'
        : 'indoors there is no degraded fix — there is no fix';
      position.sats($('sats'), g.satlist);
      $('p-satnote').textContent = g.satlist && g.satlist.length
        ? g.satlist.length + ' tracked · bars are C/N₀, dimmed = not in solution'
        : 'no per-satellite detail — receiver is sending GGA only';
      position.sources([
        { name: 'GNSS · SE100', state: g.fix ? 'ACCEPTED' : 'ABSENT',
          sigma: g.fix ? (g.hdop != null ? (g.hdop * 2.5).toFixed(1) + ' m' : '—') : null },
        { name: 'Wheel odometry', state: h.vesc ? 'ACCEPTED' : 'ABSENT', sigma: h.vesc ? '0.02 m/s' : null },
        { name: 'LiDAR SLAM', state: h.loc ? 'ACCEPTED' : (S.mode === 'navigate' ? 'DEGRADED' : 'IDLE'),
          sigma: h.loc ? '0.05 m' : null },
        { name: 'Compass · IST8310', state: 'REJECTED', sigma: 'uncalibrated' },
        { name: 'IMU', state: 'ABSENT', sigma: 'not installed' }
      ]);
    } else if (cur === 'diagnose') {
      drawDiagnose();
    }

    /* cameras — only the visible ones are subscribed, because each MJPEG
       stream is a live socket and a hidden one is pure cost on an 8 GB box */
    $('rear-off').style.display = S.rear_on ? 'none' : 'flex';
    $('rear-off2').style.display = S.rear_on ? 'none' : 'flex';
    $('rear-toggle').setAttribute('aria-pressed', S.rear_on ? 'true' : 'false');
    $('rec-btn').firstChild.nodeValue = rec.on ? 'STOP RECORDING' : 'START RECORDING';
    $('cmd-arm').setAttribute('aria-pressed', d.armed ? 'true' : 'false');
    $('cam-f-lat').textContent = h.cam ? 'LIVE' : 'OFF';
    $('cam-r-lat').textContent = S.rear_on ? 'LIVE' : 'OFF';

    /* the scale bar is drawn from the CURRENT zoom, not assumed: a legend
       that does not track the thing it describes is worse than none */
    var sc = radar.scaleFor();
    var nice = sc > 90 ? 0.5 : (sc > 34 ? 1 : (sc > 14 ? 2 : 5));
    D.querySelector('#s-drive .scalebar').style.width = (nice * sc).toFixed(0) + 'px';
    $('r-scale').textContent = nice + ' m';

    drawStrip();

    /* transitions worth a log line */
    if (S.mode !== lastMode && lastMode !== null)
      log.add('T+' + met(), 'MODE', lastMode.toUpperCase() + ' → ' + String(S.mode).toUpperCase() + ' (operator)');
    if (d.armed !== lastArmed && lastArmed !== null)
      log.add('T+' + met(), 'PREARM', d.armed ? 'armed' : 'disarmed');
    if (d.estop !== lastEstop && lastEstop !== null)
      log.add('T+' + met(), 'ESTOP', d.estop ? 'LATCHED' : 'cleared by operator (still disarmed)');
    if (rec.on !== lastRec && lastRec !== null)
      log.add('T+' + met(), 'REC', rec.on ? ('recording started' + (rec.note ? ' · “' + rec.note + '”' : '')) : 'recording stopped');
    lastMode = S.mode; lastArmed = d.armed; lastEstop = d.estop; lastRec = rec.on;

    $('f-session').textContent = 'SESSION ' + ((S.session && S.session.started) || '—') +
                                 ' · ' + ((S.session && S.session.sha) || '—') +
                                 ((S.session && S.session.dirty) ? ' · TREE DIRTY' : ' · TREE CLEAN');
  }

  /* ─────────────────────── polling ─────────────────────── */

  function pollState() {
    var t = performance.now();
    fetch('/state', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (j) {
        lastLatency = performance.now() - t;
        S = j; lastOK = Date.now();
        if (!linkUp) { linkUp = true; log.add('T+' + met(), 'LINK', 'telemetry link up'); }
        if (j.session && j.session.epoch_ms) t0 = j.session.epoch_ms;
        if (j.config) { Object.keys(j.config).forEach(function (k) { CAL[k] = j.config[k]; });
                        if (j.config.est) CAL.est = j.config.est; drawTune(); }
      })
      .catch(function () {
        if (linkUp) { linkUp = false; log.add('T+' + met(), 'LINK', 'telemetry link LOST'); }
      });
  }

  function pollScan() {
    if (cur === 'diagnose' || cur === 'position') return;   /* nobody is looking */
    fetch('/scan', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (j) { SCAN = j.pts || []; })
      .catch(function () {});
  }

  /* ─────────────────────── screens ─────────────────────── */

  var cur = 'drive';
  function show(name) {
    cur = name;
    D.querySelectorAll('.screen').forEach(function (s) { s.classList.toggle('on', s.id === 's-' + name); });
    D.querySelectorAll('#rail button[data-screen]').forEach(function (b) {
      b.setAttribute('aria-current', b.dataset.screen === name ? 'true' : 'false');
    });
    /* MJPEG sockets follow the visible screen */
    $('img-front').src  = name === 'drive'  ? '/cam/front.mjpg' : '';
    $('img-rear').src   = (name === 'drive'  && S.rear_on) ? '/cam/rear.mjpg' : '';
    $('img-front2').src = name === 'vision' ? '/cam/front.mjpg' : '';
    $('img-rear2').src  = (name === 'vision' && S.rear_on) ? '/cam/rear.mjpg' : '';
    $('slamimg').src    = name === 'position' ? '/map.jpg?t=' + Date.now() : '';
    resize();
    try { localStorage.setItem('rc.screen', name); } catch (e) {}
  }
  D.querySelectorAll('#rail button[data-screen]').forEach(function (b) {
    b.addEventListener('click', function () { show(b.dataset.screen); });
  });

  function resize() { radar.fit(); vision.fit(); position.fit(); fitDets(); }
  W.addEventListener('resize', resize);

  function fitDets() {
    ['det-front', 'det-rear', 'det-front2', 'det-rear2'].forEach(function (id) {
      var c = $(id); if (!c) return;
      var b = c.parentNode.getBoundingClientRect();
      c.width = Math.max(1, b.width); c.height = Math.max(1, b.height);
      c.style.width = b.width + 'px'; c.style.height = b.height + 'px';
    });
  }

  /* ─────────────────────── detections ─────────────────────── */
  /* Corner brackets, not filled boxes: the operator has to see the object,
     and a solid rectangle hides the thing it is pointing at. */
  function drawDets(canvasId, dets) {
    var c = $(canvasId); if (!c) return;
    var x = c.getContext('2d');
    x.clearRect(0, 0, c.width, c.height);
    if (!dets || !dets.length) return;
    x.strokeStyle = getComputedStyle(D.documentElement).getPropertyValue('--caution').trim();
    x.fillStyle = x.strokeStyle;
    x.lineWidth = 1.4; x.font = '8.5px "IBM Plex Mono", monospace';
    dets.forEach(function (dt) {
      /* dets are normalised [x0,y0,x1,y1,label,conf] in image space */
      var X0 = dt[0] * c.width, Y0 = dt[1] * c.height,
          X1 = dt[2] * c.width, Y1 = dt[3] * c.height;
      var L = Math.min(14, (X1 - X0) * 0.35), M = Math.min(14, (Y1 - Y0) * 0.35);
      [[X0, Y0 + M, X0, Y0, X0 + L, Y0], [X1 - L, Y0, X1, Y0, X1, Y0 + M],
       [X0, Y1 - M, X0, Y1, X0 + L, Y1], [X1 - L, Y1, X1, Y1, X1, Y1 - M]]
        .forEach(function (p) {
          x.beginPath(); x.moveTo(p[0], p[1]); x.lineTo(p[2], p[3]); x.lineTo(p[4], p[5]); x.stroke();
        });
      x.fillText(dt[4] + ' ' + Number(dt[5]).toFixed(2), X0, Y0 - 4);
    });
  }

  /* ─────────────────────── controls ─────────────────────── */

  function toast(msg) {
    var t = $('toast'); t.textContent = msg; t.classList.add('on');
    clearTimeout(toast._t); toast._t = setTimeout(function () { t.classList.remove('on'); }, 1600);
  }

  $('estop').addEventListener('click', function () { post('estop=1'); toast('E-STOP latched'); });
  $('cmd-arm').addEventListener('click', function () { post('arm=on'); });
  $('cmd-disarm').addEventListener('click', function () { post('arm=off&throttle=0'); });
  $('cmd-go').addEventListener('click', function () { post('follow=1'); });
  $('cmd-mark').addEventListener('click', function () {
    log.add('T+' + met(), 'MARK', 'operator mark'); toast('marked');
  });
  $('rear-toggle').addEventListener('click', function () {
    post('rear=' + (S.rear_on ? '0' : '1')).then(function () { setTimeout(function () { show(cur); }, 300); });
  });
  $('rec-btn').addEventListener('click', function () {
    var on = S.rec && S.rec.on;
    post(on ? 'rec=0' : 'rec=1&note=' + encodeURIComponent($('rec-note').value || ''));
  });
  $('save-btn').addEventListener('click', function () { post('save=1'); toast('map saved'); });

  /* click the radar to set a goal, in metres, in the car frame */
  $('radar').addEventListener('click', function (e) {
    if (S.mode !== 'navigate') { toast('goals are set in NAVIGATE mode'); return; }
    var b = e.target.getBoundingClientRect();
    var m = radar.unproject(e.clientX - b.left, e.clientY - b.top);
    post('goal_fwd=' + m[0].toFixed(3) + '&goal_lat=' + m[1].toFixed(3));
    toast('goal ' + m[0].toFixed(2) + ' m ahead, ' + m[1].toFixed(2) + ' m across');
  });

  /* the slide. Deliberately awkward: a latched stop must not be cleared, and
     a running car must not be stopped, by one stray click on a touchpad. */
  (function () {
    var box = $('slide'), fill = box.querySelector('.fillbar'), dragging = false, x0 = 0;
    function at(e) { return (e.touches ? e.touches[0].clientX : e.clientX); }
    function start(e) { dragging = true; x0 = at(e); e.preventDefault(); }
    function move(e) {
      if (!dragging) return;
      var f = Math.max(0, Math.min(1, (at(e) - x0) / (box.offsetWidth * 0.7)));
      fill.style.width = (f * 100) + '%';
      if (f >= 1) { done(); }
    }
    function done() {
      dragging = false; fill.style.width = '0';
      if (S.drive && S.drive.estop) { post('clearstop=1'); toast('E-STOP cleared — still disarmed'); }
      else { post('estop=1'); toast('E-STOP latched'); }
    }
    function end() { dragging = false; fill.style.width = '0'; }
    box.addEventListener('mousedown', start); W.addEventListener('mousemove', move); W.addEventListener('mouseup', end);
    box.addEventListener('touchstart', start, { passive: false });
    box.addEventListener('touchmove', move, { passive: false }); box.addEventListener('touchend', end);
  })();

  /* ─────────────────────── driving ─────────────────────── */
  /* The deadman lives here. keys held -> repost at 6 Hz. Anything that means
     the operator has stopped watching (blur, hidden tab, key up) stops the
     reposting immediately, and the server's own 0.5 s timeout does the rest. */

  var held = {}, driveTimer = null;

  function driveTick() {
    var th = 0, st = 0;
    if (held.w) th += 1; if (held.s) th -= 1;
    if (held.a) st -= 1; if (held.d) st += 1;
    var duty = th * (CAL.maxDuty || 0.2) * 0.55;
    post('throttle=' + duty.toFixed(3) + '&steer=' + st.toFixed(2));
  }
  function startDriving() { if (!driveTimer) { driveTick(); driveTimer = setInterval(driveTick, 160); } }
  function stopDriving() {
    if (driveTimer) { clearInterval(driveTimer); driveTimer = null; }
    held = {};
    post('throttle=0&steer=0');
  }

  D.addEventListener('keydown', function (e) {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
    var k = e.key.toLowerCase();
    if (e.key === 'Escape' && e.shiftKey) { post('estop=1'); toast('E-STOP latched'); return; }
    if ('wasd'.indexOf(k) >= 0) { held[k] = true; startDriving(); e.preventDefault(); return; }
    if (k === ' ') { post('arm=toggle'); e.preventDefault(); return; }
    if (k === 'x') { stopDriving(); post('arm=off'); return; }
    if (k === 'b') { $('rear-toggle').click(); return; }
    if (k === 'g') { $('rec-btn').click(); return; }
    if (k === 'm') { $('cmd-mark').click(); return; }
    if (k === 'enter') { post('follow=1'); return; }
    if (k === '1') show('drive'); if (k === '2') show('vision');
    if (k === '3') show('position'); if (k === '4') show('diagnose');
  });
  D.addEventListener('keyup', function (e) {
    var k = e.key.toLowerCase();
    if ('wasd'.indexOf(k) >= 0) {
      delete held[k];
      if (!held.w && !held.s && !held.a && !held.d) stopDriving();
    }
  });
  W.addEventListener('blur', stopDriving);
  D.addEventListener('visibilitychange', function () { if (D.hidden) stopDriving(); });

  /* ─────────────────────── ground toggle ─────────────────────── */

  $('hmi-toggle').addEventListener('click', function () {
    var next = D.documentElement.dataset.hmi === 'day' ? 'night' : 'day';
    D.documentElement.dataset.hmi = next;
    try { localStorage.setItem('rc.hmi', next); } catch (e) {}
    toast(next === 'day' ? 'daylight ground' : 'night ground');
  });
  /* Until the operator picks a ground, follow the machine they are reading it
     on: a laptop already set to light is probably a laptop in daylight. Once
     they choose, that choice wins for good — an instrument must not change
     its appearance on its own at dusk. */
  (function () {
    var saved = null;
    try { saved = localStorage.getItem('rc.hmi'); } catch (e) {}
    if (saved) { D.documentElement.dataset.hmi = saved; return; }
    var light = W.matchMedia && W.matchMedia('(prefers-color-scheme: light)').matches;
    D.documentElement.dataset.hmi = light ? 'day' : 'night';
  })();

  /* ─────────────────────── boot ─────────────────────── */

  drawTune();
  try { show(localStorage.getItem('rc.screen') || 'drive'); } catch (e) { show('drive'); }
  setInterval(pollState, 200);
  setInterval(pollScan, 100);
  setInterval(frame, 100);
  setInterval(function () {
    var d = S.drive || {}, h = S.health || {}, fol = S.follow || {};
    lanes.push({
      'MODE': S.mode === 'navigate' ? 'go' : (S.mode ? 'caut' : ''),
      'ARMED': d.estop ? 'alarm' : (d.armed ? 'go' : ''),
      'BLOCKED': (fol.note && fol.note.indexOf('blocked') === 0) ? 'caut' : '',
      'LOC OK': h.loc ? 'go' : ''
    });
  }, 1000);
  /* the SLAM map is a rendered JPEG, refreshed at a rate a human can read */
  setInterval(function () { if (cur === 'position') $('slamimg').src = '/map.jpg?t=' + Date.now(); }, 1000);
  setInterval(function () {
    if (cur === 'drive')  { drawDets('det-front', S.front_dets); drawDets('det-rear', S.rear_dets); }
    if (cur === 'vision') { drawDets('det-front2', S.front_dets); drawDets('det-rear2', S.rear_dets); }
  }, 200);

  pollState(); resize();
})(window, document);

// calib_page.h - WHIP's /calib page: line the IK (ik_gait.h) up with his servos.
// Served from flash like the control page. Every button needs the control PIN.
#pragma once
#include <pgmspace.h>

static const char CALIB_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WHIP Calibration</title>
  <link rel="icon" href="/avatar.jpg">
  <style>
    :root {
      --bg: #07090f; --panel: #0a0e1a; --border: #1e3458; --accent: #6496e6; --accent-rgb: 100,150,230;
      --text: #a0b8e8; --text-dim: #4a5f80; --text-val: #d7e6ff; --green: #46d764; --orange: #ffbe50;
      --red: #c84040; --radius: 6px; --font: 'Courier New', Courier, monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: var(--bg); color: var(--text); font-family: var(--font); padding: 16px; max-width: 760px; margin: 0 auto; }
    h1 { color: var(--accent); letter-spacing: 3px; font-size: 1.4rem; margin-bottom: 6px; }
    a { color: var(--accent); }
    p, li { font-size: 0.85rem; line-height: 1.5; margin-bottom: 6px; }
    ol { padding-left: 20px; margin-bottom: 12px; }
    .row { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; margin: 10px 0; }
    button { font-family: var(--font); font-size: 0.85rem; font-weight: bold; padding: 8px 14px; cursor: pointer;
             background: var(--panel); color: var(--text-val); border: 2px solid var(--border); border-radius: var(--radius); }
    button.on { border-color: var(--accent); color: var(--accent); background: rgba(var(--accent-rgb), 0.12); }
    button.go { border-color: var(--green); color: var(--green); }
    button.warn { border-color: var(--orange); color: var(--orange); }
    input[type=number] { width: 58px; font-family: var(--font); background: var(--bg); color: var(--text-val);
                         border: 1px solid var(--border); border-radius: 4px; padding: 3px; }
    input[type=range] { width: 120px; }
    .legs { display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 10px; }
    .leg { background: var(--panel); border: 2px solid var(--border); border-radius: var(--radius); padding: 8px; }
    .leg h3 { color: var(--accent); font-size: 0.9rem; margin-bottom: 4px; }
    .servo { display: grid; grid-template-columns: 38px 46px 1fr 30px; gap: 4px; align-items: center; font-size: 0.75rem; margin: 4px 0; }
    .servo .trim { grid-column: 1 / -1; display: flex; gap: 6px; align-items: center; }
    .servo button { padding: 2px 6px; font-size: 0.7rem; }
    #msg { min-height: 1.2em; font-size: 0.85rem; color: var(--orange); }
    #lock { display: none; }
  </style>
</head>
<body>
  <h1>WHIP CALIBRATION</h1>
  <p><a href="/">&larr; control page</a> &middot; <span id="calState">--</span></p>

  <ol>
    <li><b>Lift WHIP onto a stand, feet off the ground.</b> Unlock on the control page first (same PIN).</li>
    <li>Press <b>START</b> (he stops walking), then <b>ZERO</b>: every servo goes to its middle.</li>
    <li>In ZERO every leg should point straight out from the body (along the line from the centre through its hip),
        each thigh level, each foot hanging straight down. Nudge a servo's <b>trim</b> until it does.</li>
    <li>Press <b>WIGGLE</b> on each servo: the joint that twitches must be that leg's hip / knee / foot.
        If it's another one, fix the channel number.</li>
    <li>Press <b>HOME</b>: all six feet on the ground evenly; at height 5 that's nearly the ZERO pose (thighs about level).
        A joint bent the wrong way needs <b>rev</b> ticked.</li>
    <li><b>Height</b>: pick the lowest setting where the battery still clears the floor with room to spare
        (0 = feet 90 mm below the hips, 9 = 170 mm). Lower walks steadier; higher clears more.</li>
    <li><b>SAVE</b> (kept in flash), then <b>DONE</b>. Walk him slowly first (speed 2-3), still on the stand.</li>
  </ol>

  <div class="row">
    <button id="start" class="go">START</button>
    <button data-pose="zero">ZERO</button>
    <button data-pose="home">HOME</button>
    <button data-pose="sleep">SLEEP</button>
    <button id="save" class="go">SAVE</button>
    <button id="done">DONE</button>
    <button id="defaults" class="warn">DEFAULTS</button>
  </div>
  <div class="row">
    height <input type="range" id="height" min="0" max="9"> <span id="hVal"></span>
    speed <input type="range" id="speed" min="0" max="9"> <span id="sVal"></span>
  </div>
  <div id="msg"></div>
  <div id="legs" class="legs"></div>

  <script>
    var state = null;
    var msg = document.getElementById('msg');

    function call(url) {
      return fetch(url).then(function (r) {
        if (r.status === 403) { msg.textContent = 'Locked: unlock with the PIN on the control page first.'; throw 0; }
        if (r.status === 409) { msg.textContent = 'Press START first.'; throw 0; }
        return r.json();
      }).then(function (j) { msg.textContent = ''; show(j); return j; }).catch(function () {});
    }

    function show(j) {
      if (!j || !j.legs) return;
      state = j;
      document.getElementById('calState').textContent = (j.calibrated ? 'calibration saved' : 'default calibration (not saved yet)') +
        (j.calibrating ? ' · calibrating: he holds still' : '');
      document.querySelectorAll('[data-pose]').forEach(function (b) { b.classList.toggle('on', j.calibrating && b.dataset.pose[0] === j.pose); });
      document.getElementById('start').classList.toggle('on', !!j.calibrating);
      ['height', 'speed'].forEach(function (k) {
        document.getElementById(k).value = j[k];
        document.getElementById(k[0] + 'Val').textContent = j[k];
      });
      var html = '';
      j.legs.forEach(function (leg, l) {
        html += '<div class="leg"><h3>' + leg.name + '</h3>';
        leg.servos.forEach(function (s, k) {
          var id = l + '_' + k;
          html += '<div class="servo"><span>' + s.joint + '</span>' +
            '<input type="number" min="0" max="32" value="' + s.ch + '" data-ch="' + id + '" title="channel">' +
            '<label><input type="checkbox" data-rev="' + id + '"' + (s.rev ? ' checked' : '') + '> rev</label>' +
            '<button data-wig="' + id + '">WIG</button>' +
            '<div class="trim">trim <input type="range" min="-400" max="400" step="5" value="' + s.trim + '" data-trim="' + id + '">' +
            '<input type="number" min="-400" max="400" value="' + s.trim + '" data-trimn="' + id + '"> us</div></div>';
        });
        html += '</div>';
      });
      document.getElementById('legs').innerHTML = html;
    }

    function set(id, field, value) {
      var p = id.split('_');
      return call('/calib/set?leg=' + p[0] + '&joint=' + p[1] + '&' + field + '=' + value);
    }

    document.getElementById('legs').addEventListener('change', function (e) {
      var t = e.target, d = t.dataset;
      if (d.ch) set(d.ch, 'ch', t.value);
      if (d.rev) set(d.rev, 'rev', t.checked ? 1 : 0);
      if (d.trim) set(d.trim, 'trim', t.value);
      if (d.trimn) set(d.trimn, 'trim', t.value);
    });
    document.getElementById('legs').addEventListener('click', function (e) {
      var w = e.target.dataset.wig;
      if (w) { var p = w.split('_'); call('/calib/wiggle?leg=' + p[0] + '&joint=' + p[1]); }
    });
    document.querySelectorAll('[data-pose]').forEach(function (b) {
      b.addEventListener('click', function () { call('/calib/pose?p=' + b.dataset.pose); });
    });
    ['height', 'speed'].forEach(function (k) {
      document.getElementById(k).addEventListener('change', function (e) { call('/gait?' + k + '=' + e.target.value); });
    });
    document.getElementById('start').addEventListener('click', function () { call('/calib/start'); });
    document.getElementById('save').addEventListener('click', function () {
      call('/calib/save').then(function (j) { if (j) msg.textContent = 'Saved.'; });
    });
    document.getElementById('done').addEventListener('click', function () { call('/calib/done'); });
    document.getElementById('defaults').addEventListener('click', function () {
      if (confirm('Put every channel, trim and direction back to the defaults? (Not saved until SAVE.)')) call('/calib/defaults');
    });

    fetch('/calib/state').then(function (r) { return r.json(); }).then(show).catch(function () {});
  </script>
</body>
</html>
)rawhtml";

// lights_page.h - WHIP's /lights page: every LED-grid look's colour and effect, and the brightness.
// It talks to /leds (leds.h keeps the settings in flash). Changing them needs his PIN, as driving does.
#pragma once
#include <Arduino.h>

static const char LIGHTS_HTML[] PROGMEM = R"lights(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WHIP - lights</title>
<style>
  :root { --bg:#080A12; --panel:#0E1120; --border:#1D223F; --accent:#D34859; --text:#A89497; --val:#F2E9EA; }
  body { background:var(--bg); color:var(--text); font-family:'Courier New',monospace; margin:0; padding:16px; }
  h1 { color:var(--accent); letter-spacing:3px; font-size:1.3rem; }
  .row { display:flex; align-items:center; gap:10px; background:var(--panel); border:1px solid var(--border);
         border-radius:6px; padding:8px 10px; margin:6px 0; }
  .row b { color:var(--val); width:90px; }
  input[type=color] { width:46px; height:30px; border:none; background:none; }
  select, button, input[type=password] { background:var(--panel); color:var(--val); border:1px solid var(--border);
         border-radius:6px; padding:6px; font-family:inherit; }
  button { cursor:pointer; border-color:var(--accent); color:var(--accent); }
  a { color:var(--accent); }
</style></head><body>
<h1>WHIP LIGHTS</h1>
<p>Each look of his 4x4 grid: its colour and effect. Changes show at once and stay in his flash.</p>
<p><input type="password" id="pin" placeholder="PIN" size="6"> <button id="unlock">UNLOCK</button> <span id="msg"></span></p>
<div class="row"><b>brightness</b><input type="range" id="bright" min="5" max="120"><span id="bv"></span></div>
<div id="looks"></div>
<p><button id="reset">BACK TO HIS COLOURS</button> &nbsp; <a href="/">back</a></p>
<script>
  var FX = ['solid', 'breathe', 'blink', 'rainbow', 'off'];
  var WHAT = { obstacle: 'walking by himself', ir: 'on the IR remote', web: 'driven from here / the controller',
               mind: 'MIND mode', follow: 'follow me', locked: 'legs locked', tilt: 'tilt stop', held: 'picked up',
               shutdown: 'shutting down', starting: 'the 5 s before he walks off', body: 'BODY mode' };
  function call(q) {
    return fetch('/leds' + q).then(function (r) {
      if (r.status == 403) { document.getElementById('msg').textContent = 'unlock with his PIN first'; return null; }
      return r.json();
    });
  }
  function show(j) {
    if (!j) return;
    var b = document.getElementById('bright'); b.value = j.bright; document.getElementById('bv').textContent = j.bright;
    var box = document.getElementById('looks'); box.innerHTML = '';
    Object.keys(j.looks).forEach(function (name) {
      var l = j.looks[name], row = document.createElement('div');
      row.className = 'row';
      row.innerHTML = '<b>' + name + '</b><input type="color" value="#' + l.rgb + '"><select>' +
        FX.map(function (f) { return '<option' + (f == l.fx ? ' selected' : '') + '>' + f + '</option>'; }).join('') +
        '</select><span>' + (WHAT[name] || '') + '</span>';
      row.querySelector('input').addEventListener('change', function (e) {
        call('?look=' + name + '&rgb=' + e.target.value.slice(1)).then(show);
      });
      row.querySelector('select').addEventListener('change', function (e) {
        call('?look=' + name + '&fx=' + e.target.value).then(show);
      });
      box.appendChild(row);
    });
  }
  document.getElementById('bright').addEventListener('change', function (e) { call('?bright=' + e.target.value).then(show); });
  document.getElementById('reset').addEventListener('click', function () { call('?reset=1').then(show); });
  document.getElementById('unlock').addEventListener('click', function () {
    fetch('/unlock?pw=' + encodeURIComponent(document.getElementById('pin').value)).then(function (r) {
      document.getElementById('msg').textContent = r.ok ? 'unlocked' : 'wrong PIN';
    });
  });
  call('').then(show);
</script>
</body></html>)lights";

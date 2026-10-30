/* play.js - WHIP in your browser: his own gait engine and inverse kinematics (scripts/esp32_gaits/
   gaits.h, ik_gait.h), ported to JavaScript, drawn in CSS 3D. Every move is the one his ESP32 would
   send his servos: the same stance, stride, lift, gaits and timing (speed 0-9 = 120-30 ms a move).
   Drive him round the arena and collect the orbs. */
(function () {
  "use strict";

  // ---- his chassis (ik_gait.h) ----------------------------------------------------------------
  const THIGH = 84, FOOT = 127, HIP_TO_KNEE = 28, FOOT_TIP_DEG = 14, R2D = 180 / Math.PI;
  const LEGS = 6, LF = 0, LM = 1, LB = 2, RF = 3, RM = 4, RB = 5;
  const ANCHOR = [[74, 39], [0, 64], [-74, 39], [74, -39], [0, -64], [-74, -39]];
  const MOUNT_DEG = [27.8, 90, 152.2, -27.8, -90, -152.2];   // each leg at 1500 us (Jakob's)
  const HOME_Z_MIN = -80, HOME_Z_MAX = -160;
  const LIFT = 50, FB_X = 160, WIDTH_FB = 123, WIDTH_M = 177, WALK_X = 40;
  const REST_Z = -75;
  const US_PER_DEG = 10.49;

  let height = 5, speed = 7, gaitId = 0;
  const homeZ = () => HOME_Z_MIN + (HOME_Z_MAX - HOME_Z_MIN) * height / 9;
  const stepMs = () => 120 - speed * 10;
  const AIR_TICKS = 10;   // 2 * STEPS: his 115200 link has time for every move

  const P = (x, y, z) => ({ x, y, z });
  const copy = f => f.map(p => P(p.x, p.y, p.z));
  function stance(zA, zB) {
    return [P(FB_X, WIDTH_FB, zA), P(0, WIDTH_M, zB), P(-FB_X, WIDTH_FB, zA),
            P(FB_X, -WIDTH_FB, zB), P(0, -WIDTH_M, zA), P(-FB_X, -WIDTH_FB, zB)];
  }
  const home = () => { const z = homeZ(); return stance(z, z); };
  const ease = t => t * t * t * (10 + t * (6 * t - 15));
  const lerpF = (a, b, t) => a.map((p, i) => P(p.x + (b[i].x - p.x) * t, p.y + (b[i].y - p.y) * t, p.z + (b[i].z - p.z) * t));

  // Foot position (body frame) -> hip / knee / foot degrees (Leg::setFootPosition). null: out of reach
  function solve(leg, p) {
    const ax = ANCHOR[leg][0], ay = ANCHOR[leg][1];
    let x = p.x - ax, y = p.y - ay;
    const z = p.z;
    let hip = Math.atan2(y, x) * R2D - MOUNT_DEG[leg];
    while (hip > 180) hip -= 360;
    while (hip < -180) hip += 360;
    const len = Math.hypot(x, y);
    if (len < 1) return null;
    x = p.x - (ax + x * HIP_TO_KNEE / len);
    y = p.y - (ay + y * HIP_TO_KNEE / len);
    const h = Math.hypot(x, y), l = Math.hypot(h, z);
    const foot = Math.acos((THIGH * THIGH + FOOT * FOOT - l * l) / (2 * THIGH * FOOT)) * R2D - 90 - FOOT_TIP_DEG;
    const vB = Math.acos((l * l + THIGH * THIGH - FOOT * FOOT) / (2 * l * THIGH));
    const knee = (vB + Math.atan2(z, h)) * R2D;
    if ([hip, knee, foot].some(Number.isNaN)) return null;
    return [hip, knee, foot].map(a => Math.max(-90, Math.min(90, a)));   // his servos turn +-90
  }
  // ... and back: the leg's points for those angles (whip_body.py joints()): anchor, knee, ankle, tip
  function joints(leg, a) {
    const [ax, ay] = ANCHOR[leg];
    const yaw = (MOUNT_DEG[leg] + a[0]) / R2D, c = Math.cos(yaw), s = Math.sin(yaw);
    const kx = ax + HIP_TO_KNEE * c, ky = ay + HIP_TO_KNEE * s;
    const k = a[1] / R2D;
    const ah = THIGH * Math.cos(k), az = THIGH * Math.sin(k);
    const f = k + Math.PI + (a[2] + 90 + FOOT_TIP_DEG) / R2D;
    const fh = ah + FOOT * Math.cos(f), fz = az + FOOT * Math.sin(f);
    return [P(ax, ay, 0), P(kx, ky, 0), P(kx + ah * c, ky + ah * s, az), P(kx + fh * c, ky + fh * s, fz)];
  }

  // ---- his gait engine (gaits.h) --------------------------------------------------------------
  const GAITS = [
    { name: "TRIPOD", air: 3, lift: [0, 3, 0, 3, 0, 3] },
    { name: "RIPPLE", air: 2, lift: [4, 2, 0, 1, 5, 3] },
    { name: "WAVE", air: 1, lift: [2, 1, 0, 5, 4, 3] },
    { name: "QUAD", air: 2, lift: [4, 2, 0, 2, 0, 4] },
    { name: "AMBLE", air: 2, lift: [4, 0, 2, 2, 4, 0] },
    { name: "METACHRONAL", air: 1, lift: [2, 1, 0, 3, 4, 5] },
  ];
  const STRIDE_X = WALK_X * 0.9, STRIDE_Y = STRIDE_X, TURN_RAD = 10 / R2D * 0.9, HOME_MM = 1;
  const dist2 = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

  class Gait {
    start(now, id, airT) {
      this.id = id; this.airTicks = airT;
      this.cycleTicks = airT * 6 / GAITS[id].air;
      this.t = 0; this.vx = this.vy = this.w = this.tx = this.ty = this.tw = 0;
      this.homing = false; this.homeT = 0;
      const z = homeZ();
      this.foot = now.map(p => P(p.x, p.y, z));
      this.air = new Array(LEGS).fill(false);
      this.liftOff = copy(this.foot);
      this.running = true;
    }
    groundTicks() { return this.cycleTicks - this.airTicks; }
    liftTick(leg) { return Math.floor(GAITS[this.id].lift[leg] * this.cycleTicks / 6); }
    groundVel(p) { return P(this.w * p.y - this.vx, -this.w * p.x - this.vy, 0); }
    vMax() { return 2 * STRIDE_X / this.groundTicks(); }
    vyMax() { return 2 * STRIDE_Y / this.groundTicks(); }
    wMax() { return 2 * TURN_RAD / this.groundTicks(); }
    toward(v, target, full) {
      const r = (Math.abs(target) < Math.abs(v) || target * v < 0) ? 1 / this.airTicks : 1 / this.cycleTicks;
      v += Math.max(-full * r, Math.min(full * r, target - v));
      return Math.abs(v) < full * 1e-4 ? 0 : v;
    }
    target(leg, inTicks) {
      let x = this.vx, y = this.vy, a = this.w, X = 0, Y = 0, A = 0;
      const g = this.groundTicks();
      for (let i = 0; i < inTicks + g; i++) {
        x = this.toward(x, this.tx, this.vMax()); y = this.toward(y, this.ty, this.vyMax()); a = this.toward(a, this.tw, this.wMax());
        if (i >= inTicks) { X += x; Y += y; A += a; }
      }
      const h = home()[leg], c = Math.cos(A / 2), s = Math.sin(A / 2);
      return P(c * h.x - s * h.y + X / 2, s * h.x + c * h.y + Y / 2, h.z);
    }
    settled() {
      if (this.vx || this.vy || this.w) return false;
      const h = home();
      return this.foot.every((f, i) => !this.air[i] && dist2(f, h[i]) <= HOME_MM);
    }
    tick(fwd, left, turn, settle) {
      if (settle) fwd = left = turn = 0;
      const n = Math.hypot(fwd, left) + Math.abs(turn);
      if (n > 1) { fwd /= n; left /= n; turn /= n; }
      this.tx = fwd * this.vMax(); this.ty = left * this.vyMax(); this.tw = turn * this.wMax();
      this.vx = this.toward(this.vx, this.tx, this.vMax());
      this.vy = this.toward(this.vy, this.ty, this.vyMax());
      this.w = this.toward(this.w, this.tw, this.wMax());
      const anyAir = this.air.some(Boolean);
      if (settle && !this.homing && !anyAir && !this.vx && !this.vy && !this.w) { this.homing = true; this.homeT = 0; }
      this.t = (this.t + 1) % this.cycleTicks;
      if (this.homing) this.homeT++;
      const z = homeZ(), c = Math.cos(this.w), s = Math.sin(this.w), h = home();
      for (let i = 0; i < LEGS; i++) {
        let k;
        if (this.homing) {
          const groupB = GAITS[0].lift[i] !== 0;
          k = groupB ? (this.homeT > this.airTicks ? this.homeT - this.airTicks : 0) : this.homeT;
          if (k > this.airTicks) k = 0;
        } else {
          k = (this.t + this.cycleTicks - this.liftTick(i)) % this.cycleTicks;
        }
        const swingNow = k >= 1 && k <= this.airTicks;
        if (k === 1 && !this.air[i] && !(settle && dist2(this.foot[i], h[i]) <= HOME_MM)) {
          this.air[i] = true;
          this.liftOff[i] = P(this.foot[i].x, this.foot[i].y, this.foot[i].z);
        }
        if (this.air[i]) {
          const u = k / this.airTicks, L = this.liftOff[i], T = this.target(i, this.airTicks - k);
          const vL = this.groundVel(L), vT = this.groundVel(T);
          const u2 = u * u, u3 = u2 * u;
          const h00 = 2 * u3 - 3 * u2 + 1, h01 = 3 * u2 - 2 * u3;
          const h10 = (u3 - 2 * u2 + u) * this.airTicks, h11 = (u3 - u2) * this.airTicks;
          const up = 10 * u2 * (1 - u) * (1 - u) * 1.6 * LIFT;
          this.foot[i] = P(h00 * L.x + h01 * T.x + h10 * vL.x + h11 * vT.x,
                           h00 * L.y + h01 * T.y + h10 * vL.y + h11 * vT.y, z + up);
          if (k >= this.airTicks || !swingNow) { this.foot[i] = T; this.air[i] = false; }
        } else {
          const p = this.foot[i];
          this.foot[i] = P(c * p.x + s * p.y - this.vx, -s * p.x + c * p.y - this.vy, z);
        }
      }
      if (this.homing && this.homeT >= 2 * this.airTicks) this.homing = false;
      return copy(this.foot);
    }
  }

  // ---- what he's doing: moves of `ms` each, like his servo line "#..P..T<ms>" ---------------------
  const gait = new Gait();
  gait.running = false;
  let from = home(), to = home(), moveStart = 0, moveMs = 1;
  const queue = [];           // scripted moves (hello, dance, rest): { frame, ms }
  let resting = false, acting = "";
  // where he is in the arena: x, y mm and heading (anticlockwise, radians); the move's start and end
  let pose = { x: 0, y: 0, a: 0 }, poseFrom = { ...pose }, poseTo = { ...pose };

  function glide(target, steps, ms) {
    const start = queue.length ? queue[queue.length - 1].frame : to;
    for (let i = 1; i <= steps; i++) queue.push({ frame: lerpF(start, target, ease(i / steps)), ms });
  }
  function hello() {
    const base = home().map(p => P(p.x + 20, p.y + 15, p.z));
    glide(base, 10, 80);
    const up = copy(base);
    up[LF] = P(base[LF].x + 30, base[LF].y + 20, homeZ() + 120);
    glide(up, 10, 60);
    for (let k = 0; k < 3; k++) {
      const a = copy(up), b = copy(up);
      a[LF].y += 35; b[LF].y -= 35;
      glide(a, 5, 50); glide(b, 7, 50);
    }
    glide(up, 4, 50); glide(base, 10, 60); glide(home(), 10, 80);
  }
  function dance() {
    const h = home(), R = 25, BOB = 15, N = 48;
    for (let i = 0; i <= N; i++) {
      const a = 4 * Math.PI * i / N;
      const k = i < 6 ? ease(i / 6) : i > N - 6 ? ease((N - i) / 6) : 1;
      queue.push({ frame: h.map(p => P(p.x - k * R * Math.cos(a), p.y - k * R * Math.sin(a), p.z + k * BOB * (1 + Math.sin(2 * a)) / 2)), ms: 70 });
    }
    glide(h, 6, 60);
  }
  function act(what) {
    if (queue.length || gait.running) return;   // between moves, like his /act
    acting = what;
    if (what === "rest") {
      if (resting) { resting = false; glide(home(), 20, 100); }
      else { resting = true; glide(home().map(p => P(p.x, p.y, REST_Z)), 20, 100); }
    } else {
      if (resting) { resting = false; glide(home(), 20, 100); }
      what === "hello" ? hello() : dance();
    }
    say(what === "rest" ? (resting ? "Lying down" : "Up again") : what === "hello" ? "Hello!" : "Dance!");
  }

  // ---- input: keyboard, the on-screen pad, a gamepad -------------------------------------------
  const keys = new Set();
  const held = { f: 0, l: 0, t: 0 };   // the on-screen pad
  let padStick = { f: 0, l: 0, t: 0 };
  function drive() {
    let f = 0, l = 0, t = 0;
    if (keys.has("w") || keys.has("arrowup")) f += 1;
    if (keys.has("s") || keys.has("arrowdown")) f -= 1;
    if (keys.has("a") || keys.has("arrowleft")) t += 1;
    if (keys.has("d") || keys.has("arrowright")) t -= 1;
    if (keys.has("q")) l += 1;
    if (keys.has("e")) l -= 1;
    f += held.f + padStick.f; l += held.l + padStick.l; t += held.t + padStick.t;
    const c = v => Math.max(-1, Math.min(1, v));
    if (auto && !resting && !f && !l && !t) return avoid();   // you can always take over: let go and he carries on
    return { f: c(f), l: c(l), t: c(t) };
  }

  // Obstacle mode, as on the real WHIP: he walks by himself and turns away from anything his
  // ultrasonic sensor sees within MIN_DISTANCE_CM, towards the side with more room
  const MIN_DISTANCE_CM = 35, CLEAR_CM = 55, NOSE = 150, CONE = 0.35, LOOK = 0.9;
  let auto = false, turning = 0;
  function avoid() {
    const ahead = sonar(0);
    if (turning && ahead > CLEAR_CM) turning = 0;
    if (!turning && ahead < MIN_DISTANCE_CM) turning = sonar(LOOK) >= sonar(-LOOK) ? 1 : -1;
    return turning ? { f: 0, l: 0, t: turning } : { f: 1, l: 0, t: 0 };
  }
  function setAuto(on) {
    auto = on;
    turning = 0;
    say(auto ? "Obstacle mode: he walks by himself" : "Obstacle mode off");
    $("autoBtn").classList.toggle("on", auto);
  }

  // The next move: a scripted one, a walking tick, settling to a stop, or standing still
  let wantGait = gaitId;
  function nextMove(now) {
    from = to;
    poseFrom = { ...poseTo };
    const d = drive();
    const moving = Math.abs(d.f) > 0.05 || Math.abs(d.l) > 0.05 || Math.abs(d.t) > 0.05;
    if (queue.length) {
      const m = queue.shift();
      to = m.frame; moveMs = m.ms;
      if (!queue.length) acting = "";
    } else if (gait.running && (gait.id !== wantGait || !moving)) {
      to = gait.tick(0, 0, 0, true);     // step to a stop (a new gait starts from the stand)
      moveMs = stepMs();
      advance();
      if (gait.settled()) gait.running = false;
    } else if (moving && !resting) {
      if (!gait.running) { gaitId = wantGait; gait.start(to, gaitId, AIR_TICKS); }
      to = gait.tick(d.f, d.l, d.t, false);
      moveMs = stepMs();
      advance();
    } else {
      if (moving && resting) act("rest");   // driving him wakes him
      moveMs = 60;
    }
    moveStart = now;
  }
  // The body moves over the feet on the ground: the gait's speed, turned into the arena
  function advance() {
    const c = Math.cos(poseTo.a), s = Math.sin(poseTo.a);
    const nx = poseTo.x + gait.vx * c - gait.vy * s, ny = poseTo.y + gait.vx * s + gait.vy * c;
    if (!hitsPillar(nx, ny)) { poseTo.x = nx; poseTo.y = ny; }
    else {
      if (auto && !turning) turning = sonar(LOOK) >= sonar(-LOOK) ? 1 : -1;   // the sensor missed it: his leg didn't
      if (!bumped) { bumped = true; say("Bump! A pillar in the way"); setTimeout(() => (bumped = false), 1500); }
    }
    poseTo.a += gait.w;
  }
  let bumped = false;

  // ---- the arena: pillars and orbs -------------------------------------------------------------
  const ARENA = 2400;   // mm across, his walls
  const PILLARS = [[700, 500], [-600, 700], [-800, -500], [500, -800], [0, 1300], [1300, 0]];
  const PILLAR_R = 90, BODY_R = 170;
  function hitsPillar(x, y) {
    if (Math.abs(x) > ARENA / 2 - BODY_R || Math.abs(y) > ARENA / 2 - BODY_R) return true;
    return PILLARS.some(([px, py]) => Math.hypot(x - px, y - py) < PILLAR_R + BODY_R);
  }
  // What his sensor reads, cm, looking `off` radians from straight ahead: its cone is three rays
  function ray(x, y, dir) {
    const c = Math.cos(dir), s = Math.sin(dir), edge = ARENA / 2;
    let hit = Math.min(c ? ((c > 0 ? edge : -edge) - x) / c : Infinity, s ? ((s > 0 ? edge : -edge) - y) / s : Infinity);
    PILLARS.forEach(([px, py]) => {
      const along = (px - x) * c + (py - y) * s;
      const miss = Math.hypot(px - x - along * c, py - y - along * s);
      if (along > 0 && miss < PILLAR_R) hit = Math.min(hit, along - Math.sqrt(PILLAR_R * PILLAR_R - miss * miss));
    });
    return hit;
  }
  function sonar(off) {
    const dir = poseTo.a + off;
    const mm = Math.min(ray(poseTo.x, poseTo.y, dir), ray(poseTo.x, poseTo.y, dir + CONE), ray(poseTo.x, poseTo.y, dir - CONE));
    return Math.max(0, (mm - NOSE) / 10);
  }
  let orbs = [], score = 0;
  function newOrb() {
    for (let tries = 0; tries < 50; tries++) {
      const x = (Math.random() - 0.5) * (ARENA - 500), y = (Math.random() - 0.5) * (ARENA - 500);
      if (!hitsPillar(x, y) && Math.hypot(x - pose.x, y - pose.y) > 500) return { x, y, el: null };
    }
    return { x: 900, y: 900, el: null };
  }

  // ---- drawing: CSS 3D. 1 px = 1 mm; the floor at z 0, his feet on it -------------------------
  const $ = id => document.getElementById(id);
  const world = $("world"), bot = $("bot"), floor = $("floor"), bodyShadow = $("bodyShadow");
  const SEG = [];   // 6 legs x 3 segments: coxa, femur, tibia
  const SHADOW = [];
  function bar(cls, thick) {
    const el = document.createElement("div");
    el.className = "seg " + cls;
    el.style.height = thick + "px";
    el.style.marginTop = (-thick / 2) + "px";
    const side = document.createElement("i");   // a second face, at right angles: he has depth from every side
    el.appendChild(side);
    bot.appendChild(el);
    return el;
  }
  for (let l = 0; l < LEGS; l++) {
    SEG.push([bar("coxa", 22), bar("femur", 18), bar("tibia", 14)]);
    const sh = document.createElement("div");
    sh.className = "foot-shadow";
    floor.appendChild(sh);
    SHADOW.push(sh);
  }
  // pillars and the walls are on the floor's own frame; orbs too
  PILLARS.forEach(([x, y]) => {
    const p = document.createElement("div");
    p.className = "pillar";
    p.style.transform = `translate3d(${x}px, ${-y}px, 0)`;
    for (let i = 0; i < 4; i++) { const f = document.createElement("i"); f.style.transform = `rotateZ(${i * 90}deg) translateX(${PILLAR_R}px) translateZ(110px) rotateY(90deg)`; p.appendChild(f); }
    const top = document.createElement("b"); p.appendChild(top);
    floor.appendChild(p);
  });
  function placeOrb(o) {
    o.el = document.createElement("div");
    o.el.className = "orb";
    o.el.style.transform = `translate3d(${o.x}px, ${-o.y}px, 0)`;
    o.el.innerHTML = "<i></i><b></b>";
    floor.appendChild(o.el);
  }
  for (let i = 0; i < 5; i++) { const o = newOrb(); placeOrb(o); orbs.push(o); }

  function place(el, a, b, zOff) {   // a segment from a to b (body frame, mm)
    const ax = a.x, ay = -a.y, az = a.z - zOff, bx = b.x, by = -b.y, bz = b.z - zOff;
    const dx = bx - ax, dy = by - ay, dz = bz - az, flat = Math.hypot(dx, dy);
    el.style.width = Math.hypot(flat, dz) + "px";
    el.style.transform = `translate3d(${ax}px, ${ay}px, ${az}px) rotateZ(${Math.atan2(dy, dx) * R2D}deg) rotateY(${-Math.atan2(dz, flat) * R2D}deg)`;
  }

  let cam = { tilt: 58, spin: -30, zoom: 1 };
  let servoText = "";
  function render(frame, p) {
    const ground = homeZ();   // the floor, below his hips (body frame z 0)
    // his body: the hips at z 0 -> the floor is `ground` below them
    bot.style.transform = `translate3d(${p.x}px, ${-p.y}px, ${-ground}px) rotateZ(${-p.a * R2D}deg)`;
    bodyShadow.style.transform = `translate3d(${p.x}px, ${-p.y}px, 1px) rotateZ(${-p.a * R2D}deg)`;
    world.style.transform = `scale(${cam.zoom}) rotateX(${cam.tilt}deg) rotateZ(${cam.spin}deg) translate3d(${-p.x}px, ${p.y}px, 0)`;
    const angles = [];
    for (let l = 0; l < LEGS; l++) {
      const a = solve(l, frame[l]) || [0, 0, 0];
      angles.push(a);
      const j = joints(l, a);
      place(SEG[l][0], j[0], j[1], 0);
      place(SEG[l][1], j[1], j[2], 0);
      place(SEG[l][2], j[2], j[3], 0);
      // the foot's shadow on the floor, in arena coordinates
      const c = Math.cos(p.a), s = Math.sin(p.a);
      const fx = p.x + j[3].x * c - j[3].y * s, fy = p.y + j[3].x * s + j[3].y * c;
      const up = Math.max(0, j[3].z - ground);
      SHADOW[l].style.transform = `translate3d(${fx}px, ${-fy}px, 1px) scale(${1 + up / 60})`;
      SHADOW[l].style.opacity = String(Math.max(0.15, 0.7 - up / 90));
    }
    servoText = ["LF", "LM", "LB", "RF", "RM", "RB"].map((n, l) =>
      `${n} ${angles[l].map(d => String(Math.round(1500 + d * US_PER_DEG)).padStart(4)).join(" ")}`).join("\n");
  }

  // ---- HUD -----------------------------------------------------------------------------------
  let toastTimer = 0;
  function say(text) {
    const t = $("toast");
    t.textContent = text;
    t.classList.add("on");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => t.classList.remove("on"), 1600);
  }
  function hud() {
    $("hudGait").textContent = GAITS[wantGait].name;
    $("hudSpeed").textContent = speed;
    $("hudHeight").textContent = height;
    $("hudScore").textContent = score;
    $("hudState").textContent = resting ? "resting" : acting ? acting : auto ? (turning ? "avoiding" : "obstacle") : gait.running ? "walking" : "standing";
    $("hudSonar").textContent = Math.round(sonar(0)) + " cm";
    $("servos").textContent = servoText;
    document.querySelectorAll("[data-gait]").forEach(b => b.classList.toggle("on", +b.dataset.gait === wantGait));
  }

  // ---- the loop: moves at his pace, drawn at the screen's ------------------------------------
  function frameLoop(now) {
    pollGamepad();
    if (now - moveStart >= moveMs) nextMove(now);
    const k = Math.min(1, (now - moveStart) / moveMs);   // a servo moves evenly through its T
    const f = lerpF(from, to, k);
    const pz = { x: poseFrom.x + (poseTo.x - poseFrom.x) * k, y: poseFrom.y + (poseTo.y - poseFrom.y) * k,
                 a: poseFrom.a + (poseTo.a - poseFrom.a) * k };
    pose = pz;
    render(f, pz);
    orbs.forEach((o, i) => {
      if (Math.hypot(o.x - pz.x, o.y - pz.y) < 150) {
        o.el.remove();
        score++;
        say(score % 5 ? "Orb! " + score : score + " orbs: WHIP dances");
        if (score % 5 === 0 && !gait.running) act("dance");
        orbs[i] = newOrb();
        placeOrb(orbs[i]);
      }
    });
    hud();
    requestAnimationFrame(frameLoop);
  }

  // ---- controls ------------------------------------------------------------------------------
  function setGait(i) { wantGait = (i + GAITS.length) % GAITS.length; say(GAITS[wantGait].name); }
  function setSpeed(v) { speed = Math.max(0, Math.min(9, v)); say("Speed " + speed + " (" + stepMs() + " ms a move)"); }
  function setHeight(v) {
    if (gait.running || queue.length) return;
    height = Math.max(0, Math.min(9, v));
    glide(home(), 10, 60);
    say("Height " + height + " (" + Math.round(-homeZ()) + " mm)");
  }
  window.addEventListener("keydown", e => {
    if (e.target.closest && e.target.closest("input, select, textarea")) return;
    const k = e.key.toLowerCase();
    if (["w", "a", "s", "d", "q", "e", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(k)) { keys.add(k); e.preventDefault(); return; }
    if (e.repeat) return;
    if (k === "g") setGait(wantGait + 1);
    else if (k >= "1" && k <= "6") setGait(+k - 1);
    else if (k === "h") act("hello");
    else if (k === "n") act("dance");
    else if (k === "r") act("rest");
    else if (k === "o") setAuto(!auto);
    else if (k === "+" || k === "=") setSpeed(speed + 1);
    else if (k === "-") setSpeed(speed - 1);
    else if (k === "pageup" || k === "]") setHeight(height + 1);
    else if (k === "pagedown" || k === "[") setHeight(height - 1);
  });
  window.addEventListener("keyup", e => keys.delete(e.key.toLowerCase()));
  window.addEventListener("blur", () => keys.clear());

  // the pad: hold to walk (pointer events, so touch and mouse alike)
  document.querySelectorAll("[data-drive]").forEach(b => {
    const [f, l, t] = b.dataset.drive.split(",").map(Number);
    const down = e => { e.preventDefault(); b.setPointerCapture && b.setPointerCapture(e.pointerId); held.f = f; held.l = l; held.t = t; b.classList.add("on"); };
    const up = () => { held.f = held.l = held.t = 0; b.classList.remove("on"); };
    b.addEventListener("pointerdown", down);
    b.addEventListener("pointerup", up);
    b.addEventListener("pointercancel", up);
    b.addEventListener("lostpointercapture", up);
  });
  document.querySelectorAll("[data-act]").forEach(b => b.addEventListener("click", () => act(b.dataset.act)));
  document.querySelectorAll("[data-gait]").forEach(b => b.addEventListener("click", () => setGait(+b.dataset.gait)));
  $("autoBtn").addEventListener("click", () => setAuto(!auto));
  $("speedUp").addEventListener("click", () => setSpeed(speed + 1));
  $("speedDown").addEventListener("click", () => setSpeed(speed - 1));
  $("heightUp").addEventListener("click", () => setHeight(height + 1));
  $("heightDown").addEventListener("click", () => setHeight(height - 1));

  // the camera: drag to look round him, wheel / pinch to zoom
  const stage = $("stage");
  let drag = null;
  stage.addEventListener("pointerdown", e => { drag = { x: e.clientX, y: e.clientY, tilt: cam.tilt, spin: cam.spin }; stage.setPointerCapture(e.pointerId); });
  stage.addEventListener("pointermove", e => {
    if (!drag) return;
    cam.spin = drag.spin + (e.clientX - drag.x) * 0.4;
    cam.tilt = Math.max(15, Math.min(80, drag.tilt - (e.clientY - drag.y) * 0.3));
  });
  stage.addEventListener("pointerup", () => (drag = null));
  stage.addEventListener("wheel", e => { e.preventDefault(); cam.zoom = Math.max(0.5, Math.min(2.2, cam.zoom * (e.deltaY > 0 ? 0.92 : 1.08))); }, { passive: false });
  function fit() { cam.zoom = Math.max(0.55, Math.min(1.3, stage.clientWidth / 900)); }
  window.addEventListener("resize", fit);
  fit();

  // a gamepad: left stick walks and turns, right stick crabs; A hello, B dance, X rest, Y obstacle, LB / RB gait
  const padWas = [];
  function pollGamepad() {
    const gp = navigator.getGamepads ? [...navigator.getGamepads()].find(Boolean) : null;
    if (!gp) { padStick = { f: 0, l: 0, t: 0 }; return; }
    const dz = v => (Math.abs(v) < 0.15 ? 0 : v);
    padStick = { f: -dz(gp.axes[1] || 0), t: -dz(gp.axes[0] || 0), l: -dz(gp.axes[2] || 0) };
    const press = i => { const on = gp.buttons[i] && gp.buttons[i].pressed, was = padWas[i]; padWas[i] = on; return on && !was; };
    if (press(0)) act("hello");
    if (press(1)) act("dance");
    if (press(2)) act("rest");
    if (press(3)) setAuto(!auto);
    if (press(4)) setGait(wantGait - 1);
    if (press(5)) setGait(wantGait + 1);
    if (press(12)) setSpeed(speed + 1);
    if (press(13)) setSpeed(speed - 1);
  }
  window.addEventListener("gamepadconnected", () => say("Gamepad connected"));

  // ---- go --------------------------------------------------------------------------------------
  render(home(), pose);
  say("W A S D to walk, Q / E to crab");
  if (location.hash === "#demo") setAuto(true);   // play.html#demo: he walks off by himself
  requestAnimationFrame(t => { moveStart = t; frameLoop(t); });
})();

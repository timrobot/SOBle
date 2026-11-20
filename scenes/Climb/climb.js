/**
 * Climb + omnidrive — one gamepad skill for wheels and leg.
 *
 * Sticks: lx / ly / rx → four 45° omni wheels.
 * Buttons (climb.py 1–4):
 *   a → height 10.7 in   b → forward 11.6 in (+wheels if height > 5)
 *   y → height 2.06 in   x → forward 1.05 in (−wheels if height > 5)
 *
 * Latched a/b/y/x cannot be interrupted. Each tick plans at most **1 in** of
 * pure height **or** pure forward (never both), with a **0.1 s** joint lookahead
 * capped at **3 rpm**, J3 always from current J1/J2 (foot ∥ body). If the
 * inactive axis would drift: **height** moves constrain **J2** first (then J1);
 * **forward** moves constrain **J1** first (then J2) — model.md.
 */

const INCH = 0.0254;
const L1 = 7.5 * INCH;
const L2 = 8.5 * INCH;
const VERTICAL_OFFSET_IN = 1.75;
const REACH_LIMIT_IN = 14.8;

const HEIGHT_MIN_IN = 2.06;
const HEIGHT_MAX_IN = 10.7;
const FORWARD_MIN_IN = 1.05;
const FORWARD_MAX_IN = 11.6;
const CLIMB_HEIGHT_IN = 10.7;
const KEY3_HEIGHT_IN = 2.06;
const KEY2_FORWARD_IN = 11.6;
const HOME_FORWARD_IN = 1.05;
const WHEEL_ROLL_MIN_HEIGHT_IN = 5.0;
/** Drive wheels: ±100 rpm → rad/s (joint 3 rpm is separate — MAX_RPM below). */
const WHEEL_RPM = 100;
const WHEEL_RAD_S = (WHEEL_RPM * 2 * Math.PI) / 60;
const KEY2_WHEEL = WHEEL_RAD_S;
const KEY4_WHEEL = -WHEEL_RAD_S;

/** model.md joint planner */
const LOOKAHEAD_S = 0.1;
const MAX_RPM = 3;
const CHUNK_IN = 1.0;
const MAX_DEG = (MAX_RPM * 360) / 60 * LOOKAHEAD_S; // 1.8°
const SETTLE_S = 0.25;
const SETTLE_TOL_IN = 0.05;
const AXIS_TOL_M = 0.0005; // ~0.02 in — inactive-axis drift threshold

const J1_MIN_DEG = -90;
const J1_MAX_DEG = 90;
const J2_MIN_DEG = 0;
const J2_MAX_DEG = 165;
const J3_MIN_DEG = 0;
const J3_MAX_DEG = 155;

const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;

function clamp(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}

function rateLimit(cur, tgt, maxStep) {
  return cur + clamp(tgt - cur, -maxStep, maxStep);
}

function ankleDropIn(heightIn) {
  return heightIn;
}

/** Hip / knee / ankle degrees for level body + level foot. */
function legJointsDeg(heightIn, forwardIn) {
  const drop = ankleDropIn(heightIn) * INCH;
  const forward = forwardIn * INCH;
  const reach = Math.hypot(forward, drop);
  if (reach < 1e-8) return [0, 0, 180];
  let cosQ2 = (reach * reach - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  cosQ2 = clamp(cosQ2, -1, 1);
  const q2 = Math.acos(cosQ2);
  let cosHip = (L1 * L1 + reach * reach - L2 * L2) / (2 * L1 * reach);
  cosHip = clamp(cosHip, -1, 1);
  const hip = Math.acos(cosHip);
  const slope = Math.atan2(drop, forward);
  const q1 = slope - hip;
  const q3 = Math.PI - q1 - q2;
  return [q1 * DEG, q2 * DEG, q3 * DEG];
}

function jointsInLimits(j1, j2, j3) {
  return (
    j1 >= J1_MIN_DEG - 1e-6 && j1 <= J1_MAX_DEG + 1e-6
    && j2 >= J2_MIN_DEG - 1e-6 && j2 <= J2_MAX_DEG + 1e-6
    && j3 >= J3_MIN_DEG - 1e-6 && j3 <= J3_MAX_DEG + 1e-6
  );
}

function withinReach(heightIn, forwardIn) {
  const vertical = heightIn - VERTICAL_OFFSET_IN;
  const disk = forwardIn * forwardIn + vertical * vertical;
  const span = Math.hypot(forwardIn, ankleDropIn(heightIn));
  const linkLimit = (L1 + L2) / INCH;
  if (!(disk < REACH_LIMIT_IN * REACH_LIMIT_IN && span <= linkLimit + 1e-9)) {
    return false;
  }
  const [j1, j2, j3] = legJointsDeg(heightIn, forwardIn);
  return jointsInLimits(j1, j2, j3);
}

/** Ankle in hip frame (m): +x forward-of-hip is negative in our FK; +z up. */
function anklePos(j1Deg, j2Deg) {
  const q1 = j1Deg * RAD;
  const q2 = j2Deg * RAD;
  const x = -L1 * Math.cos(q1) - L2 * Math.cos(q1 + q2);
  const z = -L1 * Math.sin(q1) - L2 * Math.sin(q1 + q2);
  return { x, z };
}

function stanceFromJoints(j1Deg, j2Deg) {
  const { x, z } = anklePos(j1Deg, j2Deg);
  return {
    height: -z / INCH,
    forward: -x / INCH,
  };
}

function planarJ3(j1Deg, j2Deg) {
  return clamp(180 - j1Deg - j2Deg, J3_MIN_DEG, J3_MAX_DEG);
}

function readNum(map, name, fallback = 0) {
  const v = Number(map?.[name]);
  return Number.isFinite(v) ? v : fallback;
}

function padPressed(gamepad, name) {
  return (gamepad?.[name] ?? 0) > 0.5;
}

/** Pick J2 in range so ankle x matches, nearest to hint (model.md J2 constraint). */
function solveJ2ForX(j1Deg, xTarg, j2Hint) {
  const j1 = j1Deg * RAD;
  const cosSum = (-xTarg - L1 * Math.cos(j1)) / L2;
  if (Math.abs(cosSum) > 1.0001) return null;
  const sum = Math.acos(clamp(cosSum, -1, 1));
  const cands = [(sum - j1) * DEG, (-sum - j1) * DEG];
  let best = null;
  let bestDist = Infinity;
  for (const q2 of cands) {
    if (q2 < J2_MIN_DEG - 1e-3 || q2 > J2_MAX_DEG + 1e-3) continue;
    const d = Math.abs(q2 - j2Hint);
    if (d < bestDist) {
      bestDist = d;
      best = clamp(q2, J2_MIN_DEG, J2_MAX_DEG);
    }
  }
  return best;
}

function solveJ2ForZ(j1Deg, zTarg, j2Hint) {
  const j1 = j1Deg * RAD;
  const sinSum = (-zTarg - L1 * Math.sin(j1)) / L2;
  if (Math.abs(sinSum) > 1.0001) return null;
  const s = clamp(sinSum, -1, 1);
  const sumA = Math.asin(s);
  const sumB = Math.PI - sumA;
  const cands = [(sumA - j1) * DEG, (sumB - j1) * DEG];
  let best = null;
  let bestDist = Infinity;
  for (const q2 of cands) {
    if (q2 < J2_MIN_DEG - 1e-3 || q2 > J2_MAX_DEG + 1e-3) continue;
    const d = Math.abs(q2 - j2Hint);
    if (d < bestDist) {
      bestDist = d;
      best = clamp(q2, J2_MIN_DEG, J2_MAX_DEG);
    }
  }
  return best;
}

/** Binary-search J1 for target ankle x with J2 fixed. */
function solveJ1ForX(j2Deg, xTarg, j1Hint) {
  let best = null;
  let bestErr = Infinity;
  const lo = J1_MIN_DEG;
  const hi = J1_MAX_DEG;
  // Coarse then refine around hint
  for (let q1 = lo; q1 <= hi; q1 += 1) {
    const { x } = anklePos(q1, j2Deg);
    const err = Math.abs(x - xTarg);
    if (err < bestErr) {
      bestErr = err;
      best = q1;
    }
  }
  if (best == null) return null;
  let a = Math.max(lo, best - 1);
  let b = Math.min(hi, best + 1);
  for (let i = 0; i < 24; i += 1) {
    const mid = 0.5 * (a + b);
    const xMid = anklePos(mid, j2Deg).x;
    const xA = anklePos(a, j2Deg).x;
    if (Math.abs(xMid - xTarg) < Math.abs(xA - xTarg)) a = mid;
    else b = mid;
    best = mid;
  }
  return Math.abs(anklePos(best, j2Deg).x - xTarg) < 0.01 ? best : null;
}

function solveJ1ForZ(j2Deg, zTarg, j1Hint) {
  let best = null;
  let bestErr = Infinity;
  for (let q1 = J1_MIN_DEG; q1 <= J1_MAX_DEG; q1 += 1) {
    const err = Math.abs(anklePos(q1, j2Deg).z - zTarg);
    if (err < bestErr) {
      bestErr = err;
      best = q1;
    }
  }
  if (best == null) return null;
  let a = Math.max(J1_MIN_DEG, best - 1);
  let b = Math.min(J1_MAX_DEG, best + 1);
  for (let i = 0; i < 24; i += 1) {
    const mid = 0.5 * (a + b);
    if (Math.abs(anklePos(mid, j2Deg).z - zTarg) < Math.abs(anklePos(a, j2Deg).z - zTarg)) a = mid;
    else b = mid;
    best = mid;
  }
  return Math.abs(anklePos(best, j2Deg).z - zTarg) < 0.01 ? best : null;
}

/** Enforce planar J3 within rate limit by scaling J1/J2 deltas if needed. */
function applyPlanarJ3(j1, j2, j3, n1, n2) {
  let out1 = n1;
  let out2 = n2;
  let out3 = planarJ3(out1, out2);
  if (Math.abs(out3 - j3) > MAX_DEG + 1e-9) {
    const scale = MAX_DEG / Math.abs(out3 - j3);
    out1 = j1 + (out1 - j1) * scale;
    out2 = j2 + (out2 - j2) * scale;
    out3 = planarJ3(out1, out2);
  }
  return {
    j1: clamp(out1, J1_MIN_DEG, J1_MAX_DEG),
    j2: clamp(out2, J2_MIN_DEG, J2_MAX_DEG),
    j3: clamp(out3, J3_MIN_DEG, J3_MAX_DEG),
  };
}

/**
 * One lookahead step: ≤1 in on the active axis, 3 rpm joint caps, inactive
 * axis held (height→J2 then J1; forward→J1 then J2).
 */
function planJointStep(j1, j2, j3, axis, targetH, targetF, curH, curF) {
  let goalH = curH;
  let goalF = curF;
  if (axis === 'height') {
    goalF = targetF;
    goalH = curH + clamp(targetH - curH, -CHUNK_IN, CHUNK_IN);
    goalH = clamp(goalH, HEIGHT_MIN_IN, HEIGHT_MAX_IN);
  } else {
    goalH = targetH;
    goalF = curF + clamp(targetF - curF, -CHUNK_IN, CHUNK_IN);
    goalF = clamp(goalF, FORWARD_MIN_IN, FORWARD_MAX_IN);
  }
  if (!withinReach(goalH, goalF)) {
    // Shrink toward current until reachable.
    for (let i = 0; i < 16; i += 1) {
      goalH = 0.5 * (curH + goalH);
      goalF = 0.5 * (curF + goalF);
      if (withinReach(goalH, goalF)) break;
    }
    if (!withinReach(goalH, goalF)) {
      return { j1, j2, j3: planarJ3(j1, j2) };
    }
  }

  const [j1t, j2t] = legJointsDeg(goalH, goalF);
  let n1 = rateLimit(j1, j1t, MAX_DEG);
  let n2 = rateLimit(j2, j2t, MAX_DEG);
  let planned = applyPlanarJ3(j1, j2, j3, n1, n2);
  n1 = planned.j1;
  n2 = planned.j2;
  let n3 = planned.j3;

  const pStep = anklePos(n1, n2);
  if (axis === 'height') {
    // Height travel: hold forward (x) — constrain J2 first, then J1.
    const xTarg = -goalF * INCH;
    if (Math.abs(pStep.x - xTarg) > AXIS_TOL_M) {
      const j2c = solveJ2ForX(n1, xTarg, n2);
      if (j2c != null && Math.abs(j2c - j2) <= MAX_DEG + 1e-6) {
        n2 = j2c;
      } else {
        const j1c = solveJ1ForX(n2, xTarg, n1);
        if (j1c != null && Math.abs(j1c - j1) <= MAX_DEG + 1e-6) {
          n1 = j1c;
        } else if (j2c != null) {
          n2 = rateLimit(j2, j2c, MAX_DEG);
        } else if (j1c != null) {
          n1 = rateLimit(j1, j1c, MAX_DEG);
        }
      }
      planned = applyPlanarJ3(j1, j2, j3, n1, n2);
      n1 = planned.j1;
      n2 = planned.j2;
      n3 = planned.j3;
    }
  } else {
    // Forward travel: hold height (z) — constrain J1 first, then J2.
    const zTarg = -goalH * INCH;
    if (Math.abs(pStep.z - zTarg) > AXIS_TOL_M) {
      const j1c = solveJ1ForZ(n2, zTarg, n1);
      if (j1c != null && Math.abs(j1c - j1) <= MAX_DEG + 1e-6) {
        n1 = j1c;
      } else {
        const j2c = solveJ2ForZ(n1, zTarg, n2);
        if (j2c != null && Math.abs(j2c - j2) <= MAX_DEG + 1e-6) {
          n2 = j2c;
        } else if (j1c != null) {
          n1 = rateLimit(j1, j1c, MAX_DEG);
        } else if (j2c != null) {
          n2 = rateLimit(j2, j2c, MAX_DEG);
        }
      }
      planned = applyPlanarJ3(j1, j2, j3, n1, n2);
      n1 = planned.j1;
      n2 = planned.j2;
      n3 = planned.j3;
    }
  }

  return { j1: n1, j2: n2, j3: n3 };
}

function omniWheels(gamepad) {
  if (!gamepad) {
    return {
      front_left_mecanum_wheel: 0,
      front_right_mecanum_wheel: 0,
      rear_left_mecanum_wheel: 0,
      rear_right_mecanum_wheel: 0,
    };
  }
  const lx = gamepad.lx ?? 0;
  const fwd = -(gamepad.ly ?? 0);
  const rx = gamepad.rx ?? 0;
  return {
    front_left_mecanum_wheel: clamp((lx + fwd + rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
    front_right_mecanum_wheel: clamp((-lx + fwd - rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
    rear_left_mecanum_wheel: clamp((-lx + fwd + rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
    rear_right_mecanum_wheel: clamp((lx + fwd - rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
  };
}

function tankWheels(cmd) {
  const v = clamp(cmd, -WHEEL_RAD_S, WHEEL_RAD_S);
  return {
    front_left_mecanum_wheel: v,
    front_right_mecanum_wheel: v,
    rear_left_mecanum_wheel: v,
    rear_right_mecanum_wheel: v,
  };
}

let mode = 'neutral';
let axis = null;
let targetHeight = null;
let targetForward = null;
let timeStart = 0;
let timeEnd = 0;
let settleAcc = 0;
let lastNow = null;
let prevPad = { a: false, b: false, y: false, x: false };
/** Last commanded joints (seeded from sensors). */
let cmdJ1 = null;
let cmdJ2 = null;
let cmdJ3 = null;

function goNeutral() {
  mode = 'neutral';
  axis = null;
  targetHeight = null;
  targetForward = null;
  timeEnd = performance.now() / 1000;
  settleAcc = 0;
}

function startCommand(nextMode, nextAxis, tgtH, tgtF, now) {
  mode = nextMode;
  axis = nextAxis;
  targetHeight = tgtH;
  targetForward = tgtF;
  timeStart = now;
  timeEnd = 0;
  settleAcc = 0;
}

function atTarget(curH, curF) {
  if (axis === 'height') return Math.abs(curH - targetHeight) <= SETTLE_TOL_IN;
  if (axis === 'forward') return Math.abs(curF - targetForward) <= SETTLE_TOL_IN;
  return true;
}

export function step(input_output, _input, _iface, gamepad) {
  const now = performance.now() / 1000;
  const dt = lastNow == null ? 0 : Math.min(Math.max(now - lastNow, 0), 0.05);
  lastNow = now;

  const sensJ1 = readNum(input_output, 'J1', -45);
  const sensJ2 = readNum(input_output, 'J2', 165);
  const sensJ3 = readNum(input_output, 'J3', planarJ3(sensJ1, sensJ2));
  const { height: curH, forward: curF } = stanceFromJoints(sensJ1, sensJ2);

  if (cmdJ1 == null) {
    cmdJ1 = sensJ1;
    cmdJ2 = sensJ2;
    cmdJ3 = planarJ3(sensJ1, sensJ2);
  }

  const a = padPressed(gamepad, 'a');
  const b = padPressed(gamepad, 'b');
  const y = padPressed(gamepad, 'y');
  const x = padPressed(gamepad, 'x');

  if (mode === 'neutral') {
    if (a && !prevPad.a) startCommand('a', 'height', CLIMB_HEIGHT_IN, curF, now);
    else if (b && !prevPad.b) startCommand('b', 'forward', curH, KEY2_FORWARD_IN, now);
    else if (y && !prevPad.y) startCommand('y', 'height', KEY3_HEIGHT_IN, curF, now);
    else if (x && !prevPad.x) startCommand('x', 'forward', curH, HOME_FORWARD_IN, now);
  }
  prevPad = { a, b, y, x };

  if (mode !== 'neutral') {
    // Plan from last command (smooth), constrain with sensors for settle.
    const planned = planJointStep(
      cmdJ1, cmdJ2, cmdJ3,
      axis, targetHeight, targetForward,
      curH, curF,
    );
    cmdJ1 = planned.j1;
    cmdJ2 = planned.j2;
    cmdJ3 = planned.j3;

    if (atTarget(curH, curF)) {
      settleAcc += dt;
      if (settleAcc >= SETTLE_S) goNeutral();
    } else {
      settleAcc = 0;
    }
  } else {
    cmdJ1 = sensJ1;
    cmdJ2 = sensJ2;
    cmdJ3 = planarJ3(sensJ1, sensJ2);
  }

  let wheels;
  if (mode === 'b' && curH > WHEEL_ROLL_MIN_HEIGHT_IN + 1e-9 && !atTarget(curH, curF)) {
    wheels = tankWheels(KEY2_WHEEL);
  } else if (mode === 'x' && curH > WHEEL_ROLL_MIN_HEIGHT_IN + 1e-9 && !atTarget(curH, curF)) {
    wheels = tankWheels(KEY4_WHEEL);
  } else {
    wheels = omniWheels(gamepad);
  }

  return {
    ...wheels,
    J1: clamp(cmdJ1, J1_MIN_DEG, J1_MAX_DEG),
    J2: clamp(cmdJ2, J2_MIN_DEG, J2_MAX_DEG),
    J3: clamp(cmdJ3, J3_MIN_DEG, J3_MAX_DEG),
  };
}

export function update(input_output, _input, _iface, _gamepad) {
  const curJ1 = readNum(input_output, 'J1', -45);
  const curJ2 = readNum(input_output, 'J2', 165);
  const { height, forward } = stanceFromJoints(curJ1, curJ2);
  return { height, forward };
}

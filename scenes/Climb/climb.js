/**
 * Climb + omnidrive — one gamepad skill for wheels and leg.
 *
 * Sticks:
 *   generic / steam: lx / ly / rx → four 45° omni wheels.
 *   mobile:          -ly forward, lx yaw right → differential drive.
 * Buttons (climb.py 1–4):
 *   a → height 10.4 in   b → forward 11.66 in (+60 rpm wheels if height > 5 in)
 *   y → height 2.05 in   x → forward 1.07 in (−60 rpm wheels if height > 5 in)
 *
 * Runs at the 10 Hz (`LIFT_DT = 0.1 s`) motor_control plan rate (matching
 * stair_climber/env.py and hardware transmission). Position targets are
 * linearly ramped between 10 Hz packets at the physics / firmware rate.
 * Pressing a/b/y/x immediately switches the active target (no settle tolerance
 * or latch wait), holding the other axis at its current commanded value.
 */

const INCH = 0.0254;
const L1 = 7.5 * INCH;
const L2 = 8.5 * INCH;
const VERTICAL_OFFSET_IN = 1.75;
const REACH_LIMIT_IN = 14.8;

const CLIMB_HEIGHT_IN = 10.4;
const KEY3_HEIGHT_IN = 2.05;
const KEY2_FORWARD_IN = 11.66;
const HOME_FORWARD_IN = 1.07;
const WHEEL_ROLL_MIN_HEIGHT_IN = 5.0;

/** Drive wheels: ±75 rpm → rad/s; B/X roll at ±60 rpm (matching climb.py). */
const WHEEL_RPM = 75;
const WHEEL_RAD_S = (WHEEL_RPM * 2 * Math.PI) / 60;
const KEY2_WHEEL = (60 * 2 * Math.PI) / 60;
const KEY4_WHEEL = (-60 * 2 * Math.PI) / 60;

/** 10 Hz plan interval and leg speed caps (matching env.py: 15, 15, 20 rpm). */
const LIFT_DT = 0.1;
const J1_MAX_RPM = 15;
const J2_MAX_RPM = 15;
const J3_MAX_RPM = 20;
const CAP_J1_DEG = ((J1_MAX_RPM * 360) / 60) * LIFT_DT; // 9.0° per 0.1 s
const CAP_J2_DEG = ((J2_MAX_RPM * 360) / 60) * LIFT_DT; // 9.0° per 0.1 s
const CAP_J3_DEG = ((J3_MAX_RPM * 360) / 60) * LIFT_DT; // 12.0° per 0.1 s

const J1_MIN_DEG = -90;
const J1_MAX_DEG = 90;
const J2_MIN_DEG = 0;
const J2_MAX_DEG = 165;
const J3_MIN_DEG = 0;
const J3_MAX_DEG = 155;

const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;

const J1_MIN_RAD = J1_MIN_DEG * RAD;
const J1_MAX_RAD = J1_MAX_DEG * RAD;
const J2_MIN_RAD = J2_MIN_DEG * RAD;
const J2_MAX_RAD = J2_MAX_DEG * RAD;
const J3_MIN_RAD = J3_MIN_DEG * RAD;
const J3_MAX_RAD = J3_MAX_DEG * RAD;
const CAP_J1_RAD = CAP_J1_DEG * RAD;
const CAP_J2_RAD = CAP_J2_DEG * RAD;
const CAP_J3_RAD = CAP_J3_DEG * RAD;

function clamp(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}

function rateLimit(cur, tgt, maxStep) {
  return cur + clamp(tgt - cur, -maxStep, maxStep);
}

function ankleDropIn(heightIn) {
  return heightIn;
}

/** Hip / knee / ankle radians for level body + level foot. */
function legJointsRad(heightIn, forwardIn) {
  const drop = ankleDropIn(heightIn) * INCH;
  const forward = forwardIn * INCH;
  const reach = Math.hypot(forward, drop);
  if (reach < 1e-8) return [0, 0, Math.PI];
  const cosQ2 = clamp((reach * reach - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1);
  const q2 = Math.acos(cosQ2);
  const cosHip = clamp((L1 * L1 + reach * reach - L2 * L2) / (2 * L1 * reach), -1, 1);
  const hip = Math.acos(cosHip);
  const slope = Math.atan2(drop, forward);
  const q1 = slope - hip;
  const q3 = Math.PI - q1 - q2;
  return [q1, q2, q3];
}

function jointsInLimitsRad(q1, q2, q3) {
  return (
    q1 >= J1_MIN_RAD - 1e-9 && q1 <= J1_MAX_RAD + 1e-9
    && q2 >= J2_MIN_RAD - 1e-9 && q2 <= J2_MAX_RAD + 1e-9
    && q3 >= J3_MIN_RAD - 1e-9 && q3 <= J3_MAX_RAD + 1e-9
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
  const [q1, q2, q3] = legJointsRad(heightIn, forwardIn);
  return jointsInLimitsRad(q1, q2, q3);
}

/** Farthest reachable point along the line from (startH, startF) to (targetH, targetF). */
function approach(targetH, targetF, startH, startF) {
  if (withinReach(targetH, targetF)) return [targetH, targetF];
  let lo = 0;
  let hi = 1;
  for (let i = 0; i < 32; i += 1) {
    const mid = 0.5 * (lo + hi);
    const h = startH + (targetH - startH) * mid;
    const f = startF + (targetF - startF) * mid;
    if (withinReach(h, f)) lo = mid;
    else hi = mid;
  }
  return [
    startH + (targetH - startH) * lo,
    startF + (targetF - startF) * lo,
  ];
}

/** Ankle position in the hip frame (m): +x is forward, +z is up. */
function anklePosRad(q1, q2) {
  return {
    x: -L1 * Math.cos(q1) - L2 * Math.cos(q1 + q2),
    z: -L1 * Math.sin(q1) - L2 * Math.sin(q1 + q2),
  };
}

function stanceFromJointsDeg(j1Deg, j2Deg) {
  const { x, z } = anklePosRad(j1Deg * RAD, j2Deg * RAD);
  return {
    height: -z / INCH,
    forward: -x / INCH,
  };
}

function planarJ3Deg(j1Deg, j2Deg) {
  return clamp(180 - j1Deg - j2Deg, J3_MIN_DEG, J3_MAX_DEG);
}

function readNum(map, name, fallback = 0) {
  const v = Number(map?.[name]);
  return Number.isFinite(v) ? v : fallback;
}

function padPressed(gamepad, name) {
  return (gamepad?.[name] ?? 0) > 0.5;
}

function closestOption(options, hint, lo, hi) {
  const legal = options.filter((q) => q >= lo - 1e-9 && q <= hi + 1e-9);
  const pool = legal.length ? legal : options;
  let best = pool[0];
  let bestDist = Math.abs(best - hint);
  for (let i = 1; i < pool.length; i += 1) {
    const d = Math.abs(pool[i] - hint);
    if (d < bestDist) {
      bestDist = d;
      best = pool[i];
    }
  }
  return best;
}

/** Knee angle (rad) that puts the ankle at horizontal position x for this hip angle. */
function q2ForAnkleX(q1, x, hint) {
  const cosine = -(x + L1 * Math.cos(q1)) / L2;
  if (cosine < -1 - 1e-9 || cosine > 1 + 1e-9) return null;
  const angle = Math.acos(clamp(cosine, -1, 1));
  return closestOption([angle - q1, -angle - q1], hint, J2_MIN_RAD, J2_MAX_RAD);
}

/** Hip angle (rad) that puts the ankle at horizontal position x for this knee angle. */
function q1ForAnkleX(q2, x, hint) {
  const along = L1 + L2 * Math.cos(q2);
  const across = L2 * Math.sin(q2);
  const radius = Math.hypot(along, across);
  if (radius < 1e-12 || Math.abs(x) > radius + 1e-9) return null;
  const phase = Math.atan2(-along, across);
  const base = Math.asin(clamp(x / radius, -1, 1));
  const options = [];
  for (let q1 of [base - phase, Math.PI - base - phase]) {
    while (q1 - hint > Math.PI) q1 -= 2 * Math.PI;
    while (hint - q1 > Math.PI) q1 += 2 * Math.PI;
    options.push(q1);
  }
  return closestOption(options, hint, J1_MIN_RAD, J1_MAX_RAD);
}

/**
 * Joint step over LIFT_DT (0.1 s) that moves the ankle straight up or down
 * while holding ankle x fixed (exact port of env.py:lift_step).
 */
function liftStepDeg(j1Deg, j2Deg, j3Deg, targetH, targetF) {
  const q1 = j1Deg * RAD;
  const q2 = j2Deg * RAD;
  const q3 = j3Deg * RAD;
  const xGoal = -targetF * INCH;
  const [q1Target, q2Target] = legJointsRad(targetH, targetF);

  let dq1 = clamp(q1Target - q1, -CAP_J1_RAD, CAP_J1_RAD);
  let dq2 = clamp(q2Target - q2, -CAP_J2_RAD, CAP_J2_RAD);
  if (Math.abs(anklePosRad(q1 + dq1, q2 + dq2).x - xGoal) > 1e-6) {
    const q2Hit = q2ForAnkleX(q1 + dq1, xGoal, q2 + dq2);
    if (q2Hit != null && Math.abs(q2Hit - q2) <= CAP_J2_RAD + 1e-12) {
      dq2 = q2Hit - q2;
    } else {
      const q1Hit = q1ForAnkleX(q2 + dq2, xGoal, q1 + dq1);
      if (q1Hit != null) {
        dq1 = clamp(q1Hit - q1, -CAP_J1_RAD, CAP_J1_RAD);
      }
    }
  }
  const n1 = clamp(q1 + dq1, J1_MIN_RAD, J1_MAX_RAD);
  const n2 = clamp(q2 + dq2, J2_MIN_RAD, J2_MAX_RAD);
  const q3Next = clamp(Math.PI - n1 - n2, J3_MIN_RAD, J3_MAX_RAD);
  const n3 = clamp(q3 + clamp(q3Next - q3, -CAP_J3_RAD, CAP_J3_RAD), J3_MIN_RAD, J3_MAX_RAD);
  return { j1: n1 * DEG, j2: n2 * DEG, j3: n3 * DEG };
}

/**
 * Joint step over LIFT_DT (0.1 s) that slides the ankle horizontally at constant
 * drop using the ankle velocity Jacobian (exact port of env.py:horizontal_step).
 */
function horizontalStepDeg(j1Deg, j2Deg, j3Deg, targetH, targetF) {
  const q1 = j1Deg * RAD;
  const q2 = j2Deg * RAD;
  const q3 = j3Deg * RAD;
  const xGoal = -targetF * INCH;
  const zGoal = -ankleDropIn(targetH) * INCH;

  const { x, z } = anklePosRad(q1, q2);
  const s = Math.sin(q1 + q2);
  const c = Math.cos(q1 + q2);
  const det = L2 * (z * c - x * s);
  const dx = xGoal - x;
  const dz = zGoal - z;

  let dq1 = 0;
  let dq2 = 0;
  if (Math.abs(det) >= 1e-9) {
    dq1 = ((-L2 * c) * dx - (L2 * s) * dz) / det;
    dq2 = (-x * dx - z * dz) / det;
    let scale = 1;
    if (Math.abs(dq1) > CAP_J1_RAD) scale = Math.min(scale, CAP_J1_RAD / Math.abs(dq1));
    if (Math.abs(dq2) > CAP_J2_RAD) scale = Math.min(scale, CAP_J2_RAD / Math.abs(dq2));
    dq1 *= scale;
    dq2 *= scale;
  }
  const n1 = clamp(q1 + dq1, J1_MIN_RAD, J1_MAX_RAD);
  const n2 = clamp(q2 + dq2, J2_MIN_RAD, J2_MAX_RAD);
  const q3Next = clamp(Math.PI - n1 - n2, J3_MIN_RAD, J3_MAX_RAD);
  const n3 = clamp(q3 + clamp(q3Next - q3, -CAP_J3_RAD, CAP_J3_RAD), J3_MIN_RAD, J3_MAX_RAD);
  return { j1: n1 * DEG, j2: n2 * DEG, j3: n3 * DEG };
}

function pathArrived(j1Deg, j2Deg, targetH, targetF) {
  const { x, z } = anklePosRad(j1Deg * RAD, j2Deg * RAD);
  const xGoal = -targetF * INCH;
  const zGoal = -ankleDropIn(targetH) * INCH;
  return Math.abs(x - xGoal) < 1e-4 && Math.abs(z - zGoal) < 1e-4;
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
  // Virtual mobile gamepad (no right stick): -ly = forward, lx = yaw turn (right = CW).
  // In scene.xml, *_left wheels sit at y = -0.1397 (physical right) and *_right at y = +0.1397 (physical left).
  if ('screenx' in gamepad && !('rx' in gamepad)) {
    const rightSide = clamp((fwd - lx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S);
    const leftSide = clamp((fwd + lx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S);
    return {
      front_left_mecanum_wheel: rightSide,
      front_right_mecanum_wheel: leftSide,
      rear_left_mecanum_wheel: rightSide,
      rear_right_mecanum_wheel: leftSide,
    };
  }
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
let prevPad = { a: false, b: false, y: false, x: false };
/** Last commanded joints (seeded from sensors; planned open-loop so contact noise doesn't shake the foot). */
let cmdJ1 = null;
let cmdJ2 = null;
let cmdJ3 = null;

function startCommand(nextMode, nextAxis, nextH, nextF, curH, curF) {
  const [clampedH, clampedF] = approach(nextH, nextF, curH, curF);
  mode = nextMode;
  axis = nextAxis;
  targetHeight = clampedH;
  targetForward = clampedF;
}

export function step(input_output, _input, _iface, gamepad) {
  const sensJ1 = readNum(input_output, 'J1', -45.4);
  const sensJ2 = readNum(input_output, 'J2', 165);
  const sensJ3 = readNum(input_output, 'J3', 60.5);

  if (cmdJ1 == null) {
    cmdJ1 = sensJ1;
    cmdJ2 = sensJ2;
    cmdJ3 = sensJ3;
  }

  const refJ1 = mode === 'neutral' ? sensJ1 : cmdJ1;
  const refJ2 = mode === 'neutral' ? sensJ2 : cmdJ2;
  const { height: curH, forward: curF } = stanceFromJointsDeg(refJ1, refJ2);

  const a = padPressed(gamepad, 'a');
  const b = padPressed(gamepad, 'b');
  const y = padPressed(gamepad, 'y');
  const x = padPressed(gamepad, 'x');

  // Immediate state switching on button press — no settle tolerance or latch lock.
  if (a && !prevPad.a) startCommand('a', 'height', CLIMB_HEIGHT_IN, curF, curH, curF);
  else if (b && !prevPad.b) startCommand('b', 'forward', curH, KEY2_FORWARD_IN, curH, curF);
  else if (y && !prevPad.y) startCommand('y', 'height', KEY3_HEIGHT_IN, curF, curH, curF);
  else if (x && !prevPad.x) startCommand('x', 'forward', curH, HOME_FORWARD_IN, curH, curF);
  prevPad = { a, b, y, x };

  let legMoving = false;
  if (mode !== 'neutral') {
    if (!pathArrived(cmdJ1, cmdJ2, targetHeight, targetForward)) {
      const prevJ1 = cmdJ1;
      const prevJ2 = cmdJ2;
      const planned = axis === 'height'
        ? liftStepDeg(cmdJ1, cmdJ2, cmdJ3, targetHeight, targetForward)
        : horizontalStepDeg(cmdJ1, cmdJ2, cmdJ3, targetHeight, targetForward);
      cmdJ1 = planned.j1;
      cmdJ2 = planned.j2;
      cmdJ3 = planned.j3;
      legMoving = Math.abs(cmdJ1 - prevJ1) > 1e-4 || Math.abs(cmdJ2 - prevJ2) > 1e-4;
    } else {
      // Arrived on commanded stance: hold J1/J2 and smoothly finish keeping foot level.
      cmdJ3 = rateLimit(cmdJ3, planarJ3Deg(cmdJ1, cmdJ2), CAP_J3_DEG);
    }
  } else {
    // Neutral: track measured J1/J2 and smoothly align J3 toward level foot (no snap).
    cmdJ1 = sensJ1;
    cmdJ2 = sensJ2;
    cmdJ3 = rateLimit(sensJ3, planarJ3Deg(sensJ1, sensJ2), CAP_J3_DEG);
  }

  const stickActive = Math.abs(gamepad?.lx ?? 0) > 0.05
    || Math.abs(gamepad?.ly ?? 0) > 0.05
    || Math.abs(gamepad?.rx ?? 0) > 0.05;

  const cmdHeight = stanceFromJointsDeg(cmdJ1, cmdJ2).height;
  let wheels;
  if (mode === 'b' && cmdHeight > WHEEL_ROLL_MIN_HEIGHT_IN + 1e-9 && legMoving && !stickActive) {
    wheels = tankWheels(KEY2_WHEEL);
  } else if (mode === 'x' && cmdHeight > WHEEL_ROLL_MIN_HEIGHT_IN + 1e-9 && legMoving && !stickActive) {
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
  const curJ1 = readNum(input_output, 'J1', -45.4);
  const curJ2 = readNum(input_output, 'J2', 165);
  const { height, forward } = stanceFromJointsDeg(curJ1, curJ2);
  return { height, forward };
}

/**
 * Tankdrive — joystick-only wheel control (no climbing / leg).
 *
 * Sticks: lx / ly / rx → four wheels (same mix as Climb omni drive).
 * ±100 rpm at full stick.
 */

const WHEEL_RPM = 100
const WHEEL_RAD_S = (WHEEL_RPM * 2 * Math.PI) / 60

function clamp(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v))
}

function wheelsFromGamepad(gamepad) {
  if (!gamepad) {
    return {
      front_left_mecanum_wheel: 0,
      front_right_mecanum_wheel: 0,
      rear_left_mecanum_wheel: 0,
      rear_right_mecanum_wheel: 0,
    }
  }
  const lx = gamepad.lx ?? 0
  const fwd = -(gamepad.ly ?? 0)
  // Virtual mobile gamepad (no right stick): -ly = forward, lx = yaw turn (right = CW).
  // In scene.xml, *_left wheels sit at y = -0.1397 (physical right) and *_right at y = +0.1397 (physical left).
  if ('screenx' in gamepad && !('rx' in gamepad)) {
    const rightSide = clamp((fwd - lx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S)
    const leftSide = clamp((fwd + lx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S)
    return {
      front_left_mecanum_wheel: rightSide,
      front_right_mecanum_wheel: leftSide,
      rear_left_mecanum_wheel: rightSide,
      rear_right_mecanum_wheel: leftSide,
    }
  }
  const rx = gamepad.rx ?? 0
  return {
    front_left_mecanum_wheel: clamp((lx + fwd + rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
    front_right_mecanum_wheel: clamp((-lx + fwd - rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
    rear_left_mecanum_wheel: clamp((-lx + fwd + rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
    rear_right_mecanum_wheel: clamp((lx + fwd - rx) * WHEEL_RAD_S, -WHEEL_RAD_S, WHEEL_RAD_S),
  }
}

export function step(_input_output, _input, _iface, gamepad) {
  return wheelsFromGamepad(gamepad)
}

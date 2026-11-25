export function init() {
  return [
    'To move the robot, push the left joystick in different directions.',
    'To climb, move to the base of one of the steps, then press ▼.',
    '* ▼ - lift robot',
    '* ▶\uFE0E - move robot forward',
    '* ▲ - lift leg',
    '* ◀\uFE0E - retract robot leg',
  ].join('\n');
}

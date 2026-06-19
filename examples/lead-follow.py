#!/usr/bin/env python3
from soble import SO101Leader, SO101Platform

if __name__ == "__main__":
    leader = SO101Leader("/dev/tty.usbmodem575E0032081")
    leader.load_config("config.json")
    platform = SO101Platform("Capybara", log_state=True)

    platform.setLeftRightMotors(0, 0)

    while True:
        positions = leader.getArmPositions()
        platform.setArmPositions(positions)

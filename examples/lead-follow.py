#!/usr/bin/env python3
from soble import SO101Leader, SO101Platform

if __name__ == "__main__":
    leader = SO101Leader("/dev/ttyACM0")
    leader.load_config("angular_config.json")
    platform = SO101Platform("Capybara", log_state=True)

    platform.setLeftRightMotors(0, 0)

    try:
        while True:
            positions = leader.getArmPositions()
            platform.setArmPositions(positions)
    except KeyboardInterrupt:
        pass
    finally:
        platform.stop()
        leader.stop()
        print("Stopped.")
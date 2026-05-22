#!/usr/bin/env python3
import time
from soble import SO101Leader, SO101Platform

if __name__ == "__main__":
    leader_limits, follower_limits = SO101Leader.load_config("angular_config.json")

    leader = SO101Leader("/dev/ttyACM0", leader_limits, follower_limits)
    platform = SO101Platform("Capybara", log_state=False)

    leader.start()
    platform.start()
    platform.setLeftRightMotors(0, 0)

    try:
        while True:
            positions = leader.getPositions()
            platform.setSO101Position(positions)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        platform.stop()
        leader.stop()
        print("Stopped.")
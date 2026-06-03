#!/usr/bin/env python3
"""BLE teleop with WASD wheels, leader mirror, and AprilTag pygame overlay."""

import argparse
import threading
from pathlib import Path

from soble import SO101Leader, SO101Platform
from soble.so101_leader import JOINT_KEYS

import cv2
import numpy as np
import pygame
from pyapriltags import Detector

FRAME_W = 1280
FRAME_H = 720
MOTOR_SPEED_SCALE = 125
TURN_SPEED_SCALE = 100


class Tag16h5Bitmaps:
    _instance = None

    def __new__(cls, resolution: int = 100):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._ready = False
        return cls._instance

    def __init__(self, resolution: int = 100) -> None:
        if self._ready:
            return
        self._ready = True
        self._detector = Detector(families="tag16h5", nthreads=1, quad_decimate=1.0)
        family = self._detector.tag_families["tag16h5"]
        self._family = family
        self._grid_size = int(family.contents.total_width)
        self._nbits = int(family.contents.nbits)
        self._ncodes = int(family.contents.ncodes)
        self._bitmap_cache: dict[int, np.ndarray] = {}
        self._resolution = resolution
        r = self._resolution - 1
        self._src_corners = np.array([[0, r], [r, r], [r, 0], [0, 0]], dtype=np.float32)

    @classmethod
    def instance(cls, resolution: int = 100):
        return cls(resolution)

    def bitmap_for_id(self, tag_id: int) -> np.ndarray | None:
        if tag_id < 0 or tag_id >= self._ncodes:
            return None
        cached = self._bitmap_cache.get(tag_id)
        if cached is not None:
            return cached
        code = int(self._family.contents.codes[tag_id])
        grid = np.zeros((self._grid_size, self._grid_size), np.uint8)
        for bi in range(self._nbits):
            bx = int(self._family.contents.bit_x[bi])
            by = int(self._family.contents.bit_y[bi])
            bit = (code >> (self._nbits - 1 - bi)) & 1
            grid[by, bx] = 255 if bit else 0
        grid = grid[:-2, :-2]
        bitmap = cv2.resize(
            cv2.cvtColor(grid, cv2.COLOR_GRAY2RGB),
            (self._resolution, self._resolution),
            interpolation=cv2.INTER_NEAREST,
        )
        self._bitmap_cache[tag_id] = bitmap
        return bitmap


def draw_reconstructed_tags(
    surface: pygame.Surface,
    tags: list[tuple[int, list[tuple[float, float]]]],
) -> None:
    bg = (16, 16, 20)
    bitmaps = Tag16h5Bitmaps.instance()
    for tag_id, pixels in tags:
        bitmap = bitmaps.bitmap_for_id(tag_id)
        if bitmap is None:
            continue
        dst = np.array(pixels, dtype=np.float32)
        M = cv2.getPerspectiveTransform(bitmaps._src_corners, dst)
        warped = cv2.warpPerspective(
            bitmap,
            M,
            (FRAME_W, FRAME_H),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=bg,
        )
        warped_surface = pygame.image.frombuffer(
            warped.tobytes(), (warped.shape[1], warped.shape[0]), "RGB"
        )
        surface.blit(warped_surface, (0, 0))
        pts = [(int(round(x)), int(round(y))) for x, y in pixels]
        for i in range(4):
            pygame.draw.line(surface, (255, 255, 255), pts[i], pts[(i + 1) % 4], 2)


def pygame_loop(
    running: threading.Event,
    motor: int,
    turn: int,
    leader: SO101Leader,
    platform: SO101Platform,
) -> None:
    pygame.init()
    pygame.display.set_caption("BLE teleop — WASD drive | ESC or Q to quit")
    hud_height = 56
    screen = pygame.display.set_mode((FRAME_W, FRAME_H + hud_height))
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    view = pygame.Surface((FRAME_W, FRAME_H))

    while running.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running.clear()
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running.clear()

        keys = pygame.key.get_pressed()
        fwd = (1 if keys[pygame.K_w] else 0) - (1 if keys[pygame.K_s] else 0)
        yaw = (1 if keys[pygame.K_d] else 0) - (1 if keys[pygame.K_a] else 0)
        left = max(-125, min(125, int(round(yaw * turn + fwd * motor))))
        right = max(-125, min(125, int(round(yaw * turn - fwd * motor))))

        platform.setLeftRightMotors(left, right)
        positions = leader.getPositions()
        platform.setSO101Position(positions)

        leader_hud = leader.status_line()
        tags_viz = platform.getApriltagTags() or []
        notify_age = platform.last_notify_age_s()

        view.fill((16, 16, 20))
        if tags_viz:
            draw_reconstructed_tags(view, tags_viz)

        rx = (
            f"last_rx {notify_age:0.2f}s ago"
            if notify_age is not None
            else "waiting for notify..."
        )
        status = f"L={left:4d} R={right:4d}  {rx}"
        if tags_viz:
            status += f"  tags={len(tags_viz)}"

        screen.fill((30, 30, 30))
        screen.blit(view, (0, 0))
        screen.blit(font.render(status, True, (220, 220, 220)), (8, FRAME_H + 8))
        hud = leader_hud[:90] + ("..." if len(leader_hud) > 90 else "")
        screen.blit(font.render(hud, True, (180, 200, 180)), (8, FRAME_H + 30))
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


def main() -> int:
    p = argparse.ArgumentParser(description="BLE teleop: leader USB + SO101 follower")
    p.add_argument(
        "--config", "-c",
        type=Path,
        default=Path(__file__).resolve().parent / "angular_config.json",
        help="angular_config.json (default: examples/angular_config.json)",
    )
    p.add_argument("--leader_port", "-p", help="Leader arm serial device, e.g. /dev/ttyACM0", default="/dev/ttyACM0")
    p.add_argument("--name", "-n", help="Robot name", default="Capybara")
    p.add_argument("--motor", type=int, default=MOTOR_SPEED_SCALE)
    p.add_argument("--turn", type=int, default=TURN_SPEED_SCALE)
    args = p.parse_args()

    leader_limits, follower_limits = SO101Leader.load_config(args.config)
    print(f"Config: {args.config}")
    for i, key in enumerate(JOINT_KEYS):
        l, f = leader_limits[i], follower_limits[i]
        print(
            f"  {key}: leader [{l.min_val}, {l.max_val}]  "
            f"follower [{f.min_val}, {f.max_val}]  "
            f"ratio={f.range / l.range:.4f}"
        )

    running = threading.Event()
    running.set()

    leader = SO101Leader(args.leader_port, leader_limits, follower_limits)
    platform = SO101Platform(args.name)
    leader.start()
    platform.start()

    try:
        pygame_loop(running, args.motor, args.turn, leader, platform)
    finally:
        running.clear()
        platform.stop()
        leader.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

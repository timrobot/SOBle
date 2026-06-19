#!/usr/bin/env python3
"""BLE teleop with WASD wheels, leader mirror, and AprilTag pygame overlay."""

import argparse
from pathlib import Path

from soble import SO101Leader, SO101Platform

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


def main() -> int:
    p = argparse.ArgumentParser(description="BLE teleop: leader USB + SO101 follower")
    p.add_argument(
        "--config", "-c",
        type=Path,
        default=Path(__file__).resolve().parent / "config.json",
        help="Leader/follower joint limits JSON (default: examples/config.json)",
    )
    p.add_argument(
        "--leader_port",
        "-p",
        default="/dev/tty.usbmodem575E0032081",
        help="Leader serial port (Linux e.g. /dev/ttyACM0, Windows COM3)",
    )
    p.add_argument("--name", "-n", default="Capybara", help="Robot BLE name")
    args = p.parse_args()

    print(f"Config: {args.config}")

    leader = SO101Leader(args.leader_port)
    leader.load_config(args.config)
    platform = SO101Platform(args.name)

    pygame.init()
    pygame.display.set_caption("BLE teleop — WASD drive | ESC or Q to quit")
    hud_height = 32
    screen = pygame.display.set_mode((FRAME_W, FRAME_H + hud_height))
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    view = pygame.Surface((FRAME_W, FRAME_H))

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False

            keys = pygame.key.get_pressed()
            fwd = (1 if keys[pygame.K_w] else 0) - (1 if keys[pygame.K_s] else 0)
            yaw = (1 if keys[pygame.K_d] else 0) - (1 if keys[pygame.K_a] else 0)
            left = max(-125, min(125, int(round(yaw * TURN_SPEED_SCALE + fwd * MOTOR_SPEED_SCALE))))
            right = max(-125, min(125, int(round(yaw * TURN_SPEED_SCALE - fwd * MOTOR_SPEED_SCALE))))

            platform.drive(left, right)
            positions = leader.getMappedPositions()
            platform.setArmPositions(positions)

            tags_viz = platform.detectApriltags()
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
            status += "  arm=following" if positions else "  arm=off (waiting for leader)"
            if tags_viz:
                status += f"  tags={len(tags_viz)}"

            screen.fill((30, 30, 30))
            screen.blit(view, (0, 0))
            screen.blit(font.render(status, True, (220, 220, 220)), (8, FRAME_H + 8))
            pygame.display.flip()
            clock.tick(60)
    finally:
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

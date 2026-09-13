#!/usr/bin/env python3
"""Live occupancy-grid and scan viewer for RoboCup MAP serial telemetry."""

import argparse
import math
import queue
import threading

import matplotlib.pyplot as plt
import numpy as np
import serial


class MapViewer:
    def __init__(self):
        self.config = None
        self.log_odds = None
        self.pose = None
        self.endpoints = np.empty((0, 2))
        self.rays = []
        self.frontiers = np.empty((0, 2))
        self.path = np.empty((0, 2))
        self.figure, (self.ax_map, self.ax_scan) = plt.subplots(1, 2, figsize=(13, 6))
        self.figure.canvas.mpl_connect("key_press_event", self.on_key)

    def on_key(self, event):
        if event.key == "r" and self.log_odds is not None:
            self.log_odds.fill(0.0)

    def set_config(self, fields):
        if len(fields) != 13:
            raise ValueError("CONFIG needs 13 fields")
        width, height = int(fields[0]), int(fields[1])
        config = {
            "width": width, "height": height, "cells_per_m": float(fields[2]),
            "min_x": float(fields[3]), "min_y": float(fields[4]),
            "fov_min": float(fields[5]), "fov_max": float(fields[6]),
            "max_range": float(fields[7]), "offset_x": float(fields[8]),
            "offset_y": float(fields[9]), "num_points": int(fields[10]),
            "free_update": float(fields[11]), "occupied_update": float(fields[12]),
        }
        # CONFIG is repeated periodically by the firmware so a viewer can join
        # the serial stream at any time.  It is not a request to reset the map.
        if config == self.config:
            return

        self.config = config
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        self.frontiers = np.empty((0, 2))
        self.path = np.empty((0, 2))
        print(f"Configured {width}x{height} map at {self.config['cells_per_m']} cells/m")

    def world_to_cell(self, x, y):
        c = self.config
        mx = math.floor((x - c["min_x"]) * c["cells_per_m"])
        my = math.floor((y - c["min_y"]) * c["cells_per_m"])
        return (mx, my) if 0 <= mx < c["width"] and 0 <= my < c["height"] else None

    def update_cell(self, cell, delta):
        if cell is not None:
            x, y = cell
            self.log_odds[y, x] = np.clip(self.log_odds[y, x] + delta, -5.0, 5.0)

    def cell_index_to_world(self, index):
        c = self.config
        x, y = index % c["width"], index // c["width"]
        return (c["min_x"] + (x + 0.5) / c["cells_per_m"],
                c["min_y"] + (y + 0.5) / c["cells_per_m"])

    def update_frontiers(self, fields):
        if self.config is None or len(fields) != 1:
            return
        encoded = fields[0]
        cell_count = self.config["width"] * self.config["height"]
        if len(encoded) != (cell_count + 3) // 4:
            raise ValueError("FRONTIERS has the wrong bitmask length")

        points = []
        for index in range(cell_count):
            nibble = int(encoded[index // 4], 16)
            if nibble & (1 << (index % 4)):
                points.append(self.cell_index_to_world(index))
        self.frontiers = np.asarray(points, dtype=float).reshape((-1, 2))

    def update_path(self, fields):
        if self.config is None or len(fields) != 1:
            return
        encoded = fields[0]
        if len(encoded) % 3 != 0:
            raise ValueError("PATH must contain three hex digits per cell")

        cell_count = self.config["width"] * self.config["height"]
        points = []
        for offset in range(0, len(encoded), 3):
            index = int(encoded[offset:offset + 3], 16)
            if index >= cell_count:
                raise ValueError("PATH contains an out-of-range cell")
            points.append(self.cell_index_to_world(index))
        self.path = np.asarray(points, dtype=float).reshape((-1, 2))

    def ray_cast(self, start, end, obstacle):
        """Bresenham update matching the firmware's free/occupied ray model."""
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
        err = dx - dy
        x, y = x0, y0
        while (x, y) != (x1, y1):
            self.update_cell((x, y), self.config["free_update"])
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        if obstacle:
            self.update_cell((x1, y1), self.config["occupied_update"])

    def update_scan(self, fields):
        if self.config is None or len(fields) < 3:
            return
        x, y, theta = map(float, fields[:3])
        ranges = np.asarray([float(value) for value in fields[3:]], dtype=float)
        c = self.config
        if len(ranges) != c["num_points"]:
            return
        angles = np.linspace(c["fov_min"], c["fov_max"], len(ranges))
        robot_cell = self.world_to_cell(x, y)
        endpoints, rays = [], []
        cos_theta, sin_theta = math.cos(theta), math.sin(theta)

        for distance, bearing in zip(ranges, angles):
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            obstacle = distance < c["max_range"]
            distance = min(distance, c["max_range"])
            # Bearing is from +Y (forward), CCW-positive.
            xr = -distance * math.sin(bearing) + c["offset_x"]
            yr = distance * math.cos(bearing) + c["offset_y"]
            wx = xr * cos_theta - yr * sin_theta + x
            wy = xr * sin_theta + yr * cos_theta + y
            end_cell = self.world_to_cell(wx, wy)
            if robot_cell is not None and end_cell is not None:
                self.ray_cast(robot_cell, end_cell, obstacle)
            endpoints.append((wx, wy))
            rays.append(((x, y), (wx, wy)))

        self.pose, self.endpoints, self.rays = (x, y, theta), np.asarray(endpoints), rays

    def draw_robot(self, axis):
        x, y, theta = self.pose
        # Forward (+Y) unit vector for CCW-positive robot yaw.
        axis.arrow(x, y, -math.sin(theta) * 0.18, math.cos(theta) * 0.18,
                   width=0.015, color="tab:green", length_includes_head=True)

    def draw(self):
        if self.config is None or self.pose is None:
            return
        c = self.config
        extent = [c["min_x"], c["min_x"] + c["width"] / c["cells_per_m"],
                  c["min_y"], c["min_y"] + c["height"] / c["cells_per_m"]]
        self.ax_map.clear()
        probability = 1.0 / (1.0 + np.exp(-self.log_odds))
        self.ax_map.imshow(probability, origin="lower", extent=extent, vmin=0, vmax=1,
                           cmap="gray_r", interpolation="nearest")
        if len(self.frontiers):
            self.ax_map.scatter(self.frontiers[:, 0], self.frontiers[:, 1],
                                s=18, facecolors="none", edgecolors="tab:orange",
                                linewidths=0.9, label="Frontiers")
        if len(self.path):
            self.ax_map.plot(self.path[:, 0], self.path[:, 1], color="tab:cyan",
                             linewidth=2.0, marker=".", markersize=4, label="Path")
        self.draw_robot(self.ax_map)
        self.ax_map.set_title("Reconstructed occupancy grid (r: reset)")
        if len(self.frontiers) or len(self.path):
            self.ax_map.legend(loc="upper right")

        self.ax_scan.clear()
        for start, end in self.rays:
            self.ax_scan.plot((start[0], end[0]), (start[1], end[1]), color="tab:blue", alpha=0.12)
        if len(self.endpoints):
            self.ax_scan.scatter(self.endpoints[:, 0], self.endpoints[:, 1], s=12, c="tab:red")
        self.draw_robot(self.ax_scan)
        self.ax_scan.set_title("Latest ToF scan in world frame")

        for axis in (self.ax_map, self.ax_scan):
            axis.set_xlim(extent[0], extent[1])
            axis.set_ylim(extent[2], extent[3])
            axis.set_aspect("equal")
            axis.set_xlabel("world X (m), right")
            axis.set_ylabel("world Y (m), forward")
        self.figure.tight_layout()


def serial_reader(port, baudrate, output):
    with serial.Serial(port, baudrate, timeout=0.2) as connection:
        while True:
            line = connection.readline().decode("utf-8", errors="replace").strip()
            if line.startswith(("CONFIG,", "MAP,", "FRONTIERS,", "PATH,")):
                output.put(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="e.g. /dev/cu.usbmodem1234 or COM3")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    messages = queue.Queue()
    threading.Thread(target=serial_reader, args=(args.port, args.baud, messages), daemon=True).start()
    viewer = MapViewer()
    plt.ion()
    plt.show()
    while plt.fignum_exists(viewer.figure.number):
        dirty = False
        while True:
            try:
                message = messages.get_nowait()
            except queue.Empty:
                break
            prefix, *fields = message.split(",")
            try:
                if prefix == "CONFIG":
                    viewer.set_config(fields)
                elif prefix == "MAP":
                    viewer.update_scan(fields)
                elif prefix == "FRONTIERS":
                    viewer.update_frontiers(fields)
                elif prefix == "PATH":
                    viewer.update_path(fields)
                dirty = True
            except ValueError as error:
                print(f"Ignoring malformed telemetry: {error}")
        if dirty:
            viewer.draw()
        plt.pause(0.02)


if __name__ == "__main__":
    main()

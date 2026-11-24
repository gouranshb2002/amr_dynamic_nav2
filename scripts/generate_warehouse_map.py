#!/usr/bin/env python3
import json, os, numpy as np

RES = 0.05
X_MIN, X_MAX = -5.5, 5.5
Y_MIN, Y_MAX = -4.5, 4.5
W = int(round((X_MAX - X_MIN) / RES))
H = int(round((Y_MAX - Y_MIN) / RES))

BOXES = [
    ("wall_north", 0.0, 4.1, 10.4, 0.2), ("wall_south", 0.0, -4.1, 10.4, 0.2),
    ("wall_east", 5.1, 0.0, 0.2, 8.0), ("wall_west", -5.1, 0.0, 0.2, 8.0),
    ("rack_nw", -2.0, 1.8, 2.2, 0.8), ("rack_ne", 2.0, 1.8, 2.2, 0.8),
    ("rack_sw", -2.0, -1.8, 2.2, 0.8), ("rack_se", 2.0, -1.8, 2.2, 0.8),
]

grid = np.full((H, W), 254, dtype=np.uint8)
for _, cx, cy, sx, sy in BOXES:
    c0 = max(0, int(np.floor((cx - sx / 2.0 - X_MIN) / RES)))
    c1 = min(W, int(np.ceil((cx + sx / 2.0 - X_MIN) / RES)))
    r0 = max(0, int(np.floor((Y_MAX - (cy + sy / 2.0)) / RES)))
    r1 = min(H, int(np.ceil((Y_MAX - (cy - sy / 2.0)) / RES)))
    grid[r0:r1, c0:c1] = 0

out_dir = "amr_simulation/maps"
with open(os.path.join(out_dir, "dev_warehouse.pgm"), "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode("ascii"))
    f.write(grid.tobytes())

with open(os.path.join(out_dir, "dev_warehouse.yaml"), "w") as f:
    f.write(f"image: dev_warehouse.pgm\nmode: trinary\nresolution: {RES}\norigin: [{X_MIN}, {Y_MIN}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

with open(os.path.join(out_dir, "dev_warehouse_obstacles.json"), "w") as f:
    json.dump([{"name": n, "cx": cx, "cy": cy, "sx": sx, "sy": sy} for n, cx, cy, sx, sy in BOXES], f, indent=2)

print(f"Saved frozen map ({W}x{H} @ {RES}m) and obstacle geometry to {out_dir}/")

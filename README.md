
# uss_parking — Minimal Ultrasonic Parking Slot Detection (C++)

This is a tiny C++ demo for detecting parallel parking slots using ultrasonic sensor (USS) points only,
under lane-marking-free conditions. It implements:

- Clustering USS points along the curb to form obstacle segments
- Gap-based slot hypothesis between consecutive obstacles
- Simple parameterization suitable for APA prototyping

## Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
./uss_demo
```

## Integrate

Link `libuss.a` and include `include/USS.hpp`. Feed your fused/motion-compensated USS points per frame
and call:

```cpp
auto obstacles = uss::clusterObstacles(points, params);
auto slots     = uss::detectSlots(obstacles, params);
```

Then pass `slots` into your APA/slot manager for validation and trajectory planning.

> Note: This is a minimal baseline. In production, add multi-frame accumulation, motion compensation,
> curb/road-edge estimation, and freespace grid fusion for robustness.

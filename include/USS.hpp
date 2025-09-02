
#pragma once
#include <vector>
#include <cstdint>
#include <cmath>

namespace uss {

struct Pose2D {
    float x{0.0f};    // m
    float y{0.0f};    // m
    float yaw{0.0f};  // rad
    double timestamp{0.0}; // seconds
};
struct USSPoint {
    float x{0.0f};    // m
    float y{0.0f};    // m
    bool  valid{false};
};
struct Frame {
    std::vector<USSPoint> points;
    Pose2D pose; // vehicle pose at that frame (in world or some global)
};
struct Slot {
    float x_min, x_max;
    float y_min, y_max;
};

struct ParkingSlot {
    float cx, cy;   // center
    float length;   // along x
    float width;    // along y
    float angle;    // radians, 0 -> along x
    bool  isValid{false};
};

struct Params {
    float car_length{4.5f};
    float car_width{1.8f};
    float slot_margin{0.3f};
    float cluster_dx_thresh{1};     // mafx gap in x to continue a cluster
    float lateral_band_min{-0.8f};     // consider obstacles around curb band [ymin, ymax]
    float lateral_band_max{0.8f};
    float min_width{2.2f};             // expected slot width
    float max_width{3.0f};
    float min_length_extra{0.5f};      // extra length margin beyond car length
    const double velocity = 2.0;  // 车辆速度 m/s (匀速)
    const double total_time = 10.0;  // 总模拟时间 s (从静止开始到结束，x从0到50m)
    const double sample_rate = 10.0;  // 采样率 Hz (每秒采样次数)
    const double window_size_m = 2.0;  // 滑窗大小 (空间，米)
    const double dist_threshold = 2.0;  // 距离阈值，大于此为潜在空位 (m)
    const double noise_std = 0.1;  // 距离测量噪声标准差 (m)
};
std::vector<USSPoint> accumulateFramesToCurrent(
    const std::vector<Frame>& buffer,
    const Pose2D& current_pose,
    float time_window_sec,    // keep only frames within this time window
    bool use_time_window = true);



// Single-frame simple clustering along x (sorted beforehand)
std::vector<Slot> rect_point_min_max(const std::vector<USSPoint>& all, const Params& p);

// Detect parallel parking slots by gap between consecutive obstacles
std::vector<ParkingSlot> detectSlots(const std::vector<Slot>& obs, const Params& p);

// Utility: sort points by x
void sortByX(std::vector<USSPoint>& pts);
double ideal_distance(double x) ;
void draw(bool is_draw, std::vector<double> xs, std::vector<double> ys, std::vector<float> dis_y, std::vector<float> pts_x, std::vector<float> pts_y, std::vector<Slot> obstacles);

// Multi-frame accumulation (simple) — append new points into a rolling buffer (kept outside library)
} // namespace uss

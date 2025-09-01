
#include "USS.hpp"
#include <algorithm>

namespace uss {

void sortByX(std::vector<USSPoint>& pts) {
    std::sort(pts.begin(), pts.end(), [](const USSPoint& a, const USSPoint& b){ return a.x < b.x; });
}

static inline bool inBand(const USSPoint& p, const Params& prms) {
    return p.valid && p.y >= prms.lateral_band_min && p.y <= prms.lateral_band_max;
}

std::vector<Slot> rect_point_min_max(const std::vector<USSPoint>& all, const Params& p) {
    std::vector<USSPoint> pts;
    pts.reserve(all.size());
    for (const auto& q : all) pts.push_back(q);
    if (pts.empty()) return {};

    sortByX(pts);

    std::vector<Slot> slot;
    Slot cur{pts.front().x, pts.front().x, pts.front().y, pts.front().y};

    for (size_t i = 1; i < pts.size(); ++i) {
        const auto& pt = pts[i];
        if (std::fabs(pt.x - cur.x_max) <= p.cluster_dx_thresh) {
            cur.x_max = pt.x;
            cur.y_min = std::min(cur.y_min, pt.y);
            cur.y_max = std::max(cur.y_max, pt.y);
        } else {
            slot.push_back(cur);
            cur = Slot{pt.x, pt.x, pt.y, pt.y};
        }
    }
    slot.push_back(cur);
    return slot;
}

std::vector<ParkingSlot> detectSlots(const std::vector<Slot>& obs, const Params& p) {
    std::vector<ParkingSlot> out;
    if (obs.size() < 2) return out;

    for (size_t i = 0; i + 1 < obs.size(); ++i) {
        const auto& A = obs[i];
        const auto& B = obs[i+1];

        // Gap along x between consecutive obstacles
        const float gap = B.x_min - A.x_max;
        const float length_needed = p.car_length + p.slot_margin + p.min_length_extra;
        if (gap < length_needed) continue;

        // Estimate slot lateral width using band, or fallback to nominal in params
        const float width_est = std::max(p.min_width, std::min(p.max_width, (p.max_width + p.min_width) * 0.5f));

        ParkingSlot s{};
        s.cx = 0.5f * (A.x_max + B.x_min);
        s.cy = 0.0f; // along curb line
        s.length = gap;
        s.width  = width_est;
        s.angle  = 0.0f;
        s.isValid = (width_est >= p.min_width && width_est <= p.max_width && gap >= length_needed);
        if (s.isValid) out.push_back(s);
    }
    return out;
}

double ideal_distance(double x) {
    if (x < 10.0) return 0.5;  // 有车
    else if (x < 15.0) return 3.0;  // 空位
    else if (x < 30.0) return 0.5;  // 有车
    else return 0.5;  // 有车
}

} // namespace uss

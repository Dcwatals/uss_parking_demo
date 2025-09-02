
#include "USS.hpp"
#include <iostream>
#include <iomanip>
#include "../matplotlib-cpp-master/matplotlibcpp.h"
#include <cmath>
#include <random>

using namespace uss;
namespace plt = matplotlibcpp;






// A tiny demo with synthetic curb-side obstacles: two parked cars with a gap.
int main() {
    Params params;
    // Synthetic USS scan points along the curb (y around 0), x grows forward
    std::vector<USSPoint> pts;
    // 数据存储
    std::vector<double> times;
    std::vector<double> xs;
    std::vector<double> ys;  // y固定为0
    std::vector<double> distances;

    // 初始化随机数生成器用于噪声
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, params.noise_std);



    // 模拟数据采集 (从t=0开始，匀速行驶)
    double dt = 1.0 / params.sample_rate;
    for (double t = 0.0; t <= params.total_time; t += dt) {
        double x = params.velocity * t;  // 匀速定位 x
        double y = 0.0;  // 假设直线行驶
        double d = ideal_distance(x) + noise(gen);  // 超声波距离 + 噪声

        times.push_back(t);
        xs.push_back(x);
        ys.push_back(y);
        distances.push_back(d);
    }
    std::vector<float> dis_y(ys.size());

    

    auto add_box = [&](float x0, float x1, float y0, float y1, float step){
        for (float x = x0; x <= x1; x += step) {
            pts.push_back({x, y0, true});
            pts.push_back({x, y1, true});
        }
    };
    for (int i = 0; i < ys.size(); ++i) {
        dis_y[i] = static_cast<float>(distances[i] + ys[i]); 
        std::cout << "dis_y" <<dis_y[i]<<std::endl;

    }
    
    int window_size = static_cast<int>(std::round(params.window_size_m / (params.velocity * dt)));
    std::cout << "\n滑窗分析 (窗口大小: " << window_size << " 点, 约" << params.window_size_m << "m):" << std::endl;
    int start = 0;
    int final = 0;

    for (size_t i = 1; i < dis_y.size() - window_size; ++i) {
        if(abs(dis_y[i] - dis_y[i-1])>params.dist_threshold){
            if(start ==0) start = i;
            else{
                final = i;
            }
        } 
    }

    float avgdis = 0;

    for(int i = start;i<final;i++){
        avgdis += dis_y[i]/(final-start);
    }
    add_box(xs[start],xs[final],dis_y[start],dis_y[final],0.2f);
    std::vector<float> pts_x;
    std::vector<float> pts_y;
    for(auto &p:pts){
        pts_x.push_back(p.x);
        pts_y.push_back(p.y);
    }
    bool is_draw = 1;
    auto obstacles = rect_point_min_max(pts, params);
    draw(is_draw,xs,ys,dis_y,pts_x,pts_y,obstacles);

    return 0;
}

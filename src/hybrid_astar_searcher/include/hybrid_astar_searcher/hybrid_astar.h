/**
 * @file hybrid_astar.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <chrono>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>

#include "hybrid_astar_searcher/type_defs.h" 
#include "hybrid_astar_searcher/calculate_heuristic.h" 
#include "hybrid_astar_searcher/visualize.h"
#include "hybrid_astar_searcher/ReedsSheppPath.h"
#include "hybrid_astar_searcher/node3d.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace planning {



struct HybridAstarResult {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;

};

class HybridAstar
{
private:

    double length_ = 4.933;
    double width_ = 2.11;
    double min_turn_radius_ = 5.0538;
    double max_steer_angle_ = 27;
    double max_steer_angle_rate_ = 8.552;
    double steer_ratio_ = 16;
    double wheel_base_ = 2.845;
    double front_edge_to_center_ = 3.89;  //中心是后轴中心
    double back_edge_to_center_ = 1.043; 
    double left_edge_to_center_ = 1.055;
    double right_edge_to_center_ = 1.055;
   

    double next_node_num_ = 6;
    double step_size_ = 0.7;

    //map params
    double x_min_, x_max_, y_min_, y_max_, xy_resolution_, theta_resolution_;
    int map_size_x_, map_size_y_;
    grid_map::GridMap map_;
    Visualize vis;

    //penalty 
    double traj_forward_penalty_ = 1.0;
    double traj_back_penalty_ = 10.0;
    double traj_gear_switch_penalty_ = 1.0;
    double traj_steer_penalty_ = 0.0;
    double traj_steer_change_penalty_ = 0;   

    std::shared_ptr<Node3d> start_node_;
    std::shared_ptr<Node3d> goal_node_;
    std::shared_ptr<Node3d> final_node_;
    struct cmp {
        bool operator() (const std::pair<int, double>& l, 
                         const std::pair<int, double>& r) {
            return l.second > r.second;                
        }
    };

    std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, cmp> 
        open_pq_;
    std::unordered_map<int, std::shared_ptr<Node3d>> open_set_;
    std::unordered_map<int, std::shared_ptr<Node3d>> close_set_;

    std::unique_ptr<GridSearch> grid_search_ptr_;
    std::shared_ptr<ReedShepp> reed_shepp_generator_;

public:
    HybridAstar(grid_map::GridMap map);
    ~HybridAstar();

    bool plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result);
    Vec3i getIndexFromPose(Vec3d pose);
    bool AnalyticExpansion(std::shared_ptr<Node3d> cur_node);
    void mod2Pi(double &angle);
    std::vector<Vec2d> calculateCarBoundingBox(Vec3d pose);
    bool isLinecollision(double x0, double y0, double x1, double y1);
    bool validityCheck(std::shared_ptr<Node3d> node);
    bool isInMap(double x, double y);
    bool isStateValid2(const ob::SpaceInformation *si, const ob::State *state);
    std::shared_ptr<Node3d> nextNodeGenerator(std::shared_ptr<Node3d> cur_node, int next_node_idx); 

    double TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node);
    double calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node);

    bool getHybridAstarResult(HybridAstarResult &result);
};


} //namespace planning

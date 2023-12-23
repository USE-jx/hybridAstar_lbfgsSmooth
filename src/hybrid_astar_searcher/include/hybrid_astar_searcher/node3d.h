/**
 * @file node3d.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
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

#include "hybrid_astar_searcher/type_defs.h" 

namespace planning {

/**
 * @brief 混合A星搜索用的到节点
 * 
 */
class Node3d
{
private:
    Vec3d pose_;  //{x,y,theta}

    std::vector<double> traversed_x_, traversed_y_, traversed_theta_;  

    int index_;
    double traj_cost_, heuristic_cost_;
    int step_size_ = 1;
    double steering_ = 0;
    std::shared_ptr<Node3d> parent_;
    bool direction_ = true;  //true for forward

public:
    Node3d(Vec3d pose) {
        pose_ = pose;
        traj_cost_ = 0.0;
        heuristic_cost_ = 0.0;
        traversed_x_.push_back(pose_(0));
        traversed_y_.push_back(pose_(1));
        traversed_theta_.push_back(pose_(2));
        step_size_ = 1;
        parent_ = nullptr;
        index_ = 0;
    }

    Node3d(std::vector<double> traversed_x, std::vector<double> traversed_y, 
    std::vector<double> traversed_theta) {

        pose_(0) = traversed_x.back();
        pose_(1) = traversed_y.back();
        pose_(2) = traversed_theta.back();

        traversed_x_ = traversed_x;
        traversed_y_ = traversed_y;
        traversed_theta_ = traversed_theta;

        step_size_ = traversed_x.size();
        traj_cost_ = 0.0;
        heuristic_cost_ = 0.0;
        parent_ = nullptr;
        index_ = 0;
        
    }

    void setTrajCost(const double traj_cost) {traj_cost_ = traj_cost;}
    void setHeuristicCost(const double heuristic_cost) {heuristic_cost_ = heuristic_cost;}
    void setParent(std::shared_ptr<Node3d> parent) {parent_ = parent;}
    void setIndex(Vec3i pose_map, const int map_size_x, const int map_size_y) {
        index_ = static_cast<int>(pose_map(2) * map_size_x * map_size_y + 
                                  pose_map(1) * map_size_x + pose_map(0));
    }
    void setDirection(bool direction) {direction_ = direction;}
    void setSteering(double steering) {steering_ = steering;}


    

    double getTrajCost() {return traj_cost_;}
    double getHeuristicCost() {return heuristic_cost_;}
    double getCost() {return traj_cost_ + 1 * heuristic_cost_;}
    std::shared_ptr<Node3d> getParent() {return parent_;}
    int getIndex() {return index_;}
    bool getDirection() {return direction_;}
    int getStepSize() {return step_size_;}
    double getSteering() {return steering_;}

    std::vector<double> getXs() {return traversed_x_;}
    std::vector<double> getYs() {return traversed_y_;}
    std::vector<double> getThetas() {return traversed_theta_;}

    double getX() {return pose_(0);}
    double getY() {return pose_(1);}
    double getTheta() {return pose_(2);}




};
}
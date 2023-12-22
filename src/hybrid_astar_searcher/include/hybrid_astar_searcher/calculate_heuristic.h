/**
 * @file calculate_heuristic.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-06
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

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>

#include "hybrid_astar_searcher/type_defs.h" 

namespace planning {

/**
 * @brief 用于A*搜索和构建离线的查询障碍物启发函数距离的表
 * 
 */
class Node2d
{
private:
    Vec2i pos_map_;
    double g_, h_, f_;
    int index_;
    std::shared_ptr<Node2d> parent_;

public:
    
    Node2d(Vec2i pos_map, const int map_size_x) {
        pos_map_ = pos_map;
        g_ = 0.0;
        h_ = 0.0;
        f_ = 0.0;
        index_ = pos_map_(1) * map_size_x + pos_map_(0);
        parent_ = nullptr;
    }

    void setG(const double g) {g_ = g; f_ = g_ + 1*h_;}
    void setH(const double h) {h_ = h; f_ = g_ + 1*h_;}
    void setF(const double f) {f_ = f;}
    void setParent(std::shared_ptr<Node2d> node) {parent_ = node;}

    int getPosXInMap() {return pos_map_(0);}
    int getPosYInMap() {return pos_map_(1);}
    double getG() {return g_;}
    double getH() {return h_;}
    double getF() {return f_;}
    int getIndex() {return index_;}
    std::shared_ptr<Node2d> getParentNode() {return parent_;}

    
};

class GridSearch {
private:
    double x_min_, x_max_, y_min_, y_max_, resolution_;
    int map_size_x_, map_size_y_;
    grid_map::GridMap map_;

    std::shared_ptr<Node2d> start_node_;
    std::shared_ptr<Node2d> goal_node_;
    std::shared_ptr<Node2d> final_node_;
    double heu_astar_;

    struct cmp {
        bool operator() (const std::pair<int, double>& l, 
                         const std::pair<int, double>& r) {
            return l.second > r.second;                
        }
    };
    std::unordered_map<int, std::shared_ptr<Node2d>> dp_map_; 

    

public:
    GridSearch(grid_map::GridMap map);

    Vec2i getIndexFromPosition(Vec2d position);
    Vec2d getPositionFromIndex(Vec2i index);

    std::vector<std::shared_ptr<Node2d>> getNeighborNodes(std::shared_ptr<Node2d> cur_node);
    double EuclidDistance(const double x1, const double y1,
                          const double x2, const double y2);
    
    bool isInGridMap(std::shared_ptr<Node2d> node);
    bool isOnObstacle(std::shared_ptr<Node2d> node);

    std::vector<Vec2d> getAstartPath();


    bool calculateHeuByAstar(Vec2d start_pos, Vec2d goal_pos);

    bool generateDpMap(Vec2d goal_pos);
    double lookupInDpMap(Vec2d start_pos);


};



} //namespace planning




/**
 * @file calculate_heuristic.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hybrid_astar_searcher/calculate_heuristic.h"

using namespace std;

namespace planning {

GridSearch::GridSearch(grid_map::GridMap map) {
    map_ = map;
    resolution_ = map.getResolution();
    x_min_ = map.getStartIndex()(0) * resolution_ - 25;
    x_max_ = x_min_ + map.getLength().x();
    y_min_ = map.getStartIndex()(1) * resolution_ -25;
    y_max_ = y_min_ + map.getLength().y();
    
    map_size_x_ = map.getSize()(0); 
    map_size_y_ = map.getSize()(1);
    
    cout << "x_min and x_max:" << x_min_ << " " << x_max_ << endl;
    cout << "y_min and y_max:" << y_min_ << " " << y_max_ << endl;

    cout << "map_size_x_" << map_size_x_ << endl;
    cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl;


}

bool GridSearch::calculateHeuByAstar(Vec2d start_pos, Vec2d goal_pos) {
    cout << "A star search" << endl;
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;
    unordered_map<int, shared_ptr<Node2d>> close_set;

    Vec2i start_idx = getIndexFromPosition(start_pos);
    Vec2i goal_idx = getIndexFromPosition(goal_pos);
    //cout << "start_idx:" << start_idx << endl;

    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_pq.emplace(start_node->getIndex(), start_node->getF());
    open_set.emplace(start_node->getIndex(), start_node);

    cout << "即将进入循环" << endl;
    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        if (cur_node->getIndex() == goal_node->getIndex()) {
            final_node_ = cur_node;
            break;
        }
        

        close_set.emplace(cur_index, cur_node);
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        //cout << "得到邻居节点" << endl;
        for (auto &neighbor_node : neighbor_nodes) {

            //检测是否在地图以内
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            //cout << "在地图以内" << endl;

            //是否在障碍物上
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            //cout << "不在障碍物上" << endl;

            //是否在close set 里
            if (close_set.find(neighbor_node->getIndex()) != close_set.end()) {
                continue;
            }

            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
               
                ++explored_node_num;
                double heuristic = 
                EuclidDistance(neighbor_node->getPosXInMap(),neighbor_node->getPosYInMap(),
                               goal_node->getPosXInMap(), goal_node->getPosYInMap());
                neighbor_node->setH(heuristic);
                neighbor_node->setParent(cur_node);
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getF());
            } else {
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }

        }

    }

    if (final_node_ == nullptr) {
        cout << "没找到一条路径" << endl;
        return false;
    }
    heu_astar_ = final_node_->getG() * resolution_;
    cout << "Astar Heuristic:" << heu_astar_ << endl;
    cout << "探索的节点数是：" << explored_node_num << endl;
    return true;

}

bool GridSearch::generateDpMap(Vec2d goal_pos) {
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;

    dp_map_.clear();

    Vec2i goal_idx = getIndexFromPosition(goal_pos);

    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_set.emplace(goal_node->getIndex(), goal_node);
    open_pq.emplace(goal_node->getIndex(), goal_node->getG());

    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        dp_map_.emplace(cur_index, cur_node);

        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        //cout << "得到邻居节点" << endl;
        for (auto &neighbor_node : neighbor_nodes) {

            //检测是否在地图以内
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            //cout << "在地图以内" << endl;

            //是否在障碍物上
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            //cout << "不在障碍物上" << endl;

            if (dp_map_.find(neighbor_node->getIndex()) != dp_map_.end()) {
                continue;
            }

            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                ++explored_node_num;
                neighbor_node->setParent(cur_node);
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getG());
            } else {
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }

        }

    }

    cout << "搜索的节点数是：" << explored_node_num << endl; 
    return true;
}

double GridSearch::lookupInDpMap(Vec2d start_pos) {
    Vec2i start_idx = getIndexFromPosition(start_pos);
    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);

    if (dp_map_.find(start_node->getIndex()) != dp_map_.end()) {
        return dp_map_[start_node->getIndex()]->getG() * resolution_;
    } else {
        return numeric_limits<double>::infinity();
    }
}

vector<shared_ptr<Node2d>> GridSearch::getNeighborNodes(shared_ptr<Node2d> cur_node) {
    int cur_node_x = cur_node->getPosXInMap();
    int cur_node_y = cur_node->getPosYInMap();
    int cur_node_g = cur_node->getG();
    double diagonal_distance = sqrt(2.0);
    vector<shared_ptr<Node2d>> neighbors;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            Vec2i neighbor_idx{cur_node_x + i, cur_node_y + j};
            shared_ptr<Node2d> neightbor = make_shared<Node2d>(neighbor_idx, map_size_x_);
            if (i ==0 && j == 0) continue;
            if (sqrt(i * i + j * j) > 1) {
                neightbor->setG(cur_node_g + diagonal_distance);
            } else {
                neightbor->setG(cur_node_g + 1.0);
            }
            neighbors.emplace_back(neightbor);
        }
    }
    return neighbors;
}

bool GridSearch::isInGridMap(shared_ptr<Node2d> node) {
    int index_x = node->getPosXInMap();
    int index_y = node->getPosYInMap();
    // cout << "到这了吗" << endl;
    // cout << "index_x: " << index_x << endl;
    // cout << "index_y: " << index_y << endl;
    // cout << "map_size_y_" << map_size_y_ << endl;

    if (index_x < 0 || index_x >= map_size_x_ || index_y < 0 || index_y >= map_size_y_) {
        return false;
    }
    
    return true;
}

bool GridSearch::isOnObstacle(shared_ptr<Node2d> node) {
    int index_x = node->getPosXInMap();
    int index_y = node->getPosYInMap();
    Vec2d pos = getPositionFromIndex({index_x, index_y});
    // cout << "index_x: " << pos(0) << endl;
    // cout << "index_y: " << pos(1) << endl;
    if (map_.atPosition("elevation", pos) > 0) {
        return true;
    }
    return false;
}

vector<Vec2d> GridSearch::getAstartPath() {
    shared_ptr<Node2d> cur_node = final_node_;
    vector<shared_ptr<Node2d>> vec;
    vector<Vec2d> res;
    //cout << "回溯 " << endl;

    while (cur_node->getParentNode() != nullptr) {
        vec.emplace_back(cur_node);
        cur_node = cur_node->getParentNode();

    }
    reverse(vec.begin(), vec.end());
    //cout << "vec 大小：" << vec.size() << endl;

    for (auto &node : vec) {
        
        res.push_back({node->getPosXInMap() * resolution_ + x_min_,
                       node->getPosYInMap() * resolution_ + y_min_ });
        //cout << "cur_node->getPosXInMap():" << node->getPosXInMap() << endl;
    }
    return res;
}


double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

Vec2i GridSearch::getIndexFromPosition(Vec2d position) {
    Vec2i index;
    index(0) = static_cast<int>((position(0) - x_min_) / resolution_);
    index(1) = static_cast<int>((position(1) - y_min_) / resolution_);
    return index;
}
Vec2d GridSearch::getPositionFromIndex(Vec2i index) {
    Vec2d pos;
    pos(0) = x_min_ + (index(0) + 0.5) * resolution_;
    pos(1) = y_min_ + (index(1) + 0.5) * resolution_;
    return pos;
}

} // namespace planning


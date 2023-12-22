#pragma once

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "type_defs.h"

namespace planning
{

class Visualize
{
private:
    ros::NodeHandle nh;
    ros::Publisher explored_nodes_pub, vehicle_boxes_pub, path_points_pub;
    geometry_msgs::PoseArray nodes_pose;
    visualization_msgs::MarkerArray vehicle_boxes, path_points;
public:
    Visualize(/* args */) {
        explored_nodes_pub = nh.advertise<geometry_msgs::PoseArray>("/visualize_nodes_pose", 10);
        vehicle_boxes_pub = nh.advertise<visualization_msgs::MarkerArray>("/vehicle_boxes", 10);
        path_points_pub = nh.advertise<visualization_msgs::MarkerArray>("/path_points", 10);
    }

    void clear() {
        nodes_pose.poses.clear();
        vehicle_boxes.markers.clear();
    }

    void publishExploredNodes(Vec3d node_pose) {
        
        nodes_pose.header.frame_id = "map";
        nodes_pose.header.stamp = ros::Time::now();
        geometry_msgs::Pose pose;
        pose.position.x = node_pose(0);
        pose.position.y = node_pose(1);
        pose.orientation = tf::createQuaternionMsgFromYaw(node_pose(2));
        nodes_pose.poses.push_back(pose);
        explored_nodes_pub.publish(nodes_pose);
    }
    
    void publishVehicleBoxes(Vec3d node_pose, int i) {

        visualization_msgs::Marker vehicle_box;
        if (i == 0) {
            vehicle_box.action = 3;
        }
        vehicle_box.header.frame_id = "map";
        vehicle_box.header.stamp = ros::Time(0);
        vehicle_box.id = i;
        vehicle_box.type = visualization_msgs::Marker::CUBE;
        vehicle_box.scale.x = 4.933;
        vehicle_box.scale.y = 2.11;
        vehicle_box.scale.z = 1;
        vehicle_box.color.a = 0.05;
        vehicle_box.color.r = 0;
        vehicle_box.color.b = 1;
        vehicle_box.color.g = 0;

        
        Vec2d center = {node_pose(0) + 1.45 * std::cos(node_pose(2)),
                    node_pose(1) + 1.45 * std::sin(node_pose(2))};
        vehicle_box.pose.position.x = center(0);
        vehicle_box.pose.position.y = center(1);
        vehicle_box.pose.orientation = tf::createQuaternionMsgFromYaw(node_pose(2));
        vehicle_boxes.markers.push_back(vehicle_box);
        vehicle_boxes_pub.publish(vehicle_boxes);
    }

    void publishPathPoint(Vec2d position, int i) {
        visualization_msgs::Marker path_point;

        if (i == 0) {
            path_point.action = 3;
        }

        path_point.header.frame_id = "map";
        path_point.header.stamp = ros::Time(0);
        path_point.id = i;
        path_point.type = visualization_msgs::Marker::SPHERE;
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.scale.z = 0.1;

        path_point.color.a = 1;
        path_point.color.r = 1;
        path_point.color.g = 0;
        path_point.color.b = 0;

        path_point.pose.position.x = position(0);
        path_point.pose.position.y = position(1);
        path_point.pose.position.z = 0.1;

        path_point.pose.orientation.w = 1.0;

        path_points.markers.push_back(path_point);

        path_points_pub.publish(path_points);

    }
};










} // namespace planning








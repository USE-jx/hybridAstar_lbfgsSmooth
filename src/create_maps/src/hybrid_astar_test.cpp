#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <boost/program_options.hpp>
#include <ompl/config.h>


#include "hybrid_astar_searcher/calculate_heuristic.h"
#include "hybrid_astar_searcher/hybrid_astar.h"
#include "hybrid_astar_searcher/dynamicvoronoi.h"
#include "hybrid_astar_searcher/smooth.h"



#include <tf2/utils.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace planning;
using namespace std;

ros::Publisher path_vis_pub, marker_pub;

/*---------optimize related-------------*/
ros::Publisher  optimized_traj_pub;

nav_msgs::Path path;
geometry_msgs::PoseStamped pose;

int N = 100;
double dt = 0.01;
Eigen::Vector3d start, goal;




ros::Publisher ellipsoid_pub, polyhedron_pub,voronoi_pub;

typedef ompl::base::SE2StateSpace::StateType State;
using namespace grid_map;
GridMap gmap;
std::unique_ptr<HybridAstar> hybrid_astar_ptr;
HybridAstarResult result;
//std::unique_ptr<DynamicVoronoi> voronoiDiagram_ptr;
visualization_msgs::Marker voronoi;

std::unique_ptr<Smoother> smoother_ptr;
DynamicVoronoi voronoiDiagram;

bool has_start, has_goal;
Vec3d start_pose, goal_pose;

void Gridmap_Callback(grid_map_msgs::GridMap msg){
    if (gmap.exists("elevation")) return;
    std::cout << "receive map" << std::endl; 
    GridMapRosConverter::fromMessage(msg, gmap);
    std::cout << "FrameId:" << gmap.getFrameId() << std::endl;
    std::cout << "map :" << gmap.getLength().x() << std::endl;
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    gmap.getLength().x(), gmap.getLength().y(),
    gmap.getSize()(0), gmap.getSize()(1));
    hybrid_astar_ptr.reset(new HybridAstar(gmap));

    int size_x = gmap.getSize()(0);
    int size_y = gmap.getSize()(1);
    //cout << "size :" << size_x << " " << size_y << endl;

    bool **bin_map;
    bin_map = new bool *[size_x];
    for (int i = 0; i < size_x; ++i) {
        bin_map[i] = new bool[size_y];
        for (int j = 0; j < size_y; ++j) {
            Vec2d pos;
            pos(0) = -25 + (i + 0.5) * gmap.getResolution();
            pos(1) = -25 + (j + 0.5) * gmap.getResolution();
            //cout << "gmap.getResolution():" <<gmap.getResolution() << endl;
            //cout << map.atPosition("elevation", pos);
            //cout << pos(0) << "x" << pos(1) << endl;
            if (gmap.atPosition("elevation", pos) > 0) {
                bin_map[i][j] = true;
            } else {
                bin_map[i][j] = false;
            }
        }
    }

    voronoiDiagram.initializeMap(size_x, size_y, bin_map);
    
    //voronoiDiagram.update();
    //voronoiDiagram.prune();
   
    

    // voronoiDiagram.visualize("../result.pgm");
    // //cout << "voronoi图可视化" << std::endl;

    
    // voronoi.header.frame_id = "map";
    // voronoi.header.stamp = ros::Time();
    // voronoi.ns = "voronoi";
    // voronoi.id = 0;
    // voronoi.type = visualization_msgs::Marker::SPHERE_LIST;
    // voronoi.action = visualization_msgs::Marker::ADD;

    // voronoi.color.b = 1.0;
    // voronoi.color.a = 1.0;

    // voronoi.scale.x = 0.1;
    // voronoi.scale.y = 0.1;
    // voronoi.scale.z = 0.1;
    // geometry_msgs::Point p;
    // for (int i = size_y-1; i >= 0; --i) {
    //     for (int j = 0; j < size_x; ++j) {
    //         if (voronoiDiagram.isVoronoi(i, j)) {
    //             Vec2d pos;
    //             pos(0) = -25 + (i + 0.5) * gmap.getResolution();
    //             pos(1) = -25 + (j + 0.5) * gmap.getResolution();
    //             p.x = pos(0);
    //             p.y = pos(1);
    //             p.z = 0.05;
    //             voronoi.points.push_back(p);
    //         }
    //     }
    // }
    

}   

void visPath(std::vector<Eigen::Vector3d> path);
void run() {
    //ROS_INFO("run()");
    if (has_start && has_goal) {
        ROS_INFO("has start and goal");
        voronoi_pub.publish(voronoi);
        auto hybrid_astar_start = std::chrono::high_resolution_clock::now();
        Visualize vis;
        if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
            ROS_INFO("search success");
            auto hybrid_astar_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start;
            cout << "hybrid astar use time:" << hybrid_astar_use_time.count() * 1000 << "ms" << std::endl;
            vector<Vec3d> path2smooth;
            for (int i = 0; i < result.x.size(); ++i) {
                path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});
            }
            
            auto smooth_path_start = std::chrono::high_resolution_clock::now();
            smoother_ptr->optimize(voronoiDiagram, path2smooth);
            //smoother_ptr->smoothPath(voronoiDiagram, path2smooth);
            
            vector<Vec3d> smooth_path;
            smoother_ptr->getSmoothPath(smooth_path);
            auto smooth_path_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> smooth_path_use_time = smooth_path_end - smooth_path_start;
            cout << "smooth path use time:" << smooth_path_use_time.count() * 1000 << "ms" << std::endl;

           

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map"; // 
            marker.header.stamp = ros::Time::now();
            marker.ns = "rs_path";
            marker.id = 1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.scale.x = 0.05; // Line width

           // Set marker color and opacity
            marker.color.a = 1.0; // Fully opaque
            marker.color.r = 0.0; // Red color
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            
            for (size_t i = 0; i < result.x.size(); ++i)
            {
                geometry_msgs::Point point;
                point.x = result.x[i];
                point.y = result.y[i];
                point.z = 0.1;
                marker.points.push_back(point);
                vis.publishPathPoint({result.x[i], result.y[i]}, i);

                vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i);
            }
            marker_pub.publish(marker);
            visPath(smooth_path);

            has_start = false;
            has_goal = false;
            
            
        } else {
            ROS_WARN("search fail");
        }}
    // } else {
    //     ROS_INFO("waiting start or goal");
    // }

}

void visPath(std::vector<Eigen::Vector3d> path) {
    nav_msgs::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pos;
    for (const auto pose : path) {
        pos.pose.position.x = pose[0];
        pos.pose.position.y = pose[1];
        pos.pose.position.z = 0.05;
        //std::cout << "x,y:" << pose[0] << " " << pose[1] << std::endl; 
        nav_path.poses.push_back(pos);
    }
    path_vis_pub.publish(nav_path);

}
void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {

    start_pose[0] = msg.pose.pose.position.x;
    start_pose[1] = msg.pose.pose.position.y;
    start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);
    has_start = true;
    ROS_INFO("receive start position");

}

void goalCallback(const geometry_msgs::PoseStamped msg) {

    goal_pose[0] = msg.pose.position.x;
    goal_pose[1] = msg.pose.position.y;
    goal_pose[2] = tf2::getYaw(msg.pose.orientation);
    has_goal = true;
    ROS_INFO("receive goal position");
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "test ompl");
    ros::NodeHandle nh;

    ros::Subscriber gridmap_sub = nh.subscribe("/grid_map", 1, Gridmap_Callback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("rs_path_marker", 10);
    
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, &startCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);

    path_vis_pub = nh.advertise<nav_msgs::Path>("/path", 1);
    optimized_traj_pub = nh.advertise<nav_msgs::Path>("/optimized_path", 1);

    voronoi_pub = nh.advertise<visualization_msgs::Marker>("voronoi", 10);

    //voronoiDiagram_ptr.reset(new DynamicVoronoi());
    smoother_ptr.reset(new Smoother());



    ros::Rate rate(10);
    while (ros::ok()) {
        run();   
        
        ros::spinOnce();
        rate.sleep();
    }

    //ros::spin();
    return 0;
}
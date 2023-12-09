#ifndef GET_PLAN_H
#define GET_PLAN_H
#include <iostream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>
#include <string>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Surface_sweep_2_algorithms.h>
#include <CGAL/squared_distance_2.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cgal_definitions.hpp"
#include "cgal_comm.hpp"
#include "decomposition.hpp"
#include "sweep.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "capston_msgs/srv/waypoints.hpp"
#include "capston_msgs/srv/robotstates.hpp"

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <future>


using namespace std::chrono_literals;

class CellNode
{
public:
    CellNode()
    {
        isVisited = false;
        isCleaned = false;
        parentIndex = INT_MAX;
        cellIndex = INT_MAX;
    }
    bool isVisited;
    bool isCleaned;

    int parentIndex;
    std::deque<int> neighbor_indices;

    int cellIndex;
};

class GetWaypoints : public rclcpp::Node
{
public:
    GetWaypoints();

private:
    bool makeplan();

    int robot_radius;
    int sweep_step;
    int map_y;
    double origin_x;
    double origin_y;
    double start_position_x;
    double start_position_y;
    std::string map_location;

    bool start_robot = false;
    std::shared_ptr<capston_msgs::srv::Waypoints::Request> waypoint_request = std::make_shared<capston_msgs::srv::Waypoints::Request>();

    rclcpp::Service<capston_msgs::srv::Robotstates>::SharedPtr robot_state_service_;
    rclcpp::Client<capston_msgs::srv::Waypoints>::SharedPtr waypoint_client_;

    int getCellIndexOfPoint(const std::vector<Polygon_2>& decompositions, const Point_2& point);
    std::vector<std::map<int, std::list<Point_2 >>> calculateCellIntersections(std::vector<Polygon_2>& decompositions, std::vector<CellNode>& cell_graph);
    bool doReverseNextSweep(const Point_2& curr_point, const std::vector<Point_2>& next_sweep);
    std::vector<Point_2> getShortestPath(const Polygon_2& polygon, const Point_2& start, const Point_2& goal);
    Point_2 findNextGoal(const Point_2& start, const Point_2& goal, const std::list<Point_2>& candidates);
    std::vector<CellNode> calculateDecompositionAdjacency(const std::vector<Polygon_2>& decompositions);
    void walkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path);
    std::deque<int> getTravellingPath(const std::vector<CellNode>& cell_graph, int first_cell_index);
};


#endif
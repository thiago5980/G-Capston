#include "capston_plan/capstonplanning.hpp"

GetWaypoints::GetWaypoints() : Node("coverage_pathplanner_node")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    waypoint_client_ = this->create_client<capston_msgs::srv::Waypoints>("/ccpp_waypoints");

    while (!waypoint_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    robot_state_service_ = this->create_service<capston_msgs::srv::Robotstates>("calculate_plan", [this](const std::shared_ptr<capston_msgs::srv::Robotstates::Request> request,
                                                                                                std::shared_ptr<capston_msgs::srv::Robotstates::Response> states_response) -> void
    {
        this->robot_radius = request->robot_radius;
        this->sweep_step = request->sweep_step;
        this->map_y = request->map_height;
        this->origin_x = request->origin.x;
        this->origin_y = request->origin.y;
        this->start_position_x = request->start_position.x;
        this->start_position_y = request->start_position.y;
        this->map_location = request->map_location;
        std::cout << "location :" << this->map_location << std::endl;
        if (this->makeplan())
        {
            using ServiceResponseFuture = rclcpp::Client<capston_msgs::srv::Waypoints>::SharedFuture;
            auto response_received_callback = [this, states_response](ServiceResponseFuture future){
                auto response = future.get();
                if (response->getpoint)
                    this->start_robot = true;
                else
                    this->start_robot = false;
                return;
            };
            auto future_result = waypoint_client_->async_send_request(this->waypoint_request, response_received_callback);
            states_response->end_cal = true;
        }
        else
        {   
            states_response->end_cal = false;
        }
    });
}


bool GetWaypoints::makeplan()
{
    double pixel_origin_x = -origin_x*20.0;
    double pixel_origin_y = double(this->map_y) + origin_y*20.0;
    double pixel_start_x = pixel_origin_x + start_position_x*20.0;
    double pixel_start_y = pixel_origin_y - start_position_y*20.0;
    // double start_x = std::abs(origin_x) + start_position_x;
    // double start_y = std::abs(origin_y) + start_position_y;
    // std::cout << pixel_origin_x << " " << pixel_origin_y << " " << pixel_start_x << " " << pixel_start_y << "\n";
    cv::Mat img = cv::imread(this->map_location);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::Mat img_ = gray.clone();

    cv::threshold(img_, img_, 250, 255, 0);
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_radius,robot_radius), cv::Point(-1,-1)); // size: robot radius
    cv::morphologyEx(img_, img_, cv::MORPH_ERODE, erode_kernel);


    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1,-1));
    cv::morphologyEx(img_, img_, cv::MORPH_OPEN, open_kernel);

    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Vec4i> hierarchy; // index: next, prev, first_child, parent
    cv::findContours(img_, cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<int> cnt_indices(cnts.size());
    std::iota(cnt_indices.begin(), cnt_indices.end(), 0);
    std::sort(cnt_indices.begin(), cnt_indices.end(), [&cnts](int lhs, int rhs){return cv::contourArea(cnts[lhs]) > cv::contourArea(cnts[rhs]);});
    int ext_cnt_idx = cnt_indices.front();

    cv::Mat cnt_canvas = img.clone();
    cv::drawContours(cnt_canvas, cnts, ext_cnt_idx, cv::Scalar(0,0,255));
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(cnts[ext_cnt_idx]);
    // find all the contours of obstacle
    for(int i = 0; i < hierarchy.size(); i++){
        if(hierarchy[i][3]==ext_cnt_idx){ // parent contour's index equals to external contour's index
            contours.emplace_back(cnts[i]);
            cv::drawContours(cnt_canvas, cnts, i, cv::Scalar(255,0,0));
        }
    }
    // cv::imshow("contours", cnt_canvas);

    cv::Mat cnt_img = cv::Mat(img.rows, img.cols, CV_8UC3);
    cnt_img.setTo(255);
    for(int i = 0; i < contours.size(); i++){
        cv::drawContours(cnt_img, contours, i, cv::Scalar(0,0,0));
    }
    // cv::imshow("only contours", cnt_img);

    cv::Mat poly_canvas = img.clone();
    std::vector<cv::Point> poly;
    std::vector<std::vector<cv::Point>> polys;
    for(auto& contour : contours){
        cv::approxPolyDP(contour, poly, 3, true);
        polys.emplace_back(poly);
        poly.clear();
    }
    for(int i = 0; i < polys.size(); i++){
        cv::drawContours(poly_canvas, polys, i, cv::Scalar(255,0,255));
    }
//    cv::imshow("polygons", poly_canvas);
    cv::Mat poly_img = cv::Mat(img.rows, img.cols, CV_8UC3);
    poly_img.setTo(255);
    for(int i = 0; i < polys.size(); i++){
        cv::drawContours(poly_img, polys, i, cv::Scalar(0,0,0));
    }
    // compute main direction

    // [0,180)
    std::vector<int> line_deg_histogram(180);
    double line_len; // weight
    double line_deg;
    int line_deg_idx;

    cv::Mat line_canvas = img.clone();
    auto ext_poly = polys.front();
    ext_poly.emplace_back(ext_poly.front());          
    for(int i = 1; i < ext_poly.size(); i++){
        line_len = std::sqrt(std::pow((ext_poly[i].x-ext_poly[i-1].x),2)+std::pow((ext_poly[i].y-ext_poly[i-1].y),2));
        // y-axis towards up, x-axis towards right, theta is from x-axis to y-axis
        line_deg = std::round(atan2(-(ext_poly[i].y-ext_poly[i-1].y),(ext_poly[i].x)-ext_poly[i-1].x)/M_PI*180.0); // atan2: (-180, 180]
        line_deg_idx = (int(line_deg) + 180) % 180; // [0, 180)
        line_deg_histogram[line_deg_idx] += int(line_len);

    //    std::cout<<"deg: "<<line_deg_idx<<std::endl;
    //    cv::line(line_canvas, ext_poly[i], ext_poly[i-1], cv::Scalar(255,255,0));
    //    cv::imshow("lines",line_canvas);
    //    cv::waitKey();
    }

    auto it = std::max_element(line_deg_histogram.begin(), line_deg_histogram.end());
    int main_deg = (it-line_deg_histogram.begin());


    // construct polygon with holes

    std::vector<cv::Point> outer_poly = polys.front();
    polys.erase(polys.begin());
    std::vector<std::vector<cv::Point>> inner_polys = polys;


    Polygon_2 outer_polygon;
    for(const auto& point : outer_poly){
        outer_polygon.push_back(Point_2(point.x, point.y));
    }

    int num_holes = inner_polys.size();
    std::vector<Polygon_2> holes(num_holes);
    for(int i = 0; i < inner_polys.size(); i++){
        for(const auto& point : inner_polys[i]){
            holes[i].push_back(Point_2(point.x, point.y));
        }
    }

    PolygonWithHoles pwh(outer_polygon, holes.begin(), holes.end());

    // cell decomposition

    std::vector<Polygon_2> bcd_cells;

//    polygon_coverage_planning::computeBestTCDFromPolygonWithHoles(pwh, &bcd_cells);
    polygon_coverage_planning::computeBestBCDFromPolygonWithHoles(pwh, &bcd_cells);

    auto cell_graph = calculateDecompositionAdjacency(bcd_cells);


    //////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////start position////////////////////////////////////////////////////
    // Point_2 start = getStartingPoint(img);
    Point_2 start; 

    start = Point_2(static_cast<int>(pixel_start_x), static_cast<int>(pixel_start_y));
    // std::cout << "x: " << static_cast<int>(pixel_start_x) << " y: " << map_y - static_cast<int>(pixel_start_y);
    // start.x = abs(int(origin_x))
    // start.y = map_y - abs(int(origin_y))    
    //////////////////////////////////////////////////////////////////////////////////////////

    int starting_cell_idx = getCellIndexOfPoint(bcd_cells, start);
    auto cell_idx_path = getTravellingPath(cell_graph, starting_cell_idx);
    std::cout<<"path length: "<<cell_idx_path.size()<<std::endl;
    // std::cout<<"start";
    // for(auto& cell_idx:cell_idx_path){
    //     std::cout<<"->"<<cell_idx;
    // }

    std::vector<std::vector<Point_2>> cells_sweeps;

    for (size_t i = 0; i < bcd_cells.size(); ++i) {
        // Compute all cluster sweeps.
        std::vector<Point_2> cell_sweep;
        Direction_2 best_dir;
        polygon_coverage_planning::findBestSweepDir(bcd_cells[i], &best_dir);
        polygon_coverage_planning::visibility_graph::VisibilityGraph vis_graph(bcd_cells[i]);

        bool counter_clockwise = true;
        polygon_coverage_planning::computeSweep(bcd_cells[i], vis_graph, sweep_step, best_dir, counter_clockwise, &cell_sweep);
        cells_sweeps.emplace_back(cell_sweep);
    }

    auto cell_intersections = calculateCellIntersections(bcd_cells, cell_graph);
    std::vector<Point_2> way_points;

//////////////////////////////////////////////////////////////////////////////////////////
    Point_2 point = start;
    std::list<Point_2> next_candidates;
    Point_2 next_point;
    std::vector<Point_2> shortest_path;

    if(doReverseNextSweep(start, cells_sweeps[cell_idx_path.front()])){
        shortest_path = getShortestPath(bcd_cells[cell_idx_path.front()], start, cells_sweeps[cell_idx_path.front()].back());
        way_points.insert(way_points.end(), shortest_path.begin(), std::prev(shortest_path.end()));
    } else{
        shortest_path = getShortestPath(bcd_cells[cell_idx_path.front()], start, cells_sweeps[cell_idx_path.front()].front());
        way_points.insert(way_points.end(), shortest_path.begin(), std::prev(shortest_path.end()));
    }

    point = way_points.back();

    for(size_t i = 0; i < cell_idx_path.size(); ++i){
        // has been cleaned?
        if(!cell_graph[cell_idx_path[i]].isCleaned){
            // need to reverse?
            if(doReverseNextSweep(point, cells_sweeps[cell_idx_path[i]])){
                way_points.insert(way_points.end(), cells_sweeps[cell_idx_path[i]].rbegin(), cells_sweeps[cell_idx_path[i]].rend());
            }else{
                way_points.insert(way_points.end(), cells_sweeps[cell_idx_path[i]].begin(), cells_sweeps[cell_idx_path[i]].end());
            }
            // now cleaned
            cell_graph[cell_idx_path[i]].isCleaned = true;
            // update current point
            point = way_points.back();
            // find shortest path to next cell
            if((i+1)<cell_idx_path.size()){
                next_candidates = cell_intersections[cell_idx_path[i]][cell_idx_path[i+1]];
                if(doReverseNextSweep(point, cells_sweeps[cell_idx_path[i+1]])){
                    next_point = findNextGoal(point, cells_sweeps[cell_idx_path[i+1]].back(), next_candidates);
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
                    way_points.insert(way_points.end(), std::next(shortest_path.begin()), std::prev(shortest_path.end()));
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i+1]], next_point, cells_sweeps[cell_idx_path[i+1]].back());
                }else{
                    next_point = findNextGoal(point, cells_sweeps[cell_idx_path[i+1]].front(), next_candidates);
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
                    way_points.insert(way_points.end(), std::next(shortest_path.begin()), std::prev(shortest_path.end()));
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i+1]], next_point, cells_sweeps[cell_idx_path[i+1]].front());
                }
                way_points.insert(way_points.end(), shortest_path.begin(), std::prev(shortest_path.end()));
                point = way_points.back();
            }
        }else{
            shortest_path = getShortestPath(bcd_cells[cell_idx_path[i]],
                                            cells_sweeps[cell_idx_path[i]].front(),
                                            cells_sweeps[cell_idx_path[i]].back());
            if(doReverseNextSweep(point, cells_sweeps[cell_idx_path[i]])){
                way_points.insert(way_points.end(), shortest_path.rbegin(), shortest_path.rend());
            }else{
                way_points.insert(way_points.end(), shortest_path.begin(), shortest_path.end());
            }
            point = way_points.back();

            if((i+1)<cell_idx_path.size()){
                next_candidates = cell_intersections[cell_idx_path[i]][cell_idx_path[i+1]];
                if(doReverseNextSweep(point, cells_sweeps[cell_idx_path[i+1]])){
                    next_point = findNextGoal(point, cells_sweeps[cell_idx_path[i+1]].back(), next_candidates);
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
                    way_points.insert(way_points.end(), std::next(shortest_path.begin()), std::prev(shortest_path.end()));
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i+1]], next_point, cells_sweeps[cell_idx_path[i+1]].back());
                }else{
                    next_point = findNextGoal(point, cells_sweeps[cell_idx_path[i+1]].front(), next_candidates);
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i]], point, next_point);
                    way_points.insert(way_points.end(), std::next(shortest_path.begin()), std::prev(shortest_path.end()));
                    shortest_path = getShortestPath(bcd_cells[cell_idx_path[i+1]], next_point, cells_sweeps[cell_idx_path[i+1]].front());
                }
                way_points.insert(way_points.end(), shortest_path.begin(), std::prev(shortest_path.end()));
                point = way_points.back();
            }
        }
    }
    std::cout << "\n";
    cv::Point p1, p2;
    // cv::namedWindow("cover",cv::WINDOW_NORMAL);
    // cv::imshow("cover", img);
    // cv::waitKey();


    std::cout << "way points size : " << way_points.size() <<  "\n";
    auto PoseArray = geometry_msgs::msg::PoseArray();
    auto pose = geometry_msgs::msg::Pose();
    pose.position.x = (CGAL::to_double(way_points[0].x()) - double(pixel_origin_x))*0.05; // +origin_x is because minus 
    pose.position.y = (double(pixel_origin_y) - CGAL::to_double(way_points[0].y()))*0.05; // resolution 0.05
    // std::cout << "first point : " << way_points[0].x() << ", " << way_points[0].y() << std::endl;    
    PoseArray.poses.push_back(pose);   
    for(size_t i = 1; i < way_points.size(); ++i){
        p1 = cv::Point(CGAL::to_double(way_points[i-1].x()),CGAL::to_double(way_points[i-1].y()));
        p2 = cv::Point(CGAL::to_double(way_points[i].x()),CGAL::to_double(way_points[i].y()));
        pose.position.x = (double(p2.x) - pixel_origin_x)*0.05;
        pose.position.y = (pixel_origin_y - double(p2.y))*0.05;
        
        PoseArray.poses.push_back(pose);
        cv::line(img, p1, p2, cv::Scalar(0, 64, 255));
        cv::namedWindow("cover",cv::WINDOW_NORMAL);
        cv::imshow("cover", img);
        cv::waitKey(5);
        cv::line(img, p1, p2, cv::Scalar(200, 200, 200));
    }

    this->waypoint_request->points = PoseArray;
    return true;
}

int GetWaypoints::getCellIndexOfPoint(const std::vector<Polygon_2>& decompositions, const Point_2& point)
{
    int index = -1;
    for(int i = 0; i < decompositions.size(); i++){
        if(polygon_coverage_planning::pointInPolygon(decompositions[i], point)){
            index = i;
            break;
        }
    }
    return index;    
}

std::vector<std::map<int, std::list<Point_2 >>> GetWaypoints::calculateCellIntersections(std::vector<Polygon_2>& decompositions, std::vector<CellNode>& cell_graph){

    std::vector<std::map<int, std::list<Point_2 >>> cell_intersections(cell_graph.size());

    for(size_t i = 0; i < cell_graph.size(); ++i){
        for(size_t j = 0; j < cell_graph[i].neighbor_indices.size(); ++j){
            std::list<Point_2> pts;
            for(auto m = decompositions[i].edges_begin(); m != decompositions[i].edges_end(); ++m){
                for(auto n = decompositions[cell_graph[i].neighbor_indices[j]].edges_begin();
                    n != decompositions[cell_graph[i].neighbor_indices[j]].edges_end();
                    ++n){
                    Segment_2 segments[] = {*m, *n};
                    CGAL::compute_intersection_points(segments, segments+2, std::back_inserter(pts));
                }
            }

            for(auto p = decompositions[i].vertices_begin(); p != decompositions[i].vertices_end(); ++p){
                for(auto q = decompositions[cell_graph[i].neighbor_indices[j]].vertices_begin(); q != decompositions[cell_graph[i].neighbor_indices[j]].vertices_end(); ++q){
                    if(CGAL::to_double(p->x())==CGAL::to_double(q->x()) && CGAL::to_double(p->y())==CGAL::to_double(q->y())){
                        pts.insert(pts.end(), *p);
                    }
                }
            }

            auto verbose = std::unique(pts.begin(), pts.end());
            pts.erase(verbose, pts.end());
            cell_intersections[i].insert(std::make_pair(cell_graph[i].neighbor_indices[j], pts));
            cell_intersections[cell_graph[i].neighbor_indices[j]].insert(std::make_pair(i, pts));
        }
    }

    return cell_intersections;
}

bool GetWaypoints::doReverseNextSweep(const Point_2& curr_point, const std::vector<Point_2>& next_sweep){
    return CGAL::to_double(CGAL::squared_distance(curr_point, next_sweep.front())) > CGAL::to_double(CGAL::squared_distance(curr_point, next_sweep.back()));
}

std::vector<Point_2> GetWaypoints::getShortestPath(const Polygon_2& polygon, const Point_2& start, const Point_2& goal)
{
    polygon_coverage_planning::visibility_graph::VisibilityGraph vis_graph(polygon);
    std::vector<Point_2> shortest_path;
    polygon_coverage_planning::calculateShortestPath(vis_graph, start, goal, &shortest_path);
    return shortest_path;
}

Point_2 GetWaypoints::findNextGoal(const Point_2& start, const Point_2& goal, const std::list<Point_2>& candidates)
{
    double min_cost = DBL_MAX;
    double cost;
    Point_2 next_point = start;
    Segment_2 seg_from_start, seg_to_goal;
    for(auto point = candidates.begin(); point != candidates.end(); ++point){
        seg_from_start = Segment_2(start, *point);
        seg_to_goal = Segment_2(*point, goal);
        cost = CGAL::to_double(seg_from_start.squared_length())+CGAL::to_double(seg_to_goal.squared_length());
        if(cost < min_cost){
            min_cost = cost;
            next_point = *point;
        }
    }
    return next_point;
}

std::vector<CellNode> GetWaypoints::calculateDecompositionAdjacency(const std::vector<Polygon_2>& decompositions)
{
    std::vector<CellNode> polygon_adj_graph(decompositions.size());
    for (size_t i = 0; i < decompositions.size() - 1; ++i) {
        polygon_adj_graph[i].cellIndex = i;
        for (size_t j = i + 1; j < decompositions.size(); ++j) {
            PolygonWithHoles joined;
            if (CGAL::join(decompositions[i], decompositions[j], joined)) {
                polygon_adj_graph[i].neighbor_indices.emplace_back(j);
                polygon_adj_graph[j].neighbor_indices.emplace_back(i);
            }
        }
    }
    polygon_adj_graph.back().cellIndex = decompositions.size()-1;

    return polygon_adj_graph;
}

void GetWaypoints::walkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path)
{
    if(!cell_graph[cell_index].isVisited){
        cell_graph[cell_index].isVisited = true;
        unvisited_counter--;
    }
    path.emplace_front(cell_graph[cell_index]);

//    for debugging
//    std::cout<< "cell: " <<cell_graph[cell_index].cellIndex<<std::endl;
//

    CellNode neighbor;
    int neighbor_idx = INT_MAX;

    for(int i = 0; i < cell_graph[cell_index].neighbor_indices.size(); i++){
        neighbor = cell_graph[cell_graph[cell_index].neighbor_indices[i]];
        neighbor_idx = cell_graph[cell_index].neighbor_indices[i];
        if(!neighbor.isVisited){
            break;
        }
    }

    // unvisited neighbor found
    if(!neighbor.isVisited){
        cell_graph[neighbor_idx].parentIndex = cell_graph[cell_index].cellIndex;
        walkThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
    }
    // unvisited neighbor not found
    else{
        // cannot go on back-tracking
        if (cell_graph[cell_index].parentIndex == INT_MAX){
            return;
        }else if(unvisited_counter == 0){
            return;
        }else{
            walkThroughGraph(cell_graph, cell_graph[cell_index].parentIndex, unvisited_counter, path);
        }
    }
}

std::deque<int> GetWaypoints::getTravellingPath(const std::vector<CellNode>& cell_graph, int first_cell_index)
{
    std::deque<int> travelling_path;

    std::deque<CellNode> _cell_path;
    std::vector<CellNode> _cell_graph = cell_graph;

    if(_cell_graph.size()==1){
        travelling_path.emplace_back(0);
    }else{
        int unvisited_counter = _cell_graph.size();
        walkThroughGraph(_cell_graph, first_cell_index, unvisited_counter, _cell_path);
        std::reverse(_cell_path.begin(), _cell_path.end());
    }

    for(auto& cell : _cell_path){
        travelling_path.emplace_back(cell.cellIndex);
    }

    return travelling_path;
}


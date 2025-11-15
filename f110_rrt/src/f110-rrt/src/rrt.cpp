#include "rrt/rrt.h"
#include "rrt/csv_reader.h"
#include <thread>
#include <chrono>

using namespace std;

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) , tf2_listener_(tf_buffer_){

    // Reading ros parameters form the yaml file
    std::string pose_topic, scan_topic, CSV_path;
    nh_.getParam("/pose_topic", pose_topic);
    nh_.getParam("/scan_topic", scan_topic);
    nh_.getParam("lookahead_distance", lookahead_distance_);
    nh_.getParam("inflation_radius", inflation_radius_);
    nh_.getParam("CSV_path", CSV_path);
    nh_.getParam("max_rrt_iters", max_rrt_iters_);
    nh_.getParam("goal_tolerance", goal_tolerance_);
    nh_.getParam("max_expansion_distance", max_expansion_distance_);
    nh_.getParam("search_radius", search_radius_);
    nh_.getParam("collision_checking_points", collision_checking_points_);
    nh_.getParam("local_lookahead_distance", local_lookahead_distance_);
    nh_.getParam("high_speed", high_speed_);
    nh_.getParam("medium_speed", medium_speed_);
    nh_.getParam("low_speed", low_speed_);
    nh_.getParam("path_reach_tol", path_reach_tol_);

    // get the map
    //input_map_ = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2.0)));
std::string map_topic = "map";
nh_.param("map_topic", map_topic, map_topic);   // 필요하면 파라미터로 바꿔쓰기

ROS_INFO_STREAM("Waiting for " << map_topic << " ...");
auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh_, ros::Duration(10.0));
if (!map_msg) {
    ROS_ERROR_STREAM("Timed out waiting for " << map_topic
                     << ". map_server/시뮬레이터가 실행 중인지와 토픽명이 맞는지 확인하세요.");
    throw std::runtime_error("No map received");
}
input_map_ = *map_msg;

map_origin_x   = input_map_.info.origin.position.x;
map_origin_y   = input_map_.info.origin.position.y;
map_cols_      = input_map_.info.width;
map_resolution_= input_map_.info.resolution;

ROS_INFO("Map loaded successfully");

    if(input_map_.data.empty()){
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }

    ROS_INFO("Map loaded succesfully");

    map_origin_x = input_map_.info.origin.position.x;
    map_origin_y = input_map_.info.origin.position.y;
    map_cols_ = input_map_.info.width;
    map_resolution_ = input_map_.info.resolution; 
    
    //read global waypoints csv
    rrt::CSVReader reader(CSV_path);
    global_path_ = reader.getData();

    //get laser to map transform
    try{
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map","laser",ros::Time(0));
    }
    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }  

    // subscribers
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    sleep(1);

    // publishers
    dynamic_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_map",1); 
    line_pub_ = nh_.advertise<visualization_msgs::Marker>("show_rrt_path",1); 
    waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("show_global_waypoints",1);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
    spline_pub_ = nh_.advertise<visualization_msgs::Marker>("show_spline",1); 

    //map
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;

    //path freezing
    path_found_ = false;
    
    ROS_INFO("Created new RRT Object.");
}

/**
 * @brief This function takes the x,y coordinates of a map cell and 
 * returns the row major index of that cell in the map vector 
 * @return index
 */
int RRT::get_row_major_index(const double x_map, const double y_map)
{
    int xi = static_cast<int>((x_map - map_origin_x)/map_resolution_);
    int yi = static_cast<int>((y_map - map_origin_y)/map_resolution_);
    int rows = static_cast<int>(input_map_.info.height);
    if (xi < 0 || yi < 0 || xi >= static_cast<int>(map_cols_) || yi >= rows) return -1;
    return yi*map_cols_ + xi;
}

/**
 * @brief This function recieves latest laser scans.
 * The occupancy grid is updated here
 * @param pointer to scan_msg 
 */
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) 
{   
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    const auto translation = tf_laser_to_map_.transform.translation;
    const double yaw = tf::getYaw(tf_laser_to_map_.transform.rotation);

    // field of view(fov) of the robot
    const auto start = static_cast<int>(scan_msg->ranges.size()/6);
    const auto end = static_cast<int>(5*scan_msg->ranges.size()/6); 
    const double angle_increment = scan_msg->angle_increment;
    double theta = scan_msg->angle_min + angle_increment*start;

    // looping through all the scans in fov  
    for(int i=start; i < end; i++){

        theta += angle_increment;

        const double hit_range = scan_msg->ranges[i];
        if(std::isinf(hit_range) || std::isnan(hit_range)) continue;

        const double x_base_link = hit_range*cos(theta);
        const double y_base_link = hit_range*sin(theta);

        if(x_base_link > lookahead_distance_ || y_base_link > lookahead_distance_) continue;

        //transform to map
        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

        // getting indices of inflated cells
        std::vector<int> index_of_inflated_obstacles = get_inflated_row_major_indices(x_map, y_map);

        // marking inflated cells as occupied
        for(const auto& index: index_of_inflated_obstacles)
        {
            if(input_map_.data[index] !=  100)
            {
                input_map_.data[index] = 100;
                new_obstacles_.emplace_back(index);
            }
        }
    }

    // local cost map clearing 
    clear_obstacles_count_++;
    if(clear_obstacles_count_ > 50){
        for(const auto index: new_obstacles_){
            input_map_.data[index] = 0;
        }
        new_obstacles_.clear();
        clear_obstacles_count_ = 0;
    }
         
    // Publishing local cost map
    dynamic_map_pub_.publish(input_map_);
}

/**
 * @brief This function recieves latest robot pose in map frame, typically from a particle filter
 * RRT main loop is also present in this function
 * @param pointer to pose_msg 
 */
void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{    
    if(path_found_ == true)
    {       
        //ROS_ERROR("Following the local path");
    
        //viz_path(local_path_, pose_msg->pose);

        line_pub_.publish(tree_marker_);
        waypoint_pub_.publish(node_marker_);

        auto spline = smooth_path(local_path_, 100);
	    local_path_ = spline;

        visualization_msgs::Marker path_marker = gen_path_marker(spline);

        spline_pub_.publish(path_marker);

        // Executing Pure Pursuit control   
        const auto trackpoint_and_distance =    
            get_best_local_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

        const auto local_trackpoint_map_frame = trackpoint_and_distance.first;
        const double distance = trackpoint_and_distance.second;

        geometry_msgs::Pose goal_way_point_car_frame;

        tf2::doTransform(local_trackpoint_map_frame, goal_way_point_car_frame, tf_map_to_laser_);
            
        const double steering_angle = 2*(goal_way_point_car_frame.position.y)/pow(distance, 2);

	    //ros::Duration(0.01).sleep();

        // publishing pure pursuit velocity and steering angle
        execute_control(steering_angle);
        
        std::array<double , 2> local_trackpoint;
        local_trackpoint[0] = local_trackpoint_map_frame.position.x;
        local_trackpoint[1] = local_trackpoint_map_frame.position.y;
        viz_point(local_trackpoint, true);
        
	
	//double dist = sqrt(pow(pose_msg->pose.position.x - spline.back().at(0) ,2) - pow(pose_msg->pose.position.y - spline.back().at(1), 2));
	
	//ROS_INFO_STREAM(dist);
	
        //if(dist < path_reach_tol_)
        //{
            path_found_ = false;
        //}
        return;  
    }
    
    // Updating current pose
    current_x_ = pose_msg->pose.position.x;
    current_y_ = pose_msg->pose.position.y;

    // Goal point for RRT algorithm
    const auto trackpoint = get_best_global_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

    //publishing this point
    viz_point(trackpoint, true);

    // RRT tree
    std::vector<Node> tree;

    // setting root node cost as 0.0 and its parent index as -1
    tree.emplace_back(Node(pose_msg->pose.position.x, pose_msg->pose.position.y, -1, 0.0));    

    int count=0;
    // Main RRT loop
    while(count < max_rrt_iters_){
        count++;
        
        //sample a node
        auto sample_node = sample();

        // check is node is occupied
        if(is_collided(sample_node[0], sample_node[1])){
            continue;
        }
        
        //get nearest node in the tree
        const int nearest_node_id = nearest(tree, sample_node); 

        //move the current node closer 
        Node new_node = Steer(tree[nearest_node_id], nearest_node_id, sample_node);

        const auto current_node_index = tree.size();

        //check if edge is collision free
        if(is_edge_collided(tree[nearest_node_id], new_node))
        {
            continue;
        }

        //RRT STAR
        //calculate cost of the node
        new_node.cost = cost(tree, new_node);

        // get neighbouring nodes for the current node
        const auto neigh_vec = near(tree, new_node);
        
        // rewire neighbouring nodes
        rewire(neigh_vec, tree, new_node);

        // add current node to the tree
        tree.emplace_back(new_node);

        if(tree.size() > 2 * max_rrt_iters_)
        {
            ROS_ERROR("CYCLE FOUND TREE SIZE EXCEEDED");
            return;
        }
        
	//cout << "running\n";
        //check if new node is the goal
        if(is_goal(new_node ,trackpoint[0], trackpoint[1])){
		
            // backtracking path
            local_path_ = find_path(tree, new_node);

            ROS_WARN("RRT path found, freezing this path");
            tree_marker_ = gen_tree_marker(tree, 1, 0, 0);
            node_marker_ = gen_node_marker(tree, 0, 0, 1);

            // publish tree

            path_found_ = true;
            return;
        }
        
    }

}

/**
 * @brief randomly generates a point in map frame
 * @return std::array<double, 2> 
 */
std::array<double ,2> RRT::sample() {

    // sampling is biased towards the positive x direction
    std::uniform_real_distribution<>::param_type x_param(0, lookahead_distance_ * 1.5);
    std::uniform_real_distribution<>::param_type y_param(-lookahead_distance_ * 1.5, lookahead_distance_ * 1.5);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::Pose sample_point;
    sample_point.position.x = x_dist(gen);
    sample_point.position.y = y_dist(gen);
    sample_point.position.z = 0;
    sample_point.orientation.x = 0;
    sample_point.orientation.y = 0;
    sample_point.orientation.z = 0;
    sample_point.orientation.w = 1;

    // sampled point is transformed to map frame
    tf2::doTransform(sample_point, sample_point, tf_laser_to_map_);
    return {sample_point.position.x, sample_point.position.y};
}

/**
 * @brief returns the index of node in the tree vector that is nearest to the node that is passed in the function 
 * @param tree - tree vector
 * @param sampled_point - sampled point in free space
 * @return int - index in tree vector
 */
int RRT::nearest(std::vector<Node> &tree, std::array<double,2> &sampled_point) {

    int nearest_node = 0;
    double nearest_node_distance = std::numeric_limits<double>::max();

    for(int i=0; i < tree.size(); i++){
        const auto dist = pow( pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2) ,0.5);
        if(dist < nearest_node_distance){
            nearest_node = i;
            nearest_node_distance = dist;
        }
    }
    return nearest_node;
}

/**
 * @brief Expands the tree towards the sample point (within a max distance)
 * @param nearest_node - nearest node to the sampled point
 * @param sampled_point - sampled point in free space
 * @return Node - new node
 */
Node RRT::Steer(Node &nearest_node, const int nearest_node_index, std::array<double, 2> &sampled_point) {
    
    const double x_diff = sampled_point[0] - nearest_node.x;
    const double y_diff = sampled_point[1] - nearest_node.y;
    const double distance = pow(pow(x_diff, 2) + pow(y_diff, 2), 0.5);

    Node new_node{};

    if(distance < max_expansion_distance_){
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else{
        const double theta = atan2(y_diff, x_diff);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance_;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance_;
    }
    new_node.parent_index = nearest_node_index;

    return new_node;
}

/**
 * @brief Checks if the lastest added node is the goal node or not
 * @param latest_added_node 
 * @param goal_x - x coordinate of goal in map 
 * @param goal_y - y coordinate of goal in map
 * @return true/false
 */
bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    
    const double distance = sqrt(pow(latest_added_node.x - goal_x,2)+pow(latest_added_node.y - goal_y,2));
    return distance < goal_tolerance_;
}

/**
 * @brief Backtracks the tree from goal node to root node 
 * @param tree - rrt tree vector
 * @param latest_added_node - goal node
 * @return std::vector<std::array<double ,2>> - rrt solution path 
 */
std::vector<std::array<double ,2>>
catmull_rom(const std::vector<std::array<double,2>>& pts, int samples)
{
    if (pts.size() < 4 || samples < 2) return pts;

    auto P = [&](int i)->std::array<double,2> {
        if (i < 0) return pts.front();
        if (i >= static_cast<int>(pts.size())) return pts.back();
        return pts[i];
    };

    std::vector<std::array<double,2>> out;
    out.reserve((pts.size()-1) * samples);

    for (int i = 0; i < static_cast<int>(pts.size()) - 1; ++i) {
        auto p0 = P(i-1), p1 = P(i), p2 = P(i+1), p3 = P(i+2);
        for (int s = 0; s < samples; ++s) {
            double t  = static_cast<double>(s) / static_cast<double>(samples);
            double t2 = t * t;
            double t3 = t2 * t;

            // Catmull–Rom basis (0.5 tension)
            double b0 = -0.5*t3 +     t2 - 0.5*t;
            double b1 =  1.5*t3 - 2.5*t2 + 1.0;
            double b2 = -1.5*t3 + 2.0*t2 + 0.5*t;
            double b3 =  0.5*t3 - 0.5*t2;

            double x = b0*p0[0] + b1*p1[0] + b2*p2[0] + b3*p3[0];
            double y = b0*p0[1] + b1*p1[1] + b2*p2[1] + b3*p3[1];
            out.push_back({x, y});
        }
    }
    out.push_back(pts.back());
    return out;
}
std::vector<std::array<double,2>>
RRT::smooth_path(std::vector<std::array<double,2>> path, int discrete)
{
    if (path.size() < 4) return path;
    // 구간 수 × 구간당 샘플 수 = 대략 discrete가 되도록 조정
    int segs = static_cast<int>(path.size()) - 1;
    int samples = std::max(2, discrete / std::max(1, segs));
    return catmull_rom(path, samples);
}


/**
 * @brief Get the best local trackpoint from the local path for pure pursuit following
 * This local point is atleast one local lookahead distance away from the robot 
 * @return std::pair<geometry_msgs::Pose, double> 
 */
std::pair<geometry_msgs::Pose, double> RRT::get_best_local_trackpoint(const std::array<double, 2> &current_pose){

    try{
        tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }

    geometry_msgs::Point goal_way_point;
    geometry_msgs::Pose closest_point;
    double closest_distance_to_current_pose =std::numeric_limits<double>::max();
    double closest_distance = std::numeric_limits<double>::max();

    for(const auto& itr : local_path_){
        
	goal_way_point.x = itr[0];
	goal_way_point.y = itr[1];

	tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser_);
        if(goal_way_point.x < 0){continue;}

        double dist = sqrt(pow(itr[0] - current_pose[0], 2)
                               + pow(itr[1] - current_pose[1], 2));

        double diff_distance = std::abs(local_lookahead_distance_ - dist);
        if(diff_distance < closest_distance)
        {
            closest_distance_to_current_pose = dist;
            closest_distance = diff_distance;

            closest_point.position.x = itr[0];
            closest_point.position.y = itr[1];
            closest_point.position.z = 0;
            // closest_point.orientation.x = 0;
            // closest_point.orientation.y = 0;
            // closest_point.orientation.z = 0;
            // closest_point.orientation.w = 1;
        }
    }    
    return {closest_point, closest_distance_to_current_pose};
}

// RRT* methods

/**
 * @brief Calcutes the cumulative cost of node from the root of the tree
 * @param tree - tree vector
 * @param node - current node 
 * @return double - calculated cost
 */
double RRT::cost(std::vector<Node> &tree, Node &node)
{
    return tree[node.parent_index].cost + line_cost (tree[node.parent_index], node);
}

/**
 * @brief Connects the current node to the least cost parent and finds a child for the current node if feasible
 * @param neigh_vec - vector of nodes that lie in the search radius of the current node
 * @param tree - tree vector
 * @param node - current node 
 */
void RRT::rewire(std::vector<int> neighbor, std::vector<Node> &tree, Node &new_node)
{
    int min_cost_idx = -1;

    if(neighbor.size() == 0)
    {
        ROS_ERROR("No Neighbours");
        return;
    }

    // FINDING MIN COST PARENT FOR NEW NODE
    for(int i=0 ; i< neighbor.size() ; i++)
    {
        if( is_edge_collided( new_node, tree.at( neighbor.at(i) ) ) ) 
        {
            continue;
        }

        double cost_i = line_cost( tree.at( neighbor.at(i) ) , new_node);
        
        if(tree.at(neighbor.at(i)).cost + cost_i < new_node.cost)
        {   
            new_node.cost = tree.at(neighbor.at(i)).cost + cost_i;
            min_cost_idx = neighbor.at(i);
            new_node.parent_index = min_cost_idx;
        }
    }

    // REWIRING
    // FIND IF NEW NODE CAN BE A LOW COST PARENT TO ANOTHER NODE ALREADY IN THE TREE 
    for(int i=0 ; i< neighbor.size() ; i++)
    {   
        // skipping over current parent of new node
        // this will avoid a cycle
        if( is_edge_collided( new_node, tree.at( neighbor.at(i))) || i == min_cost_idx ) 
        {
            continue;
        }

        if(tree.at(neighbor.at(i)).cost > new_node.cost + line_cost( new_node , tree.at(neighbor.at(i))) )
        {
            // new node index is the current tree size
            tree.at(neighbor.at(i)).parent_index = tree.size();
        }
    }
}
std::vector<std::array<double, 2>>
RRT::find_path(std::vector<Node>& tree, Node& latest_added_node)
{
    // goal(마지막 노드)에서 root까지 부모 인덱스로 역추적
    std::vector<std::array<double, 2>> path;
    path.push_back({latest_added_node.x, latest_added_node.y});  // goal 포함

    int parent_idx = latest_added_node.parent_index;
    while (parent_idx >= 0 && parent_idx < static_cast<int>(tree.size())) {
        const Node& p = tree[parent_idx];
        path.push_back({p.x, p.y});
        parent_idx = p.parent_index;   // root의 parent_index는 -1
    }

    // 추적은 goal->root 순서로 쌓였으니, 주행용으로 root->goal 순서로 뒤집기
    std::reverse(path.begin(), path.end());
    return path;
}


/**
 * @brief Returns a vector of nodes that lie within a search radius of the current node
 * @param tree - tree vector
 * @param node - current node
 * @return std::vector<int> - vector of indices of nodes inside the radius
 */
std::vector<int> RRT::near(const std::vector<Node> &tree, Node &node) {

    std::vector<int> neighborhood;
        
    for(int i=0; i < tree.size() ; i++)
    {
        const double dist = sqrt(pow(tree[i].x - node.x, 2)
                               + pow(tree[i].y - node.y, 2));

        if(dist <= search_radius_)
        {
            neighborhood.push_back(i);
        }
    }

    // indices are wrt tree
    return neighborhood;
}

/**
 * @brief Calculates the cost between two nodes
 * @param n1 - node 1
 * @param n2 - nopde 2
 * @return double - calculated cost
 */
double RRT::line_cost(Node &n1, Node &n2) 
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

/**
 * @brief This function inflates all the occupied indices in the map and returns all the indices of the cells
 * that have been inflated  
 * @return vector of indices 
 */
std::vector<int> RRT::get_inflated_row_major_indices(const double x_map, const double y_map)
{
    std::vector<int> out;
    int xi = static_cast<int>((x_map - map_origin_x)/map_resolution_);
    int yi = static_cast<int>((y_map - map_origin_y)/map_resolution_);
    int rows = static_cast<int>(input_map_.info.height);

    for (int i = xi - inflation_radius_; i <= xi + inflation_radius_; ++i) {
        if (i < 0 || i >= static_cast<int>(map_cols_)) continue;
        for (int j = yi - inflation_radius_; j <= yi + inflation_radius_; ++j) {
            if (j < 0 || j >= rows) continue;
            out.emplace_back(j*map_cols_ + i);
        }
    }
    return out;
}

/**
 * @brief This function returns the x,y cordinates of a point the the global path
 * This point is atleast one lookahed distance away from the robot
 * @return std::array<double, 2> 
 */
std::array<double, 2> RRT::get_best_global_trackpoint(const std::array<double, 2>& current_pose){

    try{
        tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }

    int best_trackpoint_index = -1;
    double best_trackpoint_distance = std::numeric_limits<double>::max();

    for(int i=0; i<global_path_.size(); ++i){
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = global_path_[i][0];
        goal_way_point.position.y = global_path_[i][1];
        goal_way_point.position.z = 0.0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser_);

        // making sure waypoint is in front of the car
        if(goal_way_point.position.x < 0) continue;

        double distance = std::abs(lookahead_distance_ -
                sqrt(pow(goal_way_point.position.x, 2)+ pow(goal_way_point.position.y, 2)));

        if(distance < best_trackpoint_distance){
            const auto row_major_index = get_row_major_index(global_path_[i][0], global_path_[i][1]);
            if (input_map_.data[row_major_index] == 100) continue;
            best_trackpoint_distance = distance;
            best_trackpoint_index = i;
        }
    }
    
    return global_path_[best_trackpoint_index];
}

/**
 * @brief Checks if a randomly generated node is free or occupied
 * @param x,y coordinates of node in map frame
 */
bool RRT::is_collided(const double x_map, const double y_map)
{
    int idx = get_row_major_index(x_map, y_map);
    if (idx < 0) return true;                 // 바깥은 충돌로 처리(안전)
    return input_map_.data[idx] == 100;
}

/**
 * @brief Checks if a edge connecting two nodes is free or occupied
 * @param x,y coordinates of both nodes in map frame
 */
bool RRT::is_edge_collided(const Node &nearest_node, const Node &new_node){
    
    double x_increment = (new_node.x - nearest_node.x)/collision_checking_points_;
    double y_increment = (new_node.y - nearest_node.y)/collision_checking_points_;

    double current_x = nearest_node.x;
    double current_y = nearest_node.y;
    for(int i=0; i<collision_checking_points_; i++)
    {
        current_x += x_increment;
        current_y += y_increment;
        if(is_collided(current_x, current_y))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief This function publishes the drive messages to the robot
 * linear velocity is inversely proportional to the steering angle
 */
void RRT::execute_control(double steering_angle)
{
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "base_link";

    drive_msg.drive.steering_angle = steering_angle;
    if(steering_angle > 0.1)
    {
        if (steering_angle > 0.2)
        {
            drive_msg.drive.speed = low_speed_;
            if (steering_angle > 0.4)
            {
                drive_msg.drive.steering_angle = 0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else if(steering_angle < -0.1)
    {
        if (steering_angle < -0.2)
        {
            drive_msg.drive.speed = low_speed_;
            if (steering_angle < -0.4)
            {
                drive_msg.drive.steering_angle = -0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else
    {
        drive_msg.drive.speed = high_speed_;
    }

    drive_pub_.publish(drive_msg);
}



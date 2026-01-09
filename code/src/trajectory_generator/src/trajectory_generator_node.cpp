#include <algorithm>
#include <cctype>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <limits>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Useful customized headers
#include "Astar_searcher.h"
#include "backward.hpp"
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;
std::ofstream dataFiles;

TrajectoryGeneratorWaypoint *_trajGene = new TrajectoryGeneratorWaypoint();
Astarpath *_astar_path_finder = new Astarpath();

// Set the obstacle map
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;
int demox = 0;
double _los_step;
double _time_vel_scale = 1.25;
double _time_acc_scale = 1.15;
double _time_global_scale = 1.0;
double _path_simplify_scale = 1.0;
int _shortcut_passes = 1;
bool _use_los_shortest = false;
bool _use_raw_occ = false;
int _occ_inflate_level = 2;
double _straight_time_scale = 1.0;
double _turn_time_scale = 1.0;
double _turn_angle_deg = 90.0;
ros::Time start_record_time;
ros::Time finish_record_time;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _traj_vis_pub, _traj_before_vis_pub,_traj_pub, _path_vis_pub,_astar_path_vis_pub;

// for planning
Vector3d odom_pt, odom_vel, start_pt, target_pt, start_vel;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;

// for replanning
enum STATE {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ,
  EMER_STOP
};
STATE exec_state;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
ros::Timer _safety_timer_;
void execCallback(const ros::TimerEvent &e);
bool cracked;
int time_Count;
int target_Count;
ros::Timer _record_timer_;

static double clampDouble(double value, double low, double high) {
  return std::min(high, std::max(low, value));
}

static int clampInt(int value, int low, int high) {
  return std::min(high, std::max(low, value));
}

static std::string trimCopy(const std::string &text) {
  size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return std::string();
  size_t last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

static bool parseKeyValue(const std::string &line, std::string *key,
                          std::string *value) {
  size_t eq = line.find('=');
  if (eq == std::string::npos)
    return false;
  *key = trimCopy(line.substr(0, eq));
  *value = trimCopy(line.substr(eq + 1));
  return !key->empty();
}

static void applyOverrides(const std::string &path, std::string &search_mode,
                           double &heuristic_weight, double &ara_init_weight,
                           double &ara_step, int &ara_max_iter,
                           double &z_penalty, double &theta_rewire_z_penalty) {
  std::ifstream fin(path.c_str());
  if (!fin.is_open())
    return;

  std::string line;
  while (std::getline(fin, line)) {
    size_t hash = line.find('#');
    if (hash != std::string::npos)
      line = line.substr(0, hash);
    line = trimCopy(line);
    if (line.empty())
      continue;

    std::string key;
    std::string value;
    if (!parseKeyValue(line, &key, &value))
      continue;
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    try {
      if (key == "search_mode") {
        search_mode = value;
      } else if (key == "heuristic_weight") {
        heuristic_weight = std::stod(value);
      } else if (key == "ara_init_weight") {
        ara_init_weight = std::stod(value);
      } else if (key == "ara_step") {
        ara_step = std::stod(value);
      } else if (key == "ara_max_iter") {
        ara_max_iter = std::stoi(value);
      } else if (key == "z_penalty") {
        z_penalty = std::stod(value);
      } else if (key == "theta_rewire_z_penalty") {
        theta_rewire_z_penalty = std::stod(value);
      } else if (key == "time_vel_scale") {
        _time_vel_scale = clampDouble(std::stod(value), 0.5, 2.0);
      } else if (key == "time_acc_scale") {
        _time_acc_scale = clampDouble(std::stod(value), 0.5, 2.0);
      } else if (key == "time_global_scale") {
        _time_global_scale = clampDouble(std::stod(value), 0.5, 1.5);
      } else if (key == "path_simplify_scale") {
        _path_simplify_scale = clampDouble(std::stod(value), 0.5, 2.0);
      } else if (key == "shortcut_passes") {
        _shortcut_passes = clampInt(std::stoi(value), 0, 20);
      } else if (key == "use_los_shortest") {
        _use_los_shortest = (std::stoi(value) != 0);
      } else if (key == "use_raw_occ") {
        _use_raw_occ = (std::stoi(value) != 0);
      } else if (key == "occ_inflate_level") {
        _occ_inflate_level = clampInt(std::stoi(value), 0, 2);
      } else if (key == "straight_time_scale") {
        _straight_time_scale = clampDouble(std::stod(value), 0.5, 1.5);
      } else if (key == "turn_time_scale") {
        _turn_time_scale = clampDouble(std::stod(value), 0.5, 1.5);
      } else if (key == "turn_angle_deg") {
        _turn_angle_deg = clampDouble(std::stod(value), 5.0, 175.0);
      } else if (key == "los_step") {
        _los_step = clampDouble(std::stod(value), 0.0, 1.0);
      } else if (key == "dev_order") {
        _dev_order = clampInt(std::stoi(value), 3, 7);
      } else if (key == "min_order") {
        _min_order = clampInt(std::stoi(value), 3, 7);
      } else if (key == "path_resolution") {
        _path_resolution = clampDouble(std::stod(value), 0.05, 0.5);
      } else if (key == "replan_thresh") {
        replan_thresh = clampDouble(std::stod(value), 0.5, 5.0);
      } else if (key == "no_replan_thresh") {
        no_replan_thresh = clampDouble(std::stod(value), 0.5, 5.0);
      } else if (key == "vis_traj_width") {
        _vis_traj_width = clampDouble(std::stod(value), 0.03, 0.2);
      }
    } catch (...) {
      continue;
    }
  }
}

// declare
void changeState(STATE new_state, string pos_call);
void printState();
void visTrajectory(MatrixXd polyCoeff, VectorXd time);
void visTrajectory_before(MatrixXd polyCoeff, VectorXd time);
void visPath(MatrixXd nodes);
void visPathA(MatrixXd nodes);
void trajOptimization(Eigen::MatrixXd path);
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom);
void rcvWaypointsCallback(const nav_msgs::Path &wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);
void trajPublish(MatrixXd polyCoeff, VectorXd time);
bool trajGeneration();
VectorXd timeAllocation(MatrixXd Path);
Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);
bool lineOfSight(const Vector3d &a, const Vector3d &b);
vector<Vector3d> shortcutPath(const vector<Vector3d> &path);
vector<Vector3d> shortestPathByLOS(const vector<Vector3d> &path);

// change the state to the new state
void changeState(STATE new_state, string pos_call) {
  string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ","EMER_STOP"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

void printState() {
  string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ","EMER_STOP"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  odom_pt(0) = odom->pose.pose.position.x;
  odom_pt(1) = odom->pose.pose.position.y;
  odom_pt(2) = odom->pose.pose.position.z;

  odom_vel(0) = odom->twist.twist.linear.x;
  odom_vel(1) = odom->twist.twist.linear.y;
  odom_vel(2) = odom->twist.twist.linear.z;

  has_odom = true;
}

// Control the State changes
void execCallback(const ros::TimerEvent &e) {
  static int num = 0;
  num++;
  if (num == 100) {
    printState();
    if (!has_odom)
      cout << "no odom." << endl;
    if (!has_target)
      cout << "wait for goal." << endl;
    num = 0;
  }

  switch (exec_state) {
  case INIT: {
    if (!has_odom)
      return;
    if (!has_target)
      return;
    changeState(WAIT_TARGET, "STATE");//odom
    break;
  }

  case WAIT_TARGET: {
    if (!has_target)
      return;
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case GEN_NEW_TRAJ: {
    bool success = trajGeneration();//路径点搜索程序从这里进入
    if (success)
     { changeState(EXEC_TRAJ, "STATE");}
    else
      {changeState(WAIT_TARGET, "STATE");
      has_target=false;}
    break;
  }

  case EXEC_TRAJ: {
    time_Count++;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_replan = ros::Duration(1, 0).toSec();
    t_cur = min(time_duration, t_cur);


    if (t_cur > time_duration - 1e-2) {
      has_target = false;
      changeState(WAIT_TARGET, "STATE");
      return;
    } else if ((target_pt - odom_pt).norm() < no_replan_thresh) {
      return;
    } else if ((start_pt - odom_pt).norm() < replan_thresh) {
      return;
    } else if (t_cur < t_replan) {
      return;
    } else {
      changeState(REPLAN_TRAJ, "STATE");
    }
    break;
  }
  case REPLAN_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_delta = ros::Duration(0, 50).toSec();
    t_cur = t_delta + t_cur;
    start_pt = getPos(t_cur);
    start_vel = getVel(t_cur);
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case EMER_STOP: {
    // ROS_ERROR("YOUR DRONE HAS CRACKED!!!!");
    
  }
  }
}

void rcvWaypointsCallBack(const nav_msgs::Path &wp) {
  if (wp.poses[0].pose.position.z < 0.0)
    return;
  if(demox == 0){
    // target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y,
    //   wp.poses[0].pose.position.z;
    target_pt << 12.0, -4.0, wp.poses[0].pose.position.z;
  }else{
    target_pt << 0, 0, 5;
  }
  ROS_INFO("[node] receive the planning target");
  start_pt = odom_pt;
  start_vel = odom_vel;
  has_target = true;

  if (exec_state == WAIT_TARGET)
    changeState(GEN_NEW_TRAJ, "STATE");
  else if (exec_state == EXEC_TRAJ)
    changeState(REPLAN_TRAJ, "STATE");
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;

 _astar_path_finder->resetOccupy();//新的一帧点云到来，需将占据容器清零；

  pcl::fromROSMsg(pointcloud_map, cloud);
  
  if ((int)cloud.points.size() == 0)
    return;

  pcl::PointXYZ pt;
  for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
    pt = cloud.points[idx];
    // set obstalces into grid map for path planning
    _astar_path_finder->set_barrier(pt.x, pt.y, pt.z);
  }
}


bool trajGeneration() {

  bool astar_success =_astar_path_finder->AstarSearch(start_pt, target_pt);

  if(!astar_success){
     _astar_path_finder->resetUsedGrids();
    return false;
    }
      auto grid_path = _astar_path_finder->getPath();
      for (int pass = 0; pass < _shortcut_passes; ++pass) {
        grid_path = shortcutPath(grid_path);
      }
  // Reset map for next call
      _astar_path_finder->resetUsedGrids();
  MatrixXd path(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path.row(k) = grid_path[k].transpose();
  }
  visPathA(path);

  double simplify_resolution = _path_resolution * _path_simplify_scale;
  grid_path = _astar_path_finder->pathSimplify(grid_path, simplify_resolution);
  if (_use_los_shortest) {
    grid_path = shortestPathByLOS(grid_path);
  }
  for (int pass = 0; pass < _shortcut_passes; ++pass) {
    grid_path = shortcutPath(grid_path);
  }
  // grid_path = _astar_path_finder->getPath();
  path=MatrixXd::Zero(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path.row(k) = grid_path[k].transpose();
  }

  trajOptimization(path);
  time_duration = _polyTime.sum();

  // Publish the trajectory
  trajPublish(_polyCoeff, _polyTime);
  time_traj_start = ros::Time::now();
  if (_polyCoeff.rows() > 0)
    return true;
  else
    return false;
}

void issafe(const ros::TimerEvent &e){
    Eigen::Vector3d now_pos;
    now_pos = odom_pt;
    Eigen::Vector3i odom_index = _astar_path_finder->c2i(now_pos);
    if(_astar_path_finder->is_occupy_raw(odom_index) && !cracked){
      ROS_WARN("now place is in obstacle, the drone has cracked!!!");
      cracked = true;
      if(!dataFiles.is_open()){
        dataFiles.open("/home/user/MRPC-2025-homework/code/src/quadrotor_simulator/so3_control/src/issafe.txt", std::ios::out|std::ios::trunc);
      }
      dataFiles << 1;
      dataFiles.flush();
    }
}


void trajOptimization(Eigen::MatrixXd path) {
  // if( !has_odom ) return;
  // 
  MatrixXd vel = MatrixXd::Zero(2, 3);
  MatrixXd acc = MatrixXd::Zero(2, 3);//首末速度和加速度
  vel.row(0) = start_vel.transpose();
  vel.row(1) <<0,0,0;
  acc.row(0) <<0,0,0;
  acc.row(1) <<0,0,0;
  
  _polyTime = timeAllocation(path);
  
  _polyCoeff =
      _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
  // check if the trajectory is safe, if not, do reoptimize
  visTrajectory_before(_polyCoeff, _polyTime);
  int unsafe_segment=-1;
  
  unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
  MatrixXd repath;
 bool regen_flag=false;
  while (unsafe_segment != -1) {
    
    //通过一个循环插入一个新的路径点
    regen_flag=true;
     int count = 0;
    repath=MatrixXd::Zero(path.rows()+1,path.cols());
     while(count<=unsafe_segment)
    {
        repath.row(count)=path.row(count);
        count++;
    }
    repath.row(count)=(path.row(count)+path.row(count-1))/2;
    cout<<"Add a middle point to avoid collision!"<<endl;
    while(count<path.rows())
    {
        repath.row(count+1)=path.row(count);
        count++;
    }
    path=repath;
    // cout<<"new path is "<<path<<endl;
    _polyTime = timeAllocation(path);
    // cout<<"new time is "<<_polyTime<<endl;
    _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
    // cout<<"new poly coeff is "<<_polyCoeff<<endl;
    unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
  }
  if(regen_flag)
  cout<<"regeneration of safe path success!"<<endl;
  // visulize path and trajectory
  visPath(path);
  visTrajectory(_polyCoeff, _polyTime);
}

void trajPublish(MatrixXd polyCoeff, VectorXd time) {
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to "
             "publish.");
    return;
  }

  unsigned int poly_number;

  static int count =
      1; // The first trajectory_id must be greater than 0. zxzxzxzx

  quadrotor_msgs::PolynomialTrajectory traj_msg;

  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = std::string("world");
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

  traj_msg.num_order = 2 * _dev_order - 1; // the order of polynomial
  traj_msg.num_segment = time.size();

  Vector3d initialVel, finalVel;
  initialVel = _trajGene->getVelPoly(_polyCoeff, 0, 0);
  finalVel = _trajGene->getVelPoly(_polyCoeff, traj_msg.num_segment - 1,
                                   _polyTime(traj_msg.num_segment - 1));
  traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));
  traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));

  poly_number = traj_msg.num_order + 1;

  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    for (unsigned int j = 0; j < poly_number; j++) {
      traj_msg.coef_x.push_back(polyCoeff(i, j) * pow(time(i), j));
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) *
                                pow(time(i), j));
      traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j) *
                                pow(time(i), j));
    }
    traj_msg.time.push_back(time(i));
    traj_msg.order.push_back(traj_msg.num_order);
  }
  traj_msg.mag_coeff = 1;

  count++;
  ROS_WARN("[traj..gen...node] traj_msg publish");
  _traj_pub.publish(traj_msg);
}

//To_Do
VectorXd timeAllocation(MatrixXd Path) {
  VectorXd time(Path.rows() - 1);
  Eigen::Vector3d piece;
  double dist;
  const double vel = std::max(0.1, _Vel * _time_vel_scale);
  const double acc = std::max(0.1, _Acc * _time_acc_scale);
  const double t = vel / acc;
  const double d = 0.5 * acc * t * t;
  const double global_scale = std::max(0.1, _time_global_scale);
  //??????????????????????????????
 for(int i=0;i<int(time.size());i++)
 {
   piece = (Path.row(i + 1) - Path.row(i)).transpose();
   dist = piece.norm();
       if (dist < d + d)
    {
        time(i)= 2.0 * sqrt(dist / acc);
    }
    else
    {
        time(i) =2.0 * t + (dist - 2.0 * d) / vel;
    }
    if (i > 0) {
      Eigen::Vector3d prev = (Path.row(i) - Path.row(i - 1)).transpose();
      if (prev.norm() > 1e-6 && dist > 1e-6) {
        double cosang = prev.dot(piece) / (prev.norm() * dist);
        cosang = std::max(-1.0, std::min(1.0, cosang));
        double angle = acos(cosang);
        double angle_deg = angle * 180.0 / M_PI;
        double scale = (angle_deg < _turn_angle_deg) ? _straight_time_scale
                                                     : _turn_time_scale;
        time(i) *= scale;
      }
    }
    time(i) = std::max(time(i), 0.01);
  }
  return global_scale * time;
}

bool lineOfSight(const Vector3d &a, const Vector3d &b) {
  Vector3d diff = b - a;
  double dist = diff.norm();
  if (dist < 1e-6)
    return true;
  double step = _los_step;
  if (step <= 0.0)
    step = std::max(0.05, _resolution * 0.5);
  Vector3d dir = diff / dist;
  int steps = static_cast<int>(dist / step);
  Vector3d p = a;
  for (int i = 0; i <= steps; ++i) {
    if (_astar_path_finder->is_occupy(_astar_path_finder->c2i(p)))
      return false;
    p += dir * step;
  }
  return true;
}

vector<Vector3d> shortestPathByLOS(const vector<Vector3d> &path) {
  if (path.size() <= 2)
    return path;

  const size_t n = path.size();
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> parent(n, -1);
  dist[0] = 0.0;

  for (size_t i = 0; i < n; ++i) {
    if (dist[i] == std::numeric_limits<double>::infinity())
      continue;
    for (size_t j = i + 1; j < n; ++j) {
      if (!lineOfSight(path[i], path[j]))
        continue;
      const double d = dist[i] + (path[j] - path[i]).norm();
      if (d < dist[j]) {
        dist[j] = d;
        parent[j] = static_cast<int>(i);
      }
    }
  }

  if (parent[n - 1] < 0)
    return path;

  std::vector<Vector3d> result;
  for (int cur = static_cast<int>(n - 1); cur >= 0; cur = parent[cur]) {
    result.push_back(path[cur]);
    if (cur == 0)
      break;
  }
  if (result.back() != path[0])
    return path;
  std::reverse(result.begin(), result.end());
  return result;
}

vector<Vector3d> shortcutPath(const vector<Vector3d> &path) {
  if (path.size() <= 2)
    return path;
  vector<Vector3d> result;
  size_t i = 0;
  result.push_back(path.front());
  while (i < path.size() - 1) {
    size_t next = path.size() - 1;
    for (; next > i + 1; --next) {
      if (lineOfSight(path[i], path[next]))
        break;
    }
    result.push_back(path[next]);
    i = next;
  }
  return result;
}

void visTrajectory(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  _traj_vis.header.frame_id = "world";

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 0.0;
  _traj_vis.color.g = 0.5;
  _traj_vis.color.b = 1.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_vis_pub.publish(_traj_vis);
}

void visTrajectory_before(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  _traj_vis.header.frame_id = "world";

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 1.0;
  _traj_vis.color.g = 0.5;
  _traj_vis.color.b = 0.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_before_vis_pub.publish(_traj_vis);
}

void visPath(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_vis_pub.publish(points);
}

void visPathA(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r =1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _astar_path_vis_pub.publish(points);
}

Vector3d getPos(double t_cur) {
  double time = 0;
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t);
        return pos;
      }
    }
  }
  return pos;
}

Vector3d getVel(double t_cur) {
  double time = 0;
  Vector3d Vel = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _trajGene->getVelPoly(_polyCoeff, i, t);
        return Vel;
      }
    }
  }
  return Vel;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_node");
  ros::NodeHandle nh("~");
  std::string search_mode;
  double heuristic_weight = 1.0;
  double ara_init_weight = 1.6;
  double ara_step = 0.2;
  int ara_max_iter = 3;
  double z_penalty = 0.2;
  double theta_rewire_z_penalty = 0.0;

  nh.param("planning/vel", _Vel, 1.0);
  nh.param("planning/acc", _Acc, 1.0);
  nh.param("planning/dev_order", _dev_order, 3);
  nh.param("planning/min_order", _min_order, 3);
  nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
  nh.param("map/resolution", _resolution, 0.2);
  nh.param("map/x_size", _x_size, 50.0);
  nh.param("map/y_size", _y_size, 50.0);
  nh.param("map/z_size", _z_size, 5.0);
  nh.param("path/resolution", _path_resolution, 0.05);
  nh.param("replanning/thresh_replan", replan_thresh, -1.0);
  nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0);
  nh.param("planning/demox", demox, 0);
  nh.param("planning/search_mode", search_mode, std::string("theta_star"));
  nh.param("planning/heuristic_weight", heuristic_weight, 1.0);
  nh.param("planning/ara_init_weight", ara_init_weight, 1.6);
  nh.param("planning/ara_step", ara_step, 0.2);
  nh.param("planning/ara_max_iter", ara_max_iter, 3);
  nh.param("planning/los_step", _los_step, 0.0);
  nh.param("planning/z_penalty", z_penalty, 0.2);
  nh.param("planning/theta_rewire_z_penalty", theta_rewire_z_penalty, 0.0);

  applyOverrides("/home/user/MRPC-2025-homework/solutions/variant_override.txt",
                 search_mode, heuristic_weight, ara_init_weight, ara_step,
                 ara_max_iter, z_penalty, theta_rewire_z_penalty);

 

  _poly_num1D = 2 * _dev_order;
  cracked = false;
  exec_state = INIT;
  time_Count=0;
  target_Count = 0;
  
  std::string issafe_path = "/home/user/MRPC-2025-homework/code/src/quadrotor_simulator/so3_control/src/issafe.txt";
  std::ofstream clear_file(issafe_path, std::ios::out | std::ios::trunc);
  clear_file.close();

  _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);
  _safety_timer_ = nh.createTimer(ros::Duration(0.05), issafe);

  _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);
  _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack);
  _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);

  _traj_pub =
      nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
  _traj_before_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory_before", 1);
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);
  _astar_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path_astar", 1);
  // set the obstacle map
  _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
  _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;
  _inv_resolution = 1.0 / _resolution;
  _max_x_id = (int)(_x_size * _inv_resolution);
  _max_y_id = (int)(_y_size * _inv_resolution);
  _max_z_id = (int)(_z_size * _inv_resolution);

  _astar_path_finder = new Astarpath();
  std::string mode = search_mode;
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  bool use_theta = false;
  bool use_lazy = false;
  bool use_ara = false;
  if (mode == "theta_star" || mode == "theta") {
    use_theta = true;
  } else if (mode == "lazy_theta_star" || mode == "lazy_theta") {
    use_theta = true;
    use_lazy = true;
  } else if (mode == "ara_star" || mode == "ara") {
    use_ara = true;
  } else if (mode == "weighted_astar" || mode == "weighted") {
    // weighted A*
  } else if (mode != "astar") {
    ROS_WARN("[planner] unknown search_mode '%s', fallback to astar",
             mode.c_str());
  }
  if (use_ara) {
    use_theta = false;
    use_lazy = false;
  }
  _astar_path_finder->setThetaOptions(use_theta, use_lazy);
  _astar_path_finder->setHeuristicWeight(heuristic_weight);
  _astar_path_finder->setAraParams(use_ara, ara_init_weight, ara_step,
                                   ara_max_iter);
  _astar_path_finder->setLineOfSightStep(_los_step);
  _astar_path_finder->setZPenalty(z_penalty);
  _astar_path_finder->setThetaRewireZPenalty(theta_rewire_z_penalty);
  _astar_path_finder->setUseRawOcc(_use_raw_occ);
  _astar_path_finder->setOccInflateLevel(_occ_inflate_level);
  ROS_INFO("[planner] mode=%s theta=%d lazy=%d ara=%d weight=%.3f z_pen=%.2f theta_z=%.2f",
           mode.c_str(), use_theta, use_lazy, use_ara, heuristic_weight,
           z_penalty, theta_rewire_z_penalty);
  _astar_path_finder->begin_grid_map(_resolution, _map_lower, _map_upper,
                                  _max_x_id, _max_y_id, _max_z_id);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

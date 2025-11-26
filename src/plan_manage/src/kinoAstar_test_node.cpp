

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <traj_utils/planning_visualization.h>

enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ };
bool trigger_, have_target_, have_odom_;

Eigen::Vector3d start_pt_, end_pt_, end_vel_;                              // target state
Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
Eigen::Quaterniond odom_orient_;

FSM_EXEC_STATE exec_state_;

double max_vel_, max_acc_;


SDFMap::Ptr sdf_map_;
EDTEnvironment::Ptr edt_environment_;
unique_ptr<KinodynamicAstar> kino_path_finder_;
PlanningVisualization::Ptr visualization_;

void waypointCallback(const nav_msgs::PathConstPtr& msg) ;
void odometryCallback(const nav_msgs::OdometryConstPtr& msg) ;
void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) ;
void printFSMExecState() ;
void execFSMCallback(const ros::TimerEvent& e) ;
bool callKinodynamicReplan() ; // 定义一个函数，用于调用动力学重规划，返回一个布尔值表示是否成功
bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) ;
void getPath();

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar_test_node");
  ros::NodeHandle nh("~");

    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(sdf_map_);

    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();

    visualization_.reset(new PlanningVisualization(nh));
 
    ros::Timer exec_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_;
    //ros::Publisher replan_pub_, new_pub_, bspline_pub_;
    
    exec_timer_   = nh.createTimer(ros::Duration(0.01), execFSMCallback);
    waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, waypointCallback);
    odom_sub_     = nh.subscribe("/odom_world", 1, odometryCallback);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}


void waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;
  cout << "New target: " << msg->poses[0].pose.position.x << ", "
       << msg->poses[0].pose.position.y << ", " << msg->poses[0].pose.position.z << endl;

  end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;


  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}



void execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
        return;
    }

    case REPLAN_TRAJ: {

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

bool callKinodynamicReplan()  { // 定义一个函数，用于调用动力学重规划，返回一个布尔值表示是否成功
  bool plan_success = // 定义一个布尔变量，存储规划是否成功的结果
      kinodynamicReplan(start_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), end_pt_, end_vel_); // 调用规划管理器的动力学重规划方法，传入起点和终点的状态（位置、速度、加速度）

  if (plan_success) { // 如果路径规划成功

    // 获取轨迹点并可视化
    std::vector<Eigen::Vector3d> path_points = kino_path_finder_->getKinoTraj(0.05); // 0.05是时间步长
    
    // 使用PlanningVisualization类显示轨迹线
    visualization_->drawGeometricPath(kino_path_finder_-> getKinoTraj(0.05), 0.1, Eigen::Vector4d(0, 1, 0, 1), 200);

    ROS_INFO("KINO A* path visualized");
    return true; // 函数返回true，表示规划和后续处理全部成功

  } else { // 如果路径规划失败
    cout << "generate new traj fail." << endl; // 在控制台打印失败信息
    return false; // 函数返回false，表示规划失败
  }
}

bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }


  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }
  return true;
}
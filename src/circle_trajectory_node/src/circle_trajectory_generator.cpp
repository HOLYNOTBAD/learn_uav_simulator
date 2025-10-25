#include <ros/ros.h> // ROS主头文件
#include <quadrotor_msgs/PositionCommand.h> // 自定义无人机位置指令消息
#include <geometry_msgs/Point.h> // 点类型消息
#include <visualization_msgs/Marker.h> // rviz可视化Marker消息
#include <nav_msgs/Path.h> // 路径消息（本例未用到）
#include <math.h> // 数学函数库

// 圆形轨迹生成器类
class CircleTrajectoryGenerator {
private:
    ros::NodeHandle nh_; // ROS节点句柄
    ros::Publisher cmd_pub_; // 位置指令发布器
    ros::Publisher traj_vis_pub_; // 轨迹可视化发布器
    ros::Timer timer_; // 定时器
    
    // 圆形轨迹参数
    double center_x_, center_y_, center_z_; // 圆心坐标
    double radius_, angular_velocity_; // 半径和角速度
    ros::Time start_time_; // 起始时间
    
public:
    CircleTrajectoryGenerator() : nh_("~") {
        // 读取参数，若无则用默认值
        nh_.param("circle/center_x", center_x_, 0.0); // 圆心x
        nh_.param("circle/center_y", center_y_, 0.0); // 圆心y
        nh_.param("circle/center_z", center_z_, 2.0); // 圆心z
        nh_.param("circle/radius", radius_, 3.0); // 半径
        nh_.param("circle/angular_velocity", angular_velocity_, 0.5); // 角速度
        
        // 发布器初始化
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10); // 位置指令话题
        traj_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis", 10); // 轨迹可视化话题
        
        // 定时器，每20ms调用一次回调
        timer_ = nh_.createTimer(ros::Duration(0.02), &CircleTrajectoryGenerator::timerCallback, this);
        start_time_ = ros::Time::now(); // 记录起始时间
        
        publishTrajectoryVisualization(); // 发布一次完整圆形轨迹用于rviz显示
        ROS_INFO("Circle trajectory generator started"); // 控制台提示
    }
    
    // 定时器回调函数，实时发布期望位置、速度、加速度
    void timerCallback(const ros::TimerEvent& event) {
        double t = (ros::Time::now() - start_time_).toSec(); // 距起始的时间
        double angle = angular_velocity_ * t; // 当前角度
        
        // 计算期望位置
        double x = center_x_ + radius_ * cos(angle); // x坐标
        double y = center_y_ + radius_ * sin(angle); // y坐标
        double z = center_z_; // z坐标
        
        // 计算期望速度
        double vx = -radius_ * angular_velocity_ * sin(angle); // x方向速度
        double vy = radius_ * angular_velocity_ * cos(angle);  // y方向速度
        double vz = 0.0; // z方向速度
        
        // 计算期望加速度
        double ax = -radius_ * angular_velocity_ * angular_velocity_ * cos(angle); // x方向加速度
        double ay = -radius_ * angular_velocity_ * angular_velocity_ * sin(angle); // y方向加速度
        double az = 0.0; // z方向加速度
        
        // 创建位置指令消息
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now(); // 时间戳
        cmd.header.frame_id = "world"; // 坐标系
        
        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;
        
        cmd.velocity.x = vx;
        cmd.velocity.y = vy;
        cmd.velocity.z = vz;
        
        cmd.acceleration.x = ax;
        cmd.acceleration.y = ay;
        cmd.acceleration.z = az;
        
        cmd.yaw = atan2(vy, vx);  // 期望偏航角，切线方向
        cmd.yaw_dot = 0.0; // 偏航角速度
        
        // 控制增益设置
        cmd.kx[0] = 5.7; cmd.kx[1] = 5.7; cmd.kx[2] = 6.2; // 位置增益
        cmd.kv[0] = 3.4; cmd.kv[1] = 3.4; cmd.kv[2] = 4.0; // 速度增益
        
        cmd_pub_.publish(cmd); // 发布指令
    }
    
    // 发布圆形轨迹用于rviz可视化
    void publishTrajectoryVisualization() {
        visualization_msgs::Marker marker; // Marker消息
        marker.header.frame_id = "world"; // 坐标系
        marker.header.stamp = ros::Time::now(); // 时间戳
        marker.ns = "circle_trajectory"; // 命名空间
        marker.id = 0; // id
        marker.type = visualization_msgs::Marker::LINE_STRIP; // 线条类型
        marker.action = visualization_msgs::Marker::ADD; // 添加
        
        marker.scale.x = 0.1; // 线宽
        marker.color.r = 1.0; marker.color.a = 0.8; // 红色，透明度0.8
        
        // 生成圆形轨迹点
        for (int i = 0; i <= 100; ++i) {
            double angle = 2.0 * M_PI * i / 100.0; // 均匀采样
            geometry_msgs::Point p;
            p.x = center_x_ + radius_ * cos(angle); // x坐标
            p.y = center_y_ + radius_ * sin(angle); // y坐标
            p.z = center_z_; // z坐标
            marker.points.push_back(p); // 加入点集
        }
        
        traj_vis_pub_.publish(marker); // 发布Marker
    }
};

// 主函数，初始化ROS节点并进入循环
int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_trajectory_node"); // 初始化节点
    CircleTrajectoryGenerator generator; // 创建轨迹生成器对象
    ros::spin(); // 循环等待回调
    return 0;
}
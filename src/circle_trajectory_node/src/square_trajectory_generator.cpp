#include <ros/ros.h> // ROS主头文件
#include <quadrotor_msgs/PositionCommand.h> // 自定义无人机位置指令消息
#include <geometry_msgs/Point.h> // 点类型消息
#include <visualization_msgs/Marker.h> // rviz可视化Marker消息
#include <nav_msgs/Path.h> // 路径消息（本例未用到）
#include <math.h> // 数学函数库

// 圆形轨迹生成器类
class SquareTrajectoryGenerator {
private:
    ros::NodeHandle nh_; // ROS节点句柄
    ros::Publisher cmd_pub_; // 位置指令发布器
    ros::Publisher traj_vis_pub_; // 轨迹可视化发布器
    ros::Timer timer_; // 定时器
    
    // 矩形轨迹参数
    double center_x_, center_y_, center_z_; // 矩形中心坐标
    double length_ ; // 矩形边长
    double velocity_; // 线速度
    ros::Time start_time_; // 起始时间
    
public:
    //这是构造函数，初始化参数和发布器
    SquareTrajectoryGenerator() : nh_("~") {
        // 读取参数，若无则用默认值
        nh_.param("square/center_x", center_x_, 0.0); // 矩形中心x
        nh_.param("square/center_y", center_y_, 0.0); // 矩形中心y
        nh_.param("square/center_z", center_z_, 2.0); // 矩形中心z
        nh_.param("square/length", length_, 3.0); // 边长
        nh_.param("square/angular_velocity", velocity_, 0.5); // 线速度
        
        // 发布器初始化
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10); // 位置指令话题
        traj_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis", 10); // 轨迹可视化话题
        
        // 定时器，每20ms调用一次回调
        timer_ = nh_.createTimer(ros::Duration(0.02), &SquareTrajectoryGenerator::timerCallback, this);
        start_time_ = ros::Time::now(); // 记录起始时间
        
        publishTrajectoryVisualization(); // 发布一次完整矩形轨迹用于rviz显示
        ROS_INFO("Square trajectory generator started"); // 控制台提示
    }
    
    // 定时器回调函数，实时发布期望位置、速度、加速度
    void timerCallback(const ros::TimerEvent& event) {
        double t = (ros::Time::now() - start_time_).toSec(); // 距起始的时间
        double path = velocity_ * t; // 当前角度
        
        // 计算期望位置
        double time_for_one_side = length_ / velocity_; // 走完一边所需时间
        double total_time = 4 * time_for_one_side; // 完成一圈所需时间
        double mod_time = fmod(t, total_time); // 当前周期内的时间
        double side_index = floor(mod_time / time_for_one_side); // 当前所在边的索引
        double side_time = fmod(mod_time, time_for_one_side); // 当前边上的时间

        double x, y, z;
        if (side_index == 0) { // 第一边，正x方向
            x = center_x_ - length_/2 + velocity_ * side_time;
            y = center_y_ - length_/2;
        } else if (side_index == 1) { // 第二边，正y方向
            x = center_x_ + length_/2;
            y = center_y_ - length_/2 + velocity_ * side_time;
        } else if (side_index == 2) { // 第三边，负x
            x = center_x_ + length_/2 - velocity_ * side_time;
            y = center_y_ + length_/2;
        } else { // 第四边，负y方向
            x = center_x_ - length_/2;
            y = center_y_ + length_/2 - velocity_ * side_time;
        }
        z = center_z_; // 高度不变
        
        // 计算期望速度
        double vx, vy, vz;
        if (side_index == 0) { // 第一边，正x方向
            vx = velocity_;
            vy = 0.0;
        } else if (side_index == 1) { // 第二边，正y方向
            vx = 0.0;
            vy = velocity_;
        } else if (side_index == 2) { // 第三边，负x
            vx = -velocity_;
            vy = 0.0;
        } else { // 第四边，负y方向
            vx = 0.0;
            vy = -velocity_;
        }
        vz = 0.0; // z方向速度
        
        // 计算期望加速度
        double ax = 0.0;
        double ay = 0.0;
        double az = 0.0; // z方向加速度
        
        // 创建位置指令消息
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now(); // 时间戳
        cmd.header.frame_id = "world"; // 坐标系
        
        //这个仿真器需要输入的位置、速度、加速度和偏航角
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
        // visualization_msgs::Marker marker; // Marker消息
        // marker.header.frame_id = "world"; // 坐标系
        // marker.header.stamp = ros::Time::now(); // 时间戳
        // marker.ns = "square_trajectory"; // 命名空间
        // marker.id = 0; // id
        // marker.type = visualization_msgs::Marker::LINE_STRIP; // 线条类型
        // marker.action = visualization_msgs::Marker::ADD; // 添加
        
        // marker.scale.x = 0.1; // 线宽
        // marker.color.r = 1.0; marker.color.a = 0.8; // 红色，透明度0.8
        
        // // 生成圆形轨迹点
        // for (int i = 0; i <= 100; ++i) {
        //     double angle = 2.0 * M_PI * i / 100.0; // 均匀采样
        //     geometry_msgs::Point p;
        //     p.x = center_x_ + radius_ * cos(angle); // x坐标
        //     p.y = center_y_ + radius_ * sin(angle); // y坐标
        //     p.z = center_z_; // z坐标
        //     marker.points.push_back(p); // 加入点集
        // }
        
        // traj_vis_pub_.publish(marker); // 发布Marker
    }
};

// 主函数，初始化ROS节点并进入循环
int main(int argc, char** argv) {
    ros::init(argc, argv, "square_trajectory_node"); // 初始化节点
    SquareTrajectoryGenerator generator; // 创建轨迹生成器对象
    ros::spin(); // 循环等待回调
    return 0;
}
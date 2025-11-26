#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// 全局变量用于存储接收到的odom
nav_msgs::Odometry received_odom;
bool odom_received = false;

// 回调函数处理订阅的消息
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    received_odom = *msg;
    odom_received = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_odom");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // 定义Odometry消息
    nav_msgs::Odometry odom;

    // 创建发布者
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_world", 10);
    
    // 创建订阅者
    ros::Subscriber odom_sub = nh.subscribe("/new_odom_world", 10, odomCallback);
    // 从参数服务器获取参数
    double rate;
    nh.param("fake_odom/rate", rate, 100.0); // 默认100Hz
    
    // Odometry消息的各个字段参数
    double pos_x, pos_y, pos_z;
    double orient_x, orient_y, orient_z, orient_w;
    double vel_x, vel_y, vel_z;
    double ang_vel_x, ang_vel_y, ang_vel_z;
    
    // 从参数服务器获取位置、方向和速度参数
    nh.param("fake_odom/position/x", pos_x, 0.0);
    nh.param("fake_odom/position/y", pos_y, 0.0);
    nh.param("fake_odom/position/z", pos_z, 0.0);
    
    nh.param("fake_odom/orientation/x", orient_x, 0.0);
    nh.param("fake_odom/orientation/y", orient_y, 0.0);
    nh.param("fake_odom/orientation/z", orient_z, 0.0);
    nh.param("fake_odom/orientation/w", orient_w, 1.0);
    
    nh.param("fake_odom/linear_velocity/x", vel_x, 0.0);
    nh.param("fake_odom/linear_velocity/y", vel_y, 0.0);
    nh.param("fake_odom/linear_velocity/z", vel_z, 0.0);
    
    nh.param("fake_odom/angular_velocity/x", ang_vel_x, 0.0);
    nh.param("fake_odom/angular_velocity/y", ang_vel_y, 0.0);
    nh.param("fake_odom/angular_velocity/z", ang_vel_z, 0.0);
    
    ros::Rate loop_rate(rate);
    
    
    // 设置帧ID
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    
    // 设置位置
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = pos_z;
    
    // 设置方向
    odom.pose.pose.orientation.x = orient_x;
    odom.pose.pose.orientation.y = orient_y;
    odom.pose.pose.orientation.z = orient_z;
    odom.pose.pose.orientation.w = orient_w;
    
    // 设置线速度
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.linear.z = vel_z;
    
    // 设置角速度
    odom.twist.twist.angular.x = ang_vel_x;
    odom.twist.twist.angular.y = ang_vel_y;
    odom.twist.twist.angular.z = ang_vel_z;

    while (ros::ok()) {
        // 如果收到了新的odom消息，使用它；否则使用参数初始化的odom
        if (odom_received) {
            odom = received_odom;
            odom_received = false;  // 复位标志
        }

        // 更新时间戳
        odom.header.stamp = ros::Time::now();
        // 发布消息
        odom_pub.publish(odom);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
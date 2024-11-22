#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "eskf.hpp"
#include <fstream>  // 用于文件操作


class ESKFNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Publisher state_pub_;
    ESKF eskf_;

public:
    ESKFNode() {
        // Initialize subscribers and publishers
        imu_sub_ = nh_.subscribe("/iris_0/mavros/imu/data", 20, &ESKFNode::imuCallback, this);
        gps_position_sub_ = nh_.subscribe("/iris_0/mavros/global_position/raw/fix", 20, &ESKFNode::gpsCallback, this);
        // gps_vel_sub_.subscribe(nh_, "/iris_0/mavros/global_position/raw/gps_vel", 5);


        // 发布状态估计结果的话题
        state_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/eskf/state_estimation", 1);
        // cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/iris_0/mavros/setpoint_attitude/cmd_vel", 100);
        // Configure ESKF options
        ESKF::Options options;
        options.bias_gyro_var = 1e-3;
        options.bias_acc_var = 1e-3;
        options.gyro_var = 1e-3;
        options.acc_var = 1e-3;
        options.imu_dt = 0.02;

        // Initialize ESKF
        ESKF::Vec3 init_bg(1e-3, 1e-3, 1e-3); // Zero bias for gyroscope
        ESKF::Vec3 init_ba(1e-3, 1e-3, 1e-3); // Zero bias for accelerometer
        eskf_.SetInitialConditions(options, init_bg, init_ba);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Pass IMU data to ESKF
        if (eskf_.Predict(*msg)) {
            publishState(msg->header.stamp);
        }
    }

    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
        // Pass pressure data to ESKF
        eskf_.PressureObserver(*msg);
    }

    void gpsCallback(const sensor_msgs::NavSatFix& gps_position) {
        eskf_.GNSSObserve(gps_position);
        ROS_INFO("GNSS观测成功!");
    }

    void publishState(const ros::Time& stamp) {
         // 打开文本文件（以追加模式）
        std::ofstream outFile;
        outFile.open("/home/dcbin/catkin_ws/src/eskf_pkg/output.txt", std::ios::app);

        geometry_msgs::PoseStamped state_msg;
        state_msg.header.stamp = stamp;
        state_msg.header.frame_id = "map";

        ESKF::Vec3 p = eskf_.getp();
        ESKF::Vec3 v = eskf_.getV();
        ESKF::SO3 R = eskf_.getR();

        state_msg.pose.position.x = p(0);
        state_msg.pose.position.y = p(1);
        state_msg.pose.position.z = p(2);
        Eigen::Quaterniond q(R);
        state_msg.pose.orientation.x = q.x();
        state_msg.pose.orientation.y = q.y();
        state_msg.pose.orientation.z = q.z();
        state_msg.pose.orientation.w = q.w();
        if (outFile.is_open()) {
            // 将消息内容写入文件
            outFile << state_msg.pose.position.z << std::endl;
        } else {
            ROS_ERROR("Failed to open file for writing.");
        }
        outFile.close();
        // Publish the state
        state_pub_.publish(state_msg);
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "eskf_node");
    ESKFNode node;
    ros::spin();
    return 0;
}

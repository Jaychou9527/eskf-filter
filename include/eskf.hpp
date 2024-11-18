#ifndef ESKF_HPP
#define ESKF_HPP

#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <sensor_msgs/FluidPressure.h>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>


/**
 * 使用前需要准备好以下参数：
 * 1.过程噪声协方差阵Q的初始化配置：Options
 * 2.IMU的陀螺仪和加速度计零偏bg、ba
 * 3.传感器的观测噪声协方差矩阵V
 */
class ESKF
{
public:
    /**
     * 误差状态变量的运动过程噪声\omega的协方差矩阵Q的参数配置
     */
    class Options {
    public:
        double bias_gyro_var;
        double bias_acc_var;
        double gyro_var;
        double acc_var;
        double imu_dt;

        Options() 
            : bias_gyro_var(1e-4), 
            bias_acc_var(1e-4), 
            gyro_var(1e-4), 
            acc_var(1e-4), 
            imu_dt(0.02) 
        {}
    };


    using Vec18 = Eigen::Matrix<double, 18, 1>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Vec3 = Eigen::Matrix<double, 3, 1>;
    using Mat18 = Eigen::Matrix<double, 18, 18>;
    using Mat18x6 = Eigen::Matrix<double, 18, 6>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;
    using Mat3 = Eigen::Matrix<double, 3, 3>;
    using CovMotionNoise = Eigen::Matrix<double, 18, 18>;
    using SO3 = Eigen::Matrix<double, 3, 3>;
    using Cov = Eigen::Matrix<double, 18, 18>;
    using Imu = sensor_msgs::Imu;
    using HMat = Eigen::Matrix<double, 6, 18>;
    using SE3 = Sophus::SE3d;

 
    // 名义状态变量
    Vec3 p_ = Vec3::Zero(); //位置的名义状态变量
    Vec3 v_ = Vec3::Zero(); //速度的名义状态变量
    SO3 R_ = SO3::Identity(); //旋转矩阵的名义状态变量
    Vec3 bg_ = Vec3::Zero(); //角速度零偏的名义状态变量
    Vec3 ba_ = Vec3::Zero(); //角速度零偏的名义状态变量
    Vec3 g_{0, 0, -9.8}; //重力加速度的名义状态变量

    // 18维的误差状态变量，从这里也可以看出使用误差状态变量的优势：状态变量都是向量,
    // 不存在矩阵.由6个三维列向量组成，依次为：dp、dv、dtheta、dbg、dba、dg
    Vec18 dx_ = Vec18::Zero();

    // 误差状态变量的运动学噪声w对应的协方差矩阵
    Mat18 Q = Mat18::Zero();

    // 误差状态量的协方差矩阵
    Mat18 P_ = Mat18::Identity();

    // 观测方程噪声协方差矩阵V
    Mat6 V_ = Mat6::Identity() * 1e-4;

    // 初始气压计读数
    double first_pressure = 0;

    // 记录当前时间戳
    double current_time = 0;

    // 是否为首个气压计数据
    bool first_pressure_tag = true;
    // 是否为首个IMU数据
    bool first_imu_tag = true;

    // 误差状态变量的过程噪声协方差矩阵Q的参数配置
    Options options_;

public:
    ESKF(const Options& options = Options())
        :options_(options){}

    bool Predict(const Imu& imu_data);
    bool PressureObserver(const sensor_msgs::FluidPressure& pressure);
    bool ObserveSE3(const SE3& SE3_,
                        const HMat& H = HMat::Zero(),
                        const Mat6& V = Mat6::Zero());
    bool SetInitialConditions(Options options, const Vec3& init_bg, const Vec3& init_ba,
                                const Vec3& gravity = Vec3(0, 0, -9.8));
    Vec3 getp()const{return p_;}
    Vec3 getV()const{return v_;}
    SO3 getR()const{return R_;}

private:
    void SetNoise(const Options& options);
    void UpdateP_();
    void UpdateAndReset();
};

/**
 * 初始化函数，使用eskf前必须先使用此函数进行初始化
 * @param options 矩阵Q的配置参数
 * @param init_bg 陀螺仪零偏
 * @param init_ba 加速度计零偏
 * @param gravity 重力加速度
 */
bool ESKF::SetInitialConditions(Options options, const Vec3& init_bg, const Vec3& init_ba,
                                const Vec3& gravity)
{
    SetNoise(options);
    options_ = options;
    bg_ = init_bg;
    ba_ = init_ba;
    g_ = gravity;
    P_ = Mat18::Identity() * 1e-4;
    return true;
}

/**
 * 利用IMU数据预测状态量，包括对名义状态量、误差状态量、误差状态量协方差矩阵的预测。
 * @param imu_data sensor_msgs::Imu
 */
bool ESKF::Predict(const Imu& imu_data)
{
    double time_stamp = imu_data.header.stamp.toSec();
    assert(time_stamp >= this->current_time);

    // 判断是否为首个IMU数据并记录时间戳
    if(first_imu_tag)
    {
        this->current_time = time_stamp;
        first_imu_tag = false;
        return true;
    }
    double dt = time_stamp - current_time;
    if(dt < 0 || dt > 5 * options_.imu_dt)
    {
        this->current_time = time_stamp;
        return false;
    }

    // 获取IMU数据并保存为向量
    Vec3 imu_acc; //IMU加速度
    imu_acc << imu_data.linear_acceleration.x, imu_data.linear_acceleration.y,
    imu_data.linear_acceleration.z;
    Vec3 imu_angular_vel;
    imu_angular_vel << imu_data.angular_velocity.x, imu_data.angular_velocity.y,
    imu_data.angular_velocity.z;

    // 预测名义状态变量
    p_ = p_ + v_ * dt + 0.5*(R_*(imu_acc - bg_) + g_) * dt * dt;
    v_ = v_ + (R_*(imu_acc - ba_) + g_) * dt;
    R_ = R_ * (Sophus::SO3d::exp((imu_angular_vel - bg_) * dt)).matrix();

    // 构建雅可比矩阵F
    Mat18 F = Mat18::Identity();
    F.block<3, 3>(0, 3) = Mat3::Identity() * dt;
    F.block<3, 3>(3, 6) = -R_ * Sophus::SO3d::hat(imu_acc - ba_) * dt;
    F.block<3, 3>(3, 12) = -R_ * dt;
    F.block<3, 3>(3, 15) = Mat3::Identity() * dt;
    F.block<3, 3>(6, 6) = Sophus::SO3d::exp(-(imu_angular_vel - bg_) * dt).matrix();
    F.block<3, 3>(6, 9) = -Mat3::Identity() * dt;

    // 预测误差状态
    //dx_ = F * dx_;
    // 预测误差状态的协方差矩阵
    P_ = F * P_.eval() * F.transpose() + Q;
    // 更新时间
    this->current_time = time_stamp;
    return true;
}

/**
 * 使用气压计数据对高度进行观测,同时对高度进行修正
 * @param pressure sensor_msgs::FluidPressure；气压计测量数据
 */
bool ESKF::PressureObserver(const sensor_msgs::FluidPressure& pressure)
{
    assert(pressure.header.stamp.toSec() >= current_time);
    double current_pressure = pressure.fluid_pressure;
    SE3 SE3;
    if(this->first_pressure_tag)
    {
        this->first_pressure = current_pressure;
        this->first_pressure_tag = false;
    }
    if((first_pressure != 0))
    {
        // 高度估算公式
        SE3.translation() = Vec3(0, 0, 288.15 / 0.0065 
                    * (pow((first_pressure / current_pressure),
                    287.05 * 0.0065 / 9.80665) - 1));
    }
    HMat H = HMat::Zero();
    H(2, 2) = 1;
    this->ObserveSE3(SE3, H, V_);
    current_time = pressure.header.stamp.toSec();
    return true;
}

/**
 * 使用卡尔曼滤波更新误差状态变量和协方差矩阵P,观测函数应该提供:\n
 * 观测数据(以4X4的SE(3)矩阵表示观测数据)、观测矩阵、观测噪声的方差给这个方法。
 * 这里只考虑了对姿态和位置的观测，没有对速度的观测
 * @param SE3_ 传感器观测结果的SE(3)矩阵，若没有对某个状态变量进行观测，则对应位置设置为0
 * @param H 观测矩阵，用h(x)对误差状态量dx求偏导数得到
 * @param V 传感器的观测噪声协方差矩阵
 */
bool ESKF::ObserveSE3(const SE3& SE3_, const HMat& H, const Mat6& V)
{
    // dimension:18x6
    Mat18x6 K = P_ * H.transpose() * (H * P_ * H.transpose() + V_).inverse();
    Vec6 innov = Vec6::Zero();
    innov.block<3, 1>(0, 0) = SE3_.translation() - p_;
    Sophus::SO3d SO3_R (R_.transpose() * SE3_.rotationMatrix());
    innov.block<3, 1>(3, 0) = SO3_R.log();
    dx_ = dx_ + K * innov; //因为dx_每一次都会被置零，所以高博的代码中没有加上dx_
    P_ = (Mat18::Identity() - K * H) * P_;
    UpdateAndReset();
    return true;
}

/**
 * 1.更新滤波器最终的预测结果，实际上就是把预测的名义状态变量与更新后的误差状态变量相加；
 * 2.重置误差状态变量与协方差矩阵
 */
void ESKF::UpdateAndReset()
{
    p_ = p_ + dx_.block<3, 1>(0, 0);
    v_ = v_ + dx_.block<3, 1>(3, 0);
    R_ = R_ * (Sophus::SO3d::exp(dx_.block<3, 1>(6, 0))).matrix();
    // UpdateP_(); 加入P_的修正导致数值发散
    dx_.setZero();
}

void ESKF::UpdateP_()
{
    Mat18 J = Mat18::Identity();
    J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity()
                        -0.5 * Sophus::SO3d::hat(dx_.block<3, 1>(6, 0));
    P_ = J * P_  * J.transpose();
}

/**
 * 设置误差状态量的运动过程噪声Q矩阵
 */
void ESKF::SetNoise(const Options& options)
{
    double e_v = options.acc_var;  //加速度噪声方差要静置估计
    double e_theta = options.gyro_var; //陀螺仪噪声方差需要静置估计
    double e_g = options.bias_gyro_var; //陀螺仪零偏需要静置估计
    double e_a = options.bias_acc_var; //加速度计零偏需要静置估计
    Q.diagonal() << 0, 0, 0, e_v, e_v, e_v, e_theta, e_theta,
                    e_theta, e_g, e_g, e_g, e_a, e_a, e_a;
}

#endif
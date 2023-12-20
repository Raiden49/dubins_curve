/**
 * @file dubins_curve.hpp
 * @author Yongmeng He
 * @brief 
 * @version 0.1
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef DUBINS_CURVE_HPP_
#define DUBINS_CURVE_HPP_

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include "dubins_curve/dubins_tool.hpp"

namespace dubins_curve
{

class DubinsCurve {
    public:
        /**
         * @brief Construct a new Dubins Curve object
         * 
         * @param n 
         */
        DubinsCurve(ros::NodeHandle& n) : n_(n) {
            init_pose_pub_ = n_.advertise
                    <visualization_msgs::Marker>("init_pose_marker", 1);
            goal_pose_pub_ = n_.advertise
                    <visualization_msgs::Marker>("goal_pose_marker", 1);
        }
        ~DubinsCurve() = default;
        /**
         * @brief 创建Dubins曲线中的LSL曲线
         * 
         * @return curve_type LSL路径点
         */
        curve_type LSLCurve();
        /**
         * @brief 创建Dubins曲线中的LSR曲线
         * 
         * @return curve_type LSR路径点
         */
        curve_type LSRCurve();
        /**
         * @brief 创建Dubins曲线中的RSL曲线
         * 
         * @return curve_type RSL路径点
         */
        curve_type RSLCurve();
        /**
         * @brief 创建Dubins曲线中的RSR曲线
         * 
         * @return curve_type RSR路径点
         */
        curve_type RSRCurve();
        /**
         * @brief Get the Dubins Curve Length object
         * 
         * @param center1 第一个圆心座标点
         * @param center2 第二个圆心座标点
         * @param tangent_pos1 第一个圆上的切点
         * @param tangent_pos2 第二个圆上的切点
         * @return double 整条路径的长度
         */
        double GetDubinsCurveLength(
                const point_type& center1, const point_type& center2, 
                const point_type& tangent_pos1, const point_type& tangent_pos2);
        /**
         * @brief 求解Dubins曲线的主要函数
         * 
         * @param index 最优路径的index，0-3分别为LSL，LSR，RSR，RSL
         * @return std::array<curve_type, 4> 四条路径的所有路径点
         */
        std::array<curve_type, 4> DubinsCurveSolver(int& index);
        /**
         * @brief Dubins曲线的可视化，其中，最优路线不透明度最高，即最为明显
         * 
         * @param color 曲线的颜色
         * @param path_pub 曲线路径
         * @param dir 曲线方向，对应LSL，LSR，RSR，RSL
         * @param alpha 不透明度
         * @return visualization_msgs::Marker 发布的Marker点
         */
        visualization_msgs::Marker DrawDubinsCurve(
                const std::array<double, 3>& color, const curve_type& path_pub, 
                const std::array<std::string, 3>& dir, const double& alpha);
        /**
         * @brief 主运行函数
         * 
         */
        void DubinsCurveProcess();

    private:
        /**
         * @brief 获取初始位置点的回调函数，由rviz人为设置，并绘制在RVIZ上
         * 
         * @param msg 初始点的信息
         */
        void InitPoseCallback(
                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        /**
         * @brief 获取目标位置点的回调函数，由rviz人为设置，并绘制在RVIZ上
         * 
         * @param msg 目标点的信息
         */
        void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    private:
        double radius_ = 1;     // 最大转弯半径，根据机器人实际机动性设置
        point_type left_center1, left_center2;
        point_type right_center1, right_center2;
        point_type start_pos_, end_pos_;

    private:
        ros::NodeHandle& n_;
        ros::Subscriber init_pose_sub_, goal_pose_sub_;
        ros::Publisher init_pose_pub_, goal_pose_pub_;
        visualization_msgs::Marker init_pose_marker_, goal_pose_marker_;
        DubinsTool* dubins_tool_;
};
}


#endif // DUBINS_CURVE_HPP_
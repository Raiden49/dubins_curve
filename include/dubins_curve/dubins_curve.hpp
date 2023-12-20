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
        DubinsCurve(ros::NodeHandle& n) : n_(n) {
            init_pose_pub_ = n_.advertise
                    <visualization_msgs::Marker>("init_pose_marker", 1);
            goal_pose_pub_ = n_.advertise
                    <visualization_msgs::Marker>("goal_pose_marker", 1);
        }
        ~DubinsCurve() = default;
        curve_type LSLCurve();
        curve_type LSRCurve();
        curve_type RSLCurve();
        curve_type RSRCurve();
        double GetDubinsCurveLength(
                const point_type& center1, const point_type& center2, 
                const point_type& tangent_pos1, const point_type& tangent_pos2);
        std::array<curve_type, 4> DubinsCurveSolver(int& index);
        visualization_msgs::Marker DrawDubinsCurve(
                const std::array<double, 3>& color, const curve_type& path_pub, 
                const std::array<std::string, 3>& dir, const double& alpha);
        void DubinsCurveProcess();

    private:
        void InitPoseCallback(
                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
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
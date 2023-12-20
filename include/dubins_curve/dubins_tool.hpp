/**
 * @file dubins_tool.hpp
 * @author Yongmeng He
 * @brief 
 * @version 0.1
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef DUBINS_TOOL_HPP_
#define DUBINS_TOOL_HPP_

#include <ros/ros.h>
#include <array>
#include <vector>

namespace dubins_curve 
{
using point_type = std::pair<std::pair<double, double>, double>;
using curve_type = std::pair<std::array<point_type, 4>, double>;
class DubinsTool {
    public:
        /**
         * @brief Construct a new Dubins Tool object
         * 
         */
        DubinsTool() = default;
        ~DubinsTool() = default;
        /**
         * @brief 用于求解切点，或是左转右转的圆心，利用几何关系
         * 
         * @param input_pos 求切点时，输入的是对应圆心；求圆心则输入起点终点坐标
         * @param theta 计算得到的几何角度
         * @param radius1 求切点时的半径
         * @param radius2 求圆心时的半径
         * @param dir 主要用于求圆心时，左转右转圆心有区别
         * @param output_pos 计算出的点
         * @return true 
         * @return false 
         */
        bool CoordTransform(
                const point_type& input_pos, const double& theta, 
                const double& radius1, const double& radius2, 
                const std::string& dir, point_type& output_pos);
        /**
         * @brief 求取两个点的欧式距离
         * 
         * @param pos1 
         * @param pos2 
         * @return double 欧式距离
         */
        double GetDistance(const point_type& pos1, const point_type& pos2);
        /**
         * @brief 求取两个点的差值向量，由pos1指向pos2
         * 
         * @param pos1 
         * @param pos2 
         * @return std::vector<double> 
         */
        std::vector<double> GetDiffVec(
                const point_type& pos1, const point_type& pos2);
        /**
         * @brief 求圆上两点之间的角度，用来计算弧度长度，采用绝对值相加，这样不需要计算方向
         * 
         * @param diff_vec1 
         * @param diff_vec2 
         * @return double 角度值，标量值
         */
        double GetDelta(const std::vector<double>& diff_vec1, 
                const std::vector<double>& diff_vec2);
};
}

#endif // DUBINS_TOOL_HPP_
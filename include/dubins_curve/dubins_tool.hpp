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
        DubinsTool() = default;
        ~DubinsTool() = default;
        bool CoordTransform(
            const point_type& input_pos, const double& theta, 
            const double& radius1, const double& radius2, 
            const std::string& dir, point_type& output_pos);
        double GetDistance(const point_type& pos1, const point_type& pos2);
        std::vector<double> GetDiffVec(
                const point_type& pos1, const point_type& pos2);
        double GetDelta(const std::vector<double>& diff_vec1, 
                const std::vector<double>& diff_vec2);
};
}

#endif // DUBINS_TOOL_HPP_
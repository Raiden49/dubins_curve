#include "dubins_curve/dubins_tool.hpp"

namespace dubins_curve
{
bool DubinsTool::CoordTransform(
            const point_type& input_pos, const double& theta, 
            const double& radius1, const double& radius2, 
            const std::string& dir, point_type& output_pos) {
    
    if (dir == "left") {
        output_pos.first.first = input_pos.first.first + 
                radius1 * cos(theta) - radius2 * sin(theta);
        output_pos.first.second = input_pos.first.second + 
                radius1 * sin(theta) + radius2 * cos(theta);
        return true;
    }
    else if (dir == "right") {
        output_pos.first.first = input_pos.first.first + 
                radius1 * cos(theta) + radius2 * sin(theta);
        output_pos.first.second = input_pos.first.second + 
                radius1 * sin(theta) - radius2 * cos(theta);
        return true;
    }
    else {
        ROS_ERROR_STREAM("!!!Wrong dir:" << dir);
        return false;
    }
}

double DubinsTool::GetDistance(const point_type& pos1, const point_type& pos2) {
    return sqrt(pow(pos1.first.first - pos2.first.first, 2) 
            + pow(pos1.first.second - pos2.first.second, 2));
}

std::vector<double> DubinsTool::GetDiffVec(
        const point_type& pos1, const point_type& pos2) {

    std::vector<double> diff_vec;
    diff_vec.push_back(pos1.first.first - pos2.first.first);
    diff_vec.push_back(pos1.first.second - pos2.first.second);

    return diff_vec;
}

double DubinsTool::GetDelta(const std::vector<double>& diff_vec1, 
        const std::vector<double>& diff_vec2) {
    
    return atan2(abs(diff_vec2[1]), abs(diff_vec2[0])) + 
            atan2(abs(diff_vec1[1]), abs(diff_vec1[0]));
}
}
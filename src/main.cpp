#include "dubins_curve/dubins_curve.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "dubins_curve");
    ros::NodeHandle n;
    auto node = new dubins_curve::DubinsCurve(n);
    node->DubinsCurveProcess();

    return 0;
}
#include <math.h>

#include <pf/Constants.h>
#include <pf/MotionModel.h>

using namespace pf;

MotionModel::MotionModel() :     
    distribution_(0.0, 1.0), 
    generator_( (unsigned int)time(0)) {}

RobotState MotionModel::sampleNextState(const RobotState& state_1, const OdometryReading& odom_1, const OdometryReading& odom_2)
{
    // differentials in odometry frame
    double d_th_enc_1 = shortest_angular_distance(odom_1.theta, atan2(odom_2.y-odom_1.y, odom_2.x-odom_1.x));
    double d_s_enc = sqrt(pow((odom_2.y-odom_1.y), 2) + pow((odom_2.x-odom_1.x), 2));
    double d_th_enc_2 = shortest_angular_distance(d_th_enc_1 + odom_1.theta, odom_2.theta);

    // standard deviations for differentials' noise
    double std_1 = MotionModelParams::alpha_1*d_th_enc_1 + MotionModelParams::alpha_2*d_s_enc;
    double std_2 = MotionModelParams::alpha_3*d_s_enc + MotionModelParams::alpha_4*(d_th_enc_1 + d_th_enc_2);
    double std_3 = MotionModelParams::alpha_1*d_th_enc_2 + MotionModelParams::alpha_2*d_s_enc;
    
    // differentials in world frame
    double d_th_1 = d_th_enc_1 + distribution_(generator_)*std_1;
    double d_s = d_s_enc + distribution_(generator_)*std_2;
    double d_th_2 = d_th_enc_2 + distribution_(generator_)*std_3;

    // sample next state
    RobotState state_2;
    state_2.x(state_1.x() + d_s*cos(state_1.theta() + d_th_1));
    state_2.y(state_1.y() + d_s*sin(state_1.theta() + d_th_1));
    state_2.theta(normalize_angle(state_1.theta() + d_th_1 + d_th_2));

    return state_2;
}

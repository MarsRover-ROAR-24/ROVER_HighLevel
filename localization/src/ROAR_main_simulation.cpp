#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <chrono>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include "localization/buffer.h"

using namespace std;

Eigen::VectorXd z_measurement;
Eigen::VectorXd encoder_measurement;

const int n_state_dim = 9;  // x_state dimension
const float alpha = 0.3;
const float beta_ = 2.0;
const float kappa = 0.1;
ros::Time encoder_prev_time_stamp;
ros::Time imu_prev_time_stamp;
ros::Time gps_prev_time_stamp;
double dt = 0.0;
bool new_measurement_received = false;
bool intial_measurment = true;
double lat0 = 0.0;
double lon0 = 0.0;

// Initialize Sigma Points and UKF
MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa);
UKF ukf(sigma_points);

ros::Subscriber imu_sub;
ros::Subscriber encoder_sub;
ros::Subscriber gps_sub;

void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->velocity.size() != 6) return;

    if (encoder_prev_time_stamp.isZero()) 
    {
        encoder_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time encoder_current_time_stamp = msg->header.stamp;
    dt = (encoder_current_time_stamp - encoder_prev_time_stamp).toSec();

    for (int i = 0; i < 6; ++i) {
        encoder_measurement[i] = msg->velocity[i];
    }
    ukf.encoder_callback(encoder_measurement, dt);
    encoder_prev_time_stamp = encoder_current_time_stamp;
    // cout << "encoder dt: " << dt << endl;
    cout << "x_posterior: " << ukf.x_post.transpose() << endl;

}
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (intial_measurment == true)
    {
        lat0 = msg->latitude;
        lon0 = msg->longitude;
        intial_measurment = false;
    }
    if (encoder_prev_time_stamp.isZero()) 
    {
        encoder_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time gps_current_time_stamp = msg->header.stamp;
    dt = (gps_current_time_stamp - gps_prev_time_stamp).toSec();

    z_measurement[9] = msg->latitude;
    z_measurement[10] = msg->longitude;
    ukf.gps_callback(z_measurement, dt, lat0, lon0);
    gps_prev_time_stamp = gps_current_time_stamp;
    // cout << "gps dt: " << dt << endl;
    cout << "x_posterior: " << ukf.x_post.transpose() << endl;
}
void imuCallback(const localization::buffer::ConstPtr& msg)
{
    if (imu_prev_time_stamp.isZero()) 
    {
        imu_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time imu_current_time_stamp = msg->header.stamp;
    dt = (imu_current_time_stamp - imu_prev_time_stamp).toSec();

    for (int i = 0; i < 9; ++i) 
    {
        z_measurement[i] = msg->measurements[i];
    }
    ukf.imu_callback(z_measurement, dt);
    imu_prev_time_stamp = imu_current_time_stamp;
    cout << "x_posterior: " << ukf.x_post.transpose() << endl;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization");
    ros::NodeHandle nh;

    imu_sub = nh.subscribe("/imu_readings", 1000, imuCallback);
    encoder_sub = nh.subscribe("/joint_states", 1000, encoderCallback);
    gps_sub = nh.subscribe("/gps", 1000, gpsCallback);

    z_measurement = Eigen::VectorXd::Zero(11);
    encoder_measurement = Eigen::VectorXd::Zero(6);

    while (ros::ok())
    {
        // cout << "x_posterior: " << ukf.x_post.transpose() << endl;
        // cout << "P_posterior: " << ukf.P_post << endl;
        ros::spinOnce();
    }

	return 0;
}

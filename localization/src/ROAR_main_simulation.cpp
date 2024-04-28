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
    encoder_prev_time_stamp = encoder_current_time_stamp;

    for (int i = 0; i < 6; ++i) {
        encoder_measurement[i] = msg->velocity[i];
    }
    UKF::encoder_callback(encoder_measurement, dt);
}
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (intial_measurment == true)
    {
        lat0 = msg->measurements[9];
        lon0 = msg->measurements[10];
        intial_measurment = false;
    }
    if (encoder_prev_time_stamp.isZero()) 
    {
        encoder_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time gps_current_time_stamp = msg->header.stamp;
    dt = (gps_current_time_stamp - gps_prev_time_stamp).toSec();
    gps_prev_time_stamp = gps_current_time_stamp;

    z_measurement[9] = msg->latitude;
    z_measurement[10] = msg->longitude;
    UKF::gps_callback(z_measurement, dt, lat0, lon0);
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
    imu_prev_time_stamp = imu_current_time_stamp;

    for (int i = 0; i < 9; ++i) 
    {
        z_measurement[i] = msg->measurements[i];
    }
    UKF::imu_callback(z_measurement, dt);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization");
    ros::NodeHandle nh;

    z_measurement = Eigen::VectorXd::Zero(11);
    encoder_measurement = Eigen::VectorXd::Zero(6);

    imu_sub = nh.subscribe("/imu_readings", 1000, &SensorDataPublisher::imuCallback, this);
    encoder_sub = nh.subscribe("/joint_states", 1000, &SensorDataPublisher::encoderCallback, this);
    gps_sub = nh.subscribe("/gps", 1000, &SensorDataPublisher::gpsCallback, this);

    // Initialize Sigma Points and UKF
    MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa);
    UKF ukf(sigma_points);

    while (ros::ok())
    {
        cout << "x_posterior: " << ukf.x_posterior.transpose() << endl;
        cout << "P_posterior: " << ukf.P_posterior << endl;
        ros::spinOnce();
    }

	return 0;
}

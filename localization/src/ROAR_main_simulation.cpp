#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

using namespace std;
Eigen::VectorXd acc_measurement;
Eigen::VectorXd gyro_measurement;
Eigen::VectorXd mag_measurement ;
Eigen::VectorXd gps_measurement ;
Eigen::VectorXd z_measurement;
Eigen::VectorXd encoder_measurement;

const int n_state_dim = 9;  // x_state dimension
const float alpha = 0.3;
const float beta_ = 2.0;
const float kappa = 0.1;
const float dt = 0.01;

bool gps_started = true;
double lat0 = 0.0;
double lon0 = 0.0;

// Callback function to handle incoming IMU messages
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    acc_measurement.resize(3);
    gyro_measurement.resize(3);

    cout << "IMU Callback" << endl;
    // --- Store data into matrices ---
    // Accelerometer (m/s^2)
    acc_measurement << msg->linear_acceleration.x,
                    msg->linear_acceleration.y, 
                    msg->linear_acceleration.z;

    // Gyroscope (rad/s)
    gyro_measurement << msg->angular_velocity.x,
                    msg->angular_velocity.y, 
                    msg->angular_velocity.z;
}
// Call back function to handle incoming encoder messages
void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    encoder_measurement.resize(2);
    if (msg->velocity.size() != 6) return;
    cout << "Encoder Callback" << endl;
    double left_wheels = (msg->velocity[0]+ msg->velocity[1]+ msg->velocity[2]) / 3;
    double right_wheels = (msg->velocity[3] + msg->velocity[4] + msg->velocity[5]) / 3;
    
    encoder_measurement << right_wheels, left_wheels;
}

// Call back function to handle incoming gps messages
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gps_measurement.resize(2);
    cout << "GPS Callback" << endl;
    if (gps_started)
    {
        lat0 = msg->latitude;
        lon0 = msg->longitude;
        gps_started = false;
    }

    gps_measurement << msg->latitude,
                    msg->longitude;
}

void magCallback(const geometry_msgs::Vector3Stamped& msg)
{
    cout << "Magnetometer Callback" << endl;
    mag_measurement.resize(3);
    // Magnetometer (uT)
    mag_measurement << (msg.vector.x) * 1e-6,
                       (msg.vector.y) * 1e-6,
                       (msg.vector.z) * 1e-6;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1000, imuCallback);
    ros::Subscriber encoder_sub = nh.subscribe("/joint_states", 1000, encoderCallback);
    ros::Subscriber gps_sub = nh.subscribe("/gps", 1000, gpsCallback);
    ros::Subscriber mag_sub = nh.subscribe("/magnetometer", 1000, magCallback);

    encoder_measurement = Eigen::Vector2d::Zero(2);
    z_measurement = Eigen::VectorXd::Zero(11);
    acc_measurement = Eigen::Vector3d::Zero(3);
    gyro_measurement = Eigen::Vector3d::Zero(3);
    mag_measurement = Eigen::Vector3d::Zero(3);
    gps_measurement = Eigen::Vector2d::Zero(2);

	MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa);
	UKF ukf(sigma_points);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {   
        ukf.predict_states(encoder_measurement, dt);
        ukf.predict_measurement(dt,encoder_measurement,lat0, lon0);

        z_measurement << gyro_measurement, acc_measurement, mag_measurement, gps_measurement;
        ukf.update(z_measurement);
        
        // // --- Output to Serial ---
        // cout << "x_prior: " << ukf.x_prior << endl;
        // cout << "P_prior: " << ukf.P_prior << endl;

        cout << "x_posterior: " << endl << ukf.x_post << endl;
        // cout << "P_posterior: " << ukf.P_post << endl;

        // cout << "measurements" << z_measurement << endl;

        ros::spinOnce();
        
    }

	return 0;
}
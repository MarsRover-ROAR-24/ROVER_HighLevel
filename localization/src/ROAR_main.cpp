#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;
Eigen::Vector3d acc_measurement;
Eigen::Vector3d gyro_measurement;
Eigen::Vector3d mag_measurement;
Eigen::VectorXd encoder_measurement;
Eigen::VectorXd gps_measurement;
Eigen::VectorXd z_measurement;

const float mag_hardiron_offset_x = -31.71;
const float mag_hardiron_offset_y = 28.61;
const float mag_hardiron_offset_z = 33.985;

const float gyro_offset_x = 0.06285;
const float gyro_offset_y = -0.08785;
const float gyro_offset_z = -0.06815;

const int n_state_dim = 9;  // x_state dimension
const float alpha = 0.3;
const float beta = 2.0;
const float kappa = 0.1;
const float dt = 0.01;

bool gps_started = true;
double lat0 = 0.0;
double lon0 = 0.0;


// Callback function to handle incoming IMU messages
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // --- Store data into matrices ---
    // Accelerometer (m/s^2)
    acc_measurement << msg->linear_acceleration.x,
                    msg->linear_acceleration.y, 
                    msg->linear_acceleration.z;

    // Gyroscope (rad/s)
    gyro_measurement << msg->angular_velocity.x,
                    msg->angular_velocity.y, 
                    msg->angular_velocity.z;

    // Magnetometer (uT)
    mag_measurement << (msg->magnetic.x - mag_hardiron_offset_x) * 1e-6,
                (mag.magnetic.y - mag_hardiron_offset_y) * 1e-6,
                (mag.magnetic.z - mag_hardiron_offset_z) * 1e-6;
}
// Call back function to handle incoming encoder messages

// Call back function to handle incoming gps messages
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (gps_started)
    {
        lat0 = msg->latitude;
        lon0 = msg->longitude;
        gps_started = false;
    }

    gps_measurement << msg->latitude,
                    msg->longitude,
                    msg->altitude;

}

int main() {

	MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta, kappa);
	UKF ukf(sigma_points);

    while (true)
    {
    ukf.predict_states(encoder_measurement, dt);
    ukf.predict_measurement(dt,encoder_measurement,lat0, lon0);

    z_measurement << gyro_measurement, acc_measurement, mag_measurement, gps_measurement;
    ukf.update(z_measurement);

    // --- Output to Serial ---
    cout << "x_prior: " << ukf.x_prior << endl;
    cout << "P_prior: " << ukf.P_prior << endl;

    cout << "x_posterior: " << ukf.x_post << endl;
    cout << "P_posterior: " << ukf.P_post << endl;

    cout << "measurements" << z_measurement << endl;

    }

	return 0;
}
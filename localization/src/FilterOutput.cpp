#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include "Quaternion.h"
#include "UKF.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

Eigen::Vector3d acc_measurement;
Eigen::Vector3d gyro_measurement;
Eigen::Vector3d mag_measurement;

// const float mag_hardiron_offset_x = -31.71;
// const float mag_hardiron_offset_y = 28.61;
// const float mag_hardiron_offset_z = 33.985;

// const float gyro_offset_x = 0.06285;
// const float gyro_offset_y = -0.08785;
// const float gyro_offset_z = -0.06815;

const int n_state_dim = 7;
const float alpha = 0.3;
const float beta = 2.0;
const float kappa = 0.1;
const float dt = 0.001;

MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta, kappa);
UKF imu_ukf(sigma_points);
Quaternion q(1, 0, 0, 0);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    acc_measurement << msg->linear_acceleration.x,
                    msg->linear_acceleration.y, 
                    msg->linear_acceleration.z;

    gyro_measurement << msg->angular_velocity.x,
                    msg->angular_velocity.y, 
                    msg->angular_velocity.z;

    q.s = msg->orientation.w; q.v_1 = msg->orientation.x; q.v_2 = msg->orientation.y; q.v_3 = msg->orientation.z;
    mag_measurement << q.get_roll(), q.get_pitch(), q.get_yaw();

    // Predict step with UKF's quaternion with ang vec model
    imu_ukf.predict_with_quaternion_ang_vec_model(dt, gyro_measurement);

    // Concatenate gyroscope, accelerometer, and magnetometer for measurement
    Eigen::VectorXd z_measurement(gyro_measurement.size() + acc_measurement.size() + mag_measurement.size());
    z_measurement << gyro_measurement, acc_measurement, mag_measurement;

    // Update step UKF with measurements 
    imu_ukf.update_with_quaternion_ang_vec_model(z_measurement);

    // Print filter output
    std::cout << "Filtered Orientation: "<< std::endl << "w:" << imu_ukf.x_hat[0] << std::endl << "x:" << imu_ukf.x_hat[1]  << std::endl << "y:" << imu_ukf.x_hat[2] << std::endl << "z:" << imu_ukf.x_hat[3] << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10, imuCallback);

    ros::spin();

    return 0;
}

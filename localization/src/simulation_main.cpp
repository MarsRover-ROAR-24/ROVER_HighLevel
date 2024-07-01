#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

Eigen::VectorXd z_measurement(11);
Eigen::VectorXd encoder_measurement(6);

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
double yaw = 0.0; 

// Initialize Sigma Points and UKF
MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa);
UKF ukf(sigma_points);

ros::Subscriber imu_sub;
ros::Subscriber encoder_sub;
ros::Subscriber gps_sub;

ros::Publisher state_publisher;

void publishTransform(const Eigen::VectorXd& states) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";  // Fixed frame
    transformStamped.child_frame_id = "transformed_frame";  // Frame of your IMU

    // Fill in transform
    transformStamped.transform.translation.x = 0.0;  // Adjust as needed
    transformStamped.transform.translation.y = 0.0;  // Adjust as needed
    transformStamped.transform.translation.z = 0.0;  // Adjust as needed
    Eigen::Quaterniond quat(states[0], states[1], states[2], states[3]);
    quat.normalize(); // Ensure unit magnitude
    transformStamped.transform.rotation.w = quat.w();
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();

    // Publish transform
    static_broadcaster.sendTransform(transformStamped);
}
void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std_msgs::Float64MultiArray state_msg;

    if (msg->velocity.size() != 6) return;

    if (encoder_prev_time_stamp.isZero()) 
    {
        encoder_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time encoder_current_time_stamp = msg->header.stamp;
    dt = (encoder_current_time_stamp - encoder_prev_time_stamp).toSec();
    cout << "dt: " << dt << "\n";

    for (int i = 0; i < 6; ++i) 
    {
        encoder_measurement[i] = msg->velocity[i];
    }

    ukf.encoder_callback(encoder_measurement, dt, yaw);
    encoder_prev_time_stamp = encoder_current_time_stamp;

    state_msg.data = {ukf.x_post[0], ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[4], ukf.x_post[5], ukf.x_post[6], ukf.x_post[7], ukf.x_post[8]};
    state_publisher.publish(state_msg);
}
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    std_msgs::Float64MultiArray state_msg;

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
    ukf.gps_callback(z_measurement, lon0, lat0);
    gps_prev_time_stamp = gps_current_time_stamp;
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   Quaternion q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    yaw = q.get_yaw() + 1.57;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization");
    ros::NodeHandle nh;
    
    imu_sub = nh.subscribe("/imu", 1000, imuCallback);
    encoder_sub = nh.subscribe("/joint_states", 1000, encoderCallback);
    gps_sub = nh.subscribe("/gps", 1000, gpsCallback);

    state_publisher = nh.advertise<std_msgs::Float64MultiArray>("/filtered_state", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
    }

	return 0;
}
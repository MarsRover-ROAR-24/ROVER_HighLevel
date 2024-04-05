#include "ROAR_UKF.h"
#include <ros/ros.h>
#include "localization/buffer.h"
#include <chrono>

using namespace std;

Eigen::VectorXd z_measurement;
Eigen::VectorXd encoder_measurement;

const int n_state_dim = 9;  // x_state dimension
const float alpha = 0.3;
const float beta_ = 2.0;
const float kappa = 0.1;
ros::Time prev_time_stamp;
double dt = 0.0;
bool new_measurement_received = false;
bool intial_measurment = true;
double lat0 = 0.0;
double lon0 = 0.0;

void sensorsCallback(const localization::buffer::ConstPtr& msg)
{
    if (intial_measurment == true)
    {
        lat0 = msg->measurements[9];
        lon0 = msg->measurements[10];
        intial_measurment = false;
    }
    if (prev_time_stamp.isZero()) 
    {
        prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time current_time_stamp = msg->header.stamp;
    dt = (current_time_stamp - prev_time_stamp).toSec();

    prev_time_stamp = current_time_stamp;

    for (int i = 0; i < 11; ++i)
    {
        z_measurement[i] = msg->measurements[i];
    }
    for (int i = 0; i < 6; ++i)
    {
        encoder_measurement[i] = msg->wheel_odometry[i];
    }
    new_measurement_received = true;
}

// void timerCallback(const ros::TimerEvent& event)
// {


 
// }

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization");
    ros::NodeHandle nh;

    z_measurement = Eigen::VectorXd::Zero(11);
    encoder_measurement = Eigen::VectorXd::Zero(6);

    ros::Subscriber sensor_sub = nh.subscribe("/sensors", 1000, sensorsCallback);

    // Create a timer with the desired loop rate (e.g., 10 Hz)
    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback); // 0.1 seconds = 10 Hz

    // Initialize Sigma Points and UKF
    MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa);
    UKF ukf(sigma_points);

    while (ros::ok())
    {
        if (new_measurement_received)
        {   
            // Predict and update
            ukf.predict_states(encoder_measurement, dt);
            ukf.predict_measurement(dt, encoder_measurement, lat0, lon0);
            ukf.update(z_measurement);
            cout << "x_prior: " << endl << ukf.x_prior << endl;

            // Output to Serial or any other processing  
            new_measurement_received = false; // Reset flag
        }
        
        ros::spinOnce();
    }

	return 0;
}

/* The name of this class is "KalmanFilter".
 * This is a kalmanfilter class.
*/
#ifndef EVAROBOT_KALMAN_FILTER_H_
#define EVAROBOT_KALMAN_FILTER_H_
#include <opencv2/video/video.hpp>
#include <ros/ros.h>
#include "evarobot_adp/ObjectList.h"

namespace evarobotprocess
{
class KalmanFilter
{
public:
    KalmanFilter(const geometry_msgs::Point& position); //Constructor of this class.
    KalmanFilter(); //Constructor of this class with initial parameter
    virtual ~KalmanFilter(); //Destructor of this class
    void resetState(); //Reset the state of kalman filter
    void updateMeasurement(const geometry_msgs::Point& position); //Update the information of measurement
    void timeUpdateStep(double dt); //Predict the State vector
    geometry_msgs::Vector3 correctionStep(); //Correct the predicted state vector with measurement
protected:
    void initKalman(int state_size = 6, int measurement_size = 3); //Initialize the kalmanfilter
    cv::KalmanFilter kf_;
    cv::Mat state_;
    cv::Mat measurement_;
};
}
#endif

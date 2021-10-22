#include "evarobot_adp/kalman_filter.h"
namespace evarobotprocess {

//This Constructor initializes the kalmanfilter with a position.
KalmanFilter::KalmanFilter(const geometry_msgs::Point& position)
{
    initKalman();
    updateMeasurement(position);
    resetState();
}

//This COnstructor initializes the kalmanfilter without parameter
KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

//Reset the state of kalman filter.
void KalmanFilter::resetState()
{
    state_.at<double>(0) = measurement_.at<double>(0);
    state_.at<double>(1) = measurement_.at<double>(1);
    state_.at<double>(2) = measurement_.at<double>(2);
    state_.at<double>(3) = 0.0;
    state_.at<double>(4) = 0.0;
    state_.at<double>(5) = 0.0;
    kf_.statePost = state_;
}

//Update the position in the variable.
void KalmanFilter::updateMeasurement(const geometry_msgs::Point& position)
{
    measurement_.at<double>(0) = position.x;
    measurement_.at<double>(1) = position.y;
    measurement_.at<double>(2) = position.z;
}

//This is Time-Update step of kalmanfilter, which predicts the state vector in next timestep.
void KalmanFilter::timeUpdateStep(double dt)
{
    int state_size = state_.rows;
    int measurement_size = measurement_.rows;
    for (int i = measurement_size; i < state_size; i ++) {
        kf_.transitionMatrix.at<double>(i - measurement_size, i) = dt;
    }
    kf_.predict();
}

//This is correction step of kalmanfilter, which corrects the predicted state vector with measurement and returen the results
geometry_msgs::Vector3 KalmanFilter::correctionStep()
{
    cv::Mat temp = kf_.correct(measurement_);
    for (int i = 0; i < state_.rows; i ++) {
        state_.at<double>(i) = temp.at<double>(i);
    }
    geometry_msgs::Vector3 result;
    result.x = state_.at<double>(3);
    result.y = state_.at<double>(4);
    result.z = state_.at<double>(5);
    return result;
}

//Initialize the kalmanfilter.
void KalmanFilter::initKalman(int state_size, int measurement_size)
{
    kf_ = cv::KalmanFilter(state_size, measurement_size, 0, CV_64F);
    state_ = cv::Mat(state_size, 1, CV_64F);
    measurement_ = cv::Mat(measurement_size, 1, CV_64F);
    cv::Mat A = cv::Mat::eye(state_size, state_size, CV_64F);
    cv::Mat H = cv::Mat(measurement_size, state_size, CV_64F, cv::Scalar(0));
    for (int i = 0; i < measurement_size; i ++) H.at<double>(i, i) = 1.0;
    cv::Mat Q = cv::Mat::zeros(state_size, state_size, CV_64F);
    for (int i = 0; i < measurement_size; i ++) Q.at<double>(i, i) = 10e-5;
    for (int i = measurement_size; i < state_size; i ++) Q.at<double>(i, i) = 10e-4;
    cv::Mat R = cv::Mat::zeros(measurement_size, measurement_size, CV_64F);
    cv::setIdentity(R, cv::Scalar::all(0.001));
    kf_.transitionMatrix = A;
    kf_.measurementMatrix = H;
    kf_.processNoiseCov = Q;
    kf_.measurementNoiseCov = R;
    kf_.errorCovPre.at<double>(0) = 0.02;
    kf_.errorCovPre.at<double>(7) = 0.02;
    kf_.errorCovPre.at<double>(14) = 0.02;
    kf_.errorCovPre.at<double>(21) = 0.1;
    kf_.errorCovPre.at<double>(28) = 0.1;
    kf_.errorCovPre.at<double>(35) = 0.1;
    kf_.errorCovPost.at<double>(0) = 0.02;
    kf_.errorCovPost.at<double>(7) = 0.02;
    kf_.errorCovPost.at<double>(14) = 0.02;
    kf_.errorCovPost.at<double>(21) = 0.1;
    kf_.errorCovPost.at<double>(28) = 0.1;
    kf_.errorCovPost.at<double>(35) = 0.1;
}
}

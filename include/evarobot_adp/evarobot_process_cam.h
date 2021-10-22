/* The name of this class is "EvarobotProcessCam".
 * This class is used for processing the data from camera.
 * This process is in the callback function directly.
 * The result of this process is a object list, which contains all the detected features in the form of a list of objects.
 * The member variable of object list is "objectlist".
*/
#ifndef EVAROBOT_PROCESS_CAM_H_
#define EVAROBOT_PROCESS_CAM_H_
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "evarobot_adp/ObjectList.h"
#include "evarobot_adp/kalman_filter.h"
namespace evarobotprocess
{
class EvarobotProcessCam
{
public:
    EvarobotProcessCam(ros::NodeHandle& nh, std::vector<float>* data_sonar, std::vector<double>* data); //Constructor of this class
    virtual ~EvarobotProcessCam(); //Destructor of this class
    void update(); //Update process
    evarobot_adp::ObjectList getObjectList(); // Get the result from this process.
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg); // Callback function for depth-camera. The data processing is in this function.
    void imageCallback(const sensor_msgs::ImageConstPtr& msg); // Callback function for rgb-camera. The processing of image data is in this function.
    void recur_object(size_t i, std::vector<size_t>& contain, size_t& num_obj, int& area); // The recursion to detect object.
    bool data_processed = false; // Judge whether the processing finished.

protected:
    ros::Subscriber sub_pointcloud; //Subscriber of depth-camera
    ros::Subscriber sub_image; //Subscriber of rgb-camera
    ros::Publisher pub_depth; //Publisher of object list
    uint32_t img_height; //Height of image pixel
    uint32_t img_width; //Width of image pixel
    sensor_msgs::ImageConstPtr current_img; //The current information of rgb-camera
    sensor_msgs::PointCloud data_cam; //The current information of depth-camera
    cv::Mat input_img; //The converted image
    std::vector<std::vector<cv::Point>> convexs; //Contours of Objects
    std::vector<cv::Rect> boundb2D; //Bounding box in image
    float height_cam = 0.37f; //The height of camera-sensor
    bool have_image = false; //Judge whether the rgb-camera receive information
    evarobot_adp::ObjectList objectlist; //The results of camera-processing
    evarobot_adp::ObjectList obj_last; //The last results of camera-processing
    int object_id = 0; //Mark of object id
    std::vector<double> *odm; //The velocity of Robot

    //Vectors that store the features of the objects; This means one object each line.
    //The vectors reflect the coordinates of the boundaries of the objects; This allows the geometry to be approximated.
    //The corresponding unit is in [m].
    std::vector<float> left;
    std::vector<float> right;
    std::vector<float> top;
    std::vector<float> bottom;
    std::vector<float> depth;
    std::vector<float> depth_max;
    std::vector<float>* data_sonar;
    std::vector<int> left2;
    std::vector<int> right2;
    std::vector<int> top2;
    std::vector<int> bottom2;
    std::map<int, KalmanFilter> kfilters;
};
}

#endif

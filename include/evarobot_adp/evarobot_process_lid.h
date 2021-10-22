/* The name of this class is "EvarobotProcessLid".
 * This class is used for processing the data from lidar.
 * This process is in the callback function directly.
 * The result of this process is a object list, which contains all the detected features in the form of a list of objects.
 * The member variable of object list is "objectlist".
*/
#ifndef EVAROBOT_PROCESS_LID_H_
#define EVAROBOT_PROCESS_LID_H_
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <laser_geometry/laser_geometry.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include "evarobot_adp/ObjectList.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include "evarobot_adp/kalman_filter.h"

namespace evarobotprocess
{
class EvarobotProcessLid
{
public:
    EvarobotProcessLid(ros::NodeHandle& nh,std::vector<double>* data); //Constructor of this class
    virtual ~EvarobotProcessLid(); //Destructor of this class
    void update(); //Update process
    evarobot_adp::ObjectList getObjectList(); // Get the result from this process.
    void pointCloudCallback(const sensor_msgs::LaserScanPtr& msg); //Callback function for lidar. The data processing is in this function.
    bool data_processed = false;

protected:
    ros::Subscriber sub_LaserScan; //Subscriber of lidar
    ros::Publisher pub_lid; //Publischer of result
    evarobot_adp::ObjectList objectlist; //The results of lidar-processing
    std::vector<double> *odm_lidar; //Velocity of Robot
    evarobot_adp::ObjectList last_obj; //The last results
    std::vector<float> last_odm = std::vector<float>(3,0);
    std::vector<float> speed_odm = std::vector<float>(3,0);
    std::vector<float> speed_obj = std::vector<float>(3,0);
    int object_id = 0; //Mark of object id
    std::map<int, KalmanFilter> kfilters; //Map of kalman filters
};
}

#endif

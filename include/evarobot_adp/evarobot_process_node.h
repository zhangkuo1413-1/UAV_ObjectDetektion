/* The name of this class is "EvarobotProcessNode". This is the Subcluss of ROS-Node.
 * This class contains the all the classes of sensors. These sensors are "camera", "lidar" and "sonar".
 * This class also contains the function of data fusion. The fusion function is in "update" method.
 * The result of this process is a object list, which contains all the detected features in the form of a list of object.
 * The member variable of aggregated object list is "objectlist".
*/
#ifndef EVAROBOT_PROCESS_NODE_H_
#define EVAROBOT_PROCESS_NODE_H_
#include <ros/ros.h>
#include <evarobot_adp/evarobot_process_cam.h>
#include <evarobot_adp/evarobot_process_lid.h>
#include "evarobot_adp/evarobot_process_sonar.h"
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>

namespace evarobotprocess
{
class EvarobotProcessNode 
{
public:
    EvarobotProcessNode(ros::NodeHandle& nh); // Constructor of this class. In this Constructor, all the classes of sensors are constucted.
    virtual ~EvarobotProcessNode(); // Destructor of this class.
    void SonarPlus();
    void update(); // Fusion of data. Visualization and publishing the object list.
    void velocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg); // Callback function of the velocity of Evarobot.
    float four_quadrant_angle(float angle);
    bool sonar_object_detected(float obj_angle1, float obj_angle2, float sonar_angle1, float sonar_angle2);

protected:
    std::vector<float>* data_sonar = sonar.getdata(); //Measurement of sonar
    std::vector<double> odm =std::vector<double>(9,0.0) ; //Velocity of Evarobot
    ros::Publisher vis_pub; //Publisher of Visualisition
    ros::Publisher ob_pub; //Publisher of object list
    ros::Subscriber sub_velocity; //Subscriber of velocity of Evarobot
    EvarobotProcessSonar sonar; //Class of sonar
    EvarobotProcessCam camera; //Class of camera
    EvarobotProcessLid lidar; //Class of lidar
    evarobot_adp::ObjectList objectlist; //Object list
    visualization_msgs::MarkerArray boundingbox_array; //Booundary box for visualization
};
}
#endif

/* The name of this class is "EvarobotProcessSonar".
 * The purpose of this class is to extract sonar data.
 * The result of this process is a vector, which contains position information from sonars.
 * The member variable of vector is "data".
*/
#ifndef EVAROBOT_PROCESS_SONAR_H_
#define EVAROBOT_PROCESS_SONAR_H_
#include "evarobot_adp/ObjectList.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/point_cloud_conversion.h>
#define PI 3.14159265
namespace evarobotprocess
{
	
class EvarobotProcessSonar
{

public:
    EvarobotProcessSonar(ros::NodeHandle& nh); //Constructor of this class
    //evarobot_adp::ObjectList getObjectList();
    virtual ~EvarobotProcessSonar(); //Destructor of this class
    void RangeCallback_0(const sensor_msgs::Range::ConstPtr& msg);
    void RangeCallback_1(const sensor_msgs::Range::ConstPtr& msg);
    void RangeCallback_2(const sensor_msgs::Range::ConstPtr& msg);
    void RangeCallback_3(const sensor_msgs::Range::ConstPtr& msg);
    void RangeCallback_4(const sensor_msgs::Range::ConstPtr& msg);
    void RangeCallback_5(const sensor_msgs::Range::ConstPtr& msg);
    void RangeCallback_6(const sensor_msgs::Range::ConstPtr& msg);
    void update(); // Update function
    std::vector<float>* getdata(); //Get the results
    float coord_trans(double x0, double y0, float alpha, double range);
    float alpha[7] = {2.5, 62.5, 122.5, 152.5, 212.5, 272.5, 332.5};
    float beta[7] = {57.5, 117.5, 177.5, 207.5, 267.5, 327.5, 27.5};
    double x_coord[7] = {0.102, -0.045, -0.192, -0.213, -0.129, 0.039, 0.123};
    double y_coord[7] = {-0.04, 0.043, -0.04, -0.215, -0.361, -0.361, -0.215};

protected:
     ros::Subscriber sub_Sonar0;
     ros::Subscriber sub_Sonar1;
     ros::Subscriber sub_Sonar2;
     ros::Subscriber sub_Sonar3;
     ros::Subscriber sub_Sonar4;
     ros::Subscriber sub_Sonar5;
     ros::Subscriber sub_Sonar6;
     std::vector<float> data = std::vector<float>(7, 5.0);
     std::vector<bool> bl;
};
}
#endif

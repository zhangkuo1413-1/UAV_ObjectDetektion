#include "evarobot_adp/evarobot_process_sonar.h"
namespace evarobotprocess
{
    //sub_Sonar0 - sub_Sonar6 define the subcsribers of Sonar data(Range)
    EvarobotProcessSonar::EvarobotProcessSonar(ros::NodeHandle& nh)
    {
        bl = std::vector<bool>(7,false);
        sub_Sonar0=nh.subscribe("/sonar0", 1, &EvarobotProcessSonar::RangeCallback_0, this);
        sub_Sonar1=nh.subscribe("/sonar1", 1, &EvarobotProcessSonar::RangeCallback_1, this);
        sub_Sonar2=nh.subscribe("/sonar2", 1, &EvarobotProcessSonar::RangeCallback_2, this);
        sub_Sonar3=nh.subscribe("/sonar3", 1, &EvarobotProcessSonar::RangeCallback_3, this);
        sub_Sonar4=nh.subscribe("/sonar4", 1, &EvarobotProcessSonar::RangeCallback_4, this);
        sub_Sonar5=nh.subscribe("/sonar5", 1, &EvarobotProcessSonar::RangeCallback_5, this);
        sub_Sonar6=nh.subscribe("/sonar6", 1, &EvarobotProcessSonar::RangeCallback_6, this);
    }

    EvarobotProcessSonar::~EvarobotProcessSonar()
    {
    }
    /*
    evarobot_adp::ObjectList EvarobotProcessSonar::getObjectList()
    {
        //return objectlist;
    }
    */

    void EvarobotProcessSonar::update()
    {
        bl=std::vector<bool>(7,false);
    }

    //Function to determine the endpoints of the detection area
    float EvarobotProcessSonar::coord_trans(double x0, double y0, float alpha, double range)
    {
        double x1 = x0 + range*cos(alpha);
        double y1 = y0 + range*sin(alpha);
        float beta = atan2(y1, x1);
        if(beta < 0)
        {
            beta = beta + 2*PI;
        }
        return beta;
    }

    //7 Callback-functions, each one reading in the data of one sonar
    void EvarobotProcessSonar::RangeCallback_0(const sensor_msgs::Range::ConstPtr& msg)
    {
        bl[0]=true;
        data[0]=msg->range;
    }
    void EvarobotProcessSonar::RangeCallback_1(const sensor_msgs::Range::ConstPtr& msg)
    {
        bl[1]=true;
        data[1]=msg->range;
    }
    void EvarobotProcessSonar::RangeCallback_2(const sensor_msgs::Range::ConstPtr& msg)
    {
        bl[2]=true;
        data[2]=msg->range;
    }
    void EvarobotProcessSonar::RangeCallback_3(const sensor_msgs::Range::ConstPtr& msg)
    {
        bl[3]=true;
        data[3]=msg->range;
    }
    void EvarobotProcessSonar::RangeCallback_4(const sensor_msgs::Range::ConstPtr& msg)
    {
        bl[4]=true;
        data[4]=msg->range;
    }
    void EvarobotProcessSonar::RangeCallback_5(const sensor_msgs::Range::ConstPtr& msg)
    {
        bl[5]=true;
        data[5]=msg->range;
    }
    void EvarobotProcessSonar::RangeCallback_6(const sensor_msgs::Range::ConstPtr& msg)
    {
        bl[6]=true;
        data[6]=msg->range;
    }
    std::vector<float>* EvarobotProcessSonar::getdata()
    {
        return &data;
    }

}

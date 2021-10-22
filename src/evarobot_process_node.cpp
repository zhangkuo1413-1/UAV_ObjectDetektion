#include <evarobot_adp/evarobot_process_node.h>
#include "evarobot_process_cam.cpp"
#include "evarobot_process_lid.cpp"
#include "evarobot_process_sonar.cpp"
#include <sys/resource.h>
#include "math.h"
#define PI 3.14159265

namespace evarobotprocess
{
    EvarobotProcessNode::EvarobotProcessNode(ros::NodeHandle& nh): sonar(nh), camera(nh, data_sonar, &odm), lidar(nh,&odm)
    {
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
        ob_pub = nh.advertise<evarobot_adp::ObjectList>( "object_list", 0 );
        sub_velocity = nh.subscribe("gazebo/model_states", 1, &EvarobotProcessNode::velocityCallback, this);
    }

    //convert angle coordinates from [-pi, pi] to [0, 2*pi]
    float EvarobotProcessNode::four_quadrant_angle(float angle)
    {
        if(angle < 0)
        {
            return angle + 2*PI;
        }
        else
        {
            return angle;
        }
    }

    //check if the angle interval which is determined by boundary-box from camera/lidar overlaps the angle interval determined by sonar
    bool EvarobotProcessNode::sonar_object_detected(float obj_angle1, float obj_angle2, float sonar_angle1, float sonar_angle2)
    {
        float obj_upper_angle = std::max(obj_angle1, obj_angle2);
        float obj_lower_angle = std::min(obj_angle1, obj_angle2);
        float sonar_upper_angle = std::max(sonar_angle1, sonar_angle2);
        float sonar_lower_angle = std::min(sonar_angle1, sonar_angle2);
        if(obj_upper_angle - obj_lower_angle > PI) //in case the object cuts across 2 quadrants
        {
            float angle_buffer = obj_upper_angle;
            obj_upper_angle = obj_lower_angle;
            obj_lower_angle = angle_buffer - 2*PI;
        }
        if(sonar_upper_angle - sonar_lower_angle > PI) //in case the object cuts across 2 quadrants
        {
            float angle_buffer = sonar_upper_angle;
            sonar_upper_angle = sonar_lower_angle;
            sonar_lower_angle = angle_buffer - 2*PI;
        }
        if(obj_upper_angle > sonar_lower_angle && obj_lower_angle < sonar_upper_angle)
        {
            return true;
        }
        return false;
    }

	EvarobotProcessNode::~EvarobotProcessNode() {
	}
    void EvarobotProcessNode::SonarPlus()
    {
        std::stringstream it;
    }
	void EvarobotProcessNode::update() {
        SonarPlus();
        camera.update();
        lidar.update();
        int fusion = 1;
        if(fusion == 1 && (camera.data_processed || lidar.data_processed))
        {
            auto start = std::chrono::high_resolution_clock::now();
            evarobot_adp::ObjectList oblist_cam;
            evarobot_adp::ObjectList oblist_lid;
            std::vector<float>* sonar_data;
            std::vector<float> range_data;
            oblist_cam = camera.getObjectList();
            oblist_lid = lidar.getObjectList();
            sonar_data = sonar.getdata();
            range_data = *sonar_data;
            std::vector<size_t> match_cam;
            std::vector<size_t> match_lid;
            int match = 0;
            int test = 0;
            int obj_id = 0;
            float angle_buffer;
            float right_max; //temporary variables to determine whether boundary-boxes overlap each other
            float left_min;
            evarobot_adp::ObjectList fusionlist;
            visualization_msgs::MarkerArray marker_array; //for depiction of boundary-boxes
            marker_array.markers.clear();
            for (size_t i = 0; i < oblist_cam.objects.size() && !(oblist_cam.objects.size() < 1) ; i++)
            {
                for (size_t j = 0; j < oblist_lid.objects.size(); j++)
                {
                    //determine the most right or left position of the objects for safer data fusion
                    right_max = std::max(oblist_cam.objects[i].right, oblist_lid.objects[j].right);
                    left_min = std::min(oblist_cam.objects[i].left, oblist_lid.objects[j].left);

                    //if boundary-boxes along the height direction don't overlap, skip the loop
                    if(oblist_cam.objects[i].top < 0.2315 || oblist_cam.objects[i].bottom > 0.2315) continue;
                    if(right_max < left_min && oblist_lid.objects[j].depth_min - 0.32 > 0) //lidar and camera detected the same object
                    {
                        test++;
                        match_cam.insert(match_cam.end(),i);
                        match_lid.insert(match_lid.end(),j);
                        evarobot_adp::Object object;
                        //data fusion of those detected by both sensor
                        visualization_msgs::Marker marker;
                        visualization_msgs::Marker text;
                        marker.header.frame_id = "camera_depth_frame";
                        marker.header.stamp = ros::Time();
                        marker.ns = "evarobotprocess";
                        marker.id = match;
                        marker.type = visualization_msgs::Marker::CUBE;
                        object.center.x = (oblist_cam.objects[i].depth_max + oblist_cam.objects[i].depth_min) / 2;
                        object.center.y = (std::max(oblist_cam.objects[i].left, oblist_lid.objects[j].left + 0.045f) + std::min(oblist_cam.objects[i].right, oblist_lid.objects[j].right+0.045f))/2;
                        object.center.z = oblist_cam.objects[i].center.z;
                        object.width = std::max(0.01f, -std::min(oblist_cam.objects[i].right, oblist_lid.objects[j].right+0.045f) + std::max(oblist_cam.objects[i].left, oblist_lid.objects[j].left+0.045f));
                        object.height = std::max(0.01f, oblist_cam.objects[i].top - oblist_cam.objects[i].bottom);

                        if(oblist_lid.objects[j].right + 0.045f < oblist_cam.objects[i].right || oblist_lid.objects[j].left + 0.045f > oblist_cam.objects[i].left)
                        {
                            object.velocity.x = oblist_lid.objects[j].velocity.x;
                            object.velocity.y = oblist_lid.objects[j].velocity.y;
                            object.velocity.z = oblist_lid.objects[j].velocity.z;
                            object.dynamic = oblist_lid.objects[j].dynamic;
                        }
                        else
                        {
                            object.velocity.x = (oblist_cam.objects[i].velocity.x + oblist_lid.objects[j].velocity.x)/2;
                            object.velocity.y = (oblist_cam.objects[i].velocity.y + oblist_lid.objects[j].velocity.y)/2;
                            object.velocity.z = oblist_cam.objects[i].velocity.z;
                            if (object.velocity.x * object.velocity.x + object.velocity.y * object.velocity.y + object.velocity.z * object.velocity.z > 0.04){
                                object.dynamic = true;
                            }
                            else {
                                object.dynamic = false;
                            }
                        }

                        //visualisation of boundary-boxes through markers in RVIZ
                        marker.pose.position = object.center;
                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = oblist_cam.objects[i].depth_max - oblist_cam.objects[i].depth_min;
                        marker.scale.y = object.width;
                        marker.scale.z = object.height;
                        marker.color.a = 0.3; // Don't forget to set the alpha!
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        marker.lifetime.sec = 1;
                        marker_array.markers.insert(marker_array.markers.end(), marker);
                        object.top = oblist_cam.objects[i].top;
                        object.bottom = oblist_cam.objects[i].bottom;
                        object.left = std::max(oblist_cam.objects[i].left, oblist_lid.objects[j].left+0.045f);
                        object.right = std::min(oblist_cam.objects[i].right, oblist_lid.objects[j].right+0.045f);
                        object.depth_min = oblist_cam.objects[i].depth_min;
                        object.depth_max = oblist_cam.objects[i].depth_max;
                        object.direction = oblist_cam.objects[i].direction;
                        object.fusion = "cam, lid";

                        //calculate the angle interval of boundary-boxes determined by lidar and camera
                        std::vector<float> quadrate_angles;
                        quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_max, -object.left)));
                        quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_max, -object.right)));
                        quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_min, -object.left)));
                        quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_min, -object.right)));
                        object.lower_angle = quadrate_angles[0];
                        object.upper_angle = quadrate_angles[0];
                        for (size_t ki = 1; ki < quadrate_angles.size(); ki++)
                        {
                            if(quadrate_angles[ki] < object.lower_angle)
                            {
                                object.lower_angle = quadrate_angles[ki];
                            }
                            if(quadrate_angles[ki] > object.upper_angle)
                            {
                                object.upper_angle = quadrate_angles[ki];
                            }
                        }
                        if(object.upper_angle - object.lower_angle > PI) //in case the object cuts across the 1st and the 4th quadrants
                        {
                            angle_buffer = object.upper_angle;
                            object.upper_angle = object.lower_angle;
                            object.lower_angle = angle_buffer - 2*PI;
                        }
                        quadrate_angles.clear();
                        //include objects detected by both lidar and camera

                        for (int j = 0; j < range_data.size() ; j++)
                        {
                            if(range_data[j] < 5) //sonar detects an object
                            {
                                float sonar_angle1 = sonar.coord_trans(sonar.x_coord[j], sonar.y_coord[j], sonar.alpha[j], range_data[j]);
                                float sonar_angle2 = sonar.coord_trans(sonar.x_coord[j], sonar.y_coord[j], sonar.beta[j], range_data[j]);
                                bool detected = sonar_object_detected(object.lower_angle, object.upper_angle, sonar_angle1, sonar_angle2);
                                if(detected == true && object.height < 0.5)
                                {
                                    marker.pose.position.z = 0.12;
                                    object.height = 0.5;//the difference in height between sonar and lidar is 0.46m, rounded up to 0.5m
                                    object.fusion = "lid, sonar";
                                    marker.scale.z = 0.5;
                                    if(range_data[j] < 2)
                                    {
                                        object.height = 1;
                                        marker.scale.z = 1;
                                    }
                                }
                            }
                        }
                        fusionlist.objects.insert(fusionlist.objects.end(), object);
                        match++;
                    }
                }
            }
            obj_id = match;
            //object is only detected by camera
            //this could happen at the height where objects are not covered by the range of lidar
            for (int i = 0; i < oblist_cam.objects.size() && !(oblist_cam.objects.size() < 1) ; i++)
            {
                int inspect1 = 0;
                size_t k = 0;
                for ( ; k < match_cam.size(); k++)
                {
                    if(i == match_cam[k])
                    {
                        inspect1++;
                    }
                }
                if(inspect1 == 0)
                {
                    evarobot_adp::Object object;
                    //integration of objects which are detected only by camera
                    object.center.x = (oblist_cam.objects[i].depth_max + oblist_cam.objects[i].depth_min) / 2;
                    object.center.y = oblist_cam.objects[i].center.y;
                    object.center.z = oblist_cam.objects[i].center.z;
                    object.width = std::max(0.01f, oblist_cam.objects[i].width);
                    object.height = std::max(0.01f, oblist_cam.objects[i].height);
                    object.top = oblist_cam.objects[i].top;
                    object.bottom = oblist_cam.objects[i].bottom;
                    object.left = oblist_cam.objects[i].left;
                    object.right = oblist_cam.objects[i].right;
                    object.depth_min = oblist_cam.objects[i].depth_min;
                    object.depth_max = oblist_cam.objects[i].depth_max;
                    object.direction = oblist_cam.objects[i].direction;
                    object.fusion = "cam";

                    object.velocity.x = oblist_cam.objects[i].velocity.x;
                    object.velocity.y = oblist_cam.objects[i].velocity.y;
                    object.velocity.z = oblist_cam.objects[i].velocity.z;
                    object.dynamic = oblist_cam.objects[i].dynamic;

                    //calculate the angle interval of boundary-boxes determined by camera
                    std::vector<float> quadrate_angles;
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_max, -object.left)));
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_max, -object.right)));
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_min, -object.left)));
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_min, -object.right)));
                    object.lower_angle = quadrate_angles[0];
                    object.upper_angle = quadrate_angles[0];
                    for (size_t kj = 1; kj < quadrate_angles.size(); kj++)
                    {
                        if(quadrate_angles[kj] < object.lower_angle)
                        {
                            object.lower_angle = quadrate_angles[kj];
                        }
                        if(quadrate_angles[kj] > object.upper_angle)
                        {
                            object.upper_angle = quadrate_angles[kj];
                        }
                    }
                    if(object.upper_angle - object.lower_angle > PI) //in case the object cuts across the 1st and the 4th quadrants
                    {
                        angle_buffer = object.upper_angle;
                        object.upper_angle = object.lower_angle;
                        object.lower_angle = angle_buffer - 2*PI;
                    }
                    quadrate_angles.clear();

                    fusionlist.objects.insert(fusionlist.objects.end(), object);

                    visualization_msgs::Marker marker;
                    visualization_msgs::Marker text;
                    marker.header.frame_id = "camera_depth_frame";
                    marker.header.stamp = ros::Time();
                    marker.ns = "evarobotprocess";
                    marker.id = obj_id;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.pose.position = object.center;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = object.depth_max - object.depth_min;
                    marker.scale.y = object.width;
                    marker.scale.z = object.height;
                    marker.color.a = 0.5; // Don't forget to set the alpha!
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.0;
                    marker.lifetime.sec = 1;
                    marker_array.markers.insert(marker_array.markers.end(), marker);

                    obj_id++;
                }
            }
            //objects are located outside of the detection area of camera, which means they could only be detected by lidar
            for (int i = 0; i < oblist_lid.objects.size() && !(oblist_lid.objects.size() < 1) ; i++)
            {
                int inspect2 = 0;
                size_t k = 0;
                for ( ; k < match_lid.size(); k++)
                {
                    if(i == match_lid[k])
                    {
                        inspect2++;
                    }
                }
                if(inspect2 == 0)
                {
                    evarobot_adp::Object object;
                    object.center.x = oblist_lid.objects[i].center.x - 0.32;
                    object.center.y = oblist_lid.objects[i].center.y + 0.045;
                    object.center.z = 0.35;
                    object.width = std::max(0.01f, oblist_lid.objects[i].width);
                    object.left = oblist_lid.objects[i].left + 0.045;
                    object.right = oblist_lid.objects[i].right + 0.045;
                    object.depth_min = oblist_lid.objects[i].depth_min - 0.32;
                    object.depth_max = oblist_lid.objects[i].depth_max - 0.32;
                    float length = std::sqrt(object.depth_min * object.depth_min + object.center.y * object.center.y + 0.35f * 0.35f);
                    object.direction.x = object.depth_min / length;
                    object.direction.y = object.center.y / length;
                    object.direction.z = 0.35 / length;
                    object.fusion = "lid";

                    object.velocity.x = oblist_lid.objects[i].velocity.x;
                    object.velocity.y = oblist_lid.objects[i].velocity.y;
                    object.velocity.z = oblist_lid.objects[i].velocity.z;
                    object.dynamic = oblist_lid.objects[i].dynamic;

                    //calculate the angle interval of boundary-boxes determined by lidar
                    std::vector<float> quadrate_angles;
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_max, -object.left)));
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_max, -object.right)));
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_min, -object.left)));
                    quadrate_angles.insert(quadrate_angles.end(), four_quadrant_angle(atan2(object.depth_min, -object.right)));
                    object.lower_angle = quadrate_angles[0];
                    object.upper_angle = quadrate_angles[0];
                    for (size_t kp = 1; kp < quadrate_angles.size(); kp++)
                    {
                        if(quadrate_angles[kp] < object.lower_angle)
                        {
                            object.lower_angle = quadrate_angles[kp];
                        }
                        if(quadrate_angles[kp] > object.upper_angle)
                        {
                            object.upper_angle = quadrate_angles[kp];
                        }
                    }
                    if(object.lower_angle < PI/2 && object.upper_angle > 3*PI/2) //in case the object cuts across the 1st and the 4th quadrants
                    {
                        angle_buffer = object.upper_angle;
                        object.upper_angle = object.lower_angle;
                        object.lower_angle = angle_buffer - 2*PI;
                    }
                    quadrate_angles.clear();

                    visualization_msgs::Marker marker;
                    visualization_msgs::Marker text;
                    marker.header.frame_id = "camera_depth_frame";
                    marker.header.stamp = ros::Time();
                    marker.ns = "evarobotprocess";
                    marker.id = obj_id;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.pose.position.x = (object.depth_max + object.depth_min) / 2;
                    marker.pose.position.y = oblist_lid.objects[i].center.y + 0.045;
                    marker.pose.position.z = 0.35;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = object.depth_max - object.depth_min;
                    marker.scale.y = object.width;
                    marker.scale.z = 0.1;

                    //integration of sonar detection into previous results(increase height of detected objects under certain circumstainces)
                    for (int j = 0; j < range_data.size() ; j++)
                    {
                        if(range_data[j] < 5) //sonar detects an object
                        {
                            float sonar_angle1 = sonar.coord_trans(sonar.x_coord[j], sonar.y_coord[j], sonar.alpha[j], range_data[j]);
                            float sonar_angle2 = sonar.coord_trans(sonar.x_coord[j], sonar.y_coord[j], sonar.beta[j], range_data[j]);
                            bool detected = sonar_object_detected(object.lower_angle, object.upper_angle, sonar_angle1, sonar_angle2);
                            if(detected == true)
                            {
                                marker.pose.position.z = 0.12;
                                object.height = 0.5;//the difference in height between sonar and lidar is 0.46m, rounded up to 0.5m
                                object.fusion = "lid, sonar";
                                marker.scale.z = 0.5;
                                if(range_data[j] < 2)//object is too close to roboter, increase height to 1m
                                {
                                    object.height = 1;
                                    marker.scale.z = 1;
                                }
                            }
                        }
                    }

                    fusionlist.objects.insert(fusionlist.objects.end(), object);
                    marker.color.a = 0.5; // Don't forget to set the alpha!
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.0;
                    marker.lifetime.sec = 1;
                    marker_array.markers.insert(marker_array.markers.end(), marker);
                    obj_id++;
                }
            }

            fusionlist.header.frame_id = "camera_depth_frame";
            fusionlist.sensor_id.data = 12;
            ros::WallTime time = ros::WallTime::now();
            fusionlist.header.stamp.sec = time.sec;
            fusionlist.header.stamp.nsec = time.nsec;
            boundingbox_array = marker_array;
            objectlist = fusionlist;
            vis_pub.publish(boundingbox_array);
            ob_pub.publish(objectlist);
        }
    }

    void EvarobotProcessNode::velocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
        int index = -1;
        for (size_t i = 0; i < msg->name.size(); i ++) {
            if (msg->name[i] == "evarobot"){
                index = i;
            }
        }
        if (index == -1) {
            std::cout<<"There is no evarobot!"<<std::endl;
        }
        else {
            tf::Quaternion q1;
            tf::Vector3 v1;
            tf::Point p1;
            tf::vector3MsgToTF(msg->twist[index].linear, v1);
            tf::pointMsgToTF(msg->pose[index].position, p1);
            tf::quaternionMsgToTF(msg->pose[index].orientation, q1);
            tf::Matrix3x3 M;
            M.setRotation(q1);
            v1 = M.transpose() * v1;
            p1 = M.transpose() * p1;
            odm[0]=v1.getX();
            odm[1]=v1.getY();
            odm[2]=v1.getZ();
            odm[3]=p1.getX();
            odm[4]=p1.getY();
            odm[5]=p1.getZ();
        }
    }
}

/* This is the main process of the object detection and identification. This process contains a loop that is always executing after starting.
 * In this main programm, the size of stack and some of the parameters of ROS are set.
 * The name of ROS-Node is "evarobot_process"
 * The defaut loop rate of the ROS-Node is 1000Hz
 * All the parameters can be changed in this programm
*/
int main(int argc,char **argv)
{
    // Set the stack size. The defaut stack size is 134217728B.
    const rlim_t kStackSize = 134217728;  // Parameter of stack size
    struct rlimit rl;
    int result;
    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0) fprintf(stderr, "setrlimit returned result = %d\n", result);
        }
    }

    // Initialize the parameters of ROS.
	ros::init(argc, argv, "evarobot_process");
    ros::NodeHandle nh; //ROS-Node
    evarobotprocess::EvarobotProcessNode node(nh); // Node of this class. This class includes all the sensors.
    ros::Rate loop_rate(nh.param("evarobot_process", 1000.0)); // Set the rate of loop.

    // The main loop of the process.
	while (ros::ok())
	{
        ros::spinOnce(); // Check if there is new data from a sonsor, the callback function of the sensor will be excuted.
        node.update(); // Excute the update process of all the sonsors
        loop_rate.sleep(); // Wait to align the rate of loop.
	}
	return 0;
}

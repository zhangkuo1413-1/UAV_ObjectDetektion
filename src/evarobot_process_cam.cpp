#include <evarobot_adp/evarobot_process_cam.h>

namespace evarobotprocess
{
    /* This Constructor define the subscribers and publishers for this process.
    * This Process contains only depth-camera and rgb-camera.
    * With receiving the sensor message, the callback-function will process the original data and return the object information.
    */
    EvarobotProcessCam::EvarobotProcessCam(ros::NodeHandle& nh, std::vector<float>* data_sonar, std::vector<double>* data)
    {
        this->data_sonar = data_sonar;
        sub_pointcloud = nh.subscribe("/camera/depth/points", 1, &EvarobotProcessCam::pointCloudCallback, this);
        sub_image = nh.subscribe("/camera/rgb/image_raw", 1, &EvarobotProcessCam::imageCallback, this);
        pub_depth = nh.advertise<evarobot_adp::ObjectList>( "/object_list_cam", 0 );
        odm = data;
    }

    EvarobotProcessCam::~EvarobotProcessCam()
    {
    }

    // Function to update the processed data.
    void EvarobotProcessCam::update()
    {
    }

    // Function to get the result of this process
    evarobot_adp::ObjectList EvarobotProcessCam::getObjectList()
    {
        return objectlist;
    }

    //Callback function for rgb-camera. This function use opencv to find out the objects and draw a rectangle surrounding them.
    void EvarobotProcessCam::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        if(!data_processed) return; // If there is no data from depth-camera, this function won`t execute.

        //convert the sensor-data into opencv format.
        current_img = msg;
        img_width = 640;
        img_height = 480;
        cv_bridge::CvImagePtr cv_ptr_ = cv_bridge::toCvCopy(current_img, sensor_msgs::image_encodings::BGR8); //bgr8: CV_8UC3, color image with blue-green-red color order                                                                                   //creating a mutable copy
        input_img = cv_ptr_->image;
        cv::Mat original_img = input_img.clone();
        cv::Mat padded_img;
        cv::copyMakeBorder(input_img,padded_img,1,1,1,1,cv::BORDER_CONSTANT,255); //adding a white border around image

        //Find contours of objects: include gray-conversion, blurring, threshold.
        std::vector<std::vector<cv::Point>> contours_padded;  //This variable contains the contours
        std::vector<cv::Vec4i> hierarchy;             //contains information about image topology
        cv::Mat dst_padded;

        //the outcome of detected contours is heavily dependend on the provided quality of the binary picture
        cv::blur(padded_img, dst_padded, cv::Size(5, 5), cv::Point(-1, -1), cv::BORDER_DEFAULT); //bluring
        cv::cvtColor(dst_padded, dst_padded, CV_BGR2HSV); //HSV-conversion
        cv::inRange(dst_padded, cv::Scalar(0, 0, 0), cv::Scalar(255, 83, 53), dst_padded);
        cv::dilate(dst_padded, dst_padded, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::findContours(dst_padded, contours_padded, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(-1,-1)); //CHAIN_APPROX_SIMPLE compresses the contours to only endpoints

        //Find convexs to contain the contours and adapt rectangles
        std::vector<std::vector<cv::Point>> convexs_temp(contours_padded.size());
        std::vector<cv::Rect> rects(contours_padded.size());
        convexs = convexs_temp;
        std::vector<cv::Rect> boundb;

        for (size_t i = 0; i < contours_padded.size();i++) //Only increases in size if contour is totally inside of img
        {
            cv::convexHull(contours_padded[i], convexs[i], false, true); //Find convexs
            rects[i] = cv::boundingRect(convexs[i]); //Find rectangles
            if (rects[i].area() <= (640*480) && rects[i].area() >= 400) boundb.insert(boundb.end(), rects[i]); //Select the rectangles, delete the unmatched rectangles
        }
        for (size_t i = 0; i < boundb2D.size(); i++) //Draw the rectangles
        {
            cv::rectangle(input_img, boundb2D[i], cv::Scalar(0, 255, 0), 1, 8);
        }
        cv::Mat padded_clone = input_img.clone();
        have_image = true;
        cv::imshow("original", input_img);
        cv::waitKey(10);
    }

    //Callback function for depth-camera. It finds the geographical coordinate for obstacles and make sure the size of bounding-box.
    void EvarobotProcessCam::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        //Initialize the original data and object list
        size_t ct = 0;
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, data_cam);
        std::vector<float> data_s = *data_sonar;
        objectlist.objects.clear();
        if (data_s.size() !=0 && !(data_s[1] < 0.0f))
        {
            evarobot_adp::ObjectList objectlists_temp;
            std::vector<size_t> contain_object(data_cam.points.size(), 0);
            size_t num_obj = 1;
            left.clear();
            right.clear();
            top.clear();
            bottom.clear();
            depth.clear();
            depth_max.clear();
            left2.clear();
            right2.clear();
            top2.clear();
            bottom2.clear();

            // Fill the vectors with the objects.
            // Use the recursive method named "recur_object".
            // Assignment of the objects if the depth information is within given limits.
            for (size_t i = 0; i < data_cam.points.size(); i ++) {
                if(contain_object[i] == 0 && (!std::isnan(data_cam.points[i].z)) && data_cam.points[i].y < height_cam){
                    left.insert(left.end(), -data_cam.points[i].x);
                    right.insert(right.end(), -data_cam.points[i].x);
                    top.insert(top.end(), -data_cam.points[i].y);
                    bottom.insert(bottom.end(), -data_cam.points[i].y);
                    depth.insert(depth.end(), data_cam.points[i].z);
                    depth_max.insert(depth_max.end(), data_cam.points[i].z);
                    left2.insert(left2.end(), i);
                    right2.insert(right2.end(), i);
                    top2.insert(top2.end(), i);
                    bottom2.insert(bottom2.end(), i);
                    int area = 0;
                    contain_object[i] = num_obj;
                    EvarobotProcessCam::recur_object(i, contain_object, num_obj, area);
                    ct += area;
                    num_obj ++;
                    if (area < 200) {
                        left.pop_back();
                        right.pop_back();
                        top.pop_back();
                        bottom.pop_back();
                        depth.pop_back();
                        depth_max.pop_back();
                        num_obj --;
                    }
                }
            }

            // Calculate the attributes of the objects and save them in the object list.
            boundb2D.clear();
            objectlists_temp.header.frame_id = "camera_depth_frame";
            objectlists_temp.sensor_id.data = 1;
            ros::WallTime time = ros::WallTime::now();
            objectlists_temp.header.stamp.sec = time.sec;
            objectlists_temp.header.stamp.nsec = time.nsec;
            for (size_t i = 0; i < num_obj - 1; i ++) {
                evarobot_adp::Object object;
                cv::Rect rec;
                object.center.x = depth[i] ;
                object.center.y = (right[i] + left[i]) / 2;
                object.center.z = (top[i] + bottom[i]) / 2;
                object.width = -(right[i] - left[i]);
                object.height = (top[i] - bottom[i]);
                object.depth_max = depth_max[i];
                object.depth_min = depth[i];
                object.top = top[i];
                object.bottom = bottom[i];
                object.left = left[i];
                object.right = right[i];
                object.fusion = "cam, ";
                float distance = std::sqrt(object.center.x * object.center.x + object.center.y * object.center.y + object.center.z * object.center.z);
                object.direction.x = object.center.x / distance;
                object.direction.y = object.center.y / distance;
                object.direction.z = object.center.z / distance;

                // Track the objects
                double dt = 1;
                if (obj_last.objects.size() != 0){
                    ros::Duration dura = objectlists_temp.header.stamp - obj_last.header.stamp;
                    dt = dura.sec + double(dura.nsec) / 1000000000;
                }
                bool find = false;
                for (size_t i = 0; i < obj_last.objects.size(); i++) {
                    double dist = (obj_last.objects[i].center.x - object.center.x) * (obj_last.objects[i].center.x - object.center.x) + (obj_last.objects[i].center.y - object.center.y) * (obj_last.objects[i].center.y - object.center.y) + (obj_last.objects[i].center.y - object.center.y) * (obj_last.objects[i].center.z - object.center.z);
                    if (dist < 0.04) {
                        object.id = obj_last.objects[i].id;
                        KalmanFilter& kf = kfilters[object.id];
                        kf.timeUpdateStep(dt);
                        kf.updateMeasurement(object.center);
                        geometry_msgs::Vector3 velocity = kf.correctionStep();
                        object.velocity.x = velocity.x + odm[0][0];
                        object.velocity.y = velocity.y + odm[0][1];
                        object.velocity.z = velocity.z + odm[0][2];
                        obj_last.objects.erase(obj_last.objects.begin() + i);
                        find = true;
                        break;
                    }
                }
                if (!find) {
                    object.id = object_id;
                    object_id ++;
                    kfilters[object.id] = KalmanFilter(object.center);
                }
                objectlists_temp.objects.insert(objectlists_temp.objects.end(), object);
                rec.x = left2[i] % 640;
                rec.y = top2[i] / 640;
                rec.width = right2[i] % 640 - left2[i] % 640;
                rec.height = bottom2[i] / 640 - top2[i] / 640;
                boundb2D.insert(boundb2D.end(), rec);
                if (object.velocity.x * object.velocity.x + object.velocity.y * object.velocity.y + object.velocity.z * object.velocity.z > 0.04){ //This Threshold of dynamic and static is 0.2 m/s, this value can be changed
                    object.dynamic = true;
                }
                else {
                    object.dynamic = false;
                }
            }
            objectlist = objectlists_temp;
            data_processed = true;
        }
        obj_last = objectlist;
        pub_depth.publish(objectlist);
    }

    // This is the recursive function
    void EvarobotProcessCam::recur_object(size_t i, std::vector<size_t>& contain, size_t& num_obj, int& area){
        area ++;
        if(-data_cam.points[i].x > left[num_obj-1]){
            left[num_obj-1] = -data_cam.points[i].x;
            left2[num_obj-1] = i;
        }
        if(-data_cam.points[i].x < right[num_obj-1]){
            right[num_obj-1] = -data_cam.points[i].x;
            right2[num_obj-1] = i;
        }
        if(-data_cam.points[i].y > top[num_obj-1]){
            top[num_obj-1] = -data_cam.points[i].y;
            top2[num_obj-1] = i;
        }
        if(-data_cam.points[i].y < bottom[num_obj-1]){
            bottom[num_obj-1] = -data_cam.points[i].y;
            bottom2[num_obj-1] = i;
        }
        if(data_cam.points[i].z > depth_max[num_obj-1]) depth_max[num_obj-1] = data_cam.points[i].z;
        if(data_cam.points[i].z < depth[num_obj-1]) depth[num_obj-1] = data_cam.points[i].z;
        if(i%640 < 639 && contain[i+1] == 0 && std::fabs(data_cam.points[i+1].z - data_cam.points[i].z) < 0.7f && data_cam.points[i+1].y < height_cam){
            contain[i+1] = num_obj;
            EvarobotProcessCam::recur_object(i+1, contain, num_obj, area);
        }
        if(i >= 640 && contain[i-640] == 0 && std::fabs(data_cam.points[i-640].z - data_cam.points[i].z) < 0.7f && data_cam.points[i-640].y < height_cam){
            contain[i-640] = num_obj;
            EvarobotProcessCam::recur_object(i-640, contain, num_obj, area);
        }
        if(i%640 > 0 && contain[i-1] == 0 && std::fabs(data_cam.points[i-1].z - data_cam.points[i].z) < 0.7f && data_cam.points[i-1].y < height_cam){
            contain[i-1] = num_obj;
            EvarobotProcessCam::recur_object(i-1, contain, num_obj, area);
        }
        if(i < 306560 && contain[i+640] == 0 && std::fabs(data_cam.points[i+640].z - data_cam.points[i].z) < 0.7f && data_cam.points[i+640].y < height_cam){
            contain[i+640] = num_obj;
            EvarobotProcessCam::recur_object(i+640, contain, num_obj, area);
        }
    }
}

#include <evarobot_adp/evarobot_process_lid.h>
#include "kalman_filter.cpp"
namespace evarobotprocess
{
    /* This Constructor define the subscribers and publishers for this process.
    * This Process contains only lidar.
    * With receiving the sensor message, the callback-function will process the original data and return the object information.
    */
    EvarobotProcessLid::EvarobotProcessLid(ros::NodeHandle& nh,std::vector<double>* data)
    {
        sub_LaserScan = nh.subscribe("/lidar", 1, &EvarobotProcessLid::pointCloudCallback, this);
        pub_lid = nh.advertise<evarobot_adp::ObjectList>( "/object_list_lid", 0 );
        odm_lidar=data;
    }

    EvarobotProcessLid::~EvarobotProcessLid()
    {
    }

    // Function to get the result.
    evarobot_adp::ObjectList EvarobotProcessLid::getObjectList()
    {
        return objectlist;
    }

    // Function to update the processed data.
    void EvarobotProcessLid::update()
    {
        if (data_processed)
        {
        }
    }

    //Callback function of lidar
    void EvarobotProcessLid::pointCloudCallback(const sensor_msgs::LaserScanPtr& msg)
    {
        size_t ct = 0;

        //Transformate Laser to PCL Pointcloud
        sensor_msgs::PointCloud2 pcloud;
        laser_geometry::LaserProjection projector_;
        projector_.projectLaser(*msg, pcloud);
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2Ptr cloudPtr(cloud);
        pcl::PCLPointCloud2 cloud_filtered;
        pcl_conversions::toPCL(pcloud, *cloud);
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloudPtr);
        sor.setLeafSize(0.02, 0.02, 0.02);
        sor.filter(cloud_filtered);
        pcl::PointCloud<pcl::PointXYZ> point_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(cloud_filtered, point_cloud);
        pcl::copyPointCloud(point_cloud, *point_cloudPtr);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(point_cloudPtr);

        //Clustering = Combine individual objects
        std::vector<pcl::PointIndices> cluster_indicies;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(1.0);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(99000000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(point_cloudPtr);
        ec.extract(cluster_indicies);
        int j = 0;
        evarobot_adp::ObjectList objectlist_temp;
        ros::WallTime time = ros::WallTime::now();
        objectlist_temp.header.stamp.sec = time.sec;
        objectlist_temp.header.stamp.nsec = time.nsec;

        //Find the Geometry of objects. Each object is a loop pass.
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indicies.begin(); it != cluster_indicies.end(); ++it)
        {

            std::vector<int>::const_iterator pit = it->indices.begin();
            float y_min = -point_cloudPtr ->points[*pit].y;
            float y_max = -point_cloudPtr ->points[*pit].y;
            float depth = -point_cloudPtr ->points[*pit].x;
            float depth_max = -point_cloudPtr ->points[*pit].x;

            //y_min = left, y_max = rechts for one object.
            //From one Object of Points on furthest links, etc;
            //Loop pass for one object
            for ( ;pit != it->indices.end(); ++pit)
            {
                ct ++;
                pcl::PointXYZRGB point;
                point.x = point_cloudPtr ->points[*pit].x;
                point.y = point_cloudPtr ->points[*pit].y;
                point.z = point_cloudPtr ->points[*pit].z;
                if (y_min > -point.y) y_min = -point.y;
                if (y_max < -point.y) y_max = -point.y;
                if (std::fabs(depth) > std::fabs(point.x)) depth = -point.x;
                if (std::fabs(depth_max) < std::fabs(point.x)) depth_max = -point.x;
            }

            // Calculation and insertion of the attribute in the object list.
            float y_center = (y_max + y_min)/2;
            float x_center = (depth + depth_max) / 2;
            evarobot_adp::Object object;
            object.center.x = x_center;
            object.center.y = y_center;
            object.center.z = 0;
            object.left = y_max;
            object.right = y_min;
            object.width = y_max - y_min;
            object.depth_min = depth;
            object.depth_max = depth_max;
            object.fusion = "lid, ";
            float length = std::sqrt(x_center * x_center + y_center * y_center);
            object.direction.x = depth / length;
            object.direction.y = y_center / length;
            object.direction.z = 0;

            //Track the objects.
            double dt = 1;
            if (last_obj.objects.size() != 0){
                ros::Duration dura = objectlist_temp.header.stamp - last_obj.header.stamp;
                dt = dura.sec + double(dura.nsec) / 1000000000;
            }
            bool find = false;
            for (size_t i = 0; i < last_obj.objects.size(); i++) {
                double dist = (last_obj.objects[i].center.x - object.center.x) * (last_obj.objects[i].center.x - object.center.x) + (last_obj.objects[i].center.y - object.center.y) * (last_obj.objects[i].center.y - object.center.y);
                if (dist < 0.04) {
                    object.id = last_obj.objects[i].id;
                    KalmanFilter& kf = kfilters[object.id];
                    kf.timeUpdateStep(dt);
                    kf.updateMeasurement(object.center);
                    geometry_msgs::Vector3 velocity = kf.correctionStep();
                    object.velocity.x = velocity.x + odm_lidar[0][0];
                    object.velocity.y = velocity.y + odm_lidar[0][1];
                    object.velocity.z = odm_lidar[0][2];
                    last_obj.objects.erase(last_obj.objects.begin() + i);
                    find = true;
                    break;
                }
            }
            if (!find) {
                object.id = object_id;
                object_id ++;
                kfilters[object.id] = KalmanFilter(object.center);
            }
            if (object.velocity.x * object.velocity.x + object.velocity.y * object.velocity.y + object.velocity.z * object.velocity.z > 0.04){
                object.dynamic = true;
            }
            else {
                object.dynamic = false;
            }
            objectlist_temp.objects.insert(objectlist_temp.objects.end(), object);
            j++;
        }
        objectlist_temp.sensor_id.data = 2;
        objectlist_temp.header.frame_id = "lidar_link";
        objectlist = objectlist_temp;
        last_obj=objectlist;
        data_processed = true;
    }
}

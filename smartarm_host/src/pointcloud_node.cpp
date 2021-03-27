#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>
#include <smartarm_msgs/Position.h>


class PointCloudSubPub
{
    public:
        PointCloudSubPub()
        {
            //Topic you want to publish
            pub_ = n_.advertise<sensor_msgs::PointCloud2>("/position_pointcloud_1", 1);
            //Topic you want to subscribe
            
            //在这里也需要改一下话题，这样才可以显示点云
            //sub_ = n_.subscribe("/fake_position", 1, &PointCloudSubPub::callback, this); //有了this指针，是不是就不用犯愁回调函数的传参问题了？
            sub_ = n_.subscribe("/end_eff_position", 1, &PointCloudSubPub::callback, this);
            //pcl init:
            cloud_.width = 1; 
	        cloud_.height = 1;
	        cloud_.points.resize(cloud_.width * cloud_.height); 
        }

        void callback(const smartarm_msgs::Position& input)
        {
            sensor_msgs::PointCloud2 output;
            //.... do something with the input and generate the output...
            cloud_.points[0].x = input.x; 
            cloud_.points[0].y = input.y; 
            cloud_.points[0].z = input.z;

            //Convert the cloud to ROS message 
            pcl::toROSMsg(cloud_, output); 
            output.header.frame_id = "base_link";
            if(input.num == 1)
            {
                pub_.publish(output);
            }
        }

    private:
        ros::NodeHandle n_; 
        ros::Publisher pub_;
        ros::Subscriber sub_;
        pcl::PointCloud<pcl::PointXYZ> cloud_; 

};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "sub_and_pub_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    PointCloudSubPub SAPObject;

    ros::spin();

    return 0;
}


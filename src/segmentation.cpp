#include <ros/ros.h>

// Point cloud message definition
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL segmentation lib and dpendencies 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// Logging capabilities
#include <ros/console.h>

// Global ros publisher variable
ros::Publisher pub;
// Define Node Logging Tag 
static const string LOG_TAG = "Segmentation Node";

// Callback for the cloud_msg topic 
void segment(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_segmented;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  
  // Create intance of a geometric model parameters/coefficients and a structure to place 
  // inliner indices in the cloud structure.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Use a plane model for the fitting algorithm, the plane models are:
  //   Its Hessian Normal coeficients: [normal_x normal_y normal_z distance_to_origin]
  seg.setModelType (pcl::SACMODEL_PLANE);
  // Use RANSAC model fitting algorithm to detect:
  //    1. Model parameters ()
  //    2. Inliner points with a maximum distance to the fitted plane model of 1 [cm] 
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  // Perform segmentation
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);  
  
  // Warn user if no inliners were detected.
  ROS_WARN_COND(inliner->indices.size() < 0, "No inliners points were detected");
  ROS_DEBUG_NAMED(LOG_TAG, "PLane Model coefficients: [ normal_x:%.3f,normal_x:%.3f,normal_x:%.3f,normal_x:%.3f ]",
                                                             coefficients->values[0] ,
                                                             coefficients->values[1] ,
                                                             coefficients->values[2] ,
                                                             coefficients->values[3]);
  ROS_DEBUG_NAMED(LOG_TAG,cloud->points[0])

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_segmented, output);
  // Publish the data
  pub.publish (output);
  ROS_DEBUG_NAMED( LOG_TAG,"Publishing downsampled cloud");
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "segmentation");
  ros::NodeHandle nh;

  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  ROS_INFO_NAMED(LOG_TAG, "Segmentation Node initiaded");
  
  // Create a ROS subscriber for the input point cloud
  // Use the previously defined callback to manage the subsctiption to the mesages.
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, segment);
  ROS_DEBUG_NAMED(LOG_TAG, "Subscribed to PointCloud2 messages");

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("robot_eyes/segmentation", 1);
  ROS_DEBUG_NAMED(LOG_TAG, "Publisher of downsampled pointcloud instanciated");
  // Spin
  ros::spin ();
}
#include <ros/ros.h>
#include <vector>
// Point cloud message definition
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL segmentation lib and dpendencies 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
// Logging capabilities
#include <ros/console.h>
// Robot_eyes messages
#include <robot_eyes/segmented_cloud.h>


// Global ros publisher instances
ros::Publisher pub_segmented;     // Publisher or cloud segemented inliers array.

/**
  Callback method from a subscriber to a PointCloud2 topic. I will mantain the structure/order
  of the input pointcloud while searching for plane components using RANSAC algorithm to fit plannar models.

  @param cloud_msg: PointCloud2 message containing the pointcloud to process 
*/
void segment(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

  // Container for original
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL PointCloud<PointXYZRGB> since the SAC segmentation API does not work with PCL::PCLPointCloud2
  // and we dont want to loose the color information.
  pcl::fromROSMsg(*cloud_msg, *cloud);
  std::vector< int > nan_points;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_points);
   // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  robot_eyes::segmented_cloud segmented_cloud_msg;
  segmented_cloud_msg.cloud = *cloud_msg;
  robot_eyes::inliers_indices indices_msg;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    indices_msg.inliers = it->indices;
    segmented_cloud_msg.segmented_inliers.push_back(indices_msg);
  }

  ROS_DEBUG("Clusters found : %d", (int) segmented_cloud_msg.segmented_inliers.size() );
  pub_segmented.publish (segmented_cloud_msg);         // Publish cloud and model inliers.
  ROS_DEBUG("Publishing segmented cloud and inliers array");
}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "euclidian_segmenter");
  ros::NodeHandle nh;

  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, segment);

  // Create a ROS publisher for the segmented_cloud
  pub_segmented = nh.advertise<robot_eyes::segmented_cloud> ("output", 1);

  // Spin
  ros::spin ();
}
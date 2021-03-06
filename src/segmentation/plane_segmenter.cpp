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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// Logging capabilities
#include <ros/console.h>
// Robot_eyes messages
#include <robot_eyes/segmented_cloud.h>


// Forward declarations
void print_model_coefficients(pcl::ModelCoefficients::Ptr);

// Global ros publisher instances
ros::Publisher pub_segmented;     // Publisher or cloud segemented inliers array.
int max_models;

/**
  Find inliers of a point cloud

  @param segmented_cloud_msg: Message contaning the input cloud and the output array of inliers to be filled.
  @param seg: SACSegmentation instance with the segementation parameters ready for execution
  @param max_models: Maximum number of models to try to find, when achieved the search for inliers will stop.
*/
template <typename PointT>
void findInliers(robot_eyes::segmented_cloud *segmented_cloud_msg, pcl::SACSegmentation<PointT> &seg, int max_models){
  // Container for original to segment.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(segmented_cloud_msg->cloud, *cloud);        

  // Holder for model inliers and coefficients.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // Create intance of a geometric model coefficients
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);                // inliner indices in the cloud structure.
  const std::vector<pcl::PointIndices> found_clusters;
  
  int model_found = 1;
  int num_points = (int) cloud->points.size();
  int remaining_points = num_points;
  while (remaining_points > 0.3 * num_points){
    // Segment the largest plannar component from the remaining cloud
    seg.setInputCloud (cloud);                     
    seg.segment (*inliers, *coefficients);
    //Stop when no plannar component is found.
    int num_inliers = (int)inliers->indices.size();   //Get number of inliers points
    if (num_inliers == 0){           
      ROS_INFO("Plannar segmentation ended, %d plannar components found in cloud", model_found);
      break;
    }
    ROS_DEBUG("Plane model found: -id: %d -inliers: %d", model_found, num_inliers);
    print_model_coefficients(coefficients);
    // Save plane inliers for publishing.
    // segmented_cloud_msg->clusters.push_back(*inliers);
    // Remove the planar inliers, extract the rest.
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setKeepOrganized(true);                    // Replace inliners for NaN (not chanching the cloud structure)
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
    model_found++;
    remaining_points -= num_inliers;                   // Account for inliers points.
    inliers->indices.clear();
    // Stop if max number of models was defined.
    if ( model_found > max_models && max_models != -1 ){
      ROS_DEBUG("Max number of models found");
      break;    
    }
  }
}

/**
  Callback method from a subscriber to a PointCloud2 topic. I will mantain the structure/order
  of the input pointcloud while searching for plane components using RANSAC algorithm to fit plannar models.

  @param cloud_msg: PointCloud2 message containing the pointcloud to process 
*/
void segment(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

  // Msg containing an array of inliers for each plane found
  robot_eyes::segmented_cloud segmented_cloud_msg;
  segmented_cloud_msg.cloud = *cloud_msg;

  // SEGMENTATION Configuration***************************************************
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // Create intance of a geometric model coefficients
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);                // inliner indices in the cloud structure.
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;     // Create the segmentation object
  seg.setModelType (pcl::SACMODEL_PLANE);         // Use a plannar model to fit
    // Use RANSAC model fitting algorithm to detect:
    //    1. Model parameters ()
    //    2. Inliner points with a maximum distance to the fitted plane model of 1 [cm]
  seg.setOptimizeCoefficients (true);             // Optional  
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (500);
  seg.setDistanceThreshold (0.01);                // Distance to model to consider a point inliner

  // Find all plannar models in input cloud.
  findInliers( &segmented_cloud_msg , seg, max_models);

  ROS_DEBUG("Clusters found : %d", (int) segmented_cloud_msg.clusters.size() );
  pub_segmented.publish (segmented_cloud_msg);         // Publish cloud and model inliers.
  ROS_DEBUG("Publishing segmented cloud and inliers array");
}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "plane_segmenter");
  ros::NodeHandle nh;

  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  // Read Ros parameter indicating the number of plane components to search for.
  ros::param::param<int>("/plane_segmenter/max_models", max_models, -1);
  if(max_models <= 0) 
    throw std::invalid_argument("`max_models` parameter should be greather than 0");
  else 
    ROS_INFO("Plannar Segmentator: Searching for %d plane components", max_models);
  
  // // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, segment);

  // Create a ROS publisher for the segmented_cloud
  pub_segmented = nh.advertise<robot_eyes::segmented_cloud> ("output", 1);

  // Spin
  ros::spin ();
}

/**
  Small function for printing the plane model paramters found
   
  @param: coefficients: Coefficients array of the model type.
*/
void print_model_coefficients(pcl::ModelCoefficients::Ptr coefficients){
  ROS_DEBUG("Plane Model coefficients: [x:%.3f, y:%.3f, z:%.3f, d:%.3f]",
                                                             coefficients->values[0] , coefficients->values[1] ,
                                                             coefficients->values[2] , coefficients->values[3]);
}





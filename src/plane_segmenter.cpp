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
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// Logging capabilities
#include <ros/console.h>

#include <vector>

//Forward declarations
void print_model_coefficients(pcl::ModelCoefficients::Ptr);
void color_cloud(pcl::PointIndices::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

// Global ros publisher instances
ros::Publisher pub;


// Callback for the cloud_msg topic 
void segment(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  ROS_DEBUG("Plane_Segmenter: Processing new cloud...");
  // Container for original to segment.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Vector containing an array of inliers for each plane found
  std::vector<pcl::PointIndices> plane_models_inliers;

  // Convert to PCL PointCloud<PointXYZRGB> since the SAC segmentation API does not work with PCL::PCLPointCloud2
  // and we dont want to loose the color information.
  pcl::fromROSMsg(*cloud_msg, *cloud);                  //Cloud to segement 
  pcl::fromROSMsg(*cloud_msg, *cloud_segmented);        //Cloud to color (just for visualization)

  // SEGMENTATION (Configuration)***************************************************
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // Create intance of a geometric model coefficients
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);                // inliner indices in the cloud structure.
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;     // Create the segmentation object
  seg.setOptimizeCoefficients (true);             // Optional 
  seg.setModelType (pcl::SACMODEL_PLANE);         // Use a plannar model to fit
    // Use RANSAC model fitting algorithm to detect:
    //    1. Model parameters ()
    //    2. Inliner points with a maximum distance to the fitted plane model of 1 [cm] 
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);                // Distance to model to consider a point inliner

  // Find all plannar models in input cloud.
  int model_found = 1;
  int num_points = (int) cloud->points.size();
  int remaining_points = num_points;
  while (remaining_points > 0.3 * num_points){
    // Segment the largest plannar component from the remaining cloud
    seg.setInputCloud (cloud_segmented);                     
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
    plane_models_inliers.push_back(*inliers);
    // Remove the planar inliers, extract the rest.
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setKeepOrganized(true);                   // Replace inliners for NaN (not chanching the cloud structure)
    extract.setInputCloud(cloud_segmented);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_segmented);
    model_found++;
    remaining_points -= num_inliers;                  // Account for inliers points.
    inliers->indices.clear();
    // Color original cloud for visualization purposes. (Should be removed when running for realtime operation)
    color_cloud(inliers,cloud);
    break;
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  // Publish the data
  pub.publish (output);
  ROS_DEBUG("Publishing segmented cloud");
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "simple_plane_segmentation");
  ros::NodeHandle nh;

  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  ROS_INFO("Plane segmentation Node initiaded");
  
  // Create a ROS subscriber for the input point cloud
  // Use the previously defined callback to manage the subsctiption to the mesages.
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, segment);
  ROS_DEBUG("Subscribed to PointCloud2 messages");

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ROS_DEBUG("Publisher of segmented pointcloud");

  // Spin
  ros::spin ();
}

void print_model_coefficients(pcl::ModelCoefficients::Ptr coefficients){
  ROS_DEBUG("Plane Model coefficients: [x:%.3f, y:%.3f, z:%.3f, d:%.3f]",
                                                             coefficients->values[0] , coefficients->values[1] ,
                                                             coefficients->values[2] , coefficients->values[3]);
}

void color_cloud(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  // Color all inliers of the detected plane.
  for(int inlier_point = 0; inlier_point < inliers->indices.size(); inlier_point++){
    // Create RGB color code.
    uint32_t rgb = (static_cast<uint32_t>(rand() % 255) | static_cast<uint32_t>(rand() % 255) | static_cast<uint32_t>(rand() % 255));
    // Color point.
    cloud->points[inliers->indices[inlier_point]].rgb = *reinterpret_cast<float*>(&rgb);
  }
}
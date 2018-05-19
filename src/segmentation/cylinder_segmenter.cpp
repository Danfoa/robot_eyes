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
#include <pcl/features/normal_3d.h>
// Logging capabilities
#include <ros/console.h>
// Robot_eyes messages
#include <robot_eyes/segmented_cloud.h>


// Forward declarations
void print_model_coefficients(pcl::ModelCoefficients::Ptr);

// Global ros publisher instances
ros::Publisher pub_segmented;     // Publisher or cloud segemented inliers array.
int max_models;

int findInliers(robot_eyes::segmented_cloud *segmented_cloud_msg, int max_models){
  // Container for original to segment.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(segmented_cloud_msg->cloud, *cloud); 

  // Estimate point normals
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;    // Create the normal estimation class, and pass the input dataset to it       
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (kd_tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);  // Output Normals
  ne.setRadiusSearch (0.03);                                  // Use all neighbors in a sphere of radius 3cm

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;    // Create the segmentation object
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.3);
  seg.setMaxIterations (5000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.0, 0.5);                                        // Limit Radius to find Coke.

  // Holder for model inliers and coefficients.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // Create intance of a geometric model coefficients
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);                // inliner indices in the cloud structure.
  
  int model_found = 1;
  int num_points = (int) cloud->points.size();
  int remaining_points = num_points;
  while (remaining_points > 0.3 * num_points){
    ne.setInputCloud (cloud);                         // Set normals estimatro input cloud
    ne.compute (*cloud_normals);                      // Compute the normals of the remaining cloud
    seg.setInputCloud (cloud);              
    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers, *coefficients);  // Segment largest cylinder component 
    //Stop when no cylinder component is found.
    int num_inliers = (int)inliers->indices.size();   //Get number of inliers points
    if (num_inliers == 0){           
      ROS_WARN("No more models detected, %d cylinders components found in cloud", model_found);
      break;
    }
    ROS_DEBUG("Cylinder model found: -id: %d -inliers: %d", model_found, num_inliers);
    print_model_coefficients(coefficients);
    // Save Cylinder inliers for publishing.
    robot_eyes::inliers_indices indices_msg;
    indices_msg.inliers = inliers->indices;
    // indices_msg.cluster_id = MODEL_TYPE + std::to_string(model_found);
    segmented_cloud_msg->segmented_inliers.push_back(indices_msg);
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



/* post_segment: This method is the callback method from a subscriber to an already segemented cloud. It will mantain the structure of the cloud and attached
                 the new found models to the segmented_cloud msg inliers array.
*/
void pos_segment(const robot_eyes::segmented_cloud segmented_cloud){

  // Msg containing an array of inliers for each cylinder found
  robot_eyes::segmented_cloud segmented_cloud_msg = segmented_cloud;
  ROS_DEBUG("Post segmenting: Finding cylinder components");

  // // Remove the points already labeled as clusters by past segmentator nodes.
  // std::vector<robot_eyes::inliers_indices> inliers = segmented_cloud.segmented_inliers;
  // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // extract.setKeepOrganized(true); 
  // extract.setNegative(true);
  // extract.setInputCloud(cloud);
  // for(std::vector<robot_eyes::inliers_indices>::iterator cluster = inliers.begin(); cluster != inliers.end(); cluster++){
  //   extract.setIndices(cluster->inliers);
  //   extract.filter(*segmented_cloud_msg.cloud);
  // }
  // Find all cylinder models in input cloud.
  findInliers( &segmented_cloud_msg , max_models);
  
  pub_segmented.publish (segmented_cloud_msg);         // Publish cloud and model inliers.
  ROS_DEBUG("Publishing segmented cloud and %d cluster inliers", (int) segmented_cloud_msg.segmented_inliers.size() );
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segmenter");
  ros::NodeHandle nh;

  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  
  // Read Ros parameter indicating the number of cylinder components to search for.
  ros::param::param<int>("/cylinder_segmenter/max_models", max_models, -1);
  if(max_models <= 0) 
    throw std::invalid_argument("`/cylinder_segmenter/max_models` parameter should be greather than 0");
  else 
    ROS_INFO("Cylinder Segmentator: Searching for %d cylinder components", max_models);
  
  // // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, segment);
  // Create a ROS subscriber for the input point cloud that is already segmented.
  ros::Subscriber sub2 = nh.subscribe<robot_eyes::segmented_cloud> ("input_segmented", 1, pos_segment);

  // Create a ROS publisher for the segmented_cloud
  pub_segmented = nh.advertise<robot_eyes::segmented_cloud> ("output", 1);

  // Spin
  ros::spin ();
}

/* Small function for printing the plane model paramters found */
void print_model_coefficients(pcl::ModelCoefficients::Ptr coefficients){
  ROS_DEBUG("Cylinder Model coefficients: [x:%.3f, y:%.3f, z:%.3f, dx:%.3f, dy:%.3f, dz:%.3f, r:%.3f]",
                                                             coefficients->values[0] , coefficients->values[1],
                                                             coefficients->values[2] , coefficients->values[3],
                                                             coefficients->values[4] , coefficients->values[5],
                                                             coefficients->values[6] );
}





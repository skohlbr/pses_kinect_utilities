/**
 * @file "voxel_grid_filter.cpp"
 * @brief Nodelet implementation of a voxel grid filter, containing the callback functions.
 *
*/

#include <pses_kinect_utilities/voxel_grid_filter.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>


// Register as nodelet
PLUGINLIB_EXPORT_CLASS(pses_kinect_utilities::VoxelGridFilterNodelet,
                       nodelet::Nodelet);

typedef pcl::PointXYZ PointXYZ;

namespace pses_kinect_utilities
{

void VoxelGridFilterNodelet::onInit()
{
  NODELET_DEBUG("Initializing voxel grid filter nodelet...");

  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  nh_ = getNodeHandle();

  // Init dynamic reconfigure
  reconfigureServer = boost::make_shared <dynamic_reconfigure::Server<VoxelGridFilterConfig>> (private_nh);
  dynamic_reconfigure::Server<VoxelGridFilterConfig>::CallbackType f;
  f = boost::bind(&VoxelGridFilterNodelet::dynReconfCb, this, _1, _2);
  reconfigureServer->setCallback(f);

  filtered_cloud_.reset(new PointCloud);

  // Read parameters
  private_nh.param("queue_size", queue_size_, 1);
  //private_nh.param<std::string>("tf_frame", tf_frame_, "kinect2_link");

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&VoxelGridFilterNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_cloud_ =
      nh_.advertise<PointCloud>("cloud_out", 1, connect_cb, connect_cb);
}

void VoxelGridFilterNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_cloud_.getNumSubscribers() == 0)
  {
    NODELET_INFO("Stopping voxel grid filter...");
    sub_cloud_.shutdown();
  }
  else if (!sub_cloud_)
  {
    NODELET_INFO("Running voxel grid filter...");
    sub_cloud_ = nh_.subscribe<PointCloud>(
        "cloud_in", queue_size_,
        boost::bind(&VoxelGridFilterNodelet::pointCloudCb, this, _1));
  }
}

void VoxelGridFilterNodelet::pointCloudCb(const PointCloud::ConstPtr& cloud_msg)
{  
  vox_.setInputCloud(cloud_msg);
  vox_.filter(*filtered_cloud_);
  filtered_cloud_->header = cloud_msg->header;
  pub_cloud_.publish(filtered_cloud_);
}

void VoxelGridFilterNodelet::dynReconfCb(
    VoxelGridFilterConfig& inputConfig, uint32_t level)
{
  config_ = inputConfig;
  
  vox_.setLeafSize(config_.leaf_size, config_.leaf_size, config_.leaf_size);
  vox_.setMinimumPointsNumberPerVoxel(config_.min_points_per_voxel);
  
  NODELET_INFO("Reconfigured voxel grid filter params");
}

} // namespace pses_kinect_utilities

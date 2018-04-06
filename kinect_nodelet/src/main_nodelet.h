#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>


namespace kinect_nodelet
{
    using namespace message_filters::sync_policies;
    namespace enc = sensor_msgs::image_encodings;


class PointCloudXyzrgbNodelet : public nodelet::Nodelet
    {
      ros::NodeHandlePtr rgb_nh_;
      boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
      
      // Subscriptions
      image_transport::SubscriberFilter sub_depth_, sub_rgb_;
      message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
      typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
      typedef ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactSyncPolicy;
      typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
      typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
      boost::shared_ptr<Synchronizer> sync_;
      boost::shared_ptr<ExactSynchronizer> exact_sync_;
    
      // Publications
      boost::mutex connect_mutex_;
      typedef sensor_msgs::PointCloud2 PointCloud;
      ros::Publisher pub_point_cloud_;
    
      image_geometry::PinholeCameraModel model_;
    
      virtual void onInit();
    
      void connectCb();
    
      void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                   const sensor_msgs::ImageConstPtr& rgb_msg,
                   const sensor_msgs::CameraInfoConstPtr& info_msg);
    
      template<typename T>
      void convert(const sensor_msgs::ImageConstPtr& depth_msg,
                   const sensor_msgs::ImageConstPtr& rgb_msg,
                   const PointCloud::Ptr& cloud_msg,
                   int red_offset, int green_offset, int blue_offset, int color_step);
    };



} // namespace sample_nodelet_ns



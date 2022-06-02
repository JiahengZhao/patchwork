#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "patchwork/patchwork.hpp"
#include "patchwork/utils.hpp"


namespace patchwork {

using PointType = pcl::PointXYZI;

class PatchworkNodelet : public nodelet::Nodelet {
 public:
  using PointType = pcl::PointXYZI;

  PatchworkNodelet() {}
  virtual ~PatchworkNodelet() {}

  void onInit() override {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    groundPublisher =
        nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_ground", 100);
    noGroundPublisher =
        nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_no_ground", 100);
    fewGroundPublisher =
        nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_few_ground", 100);

    pointsSubscriber = nh.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_points", 100, &PatchworkNodelet::callbackNode, this);

    skip_num = private_nh.param<int>("/skip_num", 15);

    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));
  }

  void callbackNode(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!PatchworkGroundSeg) {
      NODELET_WARN("PatchworkGroundSeg is not initialised...");
      return;
    }
    if (!msg || msg == NULL) {
      NODELET_ERROR("NO lidar messages");
      return;
    }

    std::string frame_id = msg->header.frame_id;
    pcl::PointCloud<PointType> pc_curr = cloudmsg2cloud<PointType>(msg);

    if (pc_curr.empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    pcl::PointCloud<PointType>::Ptr pc_ground(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pc_non_ground(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pc_few_ground(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pc_output(new pcl::PointCloud<PointType>());

    static double time_taken;
    std::cout << __LINE__ << std::endl;
    PatchworkGroundSeg->estimate_ground(pc_curr, *pc_ground, *pc_non_ground,
                                        time_taken);
    std::cout << __LINE__ << std::endl;

    if (groundPublisher.getNumSubscribers() > 0) {
      auto msg_ground = cloud2msg(*pc_ground, frame_id);
      groundPublisher.publish(msg_ground);
    }

    if (noGroundPublisher.getNumSubscribers() > 0) {
      noGroundPublisher.publish(cloud2msg(*pc_non_ground, frame_id));
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointType> extract;
    for (int i = 0; i < pc_ground->size(); i += skip_num) {
      inliers->indices.push_back(i);
    }
    extract.setInputCloud(pc_ground);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*pc_few_ground);
    *pc_output = *pc_non_ground + *pc_few_ground;
    auto msg_few = cloud2msg(*pc_output, frame_id);
    msg_few.header.stamp = msg->header.stamp;
    fewGroundPublisher.publish(msg_few);
  }

  template <typename T>
  pcl::PointCloud<T> cloudmsg2cloud(
      const sensor_msgs::PointCloud2::ConstPtr& cloudmsg) {
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(*cloudmsg, cloudresult);
    return cloudresult;
  }

  template <typename T>
  sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud,
                                     std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
  }

 private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher groundPublisher;
  ros::Publisher noGroundPublisher;
  ros::Publisher fewGroundPublisher;
  ros::Subscriber pointsSubscriber;

  std::unique_ptr<PatchWork<PointType> > PatchworkGroundSeg;

  int skip_num;

};  // class PatchworkNodelet

}  // namespace patchwork

PLUGINLIB_EXPORT_CLASS(patchwork::PatchworkNodelet, nodelet::Nodelet)

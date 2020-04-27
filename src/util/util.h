//
// Created LI Zhiqiang(zqlee@ronghui.tech) on 2020/4/26.
//

#ifndef SIM_SRC_UTIL_UTIL_H_
#define SIM_SRC_UTIL_UTIL_H_
#include <string>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "util/line.h"
namespace lrobot {
namespace sim {
class Util {
 public:
  static void GenerateGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  static void GenerateModle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  static void Get2DObs(const pcl::PointCloud<pcl::PointXYZ>::Ptr world, pcl::PointCloud<pcl::PointXYZ>::Ptr obs);
  static void RemoveGround(pcl::PointCloud<pcl::PointXYZ>::Ptr obs);
  static double GetK(const pcl::PointCloud<pcl::PointXYZ>::Ptr obs);
  static std::vector<Line> GetLineSegement(const pcl::PointCloud<pcl::PointXYZ>::Ptr obs);

  static boost::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudVisualizer(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name);
};
} //namespace sim
} //namespace lrobot
#endif //SIM_SRC_UTIL_UTIL_H_

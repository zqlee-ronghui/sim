//
// Created LI Zhiqiang(zqlee@ronghui.tech) on 2020/4/27.
//

#ifndef SIM_SRC_UTIL_LINE_H_
#define SIM_SRC_UTIL_LINE_H_
#include "Eigen/Eigen"
#include "pcl/point_types.h"
namespace lrobot {
namespace sim {
class Line {
 public:
  Line(pcl::PointXYZ point1, pcl::PointXYZ point2);
  ~Line() = default;

  void AddPoint(const pcl::PointXYZ p);
  bool IsSimilar(const Line& line) const;
  void Merge(const Line& line);
  pcl::PointXYZ GetIntersectPoint(const Line& line);

  double theta() const;
  double length() const;
  pcl::PointXYZ start() const;
  pcl::PointXYZ end() const;
 private:
  void Update();

  double theta_;
  double length_;
  pcl::PointXYZ start_;
  pcl::PointXYZ end_;
};
} //namespace sim
} //nmaespace lrobot
#endif //SIM_SRC_UTIL_LINE_H_

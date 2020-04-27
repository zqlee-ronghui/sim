//
// Created LI Zhiqiang(zqlee@ronghui.tech) on 2020/4/27.
//

#include "util/line.h"

namespace lrobot {
namespace sim {
Line::Line(pcl::PointXYZ point1, pcl::PointXYZ point2)
    : start_(1e5, 1e5, 0), end_(-1e5, -1e5, 0) {
  AddPoint(point1);
  AddPoint(point2);
}

void Line::AddPoint(const pcl::PointXYZ p) {
  if(p.y + 1e-3 < start_.y) {
    start_ = p;
  } else if(fabs(p.y - start_.y) < 1e-3) {
    if(p.x + 1e-3 < start_.x) {
      start_ = p;
    }
  }
  if(p.y - 1e-3 > end_.y) {
    end_ = p;
  } else if(fabs(p.y - end_.y) < 1e-3) {
    if(p.x - 1e-3 > end_.x) {
      end_ = p;
    }
  }
  Update();
}

bool Line::IsSimilar(const lrobot::sim::Line &line) const {
  double delta_theta = fabs(theta_ - line.theta());
  return delta_theta < 0.01;
}

void Line::Merge(const lrobot::sim::Line &line) {
  AddPoint(line.start());
  AddPoint(line.end());
}

pcl::PointXYZ Line::GetIntersectPoint(const lrobot::sim::Line &line) {
  double a1 = end_.y - start_.y;
  double b1 = start_.x - end_.x;
  double c1 = end_.x * start_.y - start_.x * end_.y;
  double a2 = line.end().y - line.start().y;
  double b2 = line.start().x - line.end().x;
  double c2 = line.end().x * line.start().y - line.start().x * line.end().y;
  double m = a1 * b2 - a2 * b1;
  if(m == 0) {
    return pcl::PointXYZ(
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity());
  } else {
    return pcl::PointXYZ((c2 * b1 - c1 * b2) / m, (c1 * a2 - c2 * a1) / m, 0);
  }
}

double Line::theta() const {
  return theta_;
}

double Line::length() const {
  return length_;
}

pcl::PointXYZ Line::start() const {
  return start_;
}

pcl::PointXYZ Line::end() const {
  return end_;
}

void Line::Update() {
  theta_ = atan2(end_.y - start_.y, end_.x - start_.x);
  length_ = sqrt((end_.x - start_.x) * (end_.x - start_.x) + (end_.y - start_.y) * (end_.y - start_.y));
}
} //namespace sim
} //namespace lrobot
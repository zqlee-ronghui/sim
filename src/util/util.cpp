//
// Created LI Zhiqiang(zqlee@ronghui.tech) on 2020/4/26.
//
#include "util/util.h"
namespace lrobot {
namespace sim {
void Util::GenerateGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  //CHECK(cloud);//todo
  cloud->clear();
  for(float x = -5; x < 5; x += 0.01) {
    for(float y = -5; y < 5; y += 0.01) {
      cloud->points.emplace_back(pcl::PointXYZ{x, y, 0});
    }
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
}

void Util::GenerateModle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  cloud->clear();
  Eigen::Vector2f bottom_left(0, -1);
  Eigen::Vector2f top_right(5, 1);

  for(float x = bottom_left(0); x < top_right(0); x += 0.01) {
    for(float z = 0; z < 0.05; z += 0.01) {
      cloud->points.emplace_back(pcl::PointXYZ{x, bottom_left(1), z});
      cloud->points.emplace_back(pcl::PointXYZ{x, top_right(1), z});
    }
  }
  for(float y = bottom_left(1); y < top_right(1); y += 0.01) {
    for(float z = 0; z < 0.05; z += 0.01) {
      cloud->points.emplace_back(pcl::PointXYZ{bottom_left(0), y, z});
      cloud->points.emplace_back(pcl::PointXYZ{top_right(0), y, z});
    }
  }

  for(float x = bottom_left(0); x < top_right(0); x += 0.01) {
    for(float y = bottom_left(1); y < top_right(1); y += 0.01) {
      cloud->points.emplace_back(pcl::PointXYZ{x, y, 0.05});
    }
  }

  for(float y = bottom_left(1); y < top_right(1); y += 0.01) {
    for(float z = 0.06; z < 1; z += 0.01) {
      cloud->points.emplace_back(pcl::PointXYZ{bottom_left(0), y, z});
    }
  }

  for(float x = bottom_left(0); x < top_right(0); x += 0.01) {
    for(float z = 0.06; z < 1; z += 0.01) {
      cloud->points.emplace_back(pcl::PointXYZ{x, (bottom_left(1) + top_right(1)) * 0.5f, z});
    }
  }

  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
}

void Util::Get2DObs(const pcl::PointCloud<pcl::PointXYZ>::Ptr world, pcl::PointCloud<pcl::PointXYZ>::Ptr obs) {
  obs->clear();
  double step = 0.36 * M_PI / 180.0;
  std::map<int, std::map<double, pcl::PointXYZ>> origanised;
  for (auto p : world->points) {
    if (fabs(p.z) < 0.005) {
      double r = sqrt(p.x * p.x + p.y * p.y);
      int theta = atan2(p.y, p.x) / step;
      origanised[theta][r] = p;
    }
  }
  if(origanised.size() > 0) {
    for(auto p : origanised) {
      obs->points.emplace_back(p.second.begin()->second);
    }
  }
  obs->height = 1;
  obs->width = obs->points.size();
  obs->is_dense = false;
}

void Util::RemoveGround(pcl::PointCloud<pcl::PointXYZ>::Ptr obs) {
//  std::vector<int> indices;
//  for(int i = 0; i < obs->points.size(); ++i) {
//    if(obs->points[i].x > 5.95) {//todo
//      indices.emplace_back(i);
//    }
//  }
  for(auto iter = obs->points.begin(); iter != obs->points.end();) {
    if(iter->x > 5.95) {//todo
      iter = obs->points.erase(iter);
    } else {
      ++iter;
    }
  }
  obs->height = 1;
  obs->width = obs->points.size();
  obs->is_dense = false;
}

double Util::GetK(const pcl::PointCloud<pcl::PointXYZ>::Ptr obs) {
  //CHECK(obs->size() > 2);
  Eigen::MatrixXd X(obs->size(), 2);
  Eigen::VectorXd y(obs->size());
  for(int i = 0; i < obs->size(); ++i) {
    X(i, 0) = obs->points[i].x;
    X(i, 1) = 1.0;
    y(i) = obs->points[i].y;
  }
  auto w = (X.transpose() * X).inverse() * (X.transpose() * y);
  return w(0);
}

std::vector<Line> Util::GetLineSegement(const pcl::PointCloud<pcl::PointXYZ>::Ptr obs) {
  std::vector<Line> lines;
  for(int i = 1; i < obs->size(); ++i) {
    lines.emplace_back(Line(obs->points[i-1], obs->points[i]));
  }
  for(auto iter = (++lines.begin()); iter != lines.end();) {
    auto preline = iter - 1;
    if(iter->length() > 0.5) {
      iter = lines.erase(iter);
    } else {
      if(preline->IsSimilar(*iter)) {
        preline->Merge(*iter);
        iter = lines.erase(iter);
      } else {
        ++iter;
      }
    }
  }
  for(auto iter = lines.begin(); iter != lines.end();) {
    if(iter->length() < 0.15) {
      iter = lines.erase(iter);
    } else {
      ++iter;
    }
  }
  return lines;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Util::PointCloudVisualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                                const std::string &name) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}
} //namespace sim
} //namespace lrobot
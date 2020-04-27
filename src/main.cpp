#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "util/util.h"

#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"

#define Deg2Rad(x) (x * M_PI / 180.0)
#define Rad2Deg(x) (x * 180.0 / M_PI)

enum SIMTYPE {
  A,//horizon align
  B,
  C,
};

void test_a() {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);
  std::normal_distribution<double> dis(0, 10);
  double inclination = dis(gen);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>());
  lrobot::sim::Util::GenerateGround(ground);
  Eigen::AngleAxisd pitch(M_PI / 2.0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd roll(inclination * M_PI / 180.0, Eigen::Vector3d::UnitX());

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d{0, 0, 6});
  T.rotate(roll * pitch);

  pcl::PointCloud<pcl::PointXYZ>::Ptr threed_obs(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*ground, *threed_obs, T.inverse().matrix());

  pcl::PointCloud<pcl::PointXYZ>::Ptr obs(new pcl::PointCloud<pcl::PointXYZ>());
  lrobot::sim::Util::Get2DObs(threed_obs, obs);

  std::cout << "inclination: " << inclination << std::endl;
  std::cout << "theta: " << Rad2Deg(M_PI * 0.5 - atan(lrobot::sim::Util::GetK(obs))) << std::endl;
}

void test_b() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr world(new pcl::PointCloud<pcl::PointXYZ>());

  lrobot::sim::Util::GenerateGround(ground);
  lrobot::sim::Util::GenerateModle(model);

//  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ground(new pcl::PointCloud<pcl::PointXYZ>());
//  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud(ground);
//  sor.setLeafSize(0.05f, 0.05f, 0.05f);
//  sor.filter(*filtered_ground);

  *world += *ground;
  *world += *model;

  double inclination = -10;
  lrobot::sim::Util::GenerateGround(ground);
  Eigen::AngleAxisd pitch_rotation(Deg2Rad(inclination) + M_PI / 2.0, Eigen::Vector3d::UnitY());

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d{1, -3, 6});
  T.rotate(pitch_rotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr threed_obs(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*world, *threed_obs, T.inverse().matrix());

  pcl::PointCloud<pcl::PointXYZ>::Ptr obs(new pcl::PointCloud<pcl::PointXYZ>());
  lrobot::sim::Util::Get2DObs(threed_obs, obs);
  //lrobot::sim::Util::RemoveGround(obs);

  auto lines = lrobot::sim::Util::GetLineSegement(obs);

  auto p = lines[1].GetIntersectPoint(lines[2]);
  lines[1].AddPoint(p);
  lines[2].AddPoint(p);

  //todo
  lrobot::sim::Line horizon_line = lines[1];
  lrobot::sim::Line vertical_line = lines[2];

  double yaw = acos(1.0 / horizon_line.length());
  double pitch = acos(0.95 / vertical_line.length());

  std::cout << "inclination: " << inclination << std::endl;
  std::cout << "pitch: " << Rad2Deg(pitch) << std::endl;

  Eigen::AngleAxisd pitch_test1(pitch + M_PI / 2.0, Eigen::Vector3d::UnitY());
  Eigen::Isometry3d T_test1 = Eigen::Isometry3d::Identity();
  T_test1.rotate(pitch_test1);

  Eigen::AngleAxisd pitch_test2(-pitch + M_PI / 2.0, Eigen::Vector3d::UnitY());
  Eigen::Isometry3d T_test2 = Eigen::Isometry3d::Identity();
  T_test2.rotate(pitch_test2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr test(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr obs_test1(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr obs_test2(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::transformPointCloud(*obs, *obs_test1, T_test1.matrix());
  pcl::transformPointCloud(*obs, *obs_test2, T_test2.matrix());

  *test += *obs_test1;
  *test += *obs_test2;

  auto viewer = lrobot::sim::Util::PointCloudVisualizer(test, "ground");
  while(!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}

void test_c() {

}

int main(int argc, char** argv) {
//  SIMTYPE type = SIMTYPE::A; //horizon align
  SIMTYPE type = SIMTYPE::B;
  switch(type) {
    case A:
      test_a();
      break;
    case B:
      test_b();
      break;
    case C:
      test_c();
      break;
    default: break;
  }
  return 0;
}
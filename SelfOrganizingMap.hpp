#ifndef SELFORGANIZINGMAP_H
#define SELFORGANIZINGMAP_H 1

#include <vector>
#include <random>
#include <limits>
#include <algorithm>

#include <thread>
#include <mutex>
#include <atomic>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <Eigen/Dense>

#include <unordered_map>

#include "LoadingBar.hpp"


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef Eigen::Vector3f point;
typedef Eigen::Vector2i node_index;
typedef bg::model::point<float, 3, bg::cs::cartesian> rtree_point;

const point origin{0,0,0};

class SelfOrganizingMap {
 public:
  SelfOrganizingMap(int size);
  ~SelfOrganizingMap();

  node_index closest_node(point const& input);
  void train_one_sample(point const& input);
  void train(std::vector<point> const& inputs);
  point** winners(std::vector<point> const& inputs);

  point& node(int i, int j);

  void setIter(int iter);
  void setThreads(int threads);
  void setNu(float nu);
  void setFinalNu(float final_nu);
  void setSigma(float sigma);
  void setFinalSigma(float final_sigma);


 private:
  point** colormap_;
  float** bias_;
  int size_;
  int num_iter_;
  int threads_;
  float nu_;
  float final_nu_;
  float sigma_;
  float final_sigma_;
  LoadingBar loading_bar_;
};


struct rtree_point_hash{
  size_t operator()(const rtree_point &pt) const;
};

struct rtree_point_eq{
  bool operator()(const rtree_point &pt1, const rtree_point &pt2) const;
};

#endif 
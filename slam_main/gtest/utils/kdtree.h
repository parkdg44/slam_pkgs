#pragma once

#include <nanoflann/nanoflann.hpp>
#include <vector>

template <typename T>
struct PointCloud {
  struct Point {
    T x, y, z;
  };

  std::vector<Point> pts;

  inline size_t kdtree_get_point_count() const { return pts.size(); }

  inline T kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[idx].x;
    else if (dim == 1)
      return pts[idx].y;
    else
      return pts[idx].z;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
  }
};

template <typename T>
class KDTree {
 public:
  KDTree(const PointCloud<T>& cloud)
      : m_cloud(cloud), m_index(3, m_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10)) {
    m_index.buildIndex();
  }

  std::pair<size_t, T> findNearest(const T* query_point) const {
    const size_t num_results = 1;
    size_t ret_index;
    T out_dist_sqr;
    nanoflann::KNNResultSet<T> resultSet(num_results);
    resultSet.init(&ret_index, &out_dist_sqr);
    m_index.findNeighbors(resultSet, query_point, nanoflann::SearchParameters(0));
    return std::make_pair(ret_index, out_dist_sqr);
  }

  std::vector<std::pair<size_t, T>> findKNearest(const T* query_point, size_t k) const {
    std::vector<size_t> ret_indexes(k);
    std::vector<T> out_dists_sq(k);
    nanoflann::KNNResultSet<T> resultSet(k);
    resultSet.init(&ret_indexes[0], &out_dists_sq[0]);
    m_index.findNeighbors(resultSet, query_point, nanoflann::SearchParameters(0));

    std::vector<std::pair<size_t, T>> ret_matches;
    ret_matches.reserve(k);
    for (size_t i = 0; i < k; ++i) {
      ret_matches.emplace_back(ret_indexes[i], out_dists_sq[i]);
    }
    return ret_matches;
  }

 private:
  const PointCloud<T>& m_cloud;
  nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<T, PointCloud<T>>, PointCloud<T>,
                                      3>
      m_index;
};

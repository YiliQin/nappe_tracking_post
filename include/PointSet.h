#pragma once

#include <string>

#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>

namespace gen_point_set
{

  class PointSet
  {
  public:
    /*  Generate point set.
     * 
     * \param type Type of point set.
     *
     */
    PointSet(int _p_num_row, int _p_num_col);

    /* Get one point for the set.
     *
     * \return A point.
     */
    Eigen::Vector3d get_point();

    /*  Generate point set. 
     *
     */
    void gen_point_set();

    int rows();
    int cols();
    int p_num();

    Eigen::MatrixXd pointSet;

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();

  private:  
    int rows_;
    int cols_;
    int p_num_row_;
    int p_num_col_;
    int p_num_;
    Eigen::Vector3d center_;
  };

}

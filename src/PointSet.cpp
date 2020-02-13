#include "PointSet.h"

namespace gen_point_set
{

  PointSet::PointSet(int _p_num_row, int _p_num_col)
  {
    pointSet.resize(_p_num_row * _p_num_col, 3);
    p_num_row_ = _p_num_row;
    p_num_col_ = _p_num_col;
    rows_ = _p_num_col;
    cols_ = _p_num_row;
    p_num_ = _p_num_row * _p_num_col;
    center_ << 0.0, 0.0, 0.0;

    //gen_point_set();
  }

  void PointSet::gen_point_set()
  {
    //// Genreate a line
		//for (size_t i = 0; i < gen_pointset.rows(); i++)
		//{
			//double u = (double)(i);
			//gen_pointset(i, 0) = (u / 10);
			//gen_pointset(i, 1) = (u / 10);
			//gen_pointset(i, 2) = (u / 10);
		//}

		//// Genreate a plane
    //for (size_t i = 0; i < p_num_row_; i++)
    //{
      //for (size_t j = 0; j < p_num_col_; j++)
      //{
        //pointSet(i * p_num_col_ + j, 0) = 0.05 * i;
        //pointSet(i * p_num_col_ + j, 1) = ((double)(j))/p_num_col_;
        //pointSet(i * p_num_col_ + j, 2) = 0.0;
      //} 
    //}

		for (size_t i = 0; i < p_num_col_; i++)
		{
			pointSet(i, 0) = 0.05 * i;		
			pointSet(i, 1) = 0.0 ;		
			pointSet(i, 2) = 0.0;		
		}
		for (size_t i = 0; i < p_num_col_; i++)
		{
			pointSet(p_num_col_ + i, 0) = 0.05 * i;	
			pointSet(p_num_col_ + i, 1) = pointSet(i, 1) - 0.05;		
			pointSet(p_num_col_ + i, 2) = 0.0;		
		} 
		for (size_t i = 0; i < p_num_col_; i++)
		{
			pointSet(2*p_num_col_ + i, 0) = 0.05 * i;		
			pointSet(2*p_num_col_ + i, 1) = pointSet(i, 1) - 2*0.05;	
			pointSet(2*p_num_col_ + i, 2) = 0.0;		
		} 
  } 

  Eigen::Vector3d PointSet::get_point()
  {
    Eigen::Vector3d p;

    return p;  
  }

  int PointSet::rows()
  {
    return rows_; 
  }

  int PointSet::cols()
  {
    return cols_; 
  }

  int PointSet::p_num()
  {
    return p_num_; 
  }

}

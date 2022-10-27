/*
 * Automatically Generated from Mathematica.
 * Mon 15 Aug 2022 15:20:23 GMT-04:00
 */

#ifndef DJP_COM_FUNC_HH
#define DJP_COM_FUNC_HH

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE


#include "math2mat.hpp"
#include "mdefs.hpp"

namespace SymFunction
{

  void dJp_COM_func_raw(double *p_output1, const double *var1,const double *var2);

  inline void dJp_COM_func(Eigen::MatrixXd &p_output1, const Eigen::VectorXd &var1,const Eigen::VectorXd &var2)
  {
    // Check
    // - Inputs
    assert_size_matrix(var1, 30, 1);
    assert_size_matrix(var2, 30, 1);

	
    // - Outputs
    assert_size_matrix(p_output1, 3, 30);


    // set zero the matrix
    p_output1.setZero();


    // Call Subroutine with raw data
    dJp_COM_func_raw(p_output1.data(), var1.data(),var2.data());
    }
  
  
}

#endif // MATLAB_MEX_FILE

#endif // DJP_COM_FUNC_HH

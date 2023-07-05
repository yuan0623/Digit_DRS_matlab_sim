/*
 * Automatically Generated from Mathematica.
 * Tue 4 Jul 2023 17:54:23 GMT-04:00
 */

#ifndef AMWORLD_ABOUT_PA_FUNC_HH
#define AMWORLD_ABOUT_PA_FUNC_HH

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE


#include "math2mat.hpp"
#include "mdefs.hpp"

namespace SymFunction
{

  void AMworld_about_pA_func_raw(double *p_output1, const double *var1,const double *var2,const double *var3);

  inline void AMworld_about_pA_func(Eigen::MatrixXd &p_output1, const Eigen::VectorXd &var1,const Eigen::VectorXd &var2,const Eigen::VectorXd &var3)
  {
    // Check
    // - Inputs
    assert_size_matrix(var1, 1, 30);
    assert_size_matrix(var2, 1, 30);
    assert_size_matrix(var3, 1, 3);

	
    // - Outputs
    assert_size_matrix(p_output1, 3, 1);


    // set zero the matrix
    p_output1.setZero();


    // Call Subroutine with raw data
    AMworld_about_pA_func_raw(p_output1.data(), var1.data(),var2.data(),var3.data());
    }
  
  
}

#endif // MATLAB_MEX_FILE

#endif // AMWORLD_ABOUT_PA_FUNC_HH

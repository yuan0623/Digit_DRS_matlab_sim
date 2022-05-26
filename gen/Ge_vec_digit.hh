/*
 * Automatically Generated from Mathematica.
 * Mon 2 May 2022 22:37:45 GMT-04:00
 */

#ifndef GE_VEC_DIGIT_HH
#define GE_VEC_DIGIT_HH

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE


#include "math2mat.hpp"
#include "mdefs.hpp"

namespace SymFunction
{

  void Ge_vec_digit_raw(double *p_output1, const double *var1,const double *var2);

  inline void Ge_vec_digit(Eigen::MatrixXd &p_output1, const Eigen::VectorXd &var1,const Eigen::VectorXd &var2)
  {
    // Check
    // - Inputs
    assert_size_matrix(var1, 30, 1);
    assert_size_matrix(var2, 30, 1);

	
    // - Outputs
    assert_size_matrix(p_output1, 30, 1);


    // set zero the matrix
    p_output1.setZero();


    // Call Subroutine with raw data
    Ge_vec_digit_raw(p_output1.data(), var1.data(),var2.data());
    }
  
  
}

#endif // MATLAB_MEX_FILE

#endif // GE_VEC_DIGIT_HH

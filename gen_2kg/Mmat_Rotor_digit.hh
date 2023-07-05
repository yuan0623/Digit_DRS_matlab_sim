/*
 * Automatically Generated from Mathematica.
 * Tue 4 Jul 2023 15:39:40 GMT-04:00
 */

#ifndef MMAT_ROTOR_DIGIT_HH
#define MMAT_ROTOR_DIGIT_HH

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE


#include "math2mat.hpp"
#include "mdefs.hpp"

namespace SymFunction
{

  void Mmat_Rotor_digit_raw(double *p_output1, const double *var1);

  inline void Mmat_Rotor_digit(Eigen::MatrixXd &p_output1, const Eigen::VectorXd &var1)
  {
    // Check
    // - Inputs
    assert_size_matrix(var1, 30, 1);

	
    // - Outputs
    assert_size_matrix(p_output1, 30, 30);


    // set zero the matrix
    p_output1.setZero();


    // Call Subroutine with raw data
    Mmat_Rotor_digit_raw(p_output1.data(), var1.data());
    }
  
  
}

#endif // MATLAB_MEX_FILE

#endif // MMAT_ROTOR_DIGIT_HH

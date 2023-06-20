/*
 * Automatically Generated from Mathematica.
 * Mon 19 Jun 2023 22:52:48 GMT-04:00
 */

#ifdef MATLAB_MEX_FILE
#include <stdexcept>
#include <cmath>
#include<math.h>
/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
inline double Power(double x, double y) { return pow(x, y); }
inline double Sqrt(double x) { return sqrt(x); }

inline double Abs(double x) { return fabs(x); }

inline double Exp(double x) { return exp(x); }
inline double Log(double x) { return log(x); }

inline double Sin(double x) { return sin(x); }
inline double Cos(double x) { return cos(x); }
inline double Tan(double x) { return tan(x); }

inline double ArcSin(double x) { return asin(x); }
inline double ArcCos(double x) { return acos(x); }
inline double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
inline double ArcTan(double x, double y) { return atan2(y,x); }

inline double Sinh(double x) { return sinh(x); }
inline double Cosh(double x) { return cosh(x); }
inline double Tanh(double x) { return tanh(x); }

const double E	= 2.71828182845904523536029;
const double Pi = 3.14159265358979323846264;
const double Degree = 0.01745329251994329576924;

inline double Sec(double x) { return 1/cos(x); }
inline double Csc(double x) { return 1/sin(x); }

#endif

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1)
{
  double t136;
  double t2166;
  double t4229;
  double t4326;
  double t4339;
  double t4341;
  double t4315;
  double t4345;
  double t172;
  double t4218;
  double t4239;
  double t4302;
  double t4324;
  double t4350;
  double t4352;
  double t4483;
  double t4474;
  double t4396;
  double t4397;
  double t4403;
  double t4404;
  double t4408;
  double t4410;
  double t4504;
  double t4509;
  double t4515;
  double t4571;
  double t4574;
  double t4582;
  double t4683;
  double t4687;
  double t4690;
  double t4701;
  double t4702;
  double t4706;
  double t4726;
  double t4728;
  double t4731;
  double t4739;
  double t4743;
  double t4746;
  double t4747;
  double t4712;
  double t4713;
  double t4715;
  double t4740;
  double t4756;
  double t4759;
  double t4767;
  double t4775;
  double t4776;
  double t4796;
  double t4866;
  double t4877;
  double t4823;
  double t4843;
  double t4950;
  double t4941;
  double t4481;
  double t4486;
  double t4489;
  double t4495;
  double t4497;
  double t4535;
  double t4542;
  double t4547;
  double t4570;
  double t4585;
  double t4601;
  double t4607;
  double t4613;
  double t4616;
  double t4631;
  double t4632;
  double t4636;
  double t4638;
  double t4649;
  double t4660;
  double t4662;
  double t4667;
  double t4720;
  double t4760;
  double t4780;
  double t4785;
  double t4791;
  double t4797;
  double t4800;
  double t4801;
  double t4813;
  double t4816;
  double t4834;
  double t4840;
  double t4841;
  double t4842;
  double t4844;
  double t4849;
  double t4853;
  double t4859;
  double t4860;
  double t4867;
  double t4871;
  double t4873;
  double t4876;
  double t4883;
  double t4884;
  double t4886;
  double t4887;
  double t4890;
  double t4897;
  double t4898;
  double t4899;
  double t4900;
  double t4902;
  double t4903;
  double t4904;
  double t4905;
  double t4907;
  double t4916;
  double t4921;
  double t4922;
  double t4926;
  double t4927;
  double t4928;
  double t4929;
  double t4938;
  double t4942;
  double t4944;
  double t4946;
  double t4952;
  double t4954;
  double t4956;
  double t4958;
  double t4962;
  double t4967;
  double t4968;
  double t4969;
  double t4970;
  double t4973;
  double t4977;
  double t4633;
  double t4652;
  double t4676;
  double t4680;
  double t5009;
  double t5011;
  double t5013;
  double t5015;
  double t4822;
  double t4862;
  double t4891;
  double t4893;
  double t5018;
  double t5020;
  double t5021;
  double t5023;
  double t5065;
  double t5066;
  double t5068;
  double t5070;
  double t5100;
  double t5104;
  double t5106;
  double t4915;
  double t4940;
  double t4959;
  double t4960;
  double t5027;
  double t5028;
  double t5029;
  double t5030;
  double t5077;
  double t5079;
  double t5080;
  double t5083;
  double t5111;
  double t5116;
  double t5117;
  double t5119;
  double t5120;
  double t5122;
  double t5124;
  double t5127;
  double t5130;
  double t5185;
  double t5189;
  double t5190;
  double t5158;
  double t5161;
  double t5162;
  double t5166;
  double t5172;
  double t5173;
  double t5179;
  double t5180;
  double t5149;
  double t5153;
  double t5155;
  double t5156;
  double t5210;
  double t5211;
  double t5214;
  double t5216;
  double t5217;
  double t5218;
  double t5192;
  double t5194;
  double t5195;
  double t4978;
  double t4979;
  double t4981;
  double t4984;
  double t5035;
  double t5037;
  double t5039;
  double t5042;
  double t5085;
  double t5090;
  double t5091;
  double t5092;
  double t5226;
  double t5227;
  double t5228;
  double t5256;
  double t5257;
  double t5258;
  double t4985;
  double t4986;
  double t4987;
  double t5044;
  double t5046;
  double t5047;
  double t5093;
  double t5095;
  double t5099;
  double t5230;
  double t5231;
  double t5232;
  double t5260;
  double t5261;
  double t5262;
  double t5283;
  double t5284;
  double t5285;
  t136 = Cos(var1[4]);
  t2166 = Cos(var1[14]);
  t4229 = Sin(var1[14]);
  t4326 = 0.984808*t2166;
  t4339 = -0.173648*t4229;
  t4341 = t4326 + t4339;
  t4315 = Sin(var1[4]);
  t4345 = Sin(var1[5]);
  t172 = Cos(var1[5]);
  t4218 = -0.173648*t2166;
  t4239 = -0.984808*t4229;
  t4302 = t4218 + t4239;
  t4324 = t4302*t4315;
  t4350 = -1.*t136*t4341*t4345;
  t4352 = t4324 + t4350;
  t4483 = Cos(var1[3]);
  t4474 = Sin(var1[3]);
  t4396 = t4341*t4315;
  t4397 = 0.173648*t2166;
  t4403 = 0.984808*t4229;
  t4404 = t4397 + t4403;
  t4408 = -1.*t136*t4404*t4345;
  t4410 = t4396 + t4408;
  t4504 = t136*t4302;
  t4509 = t4341*t4315*t4345;
  t4515 = t4504 + t4509;
  t4571 = t136*t4341;
  t4574 = t4404*t4315*t4345;
  t4582 = t4571 + t4574;
  t4683 = -1.*t2166;
  t4687 = 1. + t4683;
  t4690 = 0.12*t4687;
  t4701 = 0.116892*t2166;
  t4702 = 0.0005569999999999742*t4229;
  t4706 = t4690 + t4701 + t4702;
  t4726 = 0.4*t4687;
  t4728 = 0.400557*t2166;
  t4731 = 0.0031079999999999997*t4229;
  t4739 = t4726 + t4728 + t4731;
  t4743 = -0.024405*t172;
  t4746 = -1.*t4706*t4345;
  t4747 = t4743 + t4746;
  t4712 = t172*t4706;
  t4713 = -0.024405*t4345;
  t4715 = t4712 + t4713;
  t4740 = t4739*t4315;
  t4756 = t136*t4747;
  t4759 = t4740 + t4756;
  t4767 = t136*t4739;
  t4775 = -1.*t4315*t4747;
  t4776 = t4767 + t4775;
  t4796 = -1.*t172*t4341*t4715;
  t4866 = t4715*t4345;
  t4877 = t172*t4404*t4715;
  t4823 = t172*t4341*t4715;
  t4843 = -1.*t172*t4404*t4715;
  t4950 = -1.*t4341*t4739;
  t4941 = t4302*t4739;
  t4481 = -1.*t172*t4474*t4315;
  t4486 = -1.*t4483*t4345;
  t4489 = t4481 + t4486;
  t4495 = -0.535396*t136*t172*t4489;
  t4497 = t4483*t172*t4341;
  t4535 = -1.*t4474*t4515;
  t4542 = t4497 + t4535;
  t4547 = 0.535396*t4352*t4542;
  t4570 = t4483*t172*t4404;
  t4585 = -1.*t4474*t4582;
  t4601 = t4570 + t4585;
  t4607 = 0.535396*t4410*t4601;
  t4613 = t4495 + t4547 + t4607;
  t4616 = t4483*t172*t4315;
  t4631 = -1.*t4474*t4345;
  t4632 = t4616 + t4631;
  t4636 = t172*t4341*t4474;
  t4638 = t4483*t4515;
  t4649 = t4636 + t4638;
  t4660 = t172*t4404*t4474;
  t4662 = t4483*t4582;
  t4667 = t4660 + t4662;
  t4720 = -1.*t4715*t4345;
  t4760 = -1.*t136*t172*t4759;
  t4780 = t172*t4315*t4776;
  t4785 = t4720 + t4760 + t4780;
  t4791 = t4352*t4785;
  t4797 = -1.*t4352*t4759;
  t4800 = -1.*t4515*t4776;
  t4801 = t4796 + t4797 + t4800;
  t4813 = -1.*t136*t172*t4801;
  t4816 = t4791 + t4813;
  t4834 = t4352*t4759;
  t4840 = t4515*t4776;
  t4841 = t4823 + t4834 + t4840;
  t4842 = t4410*t4841;
  t4844 = -1.*t4410*t4759;
  t4849 = -1.*t4582*t4776;
  t4853 = t4843 + t4844 + t4849;
  t4859 = t4352*t4853;
  t4860 = t4842 + t4859;
  t4867 = t136*t172*t4759;
  t4871 = -1.*t172*t4315*t4776;
  t4873 = t4866 + t4867 + t4871;
  t4876 = t4410*t4873;
  t4883 = t4410*t4759;
  t4884 = t4582*t4776;
  t4886 = t4877 + t4883 + t4884;
  t4887 = -1.*t136*t172*t4886;
  t4890 = t4876 + t4887;
  t4897 = -1.*t4715*t4345;
  t4898 = -1.*t172*t4747;
  t4899 = t4897 + t4898;
  t4900 = t172*t4341*t4899;
  t4902 = -1.*t4302*t4739;
  t4903 = t4341*t4345*t4747;
  t4904 = t4902 + t4796 + t4903;
  t4905 = -1.*t4345*t4904;
  t4907 = t4900 + t4905;
  t4916 = t172*t4747;
  t4921 = t4866 + t4916;
  t4922 = t172*t4404*t4921;
  t4926 = t4341*t4739;
  t4927 = -1.*t4404*t4345*t4747;
  t4928 = t4926 + t4877 + t4927;
  t4929 = -1.*t4345*t4928;
  t4938 = t4922 + t4929;
  t4942 = -1.*t4341*t4345*t4747;
  t4944 = t4941 + t4823 + t4942;
  t4946 = t172*t4404*t4944;
  t4952 = t4404*t4345*t4747;
  t4954 = t4950 + t4843 + t4952;
  t4956 = t172*t4341*t4954;
  t4958 = t4946 + t4956;
  t4962 = -1.*t4706*t4404;
  t4967 = t4950 + t4962;
  t4968 = t4967*t4302;
  t4969 = t4341*t4706;
  t4970 = t4969 + t4941;
  t4973 = t4970*t4341;
  t4977 = t4968 + t4973;
  t4633 = -0.535396*t136*t172*t4632;
  t4652 = 0.535396*t4352*t4649;
  t4676 = 0.535396*t4410*t4667;
  t4680 = t4633 + t4652 + t4676;
  t5009 = 0.535396*t4489*t4632;
  t5011 = 0.535396*t4649*t4542;
  t5013 = 0.535396*t4667*t4601;
  t5015 = t5009 + t5011 + t5013;
  t4822 = 0.535396*t4410*t4816;
  t4862 = -0.535396*t136*t172*t4860;
  t4891 = 0.535396*t4352*t4890;
  t4893 = t4822 + t4862 + t4891;
  t5018 = 0.535396*t4601*t4816;
  t5020 = 0.535396*t4489*t4860;
  t5021 = 0.535396*t4542*t4890;
  t5023 = t5018 + t5020 + t5021;
  t5065 = 0.535396*t4667*t4816;
  t5066 = 0.535396*t4632*t4860;
  t5068 = 0.535396*t4649*t4890;
  t5070 = t5065 + t5066 + t5068;
  t5100 = -0.000298*t136*t172;
  t5104 = 0.000035*t4352;
  t5106 = 0.000012*t4410;
  t4915 = 0.535396*t4410*t4907;
  t4940 = 0.535396*t4352*t4938;
  t4959 = -0.535396*t136*t172*t4958;
  t4960 = t4915 + t4940 + t4959;
  t5027 = 0.535396*t4601*t4907;
  t5028 = 0.535396*t4542*t4938;
  t5029 = 0.535396*t4489*t4958;
  t5030 = t5027 + t5028 + t5029;
  t5077 = 0.535396*t4667*t4907;
  t5079 = 0.535396*t4649*t4938;
  t5080 = 0.535396*t4632*t4958;
  t5083 = t5077 + t5079 + t5080;
  t5111 = t5100 + t5104 + t5106;
  t5116 = -0.000035*t136*t172;
  t5117 = 0.00075*t4352;
  t5119 = 0.000014*t4410;
  t5120 = t5116 + t5117 + t5119;
  t5122 = -0.000012*t136*t172;
  t5124 = 0.000014*t4352;
  t5127 = 0.000704*t4410;
  t5130 = t5122 + t5124 + t5127;
  t5185 = 0.535396*t4907*t4816;
  t5189 = 0.535396*t4958*t4860;
  t5190 = 0.535396*t4938*t4890;
  t5158 = 0.00075*t172*t4341;
  t5161 = 0.000014*t172*t4404;
  t5162 = -0.000035*t4345;
  t5166 = t5158 + t5161 + t5162;
  t5172 = 0.000014*t172*t4341;
  t5173 = 0.000704*t172*t4404;
  t5179 = -0.000012*t4345;
  t5180 = t5172 + t5173 + t5179;
  t5149 = 0.000035*t172*t4341;
  t5153 = 0.000012*t172*t4404;
  t5155 = -0.000298*t4345;
  t5156 = t5149 + t5153 + t5155;
  t5210 = 0.00075*t4302;
  t5211 = 0.000014*t4341;
  t5214 = t5210 + t5211;
  t5216 = 0.000014*t4302;
  t5217 = 0.000704*t4341;
  t5218 = t5216 + t5217;
  t5192 = 0.000035*t4302;
  t5194 = 0.000012*t4341;
  t5195 = t5192 + t5194;
  t4978 = -0.535396*t136*t172*t4977;
  t4979 = -0.01306633938*t4341*t4352;
  t4981 = 0.01306633938*t4302*t4410;
  t4984 = t4978 + t4979 + t4981;
  t5035 = 0.535396*t4977*t4489;
  t5037 = -0.01306633938*t4341*t4542;
  t5039 = 0.01306633938*t4302*t4601;
  t5042 = t5035 + t5037 + t5039;
  t5085 = 0.535396*t4977*t4632;
  t5090 = -0.01306633938*t4341*t4649;
  t5091 = 0.01306633938*t4302*t4667;
  t5092 = t5085 + t5090 + t5091;
  t5226 = 0.01306633938*t4302*t4816;
  t5227 = 0.535396*t4977*t4860;
  t5228 = -0.01306633938*t4341*t4890;
  t5256 = 0.01306633938*t4302*t4907;
  t5257 = -0.01306633938*t4341*t4938;
  t5258 = 0.535396*t4977*t4958;
  t4985 = 4.732939188496741e-6*t4352;
  t4986 = 0.0016905156540591966*t4410;
  t4987 = t4985 + t4986;
  t5044 = 4.732939188496741e-6*t4542;
  t5046 = 0.0016905156540591966*t4601;
  t5047 = t5044 + t5046;
  t5093 = 4.732939188496741e-6*t4649;
  t5095 = 0.0016905156540591966*t4667;
  t5099 = t5093 + t5095;
  t5230 = 0.0016905156540591966*t4816;
  t5231 = 4.732939188496741e-6*t4890;
  t5232 = t5100 + t5104 + t5106 + t5230 + t5231;
  t5260 = 0.0016905156540591966*t4907;
  t5261 = 4.732939188496741e-6*t4938;
  t5262 = t5149 + t5153 + t5155 + t5260 + t5261;
  t5283 = 0.0000762570345373147*t4302;
  t5284 = 0.000011884492619104737*t4341;
  t5285 = t5283 + t5284;
  p_output1[0]=0.535396*Power(t136,2)*Power(t172,2) + 0.535396*Power(t4352,2) + 0.535396*Power(t4410,2);
  p_output1[1]=t4613;
  p_output1[2]=t4680;
  p_output1[3]=t4893;
  p_output1[4]=t4960;
  p_output1[5]=t4984;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=0;
  p_output1[13]=0;
  p_output1[14]=t4987;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=t4613;
  p_output1[31]=0.535396*Power(t4489,2) + 0.535396*Power(t4542,2) + 0.535396*Power(t4601,2);
  p_output1[32]=t5015;
  p_output1[33]=t5023;
  p_output1[34]=t5030;
  p_output1[35]=t5042;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=t5047;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
  p_output1[54]=0;
  p_output1[55]=0;
  p_output1[56]=0;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=t4680;
  p_output1[61]=t5015;
  p_output1[62]=0.535396*Power(t4632,2) + 0.535396*Power(t4649,2) + 0.535396*Power(t4667,2);
  p_output1[63]=t5070;
  p_output1[64]=t5083;
  p_output1[65]=t5092;
  p_output1[66]=0;
  p_output1[67]=0;
  p_output1[68]=0;
  p_output1[69]=0;
  p_output1[70]=0;
  p_output1[71]=0;
  p_output1[72]=0;
  p_output1[73]=0;
  p_output1[74]=t5099;
  p_output1[75]=0;
  p_output1[76]=0;
  p_output1[77]=0;
  p_output1[78]=0;
  p_output1[79]=0;
  p_output1[80]=0;
  p_output1[81]=0;
  p_output1[82]=0;
  p_output1[83]=0;
  p_output1[84]=0;
  p_output1[85]=0;
  p_output1[86]=0;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=t4893;
  p_output1[91]=t5023;
  p_output1[92]=t5070;
  p_output1[93]=0.535396*Power(t4816,2) + 0.535396*Power(t4860,2) + 0.535396*Power(t4890,2) - 1.*t136*t172*t5111 + t4352*t5120 + t4410*t5130;
  p_output1[94]=-1.*t136*t172*t5156 + t4352*t5166 + t4410*t5180 + t5185 + t5189 + t5190;
  p_output1[95]=-1.*t136*t172*t5195 + t4352*t5214 + t4410*t5218 + t5226 + t5227 + t5228;
  p_output1[96]=0;
  p_output1[97]=0;
  p_output1[98]=0;
  p_output1[99]=0;
  p_output1[100]=0;
  p_output1[101]=0;
  p_output1[102]=0;
  p_output1[103]=0;
  p_output1[104]=t5232;
  p_output1[105]=0;
  p_output1[106]=0;
  p_output1[107]=0;
  p_output1[108]=0;
  p_output1[109]=0;
  p_output1[110]=0;
  p_output1[111]=0;
  p_output1[112]=0;
  p_output1[113]=0;
  p_output1[114]=0;
  p_output1[115]=0;
  p_output1[116]=0;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
  p_output1[120]=t4960;
  p_output1[121]=t5030;
  p_output1[122]=t5083;
  p_output1[123]=-1.*t4345*t5111 + t172*t4341*t5120 + t172*t4404*t5130 + t5185 + t5189 + t5190;
  p_output1[124]=0.535396*Power(t4907,2) + 0.535396*Power(t4938,2) + 0.535396*Power(t4958,2) - 1.*t4345*t5156 + t172*t4341*t5166 + t172*t4404*t5180;
  p_output1[125]=-1.*t4345*t5195 + t172*t4341*t5214 + t172*t4404*t5218 + t5256 + t5257 + t5258;
  p_output1[126]=0;
  p_output1[127]=0;
  p_output1[128]=0;
  p_output1[129]=0;
  p_output1[130]=0;
  p_output1[131]=0;
  p_output1[132]=0;
  p_output1[133]=0;
  p_output1[134]=t5262;
  p_output1[135]=0;
  p_output1[136]=0;
  p_output1[137]=0;
  p_output1[138]=0;
  p_output1[139]=0;
  p_output1[140]=0;
  p_output1[141]=0;
  p_output1[142]=0;
  p_output1[143]=0;
  p_output1[144]=0;
  p_output1[145]=0;
  p_output1[146]=0;
  p_output1[147]=0;
  p_output1[148]=0;
  p_output1[149]=0;
  p_output1[150]=t4984;
  p_output1[151]=t5042;
  p_output1[152]=t5092;
  p_output1[153]=t4302*t5120 + t4341*t5130 + t5226 + t5227 + t5228;
  p_output1[154]=t4302*t5166 + t4341*t5180 + t5256 + t5257 + t5258;
  p_output1[155]=0.0003188840125689*Power(t4302,2) + 0.0003188840125689*Power(t4341,2) + 0.535396*Power(t4977,2) + t4302*t5214 + t4341*t5218;
  p_output1[156]=0;
  p_output1[157]=0;
  p_output1[158]=0;
  p_output1[159]=0;
  p_output1[160]=0;
  p_output1[161]=0;
  p_output1[162]=0;
  p_output1[163]=0;
  p_output1[164]=t5285;
  p_output1[165]=0;
  p_output1[166]=0;
  p_output1[167]=0;
  p_output1[168]=0;
  p_output1[169]=0;
  p_output1[170]=0;
  p_output1[171]=0;
  p_output1[172]=0;
  p_output1[173]=0;
  p_output1[174]=0;
  p_output1[175]=0;
  p_output1[176]=0;
  p_output1[177]=0;
  p_output1[178]=0;
  p_output1[179]=0;
  p_output1[180]=0;
  p_output1[181]=0;
  p_output1[182]=0;
  p_output1[183]=0;
  p_output1[184]=0;
  p_output1[185]=0;
  p_output1[186]=0;
  p_output1[187]=0;
  p_output1[188]=0;
  p_output1[189]=0;
  p_output1[190]=0;
  p_output1[191]=0;
  p_output1[192]=0;
  p_output1[193]=0;
  p_output1[194]=0;
  p_output1[195]=0;
  p_output1[196]=0;
  p_output1[197]=0;
  p_output1[198]=0;
  p_output1[199]=0;
  p_output1[200]=0;
  p_output1[201]=0;
  p_output1[202]=0;
  p_output1[203]=0;
  p_output1[204]=0;
  p_output1[205]=0;
  p_output1[206]=0;
  p_output1[207]=0;
  p_output1[208]=0;
  p_output1[209]=0;
  p_output1[210]=0;
  p_output1[211]=0;
  p_output1[212]=0;
  p_output1[213]=0;
  p_output1[214]=0;
  p_output1[215]=0;
  p_output1[216]=0;
  p_output1[217]=0;
  p_output1[218]=0;
  p_output1[219]=0;
  p_output1[220]=0;
  p_output1[221]=0;
  p_output1[222]=0;
  p_output1[223]=0;
  p_output1[224]=0;
  p_output1[225]=0;
  p_output1[226]=0;
  p_output1[227]=0;
  p_output1[228]=0;
  p_output1[229]=0;
  p_output1[230]=0;
  p_output1[231]=0;
  p_output1[232]=0;
  p_output1[233]=0;
  p_output1[234]=0;
  p_output1[235]=0;
  p_output1[236]=0;
  p_output1[237]=0;
  p_output1[238]=0;
  p_output1[239]=0;
  p_output1[240]=0;
  p_output1[241]=0;
  p_output1[242]=0;
  p_output1[243]=0;
  p_output1[244]=0;
  p_output1[245]=0;
  p_output1[246]=0;
  p_output1[247]=0;
  p_output1[248]=0;
  p_output1[249]=0;
  p_output1[250]=0;
  p_output1[251]=0;
  p_output1[252]=0;
  p_output1[253]=0;
  p_output1[254]=0;
  p_output1[255]=0;
  p_output1[256]=0;
  p_output1[257]=0;
  p_output1[258]=0;
  p_output1[259]=0;
  p_output1[260]=0;
  p_output1[261]=0;
  p_output1[262]=0;
  p_output1[263]=0;
  p_output1[264]=0;
  p_output1[265]=0;
  p_output1[266]=0;
  p_output1[267]=0;
  p_output1[268]=0;
  p_output1[269]=0;
  p_output1[270]=0;
  p_output1[271]=0;
  p_output1[272]=0;
  p_output1[273]=0;
  p_output1[274]=0;
  p_output1[275]=0;
  p_output1[276]=0;
  p_output1[277]=0;
  p_output1[278]=0;
  p_output1[279]=0;
  p_output1[280]=0;
  p_output1[281]=0;
  p_output1[282]=0;
  p_output1[283]=0;
  p_output1[284]=0;
  p_output1[285]=0;
  p_output1[286]=0;
  p_output1[287]=0;
  p_output1[288]=0;
  p_output1[289]=0;
  p_output1[290]=0;
  p_output1[291]=0;
  p_output1[292]=0;
  p_output1[293]=0;
  p_output1[294]=0;
  p_output1[295]=0;
  p_output1[296]=0;
  p_output1[297]=0;
  p_output1[298]=0;
  p_output1[299]=0;
  p_output1[300]=0;
  p_output1[301]=0;
  p_output1[302]=0;
  p_output1[303]=0;
  p_output1[304]=0;
  p_output1[305]=0;
  p_output1[306]=0;
  p_output1[307]=0;
  p_output1[308]=0;
  p_output1[309]=0;
  p_output1[310]=0;
  p_output1[311]=0;
  p_output1[312]=0;
  p_output1[313]=0;
  p_output1[314]=0;
  p_output1[315]=0;
  p_output1[316]=0;
  p_output1[317]=0;
  p_output1[318]=0;
  p_output1[319]=0;
  p_output1[320]=0;
  p_output1[321]=0;
  p_output1[322]=0;
  p_output1[323]=0;
  p_output1[324]=0;
  p_output1[325]=0;
  p_output1[326]=0;
  p_output1[327]=0;
  p_output1[328]=0;
  p_output1[329]=0;
  p_output1[330]=0;
  p_output1[331]=0;
  p_output1[332]=0;
  p_output1[333]=0;
  p_output1[334]=0;
  p_output1[335]=0;
  p_output1[336]=0;
  p_output1[337]=0;
  p_output1[338]=0;
  p_output1[339]=0;
  p_output1[340]=0;
  p_output1[341]=0;
  p_output1[342]=0;
  p_output1[343]=0;
  p_output1[344]=0;
  p_output1[345]=0;
  p_output1[346]=0;
  p_output1[347]=0;
  p_output1[348]=0;
  p_output1[349]=0;
  p_output1[350]=0;
  p_output1[351]=0;
  p_output1[352]=0;
  p_output1[353]=0;
  p_output1[354]=0;
  p_output1[355]=0;
  p_output1[356]=0;
  p_output1[357]=0;
  p_output1[358]=0;
  p_output1[359]=0;
  p_output1[360]=0;
  p_output1[361]=0;
  p_output1[362]=0;
  p_output1[363]=0;
  p_output1[364]=0;
  p_output1[365]=0;
  p_output1[366]=0;
  p_output1[367]=0;
  p_output1[368]=0;
  p_output1[369]=0;
  p_output1[370]=0;
  p_output1[371]=0;
  p_output1[372]=0;
  p_output1[373]=0;
  p_output1[374]=0;
  p_output1[375]=0;
  p_output1[376]=0;
  p_output1[377]=0;
  p_output1[378]=0;
  p_output1[379]=0;
  p_output1[380]=0;
  p_output1[381]=0;
  p_output1[382]=0;
  p_output1[383]=0;
  p_output1[384]=0;
  p_output1[385]=0;
  p_output1[386]=0;
  p_output1[387]=0;
  p_output1[388]=0;
  p_output1[389]=0;
  p_output1[390]=0;
  p_output1[391]=0;
  p_output1[392]=0;
  p_output1[393]=0;
  p_output1[394]=0;
  p_output1[395]=0;
  p_output1[396]=0;
  p_output1[397]=0;
  p_output1[398]=0;
  p_output1[399]=0;
  p_output1[400]=0;
  p_output1[401]=0;
  p_output1[402]=0;
  p_output1[403]=0;
  p_output1[404]=0;
  p_output1[405]=0;
  p_output1[406]=0;
  p_output1[407]=0;
  p_output1[408]=0;
  p_output1[409]=0;
  p_output1[410]=0;
  p_output1[411]=0;
  p_output1[412]=0;
  p_output1[413]=0;
  p_output1[414]=0;
  p_output1[415]=0;
  p_output1[416]=0;
  p_output1[417]=0;
  p_output1[418]=0;
  p_output1[419]=0;
  p_output1[420]=t4987;
  p_output1[421]=t5047;
  p_output1[422]=t5099;
  p_output1[423]=t5232;
  p_output1[424]=t5262;
  p_output1[425]=t5285;
  p_output1[426]=0;
  p_output1[427]=0;
  p_output1[428]=0;
  p_output1[429]=0;
  p_output1[430]=0;
  p_output1[431]=0;
  p_output1[432]=0;
  p_output1[433]=0;
  p_output1[434]=0.0003033378538078965;
  p_output1[435]=0;
  p_output1[436]=0;
  p_output1[437]=0;
  p_output1[438]=0;
  p_output1[439]=0;
  p_output1[440]=0;
  p_output1[441]=0;
  p_output1[442]=0;
  p_output1[443]=0;
  p_output1[444]=0;
  p_output1[445]=0;
  p_output1[446]=0;
  p_output1[447]=0;
  p_output1[448]=0;
  p_output1[449]=0;
  p_output1[450]=0;
  p_output1[451]=0;
  p_output1[452]=0;
  p_output1[453]=0;
  p_output1[454]=0;
  p_output1[455]=0;
  p_output1[456]=0;
  p_output1[457]=0;
  p_output1[458]=0;
  p_output1[459]=0;
  p_output1[460]=0;
  p_output1[461]=0;
  p_output1[462]=0;
  p_output1[463]=0;
  p_output1[464]=0;
  p_output1[465]=0;
  p_output1[466]=0;
  p_output1[467]=0;
  p_output1[468]=0;
  p_output1[469]=0;
  p_output1[470]=0;
  p_output1[471]=0;
  p_output1[472]=0;
  p_output1[473]=0;
  p_output1[474]=0;
  p_output1[475]=0;
  p_output1[476]=0;
  p_output1[477]=0;
  p_output1[478]=0;
  p_output1[479]=0;
  p_output1[480]=0;
  p_output1[481]=0;
  p_output1[482]=0;
  p_output1[483]=0;
  p_output1[484]=0;
  p_output1[485]=0;
  p_output1[486]=0;
  p_output1[487]=0;
  p_output1[488]=0;
  p_output1[489]=0;
  p_output1[490]=0;
  p_output1[491]=0;
  p_output1[492]=0;
  p_output1[493]=0;
  p_output1[494]=0;
  p_output1[495]=0;
  p_output1[496]=0;
  p_output1[497]=0;
  p_output1[498]=0;
  p_output1[499]=0;
  p_output1[500]=0;
  p_output1[501]=0;
  p_output1[502]=0;
  p_output1[503]=0;
  p_output1[504]=0;
  p_output1[505]=0;
  p_output1[506]=0;
  p_output1[507]=0;
  p_output1[508]=0;
  p_output1[509]=0;
  p_output1[510]=0;
  p_output1[511]=0;
  p_output1[512]=0;
  p_output1[513]=0;
  p_output1[514]=0;
  p_output1[515]=0;
  p_output1[516]=0;
  p_output1[517]=0;
  p_output1[518]=0;
  p_output1[519]=0;
  p_output1[520]=0;
  p_output1[521]=0;
  p_output1[522]=0;
  p_output1[523]=0;
  p_output1[524]=0;
  p_output1[525]=0;
  p_output1[526]=0;
  p_output1[527]=0;
  p_output1[528]=0;
  p_output1[529]=0;
  p_output1[530]=0;
  p_output1[531]=0;
  p_output1[532]=0;
  p_output1[533]=0;
  p_output1[534]=0;
  p_output1[535]=0;
  p_output1[536]=0;
  p_output1[537]=0;
  p_output1[538]=0;
  p_output1[539]=0;
  p_output1[540]=0;
  p_output1[541]=0;
  p_output1[542]=0;
  p_output1[543]=0;
  p_output1[544]=0;
  p_output1[545]=0;
  p_output1[546]=0;
  p_output1[547]=0;
  p_output1[548]=0;
  p_output1[549]=0;
  p_output1[550]=0;
  p_output1[551]=0;
  p_output1[552]=0;
  p_output1[553]=0;
  p_output1[554]=0;
  p_output1[555]=0;
  p_output1[556]=0;
  p_output1[557]=0;
  p_output1[558]=0;
  p_output1[559]=0;
  p_output1[560]=0;
  p_output1[561]=0;
  p_output1[562]=0;
  p_output1[563]=0;
  p_output1[564]=0;
  p_output1[565]=0;
  p_output1[566]=0;
  p_output1[567]=0;
  p_output1[568]=0;
  p_output1[569]=0;
  p_output1[570]=0;
  p_output1[571]=0;
  p_output1[572]=0;
  p_output1[573]=0;
  p_output1[574]=0;
  p_output1[575]=0;
  p_output1[576]=0;
  p_output1[577]=0;
  p_output1[578]=0;
  p_output1[579]=0;
  p_output1[580]=0;
  p_output1[581]=0;
  p_output1[582]=0;
  p_output1[583]=0;
  p_output1[584]=0;
  p_output1[585]=0;
  p_output1[586]=0;
  p_output1[587]=0;
  p_output1[588]=0;
  p_output1[589]=0;
  p_output1[590]=0;
  p_output1[591]=0;
  p_output1[592]=0;
  p_output1[593]=0;
  p_output1[594]=0;
  p_output1[595]=0;
  p_output1[596]=0;
  p_output1[597]=0;
  p_output1[598]=0;
  p_output1[599]=0;
  p_output1[600]=0;
  p_output1[601]=0;
  p_output1[602]=0;
  p_output1[603]=0;
  p_output1[604]=0;
  p_output1[605]=0;
  p_output1[606]=0;
  p_output1[607]=0;
  p_output1[608]=0;
  p_output1[609]=0;
  p_output1[610]=0;
  p_output1[611]=0;
  p_output1[612]=0;
  p_output1[613]=0;
  p_output1[614]=0;
  p_output1[615]=0;
  p_output1[616]=0;
  p_output1[617]=0;
  p_output1[618]=0;
  p_output1[619]=0;
  p_output1[620]=0;
  p_output1[621]=0;
  p_output1[622]=0;
  p_output1[623]=0;
  p_output1[624]=0;
  p_output1[625]=0;
  p_output1[626]=0;
  p_output1[627]=0;
  p_output1[628]=0;
  p_output1[629]=0;
  p_output1[630]=0;
  p_output1[631]=0;
  p_output1[632]=0;
  p_output1[633]=0;
  p_output1[634]=0;
  p_output1[635]=0;
  p_output1[636]=0;
  p_output1[637]=0;
  p_output1[638]=0;
  p_output1[639]=0;
  p_output1[640]=0;
  p_output1[641]=0;
  p_output1[642]=0;
  p_output1[643]=0;
  p_output1[644]=0;
  p_output1[645]=0;
  p_output1[646]=0;
  p_output1[647]=0;
  p_output1[648]=0;
  p_output1[649]=0;
  p_output1[650]=0;
  p_output1[651]=0;
  p_output1[652]=0;
  p_output1[653]=0;
  p_output1[654]=0;
  p_output1[655]=0;
  p_output1[656]=0;
  p_output1[657]=0;
  p_output1[658]=0;
  p_output1[659]=0;
  p_output1[660]=0;
  p_output1[661]=0;
  p_output1[662]=0;
  p_output1[663]=0;
  p_output1[664]=0;
  p_output1[665]=0;
  p_output1[666]=0;
  p_output1[667]=0;
  p_output1[668]=0;
  p_output1[669]=0;
  p_output1[670]=0;
  p_output1[671]=0;
  p_output1[672]=0;
  p_output1[673]=0;
  p_output1[674]=0;
  p_output1[675]=0;
  p_output1[676]=0;
  p_output1[677]=0;
  p_output1[678]=0;
  p_output1[679]=0;
  p_output1[680]=0;
  p_output1[681]=0;
  p_output1[682]=0;
  p_output1[683]=0;
  p_output1[684]=0;
  p_output1[685]=0;
  p_output1[686]=0;
  p_output1[687]=0;
  p_output1[688]=0;
  p_output1[689]=0;
  p_output1[690]=0;
  p_output1[691]=0;
  p_output1[692]=0;
  p_output1[693]=0;
  p_output1[694]=0;
  p_output1[695]=0;
  p_output1[696]=0;
  p_output1[697]=0;
  p_output1[698]=0;
  p_output1[699]=0;
  p_output1[700]=0;
  p_output1[701]=0;
  p_output1[702]=0;
  p_output1[703]=0;
  p_output1[704]=0;
  p_output1[705]=0;
  p_output1[706]=0;
  p_output1[707]=0;
  p_output1[708]=0;
  p_output1[709]=0;
  p_output1[710]=0;
  p_output1[711]=0;
  p_output1[712]=0;
  p_output1[713]=0;
  p_output1[714]=0;
  p_output1[715]=0;
  p_output1[716]=0;
  p_output1[717]=0;
  p_output1[718]=0;
  p_output1[719]=0;
  p_output1[720]=0;
  p_output1[721]=0;
  p_output1[722]=0;
  p_output1[723]=0;
  p_output1[724]=0;
  p_output1[725]=0;
  p_output1[726]=0;
  p_output1[727]=0;
  p_output1[728]=0;
  p_output1[729]=0;
  p_output1[730]=0;
  p_output1[731]=0;
  p_output1[732]=0;
  p_output1[733]=0;
  p_output1[734]=0;
  p_output1[735]=0;
  p_output1[736]=0;
  p_output1[737]=0;
  p_output1[738]=0;
  p_output1[739]=0;
  p_output1[740]=0;
  p_output1[741]=0;
  p_output1[742]=0;
  p_output1[743]=0;
  p_output1[744]=0;
  p_output1[745]=0;
  p_output1[746]=0;
  p_output1[747]=0;
  p_output1[748]=0;
  p_output1[749]=0;
  p_output1[750]=0;
  p_output1[751]=0;
  p_output1[752]=0;
  p_output1[753]=0;
  p_output1[754]=0;
  p_output1[755]=0;
  p_output1[756]=0;
  p_output1[757]=0;
  p_output1[758]=0;
  p_output1[759]=0;
  p_output1[760]=0;
  p_output1[761]=0;
  p_output1[762]=0;
  p_output1[763]=0;
  p_output1[764]=0;
  p_output1[765]=0;
  p_output1[766]=0;
  p_output1[767]=0;
  p_output1[768]=0;
  p_output1[769]=0;
  p_output1[770]=0;
  p_output1[771]=0;
  p_output1[772]=0;
  p_output1[773]=0;
  p_output1[774]=0;
  p_output1[775]=0;
  p_output1[776]=0;
  p_output1[777]=0;
  p_output1[778]=0;
  p_output1[779]=0;
  p_output1[780]=0;
  p_output1[781]=0;
  p_output1[782]=0;
  p_output1[783]=0;
  p_output1[784]=0;
  p_output1[785]=0;
  p_output1[786]=0;
  p_output1[787]=0;
  p_output1[788]=0;
  p_output1[789]=0;
  p_output1[790]=0;
  p_output1[791]=0;
  p_output1[792]=0;
  p_output1[793]=0;
  p_output1[794]=0;
  p_output1[795]=0;
  p_output1[796]=0;
  p_output1[797]=0;
  p_output1[798]=0;
  p_output1[799]=0;
  p_output1[800]=0;
  p_output1[801]=0;
  p_output1[802]=0;
  p_output1[803]=0;
  p_output1[804]=0;
  p_output1[805]=0;
  p_output1[806]=0;
  p_output1[807]=0;
  p_output1[808]=0;
  p_output1[809]=0;
  p_output1[810]=0;
  p_output1[811]=0;
  p_output1[812]=0;
  p_output1[813]=0;
  p_output1[814]=0;
  p_output1[815]=0;
  p_output1[816]=0;
  p_output1[817]=0;
  p_output1[818]=0;
  p_output1[819]=0;
  p_output1[820]=0;
  p_output1[821]=0;
  p_output1[822]=0;
  p_output1[823]=0;
  p_output1[824]=0;
  p_output1[825]=0;
  p_output1[826]=0;
  p_output1[827]=0;
  p_output1[828]=0;
  p_output1[829]=0;
  p_output1[830]=0;
  p_output1[831]=0;
  p_output1[832]=0;
  p_output1[833]=0;
  p_output1[834]=0;
  p_output1[835]=0;
  p_output1[836]=0;
  p_output1[837]=0;
  p_output1[838]=0;
  p_output1[839]=0;
  p_output1[840]=0;
  p_output1[841]=0;
  p_output1[842]=0;
  p_output1[843]=0;
  p_output1[844]=0;
  p_output1[845]=0;
  p_output1[846]=0;
  p_output1[847]=0;
  p_output1[848]=0;
  p_output1[849]=0;
  p_output1[850]=0;
  p_output1[851]=0;
  p_output1[852]=0;
  p_output1[853]=0;
  p_output1[854]=0;
  p_output1[855]=0;
  p_output1[856]=0;
  p_output1[857]=0;
  p_output1[858]=0;
  p_output1[859]=0;
  p_output1[860]=0;
  p_output1[861]=0;
  p_output1[862]=0;
  p_output1[863]=0;
  p_output1[864]=0;
  p_output1[865]=0;
  p_output1[866]=0;
  p_output1[867]=0;
  p_output1[868]=0;
  p_output1[869]=0;
  p_output1[870]=0;
  p_output1[871]=0;
  p_output1[872]=0;
  p_output1[873]=0;
  p_output1[874]=0;
  p_output1[875]=0;
  p_output1[876]=0;
  p_output1[877]=0;
  p_output1[878]=0;
  p_output1[879]=0;
  p_output1[880]=0;
  p_output1[881]=0;
  p_output1[882]=0;
  p_output1[883]=0;
  p_output1[884]=0;
  p_output1[885]=0;
  p_output1[886]=0;
  p_output1[887]=0;
  p_output1[888]=0;
  p_output1[889]=0;
  p_output1[890]=0;
  p_output1[891]=0;
  p_output1[892]=0;
  p_output1[893]=0;
  p_output1[894]=0;
  p_output1[895]=0;
  p_output1[896]=0;
  p_output1[897]=0;
  p_output1[898]=0;
  p_output1[899]=0;
}



#ifdef MATLAB_MEX_FILE

#include "mex.h"
/*
 * Main function
 */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  size_t mrows, ncols;

  double *var1;
  double *p_output1;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "One input(s) required (var1).");
    }
  else if( nlhs > 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
    }

  /*  The input must be a noncomplex double vector or scaler.  */
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);
  if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
    ( !(mrows == 30 && ncols == 1) && 
      !(mrows == 1 && ncols == 30))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "var1 is wrong.");
    }

  /*  Assign pointers to each input.  */
  var1 = mxGetPr(prhs[0]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreateDoubleMatrix((mwSize) 30, (mwSize) 30, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,var1);


}

#else // MATLAB_MEX_FILE

#include "Mmat_L10_digit.hh"

namespace SymFunction
{

void Mmat_L10_digit_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

#endif // MATLAB_MEX_FILE

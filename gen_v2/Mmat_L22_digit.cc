/*
 * Automatically Generated from Mathematica.
 * Mon 19 Jun 2023 23:03:30 GMT-04:00
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
  double t55;
  double t674;
  double t861;
  double t1371;
  double t1417;
  double t1745;
  double t1301;
  double t1767;
  double t193;
  double t849;
  double t907;
  double t1110;
  double t1369;
  double t2023;
  double t2070;
  double t3652;
  double t3206;
  double t2217;
  double t2303;
  double t2547;
  double t2820;
  double t2972;
  double t2978;
  double t5534;
  double t5627;
  double t5718;
  double t5900;
  double t5923;
  double t5992;
  double t6272;
  double t6280;
  double t6290;
  double t6292;
  double t6297;
  double t6300;
  double t6357;
  double t6362;
  double t6366;
  double t6368;
  double t6386;
  double t6393;
  double t6395;
  double t6301;
  double t6318;
  double t6321;
  double t6373;
  double t6401;
  double t6412;
  double t6417;
  double t6420;
  double t6422;
  double t6447;
  double t6527;
  double t6543;
  double t6486;
  double t6499;
  double t6647;
  double t6632;
  double t3576;
  double t3875;
  double t3901;
  double t4117;
  double t4935;
  double t5750;
  double t5756;
  double t5759;
  double t5783;
  double t6003;
  double t6043;
  double t6047;
  double t6056;
  double t6066;
  double t6092;
  double t6122;
  double t6132;
  double t6180;
  double t6190;
  double t6205;
  double t6227;
  double t6255;
  double t6331;
  double t6413;
  double t6428;
  double t6441;
  double t6443;
  double t6457;
  double t6462;
  double t6463;
  double t6464;
  double t6471;
  double t6487;
  double t6490;
  double t6494;
  double t6496;
  double t6501;
  double t6502;
  double t6512;
  double t6515;
  double t6516;
  double t6530;
  double t6531;
  double t6540;
  double t6542;
  double t6546;
  double t6547;
  double t6551;
  double t6555;
  double t6558;
  double t6569;
  double t6570;
  double t6573;
  double t6579;
  double t6584;
  double t6586;
  double t6592;
  double t6596;
  double t6597;
  double t6603;
  double t6605;
  double t6610;
  double t6617;
  double t6619;
  double t6622;
  double t6625;
  double t6629;
  double t6634;
  double t6635;
  double t6643;
  double t6649;
  double t6650;
  double t6651;
  double t6652;
  double t6656;
  double t6657;
  double t6659;
  double t6665;
  double t6668;
  double t6669;
  double t6670;
  double t6127;
  double t6195;
  double t6257;
  double t6258;
  double t6708;
  double t6709;
  double t6717;
  double t6720;
  double t6478;
  double t6518;
  double t6559;
  double t6565;
  double t6722;
  double t6723;
  double t6724;
  double t6725;
  double t6765;
  double t6766;
  double t6771;
  double t6772;
  double t6805;
  double t6806;
  double t6812;
  double t6598;
  double t6630;
  double t6654;
  double t6655;
  double t6728;
  double t6732;
  double t6733;
  double t6739;
  double t6774;
  double t6775;
  double t6776;
  double t6778;
  double t6791;
  double t6792;
  double t6797;
  double t6802;
  double t6813;
  double t6817;
  double t6819;
  double t6821;
  double t6822;
  double t6882;
  double t6884;
  double t6885;
  double t6872;
  double t6877;
  double t6879;
  double t6880;
  double t6855;
  double t6856;
  double t6858;
  double t6867;
  double t6846;
  double t6850;
  double t6851;
  double t6853;
  double t6897;
  double t6898;
  double t6904;
  double t6910;
  double t6914;
  double t6915;
  double t6892;
  double t6894;
  double t6895;
  double t6672;
  double t6673;
  double t6674;
  double t6675;
  double t6741;
  double t6745;
  double t6746;
  double t6747;
  double t6781;
  double t6783;
  double t6784;
  double t6785;
  double t6917;
  double t6921;
  double t6922;
  double t6969;
  double t6978;
  double t6979;
  double t6679;
  double t6681;
  double t6682;
  double t6748;
  double t6751;
  double t6752;
  double t6786;
  double t6789;
  double t6790;
  double t6930;
  double t6931;
  double t6932;
  double t6985;
  double t6988;
  double t6989;
  double t7008;
  double t7009;
  double t7010;
  t55 = Cos(var1[4]);
  t674 = Cos(var1[26]);
  t861 = Sin(var1[26]);
  t1371 = 0.984808*t674;
  t1417 = 0.173648*t861;
  t1745 = t1371 + t1417;
  t1301 = Sin(var1[4]);
  t1767 = Sin(var1[5]);
  t193 = Cos(var1[5]);
  t849 = 0.173648*t674;
  t907 = -0.984808*t861;
  t1110 = t849 + t907;
  t1369 = t1110*t1301;
  t2023 = -1.*t55*t1745*t1767;
  t2070 = t1369 + t2023;
  t3652 = Cos(var1[3]);
  t3206 = Sin(var1[3]);
  t2217 = t1745*t1301;
  t2303 = -0.173648*t674;
  t2547 = 0.984808*t861;
  t2820 = t2303 + t2547;
  t2972 = -1.*t55*t2820*t1767;
  t2978 = t2217 + t2972;
  t5534 = t55*t1110;
  t5627 = t1745*t1301*t1767;
  t5718 = t5534 + t5627;
  t5900 = t55*t1745;
  t5923 = t2820*t1301*t1767;
  t5992 = t5900 + t5923;
  t6272 = -1.*t674;
  t6280 = 1. + t6272;
  t6290 = -0.12*t6280;
  t6292 = -0.116892*t674;
  t6297 = 0.0005569999999999742*t861;
  t6300 = t6290 + t6292 + t6297;
  t6357 = 0.4*t6280;
  t6362 = 0.400557*t674;
  t6366 = -0.0031079999999999997*t861;
  t6368 = t6357 + t6362 + t6366;
  t6386 = -0.024405*t193;
  t6393 = -1.*t6300*t1767;
  t6395 = t6386 + t6393;
  t6301 = t193*t6300;
  t6318 = -0.024405*t1767;
  t6321 = t6301 + t6318;
  t6373 = t6368*t1301;
  t6401 = t55*t6395;
  t6412 = t6373 + t6401;
  t6417 = t55*t6368;
  t6420 = -1.*t1301*t6395;
  t6422 = t6417 + t6420;
  t6447 = -1.*t193*t1745*t6321;
  t6527 = t6321*t1767;
  t6543 = t193*t2820*t6321;
  t6486 = t193*t1745*t6321;
  t6499 = -1.*t193*t2820*t6321;
  t6647 = -1.*t6368*t1745;
  t6632 = t1110*t6368;
  t3576 = -1.*t193*t3206*t1301;
  t3875 = -1.*t3652*t1767;
  t3901 = t3576 + t3875;
  t4117 = -0.535396*t55*t193*t3901;
  t4935 = t3652*t193*t1745;
  t5750 = -1.*t3206*t5718;
  t5756 = t4935 + t5750;
  t5759 = 0.535396*t2070*t5756;
  t5783 = t3652*t193*t2820;
  t6003 = -1.*t3206*t5992;
  t6043 = t5783 + t6003;
  t6047 = 0.535396*t2978*t6043;
  t6056 = t4117 + t5759 + t6047;
  t6066 = t3652*t193*t1301;
  t6092 = -1.*t3206*t1767;
  t6122 = t6066 + t6092;
  t6132 = t193*t1745*t3206;
  t6180 = t3652*t5718;
  t6190 = t6132 + t6180;
  t6205 = t193*t2820*t3206;
  t6227 = t3652*t5992;
  t6255 = t6205 + t6227;
  t6331 = -1.*t6321*t1767;
  t6413 = -1.*t55*t193*t6412;
  t6428 = t193*t1301*t6422;
  t6441 = t6331 + t6413 + t6428;
  t6443 = t2070*t6441;
  t6457 = -1.*t2070*t6412;
  t6462 = -1.*t5718*t6422;
  t6463 = t6447 + t6457 + t6462;
  t6464 = -1.*t55*t193*t6463;
  t6471 = t6443 + t6464;
  t6487 = t2070*t6412;
  t6490 = t5718*t6422;
  t6494 = t6486 + t6487 + t6490;
  t6496 = t2978*t6494;
  t6501 = -1.*t2978*t6412;
  t6502 = -1.*t5992*t6422;
  t6512 = t6499 + t6501 + t6502;
  t6515 = t2070*t6512;
  t6516 = t6496 + t6515;
  t6530 = t55*t193*t6412;
  t6531 = -1.*t193*t1301*t6422;
  t6540 = t6527 + t6530 + t6531;
  t6542 = t2978*t6540;
  t6546 = t2978*t6412;
  t6547 = t5992*t6422;
  t6551 = t6543 + t6546 + t6547;
  t6555 = -1.*t55*t193*t6551;
  t6558 = t6542 + t6555;
  t6569 = -1.*t6321*t1767;
  t6570 = -1.*t193*t6395;
  t6573 = t6569 + t6570;
  t6579 = t193*t1745*t6573;
  t6584 = -1.*t1110*t6368;
  t6586 = t1745*t1767*t6395;
  t6592 = t6584 + t6447 + t6586;
  t6596 = -1.*t1767*t6592;
  t6597 = t6579 + t6596;
  t6603 = t193*t6395;
  t6605 = t6527 + t6603;
  t6610 = t193*t2820*t6605;
  t6617 = t6368*t1745;
  t6619 = -1.*t2820*t1767*t6395;
  t6622 = t6617 + t6543 + t6619;
  t6625 = -1.*t1767*t6622;
  t6629 = t6610 + t6625;
  t6634 = -1.*t1745*t1767*t6395;
  t6635 = t6632 + t6486 + t6634;
  t6643 = t193*t2820*t6635;
  t6649 = t2820*t1767*t6395;
  t6650 = t6647 + t6499 + t6649;
  t6651 = t193*t1745*t6650;
  t6652 = t6643 + t6651;
  t6656 = -1.*t6300*t2820;
  t6657 = t6647 + t6656;
  t6659 = t6657*t1110;
  t6665 = t6300*t1745;
  t6668 = t6632 + t6665;
  t6669 = t6668*t1745;
  t6670 = t6659 + t6669;
  t6127 = -0.535396*t55*t193*t6122;
  t6195 = 0.535396*t2070*t6190;
  t6257 = 0.535396*t2978*t6255;
  t6258 = t6127 + t6195 + t6257;
  t6708 = 0.535396*t3901*t6122;
  t6709 = 0.535396*t6190*t5756;
  t6717 = 0.535396*t6255*t6043;
  t6720 = t6708 + t6709 + t6717;
  t6478 = 0.535396*t2978*t6471;
  t6518 = -0.535396*t55*t193*t6516;
  t6559 = 0.535396*t2070*t6558;
  t6565 = t6478 + t6518 + t6559;
  t6722 = 0.535396*t6043*t6471;
  t6723 = 0.535396*t3901*t6516;
  t6724 = 0.535396*t5756*t6558;
  t6725 = t6722 + t6723 + t6724;
  t6765 = 0.535396*t6255*t6471;
  t6766 = 0.535396*t6122*t6516;
  t6771 = 0.535396*t6190*t6558;
  t6772 = t6765 + t6766 + t6771;
  t6805 = -0.000298*t55*t193;
  t6806 = -0.000035*t2070;
  t6812 = 0.000012*t2978;
  t6598 = 0.535396*t2978*t6597;
  t6630 = 0.535396*t2070*t6629;
  t6654 = -0.535396*t55*t193*t6652;
  t6655 = t6598 + t6630 + t6654;
  t6728 = 0.535396*t6043*t6597;
  t6732 = 0.535396*t5756*t6629;
  t6733 = 0.535396*t3901*t6652;
  t6739 = t6728 + t6732 + t6733;
  t6774 = 0.535396*t6255*t6597;
  t6775 = 0.535396*t6190*t6629;
  t6776 = 0.535396*t6122*t6652;
  t6778 = t6774 + t6775 + t6776;
  t6791 = 0.000035*t55*t193;
  t6792 = 0.00075*t2070;
  t6797 = -0.000014*t2978;
  t6802 = t6791 + t6792 + t6797;
  t6813 = t6805 + t6806 + t6812;
  t6817 = -0.000012*t55*t193;
  t6819 = -0.000014*t2070;
  t6821 = 0.000704*t2978;
  t6822 = t6817 + t6819 + t6821;
  t6882 = 0.535396*t6597*t6471;
  t6884 = 0.535396*t6652*t6516;
  t6885 = 0.535396*t6629*t6558;
  t6872 = -0.000014*t193*t1745;
  t6877 = 0.000704*t193*t2820;
  t6879 = -0.000012*t1767;
  t6880 = t6872 + t6877 + t6879;
  t6855 = 0.00075*t193*t1745;
  t6856 = -0.000014*t193*t2820;
  t6858 = 0.000035*t1767;
  t6867 = t6855 + t6856 + t6858;
  t6846 = -0.000035*t193*t1745;
  t6850 = 0.000012*t193*t2820;
  t6851 = -0.000298*t1767;
  t6853 = t6846 + t6850 + t6851;
  t6897 = 0.00075*t1110;
  t6898 = -0.000014*t1745;
  t6904 = t6897 + t6898;
  t6910 = -0.000014*t1110;
  t6914 = 0.000704*t1745;
  t6915 = t6910 + t6914;
  t6892 = -0.000035*t1110;
  t6894 = 0.000012*t1745;
  t6895 = t6892 + t6894;
  t6672 = -0.535396*t55*t193*t6670;
  t6673 = -0.01306633938*t1745*t2070;
  t6674 = 0.01306633938*t1110*t2978;
  t6675 = t6672 + t6673 + t6674;
  t6741 = 0.535396*t6670*t3901;
  t6745 = -0.01306633938*t1745*t5756;
  t6746 = 0.01306633938*t1110*t6043;
  t6747 = t6741 + t6745 + t6746;
  t6781 = 0.535396*t6670*t6122;
  t6783 = -0.01306633938*t1745*t6190;
  t6784 = 0.01306633938*t1110*t6255;
  t6785 = t6781 + t6783 + t6784;
  t6917 = 0.01306633938*t1110*t6471;
  t6921 = 0.535396*t6670*t6516;
  t6922 = -0.01306633938*t1745*t6558;
  t6969 = 0.01306633938*t1110*t6597;
  t6978 = -0.01306633938*t1745*t6629;
  t6979 = 0.535396*t6670*t6652;
  t6679 = 4.732939188496741e-6*t2070;
  t6681 = -0.0016905156540591966*t2978;
  t6682 = t6679 + t6681;
  t6748 = 4.732939188496741e-6*t5756;
  t6751 = -0.0016905156540591966*t6043;
  t6752 = t6748 + t6751;
  t6786 = 4.732939188496741e-6*t6190;
  t6789 = -0.0016905156540591966*t6255;
  t6790 = t6786 + t6789;
  t6930 = -0.0016905156540591966*t6471;
  t6931 = 4.732939188496741e-6*t6558;
  t6932 = t6805 + t6806 + t6812 + t6930 + t6931;
  t6985 = -0.0016905156540591966*t6597;
  t6988 = 4.732939188496741e-6*t6629;
  t6989 = t6846 + t6850 + t6851 + t6985 + t6988;
  t7008 = -0.0000762570345373147*t1110;
  t7009 = 0.000011884492619104737*t1745;
  t7010 = t7008 + t7009;
  p_output1[0]=0.535396*Power(t2070,2) + 0.535396*Power(t2978,2) + 0.535396*Power(t193,2)*Power(t55,2);
  p_output1[1]=t6056;
  p_output1[2]=t6258;
  p_output1[3]=t6565;
  p_output1[4]=t6655;
  p_output1[5]=t6675;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=0;
  p_output1[13]=0;
  p_output1[14]=0;
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
  p_output1[26]=t6682;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=t6056;
  p_output1[31]=0.535396*Power(t3901,2) + 0.535396*Power(t5756,2) + 0.535396*Power(t6043,2);
  p_output1[32]=t6720;
  p_output1[33]=t6725;
  p_output1[34]=t6739;
  p_output1[35]=t6747;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
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
  p_output1[56]=t6752;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=t6258;
  p_output1[61]=t6720;
  p_output1[62]=0.535396*Power(t6122,2) + 0.535396*Power(t6190,2) + 0.535396*Power(t6255,2);
  p_output1[63]=t6772;
  p_output1[64]=t6778;
  p_output1[65]=t6785;
  p_output1[66]=0;
  p_output1[67]=0;
  p_output1[68]=0;
  p_output1[69]=0;
  p_output1[70]=0;
  p_output1[71]=0;
  p_output1[72]=0;
  p_output1[73]=0;
  p_output1[74]=0;
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
  p_output1[86]=t6790;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=t6565;
  p_output1[91]=t6725;
  p_output1[92]=t6772;
  p_output1[93]=0.535396*Power(t6471,2) + 0.535396*Power(t6516,2) + 0.535396*Power(t6558,2) + t2070*t6802 - 1.*t193*t55*t6813 + t2978*t6822;
  p_output1[94]=-1.*t193*t55*t6853 + t2070*t6867 + t2978*t6880 + t6882 + t6884 + t6885;
  p_output1[95]=-1.*t193*t55*t6895 + t2070*t6904 + t2978*t6915 + t6917 + t6921 + t6922;
  p_output1[96]=0;
  p_output1[97]=0;
  p_output1[98]=0;
  p_output1[99]=0;
  p_output1[100]=0;
  p_output1[101]=0;
  p_output1[102]=0;
  p_output1[103]=0;
  p_output1[104]=0;
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
  p_output1[116]=t6932;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
  p_output1[120]=t6655;
  p_output1[121]=t6739;
  p_output1[122]=t6778;
  p_output1[123]=t1745*t193*t6802 - 1.*t1767*t6813 + t193*t2820*t6822 + t6882 + t6884 + t6885;
  p_output1[124]=0.535396*Power(t6597,2) + 0.535396*Power(t6629,2) + 0.535396*Power(t6652,2) - 1.*t1767*t6853 + t1745*t193*t6867 + t193*t2820*t6880;
  p_output1[125]=-1.*t1767*t6895 + t1745*t193*t6904 + t193*t2820*t6915 + t6969 + t6978 + t6979;
  p_output1[126]=0;
  p_output1[127]=0;
  p_output1[128]=0;
  p_output1[129]=0;
  p_output1[130]=0;
  p_output1[131]=0;
  p_output1[132]=0;
  p_output1[133]=0;
  p_output1[134]=0;
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
  p_output1[146]=t6989;
  p_output1[147]=0;
  p_output1[148]=0;
  p_output1[149]=0;
  p_output1[150]=t6675;
  p_output1[151]=t6747;
  p_output1[152]=t6785;
  p_output1[153]=t1110*t6802 + t1745*t6822 + t6917 + t6921 + t6922;
  p_output1[154]=t1110*t6867 + t1745*t6880 + t6969 + t6978 + t6979;
  p_output1[155]=0.0003188840125689*Power(t1110,2) + 0.0003188840125689*Power(t1745,2) + 0.535396*Power(t6670,2) + t1110*t6904 + t1745*t6915;
  p_output1[156]=0;
  p_output1[157]=0;
  p_output1[158]=0;
  p_output1[159]=0;
  p_output1[160]=0;
  p_output1[161]=0;
  p_output1[162]=0;
  p_output1[163]=0;
  p_output1[164]=0;
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
  p_output1[176]=t7010;
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
  p_output1[420]=0;
  p_output1[421]=0;
  p_output1[422]=0;
  p_output1[423]=0;
  p_output1[424]=0;
  p_output1[425]=0;
  p_output1[426]=0;
  p_output1[427]=0;
  p_output1[428]=0;
  p_output1[429]=0;
  p_output1[430]=0;
  p_output1[431]=0;
  p_output1[432]=0;
  p_output1[433]=0;
  p_output1[434]=0;
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
  p_output1[780]=t6682;
  p_output1[781]=t6752;
  p_output1[782]=t6790;
  p_output1[783]=t6932;
  p_output1[784]=t6989;
  p_output1[785]=t7010;
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
  p_output1[806]=0.0003033378538078965;
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

#include "Mmat_L22_digit.hh"

namespace SymFunction
{

void Mmat_L22_digit_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

#endif // MATLAB_MEX_FILE

/*
 * Automatically Generated from Mathematica.
 * Tue 4 Jul 2023 20:28:11 GMT-04:00
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
  double t140;
  double t197;
  double t260;
  double t200;
  double t386;
  double t326;
  double t407;
  double t503;
  double t506;
  double t512;
  double t496;
  double t513;
  double t514;
  double t525;
  double t526;
  double t527;
  double t545;
  double t548;
  double t549;
  double t232;
  double t335;
  double t569;
  double t588;
  double t401;
  double t406;
  double t414;
  double t415;
  double t417;
  double t420;
  double t427;
  double t436;
  double t438;
  double t441;
  double t458;
  double t459;
  double t460;
  double t478;
  double t484;
  double t485;
  double t522;
  double t528;
  double t532;
  double t533;
  double t552;
  double t553;
  double t554;
  double t555;
  double t557;
  double t558;
  double t570;
  double t574;
  double t575;
  double t583;
  double t584;
  double t585;
  double t592;
  double t593;
  double t594;
  double t611;
  double t612;
  double t614;
  double t619;
  double t620;
  double t622;
  double t623;
  double t624;
  double t625;
  double t626;
  double t636;
  double t637;
  double t639;
  double t640;
  double t641;
  double t644;
  double t648;
  double t449;
  double t475;
  double t486;
  double t494;
  double t678;
  double t683;
  double t700;
  double t701;
  double t561;
  double t608;
  double t629;
  double t632;
  double t708;
  double t712;
  double t713;
  double t714;
  double t748;
  double t754;
  double t755;
  double t759;
  double t792;
  double t793;
  double t794;
  double t634;
  double t635;
  double t649;
  double t651;
  double t715;
  double t717;
  double t718;
  double t719;
  double t760;
  double t763;
  double t764;
  double t765;
  double t778;
  double t779;
  double t787;
  double t788;
  double t797;
  double t798;
  double t799;
  double t800;
  double t840;
  double t844;
  double t845;
  double t835;
  double t836;
  double t838;
  double t826;
  double t827;
  double t829;
  double t652;
  double t662;
  double t663;
  double t720;
  double t721;
  double t722;
  double t767;
  double t773;
  double t774;
  double t854;
  double t855;
  double t856;
  double t878;
  double t879;
  double t882;
  t140 = Cos(var1[4]);
  t197 = Power(t140,2);
  t260 = Sin(var1[4]);
  t200 = Cos(var1[5]);
  t386 = Sin(var1[3]);
  t326 = Sin(var1[5]);
  t407 = Cos(var1[3]);
  t503 = 0.001637*t200;
  t506 = -0.0002*t326;
  t512 = t503 + t506;
  t496 = 0.259547*t260;
  t513 = t140*t512;
  t514 = t496 + t513;
  t525 = 0.259547*t140;
  t526 = -1.*t260*t512;
  t527 = t525 + t526;
  t545 = 0.0002*t200;
  t548 = 0.001637*t326;
  t549 = t545 + t548;
  t232 = Power(t200,2);
  t335 = Power(t326,2);
  t569 = t200*t549;
  t588 = -1.*t549*t326;
  t401 = -15.028392*t140*t386*t260;
  t406 = t200*t386*t260;
  t414 = t407*t326;
  t415 = t406 + t414;
  t417 = 15.028392*t140*t200*t415;
  t420 = t407*t200;
  t427 = -1.*t386*t260*t326;
  t436 = t420 + t427;
  t438 = -15.028392*t140*t326*t436;
  t441 = t401 + t417 + t438;
  t458 = -1.*t407*t200*t260;
  t459 = t386*t326;
  t460 = t458 + t459;
  t478 = t200*t386;
  t484 = t407*t260*t326;
  t485 = t478 + t484;
  t522 = t260*t514;
  t528 = t140*t527;
  t532 = t522 + t528;
  t533 = -1.*t140*t532*t326;
  t552 = -1.*t200*t549;
  t553 = t140*t514*t326;
  t554 = -1.*t260*t527*t326;
  t555 = t552 + t553 + t554;
  t557 = t260*t555;
  t558 = t533 + t557;
  t570 = -1.*t140*t514*t326;
  t574 = t260*t527*t326;
  t575 = t569 + t570 + t574;
  t583 = t140*t200*t575;
  t584 = -1.*t140*t200*t514;
  t585 = t200*t260*t527;
  t592 = t584 + t585 + t588;
  t593 = -1.*t140*t326*t592;
  t594 = t583 + t593;
  t611 = -1.*t260*t514;
  t612 = -1.*t140*t527;
  t614 = t611 + t612;
  t619 = t140*t200*t614;
  t620 = t140*t200*t514;
  t622 = -1.*t200*t260*t527;
  t623 = t549*t326;
  t624 = t620 + t622 + t623;
  t625 = t260*t624;
  t626 = t619 + t625;
  t636 = -1.*t512*t326;
  t637 = t569 + t636;
  t639 = t326*t637;
  t640 = -1.*t200*t512;
  t641 = t640 + t588;
  t644 = t200*t641;
  t648 = t639 + t644;
  t449 = 15.028392*t407*t140*t260;
  t475 = 15.028392*t140*t200*t460;
  t486 = -15.028392*t140*t326*t485;
  t494 = t449 + t475 + t486;
  t678 = -15.028392*t407*t197*t386;
  t683 = 15.028392*t415*t460;
  t700 = 15.028392*t485*t436;
  t701 = t678 + t683 + t700;
  t561 = 15.028392*t140*t200*t558;
  t608 = 15.028392*t260*t594;
  t629 = -15.028392*t140*t326*t626;
  t632 = t561 + t608 + t629;
  t708 = 15.028392*t415*t558;
  t712 = -15.028392*t140*t386*t594;
  t713 = 15.028392*t436*t626;
  t714 = t708 + t712 + t713;
  t748 = 15.028392*t460*t558;
  t754 = 15.028392*t407*t140*t594;
  t755 = 15.028392*t485*t626;
  t759 = t748 + t754 + t755;
  t792 = 0.008378*t140*t200;
  t793 = 0.100648*t260;
  t794 = -0.000067*t140*t326;
  t634 = 3.9005740584240005*t140*t232;
  t635 = 3.9005740584240005*t140*t335;
  t649 = 15.028392*t260*t648;
  t651 = t634 + t635 + t649;
  t715 = 3.9005740584240005*t200*t415;
  t717 = -3.9005740584240005*t326*t436;
  t718 = -15.028392*t140*t386*t648;
  t719 = t715 + t717 + t718;
  t760 = 3.9005740584240005*t200*t460;
  t763 = -3.9005740584240005*t326*t485;
  t764 = 15.028392*t407*t140*t648;
  t765 = t760 + t763 + t764;
  t778 = -0.000088*t140*t200;
  t779 = 0.000067*t260;
  t787 = -0.342655*t140*t326;
  t788 = t778 + t779 + t787;
  t797 = 0.376284*t140*t200;
  t798 = 0.008378*t260;
  t799 = 0.000088*t140*t326;
  t800 = t797 + t798 + t799;
  t840 = 3.9005740584240005*t200*t558;
  t844 = 15.028392*t594*t648;
  t845 = -3.9005740584240005*t326*t626;
  t835 = 0.342655*t200;
  t836 = -0.000088*t326;
  t838 = t835 + t836;
  t826 = -0.000088*t200;
  t827 = 0.376284*t326;
  t829 = t826 + t827;
  t652 = -0.0030056784*t140*t200;
  t662 = -0.024601477704*t140*t326;
  t663 = t652 + t662;
  t720 = -0.0030056784*t415;
  t721 = 0.024601477704*t436;
  t722 = t720 + t721;
  t767 = -0.0030056784*t460;
  t773 = 0.024601477704*t485;
  t774 = t767 + t773;
  t854 = -0.0030056784*t558;
  t855 = 0.024601477704*t626;
  t856 = t792 + t793 + t794 + t854 + t855;
  t878 = -0.0007131148116848001*t200;
  t879 = 0.0019927602663599112*t326;
  t882 = t878 + t879;
  p_output1[0]=15.028392*t197*t232 + 15.028392*Power(t260,2) + 15.028392*t197*t335;
  p_output1[1]=t441;
  p_output1[2]=t494;
  p_output1[3]=t632;
  p_output1[4]=t651;
  p_output1[5]=t663;
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
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=t441;
  p_output1[31]=15.028392*t197*Power(t386,2) + 15.028392*Power(t415,2) + 15.028392*Power(t436,2);
  p_output1[32]=t701;
  p_output1[33]=t714;
  p_output1[34]=t719;
  p_output1[35]=t722;
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
  p_output1[56]=0;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=t494;
  p_output1[61]=t701;
  p_output1[62]=15.028392*t197*Power(t407,2) + 15.028392*Power(t460,2) + 15.028392*Power(t485,2);
  p_output1[63]=t759;
  p_output1[64]=t765;
  p_output1[65]=t774;
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
  p_output1[86]=0;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=t632;
  p_output1[91]=t714;
  p_output1[92]=t759;
  p_output1[93]=15.028392*Power(t558,2) + 15.028392*Power(t594,2) + 15.028392*Power(t626,2) - 1.*t140*t326*t788 + t260*(t792 + t793 + t794) + t140*t200*t800;
  p_output1[94]=t260*(0.000067*t200 + 0.008378*t326) + t140*t200*t829 - 1.*t140*t326*t838 + t840 + t844 + t845;
  p_output1[95]=t856;
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
  p_output1[116]=0;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
  p_output1[120]=t651;
  p_output1[121]=t719;
  p_output1[122]=t765;
  p_output1[123]=t200*t788 + t326*t800 + t840 + t844 + t845;
  p_output1[124]=1.0123822951417742*t232 + 1.0123822951417742*t335 + 15.028392*Power(t648,2) + t326*t829 + t200*t838;
  p_output1[125]=t882;
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
  p_output1[146]=0;
  p_output1[147]=0;
  p_output1[148]=0;
  p_output1[149]=0;
  p_output1[150]=t663;
  p_output1[151]=t722;
  p_output1[152]=t774;
  p_output1[153]=t856;
  p_output1[154]=t882;
  p_output1[155]=0.10068887375468145;
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

#include "Mmat_L1_digit.hh"

namespace SymFunction
{

void Mmat_L1_digit_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

#endif // MATLAB_MEX_FILE

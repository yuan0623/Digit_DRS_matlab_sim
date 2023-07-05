/*
 * Automatically Generated from Mathematica.
 * Tue 4 Jul 2023 15:39:32 GMT-04:00
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
  double t813;
  double t2344;
  double t2453;
  double t2490;
  double t2502;
  double t2505;
  double t2471;
  double t2506;
  double t2315;
  double t2432;
  double t2465;
  double t2466;
  double t2487;
  double t2518;
  double t2522;
  double t2649;
  double t2638;
  double t2544;
  double t2551;
  double t2562;
  double t2596;
  double t2607;
  double t2608;
  double t2704;
  double t2708;
  double t2722;
  double t2755;
  double t2760;
  double t2762;
  double t2864;
  double t2866;
  double t2868;
  double t2874;
  double t2875;
  double t2878;
  double t2895;
  double t2903;
  double t2909;
  double t2910;
  double t2925;
  double t2931;
  double t2935;
  double t2879;
  double t2888;
  double t2889;
  double t2914;
  double t2936;
  double t2937;
  double t2947;
  double t2950;
  double t2951;
  double t2980;
  double t3026;
  double t3035;
  double t2999;
  double t3009;
  double t3107;
  double t3101;
  double t2643;
  double t2652;
  double t2684;
  double t2696;
  double t2698;
  double t2723;
  double t2729;
  double t2738;
  double t2749;
  double t2767;
  double t2769;
  double t2772;
  double t2774;
  double t2776;
  double t2807;
  double t2808;
  double t2814;
  double t2819;
  double t2828;
  double t2837;
  double t2845;
  double t2855;
  double t2890;
  double t2944;
  double t2967;
  double t2970;
  double t2971;
  double t2981;
  double t2982;
  double t2986;
  double t2987;
  double t2989;
  double t3000;
  double t3002;
  double t3004;
  double t3005;
  double t3010;
  double t3011;
  double t3016;
  double t3023;
  double t3024;
  double t3030;
  double t3031;
  double t3032;
  double t3033;
  double t3037;
  double t3038;
  double t3039;
  double t3040;
  double t3054;
  double t3059;
  double t3061;
  double t3069;
  double t3070;
  double t3071;
  double t3072;
  double t3074;
  double t3075;
  double t3076;
  double t3080;
  double t3085;
  double t3086;
  double t3088;
  double t3089;
  double t3091;
  double t3092;
  double t3097;
  double t3103;
  double t3104;
  double t3105;
  double t3109;
  double t3110;
  double t3111;
  double t3112;
  double t3123;
  double t3124;
  double t3127;
  double t3129;
  double t3130;
  double t3131;
  double t3132;
  double t2809;
  double t2836;
  double t2856;
  double t2857;
  double t3168;
  double t3169;
  double t3171;
  double t3173;
  double t2993;
  double t3025;
  double t3055;
  double t3056;
  double t3175;
  double t3176;
  double t3177;
  double t3179;
  double t3223;
  double t3230;
  double t3231;
  double t3232;
  double t3264;
  double t3265;
  double t3269;
  double t3077;
  double t3099;
  double t3116;
  double t3119;
  double t3182;
  double t3183;
  double t3184;
  double t3185;
  double t3234;
  double t3236;
  double t3237;
  double t3238;
  double t3258;
  double t3259;
  double t3260;
  double t3262;
  double t3272;
  double t3274;
  double t3281;
  double t3283;
  double t3284;
  double t3341;
  double t3343;
  double t3344;
  double t3327;
  double t3328;
  double t3332;
  double t3337;
  double t3312;
  double t3320;
  double t3321;
  double t3322;
  double t3305;
  double t3306;
  double t3307;
  double t3308;
  double t3365;
  double t3366;
  double t3367;
  double t3372;
  double t3373;
  double t3374;
  double t3359;
  double t3362;
  double t3363;
  double t3135;
  double t3137;
  double t3138;
  double t3142;
  double t3190;
  double t3194;
  double t3195;
  double t3196;
  double t3242;
  double t3244;
  double t3245;
  double t3249;
  double t3376;
  double t3377;
  double t3378;
  double t3405;
  double t3406;
  double t3407;
  double t3149;
  double t3150;
  double t3152;
  double t3197;
  double t3199;
  double t3201;
  double t3250;
  double t3255;
  double t3257;
  double t3382;
  double t3383;
  double t3384;
  double t3412;
  double t3413;
  double t3414;
  double t3432;
  double t3436;
  double t3437;
  t813 = Cos(var1[4]);
  t2344 = Cos(var1[26]);
  t2453 = Sin(var1[26]);
  t2490 = 0.984808*t2344;
  t2502 = 0.173648*t2453;
  t2505 = t2490 + t2502;
  t2471 = Sin(var1[4]);
  t2506 = Sin(var1[5]);
  t2315 = Cos(var1[5]);
  t2432 = 0.173648*t2344;
  t2465 = -0.984808*t2453;
  t2466 = t2432 + t2465;
  t2487 = t2466*t2471;
  t2518 = -1.*t813*t2505*t2506;
  t2522 = t2487 + t2518;
  t2649 = Cos(var1[3]);
  t2638 = Sin(var1[3]);
  t2544 = t2505*t2471;
  t2551 = -0.173648*t2344;
  t2562 = 0.984808*t2453;
  t2596 = t2551 + t2562;
  t2607 = -1.*t813*t2596*t2506;
  t2608 = t2544 + t2607;
  t2704 = t813*t2466;
  t2708 = t2505*t2471*t2506;
  t2722 = t2704 + t2708;
  t2755 = t813*t2505;
  t2760 = t2596*t2471*t2506;
  t2762 = t2755 + t2760;
  t2864 = -1.*t2344;
  t2866 = 1. + t2864;
  t2868 = -0.12*t2866;
  t2874 = -0.116892*t2344;
  t2875 = 0.0005569999999999742*t2453;
  t2878 = t2868 + t2874 + t2875;
  t2895 = 0.4*t2866;
  t2903 = 0.400557*t2344;
  t2909 = -0.0031079999999999997*t2453;
  t2910 = t2895 + t2903 + t2909;
  t2925 = -0.024405*t2315;
  t2931 = -1.*t2878*t2506;
  t2935 = t2925 + t2931;
  t2879 = t2315*t2878;
  t2888 = -0.024405*t2506;
  t2889 = t2879 + t2888;
  t2914 = t2910*t2471;
  t2936 = t813*t2935;
  t2937 = t2914 + t2936;
  t2947 = t813*t2910;
  t2950 = -1.*t2471*t2935;
  t2951 = t2947 + t2950;
  t2980 = -1.*t2315*t2505*t2889;
  t3026 = t2889*t2506;
  t3035 = t2315*t2596*t2889;
  t2999 = t2315*t2505*t2889;
  t3009 = -1.*t2315*t2596*t2889;
  t3107 = -1.*t2910*t2505;
  t3101 = t2466*t2910;
  t2643 = -1.*t2315*t2638*t2471;
  t2652 = -1.*t2649*t2506;
  t2684 = t2643 + t2652;
  t2696 = -0.535396*t813*t2315*t2684;
  t2698 = t2649*t2315*t2505;
  t2723 = -1.*t2638*t2722;
  t2729 = t2698 + t2723;
  t2738 = 0.535396*t2522*t2729;
  t2749 = t2649*t2315*t2596;
  t2767 = -1.*t2638*t2762;
  t2769 = t2749 + t2767;
  t2772 = 0.535396*t2608*t2769;
  t2774 = t2696 + t2738 + t2772;
  t2776 = t2649*t2315*t2471;
  t2807 = -1.*t2638*t2506;
  t2808 = t2776 + t2807;
  t2814 = t2315*t2505*t2638;
  t2819 = t2649*t2722;
  t2828 = t2814 + t2819;
  t2837 = t2315*t2596*t2638;
  t2845 = t2649*t2762;
  t2855 = t2837 + t2845;
  t2890 = -1.*t2889*t2506;
  t2944 = -1.*t813*t2315*t2937;
  t2967 = t2315*t2471*t2951;
  t2970 = t2890 + t2944 + t2967;
  t2971 = t2522*t2970;
  t2981 = -1.*t2522*t2937;
  t2982 = -1.*t2722*t2951;
  t2986 = t2980 + t2981 + t2982;
  t2987 = -1.*t813*t2315*t2986;
  t2989 = t2971 + t2987;
  t3000 = t2522*t2937;
  t3002 = t2722*t2951;
  t3004 = t2999 + t3000 + t3002;
  t3005 = t2608*t3004;
  t3010 = -1.*t2608*t2937;
  t3011 = -1.*t2762*t2951;
  t3016 = t3009 + t3010 + t3011;
  t3023 = t2522*t3016;
  t3024 = t3005 + t3023;
  t3030 = t813*t2315*t2937;
  t3031 = -1.*t2315*t2471*t2951;
  t3032 = t3026 + t3030 + t3031;
  t3033 = t2608*t3032;
  t3037 = t2608*t2937;
  t3038 = t2762*t2951;
  t3039 = t3035 + t3037 + t3038;
  t3040 = -1.*t813*t2315*t3039;
  t3054 = t3033 + t3040;
  t3059 = -1.*t2889*t2506;
  t3061 = -1.*t2315*t2935;
  t3069 = t3059 + t3061;
  t3070 = t2315*t2505*t3069;
  t3071 = -1.*t2466*t2910;
  t3072 = t2505*t2506*t2935;
  t3074 = t3071 + t2980 + t3072;
  t3075 = -1.*t2506*t3074;
  t3076 = t3070 + t3075;
  t3080 = t2315*t2935;
  t3085 = t3026 + t3080;
  t3086 = t2315*t2596*t3085;
  t3088 = t2910*t2505;
  t3089 = -1.*t2596*t2506*t2935;
  t3091 = t3088 + t3035 + t3089;
  t3092 = -1.*t2506*t3091;
  t3097 = t3086 + t3092;
  t3103 = -1.*t2505*t2506*t2935;
  t3104 = t3101 + t2999 + t3103;
  t3105 = t2315*t2596*t3104;
  t3109 = t2596*t2506*t2935;
  t3110 = t3107 + t3009 + t3109;
  t3111 = t2315*t2505*t3110;
  t3112 = t3105 + t3111;
  t3123 = -1.*t2878*t2596;
  t3124 = t3107 + t3123;
  t3127 = t3124*t2466;
  t3129 = t2878*t2505;
  t3130 = t3101 + t3129;
  t3131 = t3130*t2505;
  t3132 = t3127 + t3131;
  t2809 = -0.535396*t813*t2315*t2808;
  t2836 = 0.535396*t2522*t2828;
  t2856 = 0.535396*t2608*t2855;
  t2857 = t2809 + t2836 + t2856;
  t3168 = 0.535396*t2684*t2808;
  t3169 = 0.535396*t2828*t2729;
  t3171 = 0.535396*t2855*t2769;
  t3173 = t3168 + t3169 + t3171;
  t2993 = 0.535396*t2608*t2989;
  t3025 = -0.535396*t813*t2315*t3024;
  t3055 = 0.535396*t2522*t3054;
  t3056 = t2993 + t3025 + t3055;
  t3175 = 0.535396*t2769*t2989;
  t3176 = 0.535396*t2684*t3024;
  t3177 = 0.535396*t2729*t3054;
  t3179 = t3175 + t3176 + t3177;
  t3223 = 0.535396*t2855*t2989;
  t3230 = 0.535396*t2808*t3024;
  t3231 = 0.535396*t2828*t3054;
  t3232 = t3223 + t3230 + t3231;
  t3264 = -0.000298*t813*t2315;
  t3265 = -0.000035*t2522;
  t3269 = 0.000012*t2608;
  t3077 = 0.535396*t2608*t3076;
  t3099 = 0.535396*t2522*t3097;
  t3116 = -0.535396*t813*t2315*t3112;
  t3119 = t3077 + t3099 + t3116;
  t3182 = 0.535396*t2769*t3076;
  t3183 = 0.535396*t2729*t3097;
  t3184 = 0.535396*t2684*t3112;
  t3185 = t3182 + t3183 + t3184;
  t3234 = 0.535396*t2855*t3076;
  t3236 = 0.535396*t2828*t3097;
  t3237 = 0.535396*t2808*t3112;
  t3238 = t3234 + t3236 + t3237;
  t3258 = 0.000035*t813*t2315;
  t3259 = 0.00075*t2522;
  t3260 = -0.000014*t2608;
  t3262 = t3258 + t3259 + t3260;
  t3272 = t3264 + t3265 + t3269;
  t3274 = -0.000012*t813*t2315;
  t3281 = -0.000014*t2522;
  t3283 = 0.000704*t2608;
  t3284 = t3274 + t3281 + t3283;
  t3341 = 0.535396*t3076*t2989;
  t3343 = 0.535396*t3112*t3024;
  t3344 = 0.535396*t3097*t3054;
  t3327 = -0.000014*t2315*t2505;
  t3328 = 0.000704*t2315*t2596;
  t3332 = -0.000012*t2506;
  t3337 = t3327 + t3328 + t3332;
  t3312 = 0.00075*t2315*t2505;
  t3320 = -0.000014*t2315*t2596;
  t3321 = 0.000035*t2506;
  t3322 = t3312 + t3320 + t3321;
  t3305 = -0.000035*t2315*t2505;
  t3306 = 0.000012*t2315*t2596;
  t3307 = -0.000298*t2506;
  t3308 = t3305 + t3306 + t3307;
  t3365 = 0.00075*t2466;
  t3366 = -0.000014*t2505;
  t3367 = t3365 + t3366;
  t3372 = -0.000014*t2466;
  t3373 = 0.000704*t2505;
  t3374 = t3372 + t3373;
  t3359 = -0.000035*t2466;
  t3362 = 0.000012*t2505;
  t3363 = t3359 + t3362;
  t3135 = -0.535396*t813*t2315*t3132;
  t3137 = -0.01306633938*t2505*t2522;
  t3138 = 0.01306633938*t2466*t2608;
  t3142 = t3135 + t3137 + t3138;
  t3190 = 0.535396*t3132*t2684;
  t3194 = -0.01306633938*t2505*t2729;
  t3195 = 0.01306633938*t2466*t2769;
  t3196 = t3190 + t3194 + t3195;
  t3242 = 0.535396*t3132*t2808;
  t3244 = -0.01306633938*t2505*t2828;
  t3245 = 0.01306633938*t2466*t2855;
  t3249 = t3242 + t3244 + t3245;
  t3376 = 0.01306633938*t2466*t2989;
  t3377 = 0.535396*t3132*t3024;
  t3378 = -0.01306633938*t2505*t3054;
  t3405 = 0.01306633938*t2466*t3076;
  t3406 = -0.01306633938*t2505*t3097;
  t3407 = 0.535396*t3132*t3112;
  t3149 = 4.732939188496741e-6*t2522;
  t3150 = -0.0016905156540591966*t2608;
  t3152 = t3149 + t3150;
  t3197 = 4.732939188496741e-6*t2729;
  t3199 = -0.0016905156540591966*t2769;
  t3201 = t3197 + t3199;
  t3250 = 4.732939188496741e-6*t2828;
  t3255 = -0.0016905156540591966*t2855;
  t3257 = t3250 + t3255;
  t3382 = -0.0016905156540591966*t2989;
  t3383 = 4.732939188496741e-6*t3054;
  t3384 = t3264 + t3265 + t3269 + t3382 + t3383;
  t3412 = -0.0016905156540591966*t3076;
  t3413 = 4.732939188496741e-6*t3097;
  t3414 = t3305 + t3306 + t3307 + t3412 + t3413;
  t3432 = -0.0000762570345373147*t2466;
  t3436 = 0.000011884492619104737*t2505;
  t3437 = t3432 + t3436;
  p_output1[0]=0.535396*Power(t2522,2) + 0.535396*Power(t2608,2) + 0.535396*Power(t2315,2)*Power(t813,2);
  p_output1[1]=t2774;
  p_output1[2]=t2857;
  p_output1[3]=t3056;
  p_output1[4]=t3119;
  p_output1[5]=t3142;
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
  p_output1[26]=t3152;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=t2774;
  p_output1[31]=0.535396*Power(t2684,2) + 0.535396*Power(t2729,2) + 0.535396*Power(t2769,2);
  p_output1[32]=t3173;
  p_output1[33]=t3179;
  p_output1[34]=t3185;
  p_output1[35]=t3196;
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
  p_output1[56]=t3201;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=t2857;
  p_output1[61]=t3173;
  p_output1[62]=0.535396*Power(t2808,2) + 0.535396*Power(t2828,2) + 0.535396*Power(t2855,2);
  p_output1[63]=t3232;
  p_output1[64]=t3238;
  p_output1[65]=t3249;
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
  p_output1[86]=t3257;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=t3056;
  p_output1[91]=t3179;
  p_output1[92]=t3232;
  p_output1[93]=0.535396*Power(t2989,2) + 0.535396*Power(t3024,2) + 0.535396*Power(t3054,2) + t2522*t3262 + t2608*t3284 - 1.*t2315*t3272*t813;
  p_output1[94]=t2522*t3322 + t2608*t3337 + t3341 + t3343 + t3344 - 1.*t2315*t3308*t813;
  p_output1[95]=t2522*t3367 + t2608*t3374 + t3376 + t3377 + t3378 - 1.*t2315*t3363*t813;
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
  p_output1[116]=t3384;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
  p_output1[120]=t3119;
  p_output1[121]=t3185;
  p_output1[122]=t3238;
  p_output1[123]=t2315*t2505*t3262 - 1.*t2506*t3272 + t2315*t2596*t3284 + t3341 + t3343 + t3344;
  p_output1[124]=0.535396*Power(t3076,2) + 0.535396*Power(t3097,2) + 0.535396*Power(t3112,2) - 1.*t2506*t3308 + t2315*t2505*t3322 + t2315*t2596*t3337;
  p_output1[125]=-1.*t2506*t3363 + t2315*t2505*t3367 + t2315*t2596*t3374 + t3405 + t3406 + t3407;
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
  p_output1[146]=t3414;
  p_output1[147]=0;
  p_output1[148]=0;
  p_output1[149]=0;
  p_output1[150]=t3142;
  p_output1[151]=t3196;
  p_output1[152]=t3249;
  p_output1[153]=t2466*t3262 + t2505*t3284 + t3376 + t3377 + t3378;
  p_output1[154]=t2466*t3322 + t2505*t3337 + t3405 + t3406 + t3407;
  p_output1[155]=0.0003188840125689*Power(t2466,2) + 0.0003188840125689*Power(t2505,2) + 0.535396*Power(t3132,2) + t2466*t3367 + t2505*t3374;
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
  p_output1[176]=t3437;
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
  p_output1[780]=t3152;
  p_output1[781]=t3201;
  p_output1[782]=t3257;
  p_output1[783]=t3384;
  p_output1[784]=t3414;
  p_output1[785]=t3437;
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

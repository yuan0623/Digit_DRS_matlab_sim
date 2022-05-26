/*
 * Automatically Generated from Mathematica.
 * Mon 2 May 2022 22:34:15 GMT-04:00
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
  double t1050;
  double t1192;
  double t1201;
  double t1260;
  double t1269;
  double t1274;
  double t1247;
  double t1278;
  double t1145;
  double t1193;
  double t1230;
  double t1237;
  double t1257;
  double t1287;
  double t1290;
  double t1415;
  double t1405;
  double t1304;
  double t1325;
  double t1329;
  double t1337;
  double t1338;
  double t1343;
  double t1457;
  double t1459;
  double t1464;
  double t1515;
  double t1521;
  double t1530;
  double t1593;
  double t1594;
  double t1597;
  double t1600;
  double t1607;
  double t1612;
  double t1623;
  double t1627;
  double t1629;
  double t1633;
  double t1638;
  double t1640;
  double t1613;
  double t1614;
  double t1621;
  double t1631;
  double t1643;
  double t1645;
  double t1648;
  double t1651;
  double t1656;
  double t1661;
  double t1710;
  double t1746;
  double t1678;
  double t1688;
  double t1830;
  double t1824;
  double t1414;
  double t1424;
  double t1426;
  double t1442;
  double t1453;
  double t1480;
  double t1506;
  double t1507;
  double t1508;
  double t1532;
  double t1534;
  double t1535;
  double t1536;
  double t1537;
  double t1546;
  double t1550;
  double t1552;
  double t1557;
  double t1564;
  double t1572;
  double t1573;
  double t1588;
  double t1622;
  double t1646;
  double t1657;
  double t1658;
  double t1660;
  double t1664;
  double t1666;
  double t1668;
  double t1669;
  double t1670;
  double t1680;
  double t1682;
  double t1683;
  double t1686;
  double t1689;
  double t1695;
  double t1700;
  double t1704;
  double t1705;
  double t1712;
  double t1714;
  double t1721;
  double t1742;
  double t1747;
  double t1751;
  double t1752;
  double t1753;
  double t1754;
  double t1774;
  double t1775;
  double t1781;
  double t1782;
  double t1787;
  double t1788;
  double t1792;
  double t1793;
  double t1794;
  double t1799;
  double t1806;
  double t1810;
  double t1811;
  double t1812;
  double t1813;
  double t1821;
  double t1822;
  double t1827;
  double t1828;
  double t1829;
  double t1831;
  double t1834;
  double t1835;
  double t1836;
  double t1843;
  double t1846;
  double t1847;
  double t1848;
  double t1849;
  double t1858;
  double t1859;
  double t1551;
  double t1567;
  double t1590;
  double t1592;
  double t1893;
  double t1898;
  double t1899;
  double t1903;
  double t1672;
  double t1708;
  double t1762;
  double t1764;
  double t1905;
  double t1908;
  double t1909;
  double t1917;
  double t1951;
  double t1955;
  double t1956;
  double t1957;
  double t1992;
  double t1995;
  double t2000;
  double t1795;
  double t1823;
  double t1841;
  double t1842;
  double t1918;
  double t1923;
  double t1924;
  double t1927;
  double t1958;
  double t1959;
  double t1960;
  double t1962;
  double t1982;
  double t1983;
  double t1984;
  double t1985;
  double t2001;
  double t2005;
  double t2006;
  double t2010;
  double t2011;
  double t2047;
  double t2048;
  double t2051;
  double t2042;
  double t2043;
  double t2044;
  double t2045;
  double t2034;
  double t2038;
  double t2039;
  double t2040;
  double t2026;
  double t2027;
  double t2028;
  double t2032;
  double t2057;
  double t2058;
  double t2064;
  double t2066;
  double t2067;
  double t2070;
  double t2053;
  double t2054;
  double t2055;
  double t1860;
  double t1861;
  double t1862;
  double t1869;
  double t1931;
  double t1932;
  double t1933;
  double t1935;
  double t1968;
  double t1973;
  double t1975;
  double t1976;
  double t2072;
  double t2073;
  double t2074;
  double t2106;
  double t2107;
  double t2108;
  double t1870;
  double t1871;
  double t1876;
  double t1936;
  double t1937;
  double t1938;
  double t1977;
  double t1978;
  double t1980;
  double t2076;
  double t2077;
  double t2078;
  double t2112;
  double t2113;
  double t2114;
  double t2154;
  double t2155;
  double t2156;
  t1050 = Cos(var1[4]);
  t1192 = Cos(var1[18]);
  t1201 = Sin(var1[18]);
  t1260 = 0.930418*t1192;
  t1269 = -0.366501*t1201;
  t1274 = t1260 + t1269;
  t1247 = Sin(var1[4]);
  t1278 = Sin(var1[5]);
  t1145 = Cos(var1[5]);
  t1193 = -0.366501*t1192;
  t1230 = -0.930418*t1201;
  t1237 = t1193 + t1230;
  t1257 = t1237*t1247;
  t1287 = -1.*t1050*t1274*t1278;
  t1290 = t1257 + t1287;
  t1415 = Cos(var1[3]);
  t1405 = Sin(var1[3]);
  t1304 = t1274*t1247;
  t1325 = 0.366501*t1192;
  t1329 = 0.930418*t1201;
  t1337 = t1325 + t1329;
  t1338 = -1.*t1050*t1337*t1278;
  t1343 = t1304 + t1338;
  t1457 = t1050*t1237;
  t1459 = t1274*t1247*t1278;
  t1464 = t1457 + t1459;
  t1515 = t1050*t1274;
  t1521 = t1337*t1247*t1278;
  t1530 = t1515 + t1521;
  t1593 = -1.*t1192;
  t1594 = 1. + t1593;
  t1597 = -0.091*t1594;
  t1600 = -0.091948*t1192;
  t1607 = -0.001741*t1201;
  t1612 = t1597 + t1600 + t1607;
  t1623 = -0.001741*t1192;
  t1627 = 0.0009480000000000044*t1201;
  t1629 = t1623 + t1627;
  t1633 = -0.032435*t1145;
  t1638 = -1.*t1612*t1278;
  t1640 = t1633 + t1638;
  t1613 = t1145*t1612;
  t1614 = -0.032435*t1278;
  t1621 = t1613 + t1614;
  t1631 = t1629*t1247;
  t1643 = t1050*t1640;
  t1645 = t1631 + t1643;
  t1648 = t1050*t1629;
  t1651 = -1.*t1247*t1640;
  t1656 = t1648 + t1651;
  t1661 = -1.*t1145*t1274*t1621;
  t1710 = t1621*t1278;
  t1746 = t1145*t1337*t1621;
  t1678 = t1145*t1274*t1621;
  t1688 = -1.*t1145*t1337*t1621;
  t1830 = -1.*t1274*t1629;
  t1824 = t1237*t1629;
  t1414 = -1.*t1145*t1405*t1247;
  t1424 = -1.*t1415*t1278;
  t1426 = t1414 + t1424;
  t1442 = -0.915088*t1050*t1145*t1426;
  t1453 = t1415*t1145*t1274;
  t1480 = -1.*t1405*t1464;
  t1506 = t1453 + t1480;
  t1507 = 0.915088*t1290*t1506;
  t1508 = t1415*t1145*t1337;
  t1532 = -1.*t1405*t1530;
  t1534 = t1508 + t1532;
  t1535 = 0.915088*t1343*t1534;
  t1536 = t1442 + t1507 + t1535;
  t1537 = t1415*t1145*t1247;
  t1546 = -1.*t1405*t1278;
  t1550 = t1537 + t1546;
  t1552 = t1145*t1274*t1405;
  t1557 = t1415*t1464;
  t1564 = t1552 + t1557;
  t1572 = t1145*t1337*t1405;
  t1573 = t1415*t1530;
  t1588 = t1572 + t1573;
  t1622 = -1.*t1621*t1278;
  t1646 = -1.*t1050*t1145*t1645;
  t1657 = t1145*t1247*t1656;
  t1658 = t1622 + t1646 + t1657;
  t1660 = t1290*t1658;
  t1664 = -1.*t1290*t1645;
  t1666 = -1.*t1464*t1656;
  t1668 = t1661 + t1664 + t1666;
  t1669 = -1.*t1050*t1145*t1668;
  t1670 = t1660 + t1669;
  t1680 = t1290*t1645;
  t1682 = t1464*t1656;
  t1683 = t1678 + t1680 + t1682;
  t1686 = t1343*t1683;
  t1689 = -1.*t1343*t1645;
  t1695 = -1.*t1530*t1656;
  t1700 = t1688 + t1689 + t1695;
  t1704 = t1290*t1700;
  t1705 = t1686 + t1704;
  t1712 = t1050*t1145*t1645;
  t1714 = -1.*t1145*t1247*t1656;
  t1721 = t1710 + t1712 + t1714;
  t1742 = t1343*t1721;
  t1747 = t1343*t1645;
  t1751 = t1530*t1656;
  t1752 = t1746 + t1747 + t1751;
  t1753 = -1.*t1050*t1145*t1752;
  t1754 = t1742 + t1753;
  t1774 = -1.*t1621*t1278;
  t1775 = -1.*t1145*t1640;
  t1781 = t1774 + t1775;
  t1782 = t1145*t1274*t1781;
  t1787 = -1.*t1237*t1629;
  t1788 = t1274*t1278*t1640;
  t1792 = t1787 + t1661 + t1788;
  t1793 = -1.*t1278*t1792;
  t1794 = t1782 + t1793;
  t1799 = t1145*t1640;
  t1806 = t1710 + t1799;
  t1810 = t1145*t1337*t1806;
  t1811 = t1274*t1629;
  t1812 = -1.*t1337*t1278*t1640;
  t1813 = t1811 + t1746 + t1812;
  t1821 = -1.*t1278*t1813;
  t1822 = t1810 + t1821;
  t1827 = -1.*t1274*t1278*t1640;
  t1828 = t1824 + t1678 + t1827;
  t1829 = t1145*t1337*t1828;
  t1831 = t1337*t1278*t1640;
  t1834 = t1830 + t1688 + t1831;
  t1835 = t1145*t1274*t1834;
  t1836 = t1829 + t1835;
  t1843 = -1.*t1612*t1337;
  t1846 = t1830 + t1843;
  t1847 = t1846*t1237;
  t1848 = t1274*t1612;
  t1849 = t1848 + t1824;
  t1858 = t1849*t1274;
  t1859 = t1847 + t1858;
  t1551 = -0.915088*t1050*t1145*t1550;
  t1567 = 0.915088*t1290*t1564;
  t1590 = 0.915088*t1343*t1588;
  t1592 = t1551 + t1567 + t1590;
  t1893 = 0.915088*t1426*t1550;
  t1898 = 0.915088*t1564*t1506;
  t1899 = 0.915088*t1588*t1534;
  t1903 = t1893 + t1898 + t1899;
  t1672 = 0.915088*t1343*t1670;
  t1708 = -0.915088*t1050*t1145*t1705;
  t1762 = 0.915088*t1290*t1754;
  t1764 = t1672 + t1708 + t1762;
  t1905 = 0.915088*t1534*t1670;
  t1908 = 0.915088*t1426*t1705;
  t1909 = 0.915088*t1506*t1754;
  t1917 = t1905 + t1908 + t1909;
  t1951 = 0.915088*t1588*t1670;
  t1955 = 0.915088*t1550*t1705;
  t1956 = 0.915088*t1564*t1754;
  t1957 = t1951 + t1955 + t1956;
  t1992 = -0.000766*t1050*t1145;
  t1995 = 4.e-6*t1290;
  t2000 = 0.000013*t1343;
  t1795 = 0.915088*t1343*t1794;
  t1823 = 0.915088*t1290*t1822;
  t1841 = -0.915088*t1050*t1145*t1836;
  t1842 = t1795 + t1823 + t1841;
  t1918 = 0.915088*t1534*t1794;
  t1923 = 0.915088*t1506*t1822;
  t1924 = 0.915088*t1426*t1836;
  t1927 = t1918 + t1923 + t1924;
  t1958 = 0.915088*t1588*t1794;
  t1959 = 0.915088*t1564*t1822;
  t1960 = 0.915088*t1550*t1836;
  t1962 = t1958 + t1959 + t1960;
  t1982 = -4.e-6*t1050*t1145;
  t1983 = 0.001148*t1290;
  t1984 = 3.e-6*t1343;
  t1985 = t1982 + t1983 + t1984;
  t2001 = t1992 + t1995 + t2000;
  t2005 = -0.000013*t1050*t1145;
  t2006 = 3.e-6*t1290;
  t2010 = 0.001017*t1343;
  t2011 = t2005 + t2006 + t2010;
  t2047 = 0.915088*t1794*t1670;
  t2048 = 0.915088*t1836*t1705;
  t2051 = 0.915088*t1822*t1754;
  t2042 = 3.e-6*t1145*t1274;
  t2043 = 0.001017*t1145*t1337;
  t2044 = -0.000013*t1278;
  t2045 = t2042 + t2043 + t2044;
  t2034 = 0.001148*t1145*t1274;
  t2038 = 3.e-6*t1145*t1337;
  t2039 = -4.e-6*t1278;
  t2040 = t2034 + t2038 + t2039;
  t2026 = 4.e-6*t1145*t1274;
  t2027 = 0.000013*t1145*t1337;
  t2028 = -0.000766*t1278;
  t2032 = t2026 + t2027 + t2028;
  t2057 = 0.001148*t1237;
  t2058 = 3.e-6*t1274;
  t2064 = t2057 + t2058;
  t2066 = 3.e-6*t1237;
  t2067 = 0.001017*t1274;
  t2070 = t2066 + t2067;
  t2053 = 4.e-6*t1237;
  t2054 = 0.000013*t1274;
  t2055 = t2053 + t2054;
  t1860 = -0.915088*t1050*t1145*t1859;
  t1861 = -0.02968087928*t1274*t1290;
  t1862 = 0.02968087928*t1237*t1343;
  t1869 = t1860 + t1861 + t1862;
  t1931 = 0.915088*t1859*t1426;
  t1932 = -0.02968087928*t1274*t1506;
  t1933 = 0.02968087928*t1237*t1534;
  t1935 = t1931 + t1932 + t1933;
  t1968 = 0.915088*t1859*t1550;
  t1973 = -0.02968087928*t1274*t1564;
  t1975 = 0.02968087928*t1237*t1588;
  t1976 = t1968 + t1973 + t1975;
  t2072 = 0.02968087928*t1237*t1670;
  t2073 = 0.915088*t1859*t1705;
  t2074 = -0.02968087928*t1274*t1754;
  t2106 = 0.02968087928*t1237*t1794;
  t2107 = -0.02968087928*t1274*t1822;
  t2108 = 0.915088*t1859*t1836;
  t1870 = -0.0018002532501503663*t1290;
  t1871 = 0.00022324305935101784*t1343;
  t1876 = t1870 + t1871;
  t1936 = -0.0018002532501503663*t1506;
  t1937 = 0.00022324305935101784*t1534;
  t1938 = t1936 + t1937;
  t1977 = -0.0018002532501503663*t1564;
  t1978 = 0.00022324305935101784*t1588;
  t1980 = t1977 + t1978;
  t2076 = 0.00022324305935101784*t1670;
  t2077 = -0.0018002532501503663*t1754;
  t2078 = t1992 + t1995 + t2000 + t2076 + t2077;
  t2112 = 0.00022324305935101784*t1794;
  t2113 = -0.0018002532501503663*t1822;
  t2114 = t2026 + t2027 + t2028 + t2112 + t2113;
  t2154 = 0.000011240888630050262*t1237;
  t2155 = 0.00007139121416862712*t1274;
  t2156 = t2154 + t2155;
  p_output1[0]=0.915088*Power(t1050,2)*Power(t1145,2) + 0.915088*Power(t1290,2) + 0.915088*Power(t1343,2);
  p_output1[1]=t1536;
  p_output1[2]=t1592;
  p_output1[3]=t1764;
  p_output1[4]=t1842;
  p_output1[5]=t1869;
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
  p_output1[18]=t1876;
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
  p_output1[30]=t1536;
  p_output1[31]=0.915088*Power(t1426,2) + 0.915088*Power(t1506,2) + 0.915088*Power(t1534,2);
  p_output1[32]=t1903;
  p_output1[33]=t1917;
  p_output1[34]=t1927;
  p_output1[35]=t1935;
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
  p_output1[48]=t1938;
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
  p_output1[60]=t1592;
  p_output1[61]=t1903;
  p_output1[62]=0.915088*Power(t1550,2) + 0.915088*Power(t1564,2) + 0.915088*Power(t1588,2);
  p_output1[63]=t1957;
  p_output1[64]=t1962;
  p_output1[65]=t1976;
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
  p_output1[78]=t1980;
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
  p_output1[90]=t1764;
  p_output1[91]=t1917;
  p_output1[92]=t1957;
  p_output1[93]=0.915088*Power(t1670,2) + 0.915088*Power(t1705,2) + 0.915088*Power(t1754,2) + t1290*t1985 - 1.*t1050*t1145*t2001 + t1343*t2011;
  p_output1[94]=-1.*t1050*t1145*t2032 + t1290*t2040 + t1343*t2045 + t2047 + t2048 + t2051;
  p_output1[95]=-1.*t1050*t1145*t2055 + t1290*t2064 + t1343*t2070 + t2072 + t2073 + t2074;
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
  p_output1[108]=t2078;
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
  p_output1[120]=t1842;
  p_output1[121]=t1927;
  p_output1[122]=t1962;
  p_output1[123]=t1145*t1274*t1985 - 1.*t1278*t2001 + t1145*t1337*t2011 + t2047 + t2048 + t2051;
  p_output1[124]=0.915088*Power(t1794,2) + 0.915088*Power(t1822,2) + 0.915088*Power(t1836,2) - 1.*t1278*t2032 + t1145*t1274*t2040 + t1145*t1337*t2045;
  p_output1[125]=-1.*t1278*t2055 + t1145*t1274*t2064 + t1145*t1337*t2070 + t2106 + t2107 + t2108;
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
  p_output1[138]=t2114;
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
  p_output1[150]=t1869;
  p_output1[151]=t1935;
  p_output1[152]=t1976;
  p_output1[153]=t1237*t1985 + t1274*t2011 + t2072 + t2073 + t2074;
  p_output1[154]=t1237*t2040 + t1274*t2045 + t2106 + t2107 + t2108;
  p_output1[155]=0.0009626993194468*Power(t1237,2) + 0.0009626993194468*Power(t1274,2) + 0.915088*Power(t1859,2) + t1237*t2064 + t1274*t2070;
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
  p_output1[168]=t2156;
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
  p_output1[540]=t1876;
  p_output1[541]=t1938;
  p_output1[542]=t1980;
  p_output1[543]=t2078;
  p_output1[544]=t2114;
  p_output1[545]=t2156;
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
  p_output1[558]=0.0007695961013894023;
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

#include "Mmat14_digit.hh"

namespace SymFunction
{

void Mmat14_digit_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

#endif // MATLAB_MEX_FILE

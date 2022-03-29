#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
int main() {
    std::string s = "tm";
    const char *str = s.c_str();
    std::cout << str;
    s[0] = 'm';
    std::cout << str << std::endl;
    Eigen::Matrix3d T857;
    Eigen::Matrix3d T858;
    Eigen::Matrix3d T859;
    Eigen::Matrix3d T860;
    T857 << -0.994009,-0.109297,-12.3938,
            0.109297 ,-0.994009,-5.20634,
            0,0,1;
    T858 << -0.993887, -0.110399,  -12.4161,
    0.110399, -0.993887,  -5.20769,
    0   ,      0    ,     1;
    T859 << -0.993878, -0.110486,  -12.4372,
    0.110486, -0.993878,  -5.20959,
    0 ,        0      ,   1;
    T860 << -0.993906,-0.11023,-12.4782,
            0.11023,-0.993906,-5.21122,
            0,         0,         1;
    Eigen::Matrix3d T1;
    Eigen::Matrix3d T2;
    Eigen::Matrix3d T3;
    T1 = T858 * T857.inverse();
    T2 = T859 * T858.inverse();
    T3 = T860 * T859.inverse();
    std::cout<<"T=\n"<<T1*T2*T3<<std::endl;
    std::cout<<"T=\n"<<(T3*T2*T1)<<std::endl;
    return 0;
}
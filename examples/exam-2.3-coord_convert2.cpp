/**
 * 版权：
 *  本软件遵循木兰宽松许可证第2版（MulanPSL-2.0），您可以在以下位置获取许可证的完整文本：
 *  http://license.coscl.org.cn/MulanPSL2
 *
 *  根据木兰宽松许可证第2版的规定，您可以自由地：
 *  1. 复制、使用、修改本软件；
 *  2. 将本软件用于商业用途；
 *  3. 对本软件进行再发布。
 *
 *  但您必须满足以下条件：
 *  1. 在再发布的软件或文档中保留原始版权声明；
 *  2. 在再发布的软件或文档中包含木兰宽松许可证的完整文本。
 *
 *  本软件按“原样”提供，不提供任何形式的明示或暗示的担保，包括但不限于对适销性、特定用途适用性和非侵权性的担保。
 *  作者不对因使用本软件而产生的任何直接、间接、特殊、偶然或 consequential 损害负责。
 *
 * 功能： 坐标转换例子展示（利用类来定义不同参考框架）
 *
 * 作者: 张守建
 * 联系方式: shjzhang@sgg.whu.edu.cn
 * 版本: v1.0.0
 * 日期: 2024-10-10
 *
 * 参考文献:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */

#include <iostream>
#include <cmath>
#include <stdexcept>

// 参考框架类
class ReferenceFrame {
public:
    const double a;       // 长半轴
    const double f;       // 扁率
    const double omega;   // 地球自转角速度
    const double GM;      // 地球引力常数
    const double J2;      // 二阶田谐系数

    // 构造函数
    ReferenceFrame(double a_, double f_, double omega_, double GM_, double J2_)
            : a(a_), f(f_), omega(omega_), GM(GM_), J2(J2_) {}

    // 计算第一偏心率平方 e^2
    double getE2() const {
        return 2 * f - f * f;
    }
};

// 定义常量对象
const ReferenceFrame wgs84(
        6378137.0,              // 长半轴 (米)
        1 / 298.257223563,      // 扁率
        7.292115e-5,            // 地球自转角速度 (弧度/秒)
        3.986004418e14,         // 地球引力常数 (米^3/秒^2)
        1.08263e-3              // 二阶田谐系数
);

const ReferenceFrame pz90(
        6378136.0,              // 长半轴 (米)
        1 / 298.257839303,      // 扁率
        7.2921150e-5,           // 地球自转角速度 (弧度/秒)
        3.9860044e14,           // 地球引力常数 (米^3/秒^2)
        1.08262575e-3           // 二阶田谐系数
);

// XYZ 坐标类
class XYZ {
public:
    double x, y, z;

    XYZ(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// BLH 坐标类
class BLH {
public:
    double B, L, H;

    BLH(double B_, double L_, double H_) : B(B_), L(L_), H(H_) {}
};

// 坐标转换函数
BLH xyz2blh(const XYZ &xyz, const ReferenceFrame &frame) {
    // 获取椭球参数
    double a = frame.a;
    double e2 = frame.getE2();

    // 计算水平距离 rho（即 sqrt(x^2 + y^2)）
    double rho = sqrt(xyz.x * xyz.x + xyz.y * xyz.y);

    // 定义阈值，用于判断是否在极点
    const double eps = 1.0e-13;

    // 判断是否在极点
    if (rho < eps) {
        // 在极点，根据 z 的符号判断是南极还是北极
        double B = (xyz.z > 0) ? M_PI / 2 : -M_PI / 2;  // 北极为 +90°，南极为 -90°
        double L = 0.0;  // 经度在极点无定义，通常设为 0
        double H = fabs(xyz.z) - a * sqrt(1 - e2);  // 高度计算

        return BLH(B, L, H);
    }

    // 不在极点，正常计算
    double B0 = atan2(xyz.z, rho);

    // 迭代计算大地纬度 B
    const int maxIterations = 10;
    int iterationCount = 0;
    double B1, N;
    do {
        N = a / sqrt(1 - e2 * sin(B0) * sin(B0));
        B1 = atan2(xyz.z + e2 * N * sin(B0), rho);

        if (fabs(B1 - B0) < eps) break;

        B0 = B1;
        iterationCount++;

        if (iterationCount > maxIterations) {
            throw std::runtime_error("Iteration did not converge.");
        }
    } while (true);

    // 计算大地经度 L
    double L = atan2(xyz.y, xyz.x);

    // 计算高度 H
    double H = rho / cos(B1) - N;

    // 返回大地坐标
    return BLH(B1, L, H);
}

int main() {
    try {
        // 示例 XYZ 坐标
        XYZ xyz_north_pole(0.0, 0.0, 6356752.314);  // 北极点
        XYZ xyz_south_pole(0.0, 0.0, -6356752.314); // 南极点
        XYZ xyz_normal(4081945.67, 2187689.34, 4767321.89); // 正常点

        // 使用 PZ-90 参考框架进行坐标转换
        BLH blh_north_pole = xyz2blh(xyz_north_pole, pz90);
        std::cout << "PZ-90   (B, L, H): "
                  << blh_north_pole.B << ", " << blh_north_pole.L << ", " << blh_north_pole.H << std::endl;

        BLH blh_south_pole = xyz2blh(xyz_south_pole, pz90);
        std::cout << "PZ-90   (B, L, H): "
                  << blh_south_pole.B << ", " << blh_south_pole.L << ", " << blh_south_pole.H << std::endl;

        BLH blh_normal = xyz2blh(xyz_normal, pz90);
        std::cout << "PZ-90   (B, L, H): "
                  << blh_normal.B << ", " << blh_normal.L << ", " << blh_normal.H << std::endl;

        // 使用 WGS84 参考框架进行坐标转换
        BLH blh_normal_wgs84 = xyz2blh(xyz_normal, wgs84);
        std::cout << "WGS84   (B, L, H): "
                  << blh_normal_wgs84.B << ", " << blh_normal_wgs84.L << ", " << blh_normal_wgs84.H << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

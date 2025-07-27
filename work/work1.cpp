#include <iostream>
#include <cmath>
using namespace std;

const double v0 = 17.0;
const double k1 = 0.0089;
const double g = 9.81;

double calculateThetaByHeight(double z_temp, double s) 
{
    return atan(z_temp / s);
}

double simulateTrajectory(double theta, double s) 
{
    double exp_k1s = exp(k1 * s);
    double t = (exp_k1s - 1) / (k1 * v0 * cos(theta));
    double z_actual = v0 * t * sin(theta) - 0.5 * g * t * t;
    return z_actual;
}

double solveBallisticAngle(double s, double z0, int j= 20, double tolerance = 1e-4) 
{
    double z_temp = z0;      // 初始化目标高度
    double theta = 0.0;

    for (int i = 0; i < j; ++i) {
        // 1. 根据当前 z_temp 计算角度 theta
        theta = calculateThetaByHeight(z_temp, s);

        // 2. 模拟弹道，计算实际落点 z_actual
        double z_actual = simulateTrajectory(theta, s);

        // 3. 计算误差并更新 z_temp
        double dz = z0 - z_actual;
        z_temp += dz;

        // 4. 检查收敛条件
        if (abs(dz) < tolerance) {
            cout << "Converged after " << i + 1 << " iterations." << endl;
            break;
        }
    }

    return theta;
}

int main() {
    double x = 3.0, y = 4.0, z0 = 0.25;
    double s = sqrt(x * x + y * y);  // 水平距离 s = 5 m

    double theta = solveBallisticAngle(s, z0);
    cout << "Final theta: " << theta * 180.0 / M_PI << " degrees" << endl;

    return 0;
}
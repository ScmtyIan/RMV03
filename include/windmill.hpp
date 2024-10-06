#ifndef WINDMILL_H_
#define WINDMILL_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <random>
#include <vector>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#define pi 3.1415926

using namespace std;
using namespace cv;
using namespace ceres;

namespace WINDMILL
{
    class WindMill
    {
    private:
        int cnt;
        bool direct;
        double A;
        double w;
        double A0;
        double fai;
        double now_angle;
        double start_time;
        cv::Point2i R_center;
        void drawR(cv::Mat &img, const cv::Point2i &center);
        void drawHitFan(cv::Mat &img, const cv::Point2i &center, double angle);
        void drawOtherFan(cv::Mat &img, const cv::Point2i &center, double angle);
        cv::Point calPoint(const cv::Point2f &center, double angle_deg, double r)
        {
            return center + cv::Point2f((float)cos(angle_deg / 180 * 3.1415926), (float)-sin(angle_deg / 180 * 3.1415926)) * r;
        }
        double SumAngle(double angle_now, double t0, double dt)
        {
            double dangle = A0 * dt + (A / w) * (cos(w * t0 + 1.81) - cos(w * (t0 + dt) + 1.81));
            angle_now += dangle / 3.1415926 * 180;
            if (angle_now < 0)
            {
                angle_now = 360 + angle_now;
            }
            if (angle_now > 360)
            {
                angle_now -= 360;
            }
            return angle_now;
        }

    public:
        WindMill(double time = 0);
        cv::Mat getMat(double time);
    };
} // namespace WINDMILL

namespace GETPOINT
{
    class GetPoint
    {
    public:
        void getPoint(Mat &, Point &, Point &);
    };
}

// 以下应该括在命名空间里的，但是为了方便（lazy），就不写了
class DataFitting
{
public:
    DataFitting(double);
    // 加入实际数据(tim,theta)
    void AddData(const double, const double);
    // 迭代拟合
    void DataFitting_Solver();
    // 预测
    void Data_Predict(double, double &);
    bool CheckAns();

private:
    // 两个拟合函数的代价函数
    struct CostFunctorA
    {
        CostFunctorA(double tim, double theta) : tim(tim), theta(theta) {}

        template <typename T>
        bool operator()(const T *const A0, const T *const A, const T *const w, const T *const phi, T *residual) const
        {
            residual[0] = T(theta) - A0[0] * T(tim) - A[0] / w[0] * (ceres::sin(w[0] * T(tim) + phi[0]) - ceres::sin(phi[0]));
            return true;
        }
        const double tim;
        const double theta;
    };
    struct CostFunctorOmega
    {
        CostFunctorOmega(double tim, double theta) : tim(tim), theta(theta) {}

        template <typename T>
        bool operator()(const T *const A, const T *const w, const T *const phi, T *residual) const
        {
            residual[0] = T(theta) - A[0] / w[0] * (ceres::sin(w[0] * T(tim) + phi[0]) - ceres::sin(phi[0]));
            return true;
        }
        const double tim;
        const double theta;
    };
    vector<double> tim_data;
    vector<double> theta_data;
    double start_time;
    int size;
    double A0;
    double A;
    double phi;
    double w;
    // 拟合角速度为Acos(wt+phi)+A0
};
#endif
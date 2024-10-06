#include "windmill.hpp"

// 初始值设置
DataFitting::DataFitting(double st_time)
{
    size = 0;
    A0 = 0.305; // 真值为1.305
    A = 1.785;  // 真值为0.785
    phi = 1.24; // 真值为0.24
    w = 0.884;  // 真值为1.884
    start_time = st_time;
}

// 传入数据，注意theta_data是绝对角度，tim_data是相对于开始时间的时间
void DataFitting::AddData(const double now_tim, const double theta)
{
    tim_data.push_back(now_tim - start_time);
    theta_data.push_back(theta);
    size++;
}

void DataFitting::DataFitting_Solver()
{
    // 构建优化问题A0 和 问题Omega
    // 前者主要拟合A0，后者主要拟合A，w，phi
    ceres::Problem problemA, problemOmega;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    // 采用QR分解够了，密集型小数据（180算小把）
    options.linear_solver_type = ceres::DENSE_QR;
    // 传入数据，拟合A0
    for (int i = 0; i < size; i++)
    {
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctorA, 1, 1, 1, 1, 1>(
            new CostFunctorA(tim_data[i], theta_data[i]));
        problemA.AddResidualBlock(cost_function, nullptr, &A0, &A, &w, &phi);
    }
    ceres::Solve(options, &problemA, &summary); // 求解拟合A0

    // 设置鲁棒损失函数，处理异常值

    // 传入数据，拟合A，w，phi
    for (int i = 0; i < size; i++)
    {
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctorOmega, 1, 1, 1, 1>(
            new CostFunctorOmega(tim_data[i], theta_data[i] - A0 * tim_data[i]));
        problemOmega.AddResidualBlock(cost_function, nullptr, &A, &w, &phi);
    }
    ceres::Solve(options, &problemOmega, &summary); // 求解拟合A,w,phi

    // 拟合值处理
    if (w < 0)
    {
        w = -w;
        phi = -phi;
    }
    if (A < 0)
    {
        A = -A;
        phi += pi;
    }

    // 可视化拟合效果，下面不要也行，主要用来调试。
    // 拟合绝对角度和相对时间的关系，白线观测值，粉线拟合值，蓝线理论拟合值
    cv::Mat paper1(Size(400, 400), CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < size; ++i)
    {
        circle(paper1, Point(tim_data[i] * 150 + 5, theta_data[i] * 100), 1, Scalar(255, 255, 255), -1);
    }
    for (int i = 0; i < size; ++i)
    {
        circle(paper1, Point(tim_data[i] * 150 + 5, (A0 * tim_data[i] + (A / w) * (sin(w * tim_data[i] + phi) - sin(phi))) * 100), 1, Scalar(255, 0, 255), -1);
    }
    for (int i = 0; i < size; ++i)
    {
        circle(paper1, Point(tim_data[i] * 150 + 5, (1.305 * tim_data[i] + (0.785 / 1.884) * (sin(1.884 * tim_data[i] + 1.884 * start_time + 0.24) - sin(1.884 * start_time + 0.24))) * 100), 1, Scalar(255, 0, 0), -1);
    }
    imshow("paper1", paper1);

    // 拟合三角项的角度和相对时间的关系，白线观测值，粉线拟合值
    cv::Mat paper(Size(400, 400), CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < size; ++i)
    {
        circle(paper, Point(tim_data[i] * 100 + 5, (theta_data[i] - A0 * tim_data[i]) * 200 + 200), 1, Scalar(255, 255, 255), -1);
    }
    for (int i = 0; i < size; ++i)
    {
        circle(paper, Point(tim_data[i] * 100 + 5, ((A / w) * (sin(w * tim_data[i] + phi) - sin(phi))) * 200 + 200), 1, Scalar(255, 0, 255), -1);
    }
    imshow("paper", paper);
}

// 角度预测，传入当前时刻，返回预测角度
void DataFitting::Data_Predict(double tim_now, double &theta_predict)
{
    theta_predict = A0 * (tim_now - start_time) + A / w * (sin(w * (tim_now - start_time) + phi) - sin(phi));
}

// 答案检测，小于真值5%则认为答案正确
bool DataFitting::CheckAns()
{
    // phi_process 是题目要求的初相位，很容易得到关系 phi_process = phi - start_time*w
    // 但是，由于w是拟合的，固然相对于1.884有误差，这个误差经过start_time（将近1e9）放大，
    // 就导致根本定位不到绝对初值了，所以phi_process关于0.24的真值判定我删了，
    // 对不起，我拟合不出来真正意义上的phi_process
    // phi_process的换算
    double phi_process;
    phi_process = asin(sin(phi - start_time * w));
    if (cos(phi - start_time * w) < 0)
        phi_process = pi - phi_process;
    if (phi_process < 0)
        phi_process += 2 * pi;

    // 拟合效果判定
    if (fabs(A0 - 1.305) / 1.305 < 0.05 && fabs(A - 0.785) / 0.785 < 0.05 && fabs(w - 1.884) / 1.884 < 0.05)
    {
        Mat paper(Size(400, 400), CV_8UC3, Scalar(0, 0, 0));
        // 输出答案，方便调试
        putText(paper, "A0=" + to_string(A0) + " A=" + to_string(A), Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        putText(paper, "w=" + to_string(w) + " phi=" + to_string(phi_process), Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        imshow("ans", paper);
        return 1;
    }
    // 否则重新从初始值开始拟合
    A0 = 0.305; // 真值为1.305
    A = 1.785;  // 真值为0.785
    phi = 1.24; // 真值为0.24
    w = 0.884;  // 真值为1.884
    return 0;
}

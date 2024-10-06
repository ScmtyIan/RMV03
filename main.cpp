#include "windmill.hpp"

double distance(Point Recpoint, Point Rpoint)
{
    return sqrt((Recpoint.x - Rpoint.x) * (Recpoint.x - Rpoint.x) + (Recpoint.y - Rpoint.y) * (Recpoint.y - Rpoint.y));
}

double gettheta(Point Recpoint, Point Rpoint)
{
    if (fabs(Recpoint.x - Rpoint.x) < 1)
    {
        if (Recpoint.y < Rpoint.y)
            return pi / 2;
        else
            return 3 * pi / 2;
    }
    double t = atan(double(Recpoint.y - Rpoint.y) / double(Rpoint.x - Recpoint.x));
    if (Rpoint.x > Recpoint.x)
        return t + pi;
    else if (Rpoint.x < Recpoint.x && Rpoint.y < Recpoint.y)
        return t + 2 * pi;
    else
        return t;
}

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    double t_sum = 0;
    int N = 10;
    for (int i = 0; i < N; i++)
    {
        std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        double st_time = t.count() / 1000.0;
        WINDMILL::WindMill wm(st_time);

        //==========================代码区========================//

        // 创建一个数据拟合的类，并传入开始时间
        DataFitting MyDataFit(st_time);
        // 计时点
        int64 start_time = getTickCount();
        Mat src;
        int tot = 0;           // 记录风车转了多少圈
        double theta = 0, dis; // 当前观测下风车的角度[0,2pi]，和距离圆心R的距离
        double theta_predict;  // 根据拟合模型预测下一时刻风车转过的角度，这里用不上
        bool f = 0, flag = 0;  // 用来辅助统计圈数，主要是用来防止噪声抖动引发的圈数误判

        // 采样 1.8s 180个点，刚好半个周期多一点
        vector<Point2d> shit_vec;
        for (int j = 0; j < 400; j++)
        {
            t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            double now_time = t.count() / 1000.0;
            // 读取当前时刻,并通过GetPoint函数计算R和锤子矩形的中心
            src = wm.getMat(now_time);
            GETPOINT::GetPoint MyGetPoint;
            Point Recpoint, Rpoint; // 矩阵中心，R中心(圆心)
            MyGetPoint.getPoint(src, Recpoint, Rpoint);

            // 画出识别出来的两个中心点并显示出来
            circle(src, Recpoint, 3, Scalar(255, 0, 0), -1);
            circle(src, Rpoint, 3, Scalar(255, 0, 0), -1);
            imshow("dst", src);

            // 计算锤子中心运动的角度和半径，这里计算半径用来调整R中心的位置，达到近似圆轨迹
            //  dis = distance(Recpoint, Rpoint);
            theta = gettheta(Recpoint, Rpoint);

            // 这里是根据当前状态，来得到绝对角度[0,+无穷)
            if (theta < pi * 3 / 2 && theta > pi && f == 0)
                f = 1;
            if (theta < pi / 2 && f == 1)
            {
                f = 0;
                tot++;
            }
            theta += 2 * pi * tot; // 绝对角度

            // 前10个点不参与拟合,除去噪声引发的初始偏置，且波动大
            if (j < 10)
                continue;
            waitKey(1);

            // 将当前时刻的角度和时刻传入数据拟合类，拟合theta=A0*t+A/W*(sin(wt+phi)-sin(phi)),
            // 对应角速度A0 + Acos(wt + phi),注意此处的时刻是相对于开始时刻的，原因后续说明
            MyDataFit.AddData(now_time, theta);
            if ((j + 1) % 50 == 0) // 每50个点拟合一次
            {
                // 迭代拟合
                MyDataFit.DataFitting_Solver();
                // 判断拟合效果是否符合真值5%内，如果不符合，则重新拟合(此组放弃，重新拟合)
                if (MyDataFit.CheckAns()) // 拟合成功，则不采样，跳出循环
                {
                    flag = 1;
                    break;
                }
            }
        }
        // 如果400个点都拟合不出来，说明ceres solver废了，重新做一组
        if (!flag)
            N++;
        // 当前组计时结束点
        int64 end_time = getTickCount();
        t_sum += (end_time - start_time) / getTickFrequency();

        //=======================================================//
    }
    cout << t_sum / 10 << endl;
    cout << N << endl; // 额外输出拟合组数，用来判定模型稳定性，如果拟合组数过多，则说明模型不稳定
    // 实测均值在[1.6s,1.9s],N在[10,12]
}

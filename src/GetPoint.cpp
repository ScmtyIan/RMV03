#include "windmill.hpp"

// 识别部分
namespace GETPOINT
{
    // 识别锤子的矩形中心和R的中心
    void GetPoint::getPoint(Mat &img, Point &p, Point &R)
    {
        Mat dst = img.clone();
        // idx1用来记录R的轮廓索引,idx2用来记录锤子的轮廓索引
        int idx1 = 0, idx2 = 0;
        // 灰度化，二值化，找轮廓
        cvtColor(dst, dst, COLOR_BGR2GRAY);
        threshold(dst, dst, 1, 255, THRESH_BINARY);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // 遍历所有轮廓
        for (int i = 0; i < contours.size(); i++)
        {
            // 没有儿子轮廓，没有同层级轮廓，那就只有R的内轮廓和目标锤子的内轮廓了
            if (hierarchy[i][2] == -1 && hierarchy[i][1] == -1 && hierarchy[i][0] == -1)
            {
                // 轮廓面积小，就是R的内轮廓
                if (contourArea(contours[i]) < 100.0)
                    idx1 = i;
                // 否则是目标锤子内轮廓
                else
                    idx2 = i;
                if (idx1 && idx2)
                    break;
            }
        }
        // boundingbox一下得到中心点
        Rect rect1 = boundingRect(contours[idx2]);
        Rect rect2 = boundingRect(contours[idx1]);
        p = Point(rect1.x + rect1.width / 2, rect1.y + rect1.height / 2);
        R = Point(rect2.x + rect2.width / 2, rect2.y + rect2.height / 2);

        // 实测R的中心需要加一个偏置，dis控制在[169,171)
        Point pz = Point(-4, 6);
        R += pz;
    }
}
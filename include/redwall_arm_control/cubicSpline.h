#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_

/* 三次样条函数插补 */

class cubicSpline
{
public:
    /* 边界条件下的三次样条，们要求起始和终止的位置速度必须为0，所以采用了第一类边界条件，并且边界的一阶导数为0 */
    typedef enum _BoundType
    {
        BoundType_First_Derivative,     // 第一类（已知边界一阶导数）
        BoundType_Second_Derivative     // 第二类（已知边界二阶导数）
    }BoundType;

public:
    cubicSpline();
    ~cubicSpline();

    void initParam();       // 初始化参数
    void releaseMem();      // 释放内存

    /* x_data表示时间数组  y_data表示路点数组  count表示数据点个数,也就是数组的长度 后面的是求速度和加速度样条曲线的*/
    bool loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type);
    /* 拟合好曲线,可以通过时间自变量求出对应的路点position */
    bool getYbyX(double &x_in, double &y_out);

protected:
    bool spline(BoundType type);

protected:
    double *x_sample_, *y_sample_;
    double *M_;
    int sample_count_;
    double bound1_, bound2_;
};

#endif //_CUBICSPLINE_H_

/* ROS action server */
#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>

/* 三次样条插补 */
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include "cubicSpline.h"

/* 使用变长数组 */
#include <vector>
#include <algorithm>

/* 套接字通信 */
#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

#define PORT 7788
using namespace std;
vector<double> time_from_start_;
vector<double> p_lumbar_;
vector<double> p_big_arm_;
vector<double> p_small_arm_;
vector<double> p_wrist_;
vector<double> p_hand_;
vector<double> v_lumbar_;
vector<double> v_big_arm_;
vector<double> v_small_arm_;
vector<double> v_wrist_;
vector<double> v_hand_;
vector<double> a_lumbar_;
vector<double> a_big_arm_;
vector<double> a_small_arm_;
vector<double> a_wrist_;
vector<double> a_hand_;

/* 存储的结构体 p2*/
struct vel_data
{
    double time_from_begin;
    double lumbar_pos;
    double big_arm_pos;
    double small_arm_pos;
    double wrist_pos;
    double hand_pos;
    double lumbar_vel;
    double big_arm_vel;
    double small_arm_vel;
    double wrist_vel;
    double hand_vel;
    double lumbar_acc;
    double big_arm_acc;
    double small_arm_acc;
    double wrist_acc;
    double hand_acc;
};

/* 数据收发结构体 */
struct vel_data p2;
char writebuf[sizeof(p2)];


/* 客户端套接字文件描述符和地址,端口 */
typedef struct MySocketInfo{
    int socketCon;
    char *ipaddr;
    uint16_t port;
}_MySocketInfo;

/* 客户端连接所用数据的存储数组 */
struct MySocketInfo arrConSocket[10];
int conClientCount = 0;              // 连接的客户端个数

/* 客户端连接所用套接字的存储数组 */
pthread_t arrThrReceiveClient[10];
int thrReceiveClientCount = 0;       // 接受数据线程个数

/* action 服务端声明 */
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

/* 初始化输入输出速度加速度 */
double acc = 0, vel = 0;
double x_out = 0, y_out = 0;
/* 规划的路点数目 */
int point_num;
/* 判断路点数据是否改变 */
bool point_changed = false;

/* 三次样条无参构造 */
cubicSpline::cubicSpline()
{
}
/* 析构 */
cubicSpline::~cubicSpline()
{
    releaseMem();
}
/* 初始化参数 */
void cubicSpline::initParam()
{
    x_sample_ = y_sample_ = M_ = NULL;
    sample_count_ = 0;
    bound1_ = bound2_ = 0;
}
/* 释放参数 */
void cubicSpline::releaseMem()
{
    delete x_sample_;
    delete y_sample_;
    delete M_;

    initParam();
}
/* 加载关节位置数组等信息 */
bool cubicSpline::loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type)
{
    if ((NULL == x_data) || (NULL == y_data) || (count < 3) || (type > BoundType_Second_Derivative) || (type < BoundType_First_Derivative))
    {
        return false;
    }

    initParam();

    x_sample_ = new double[count];
    y_sample_ = new double[count];
    M_        = new double[count];
    sample_count_ = count;

    memcpy(x_sample_, x_data, sample_count_*sizeof(double));
    memcpy(y_sample_, y_data, sample_count_*sizeof(double));

    bound1_ = bound1;
    bound2_ = bound2;

    return spline(type);
}
/* 计算样条插值 */
bool cubicSpline::spline(BoundType type)
{
    if ((type < BoundType_First_Derivative) || (type > BoundType_Second_Derivative))
    {
        return false;
    }

    //  追赶法解方程求二阶偏导数
    double f1=bound1_, f2=bound2_;

    double *a=new double[sample_count_];                //  a:稀疏矩阵最下边一串数
    double *b=new double[sample_count_];                //  b:稀疏矩阵最中间一串数
    double *c=new double[sample_count_];                //  c:稀疏矩阵最上边一串数
    double *d=new double[sample_count_];

    double *f=new double[sample_count_];

    double *bt=new double[sample_count_];
    double *gm=new double[sample_count_];

    double *h=new double[sample_count_];

    for(int i=0;i<sample_count_;i++)
        b[i]=2;                                //  中间一串数为2
    for(int i=0;i<sample_count_-1;i++)
        h[i]=x_sample_[i+1]-x_sample_[i];      // 各段步长
    for(int i=1;i<sample_count_-1;i++)
        a[i]=h[i-1]/(h[i-1]+h[i]);
    a[sample_count_-1]=1;

    c[0]=1;
    for(int i=1;i<sample_count_-1;i++)
        c[i]=h[i]/(h[i-1]+h[i]);

    for(int i=0;i<sample_count_-1;i++)
        f[i]=(y_sample_[i+1]-y_sample_[i])/(x_sample_[i+1]-x_sample_[i]);

    for(int i=1;i<sample_count_-1;i++)
        d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

    //  追赶法求解方程
    if(BoundType_First_Derivative == type)
    {
        d[0]=6*(f[0]-f1)/h[0];
        d[sample_count_-1]=6*(f2-f[sample_count_-2])/h[sample_count_-2];

        bt[0]=c[0]/b[0];
        for(int i=1;i<sample_count_-1;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[0]=d[0]/b[0];
        for(int i=1;i<=sample_count_-1;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-1]=gm[sample_count_-1];
        for(int i=sample_count_-2;i>=0;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];
    }
    else if(BoundType_Second_Derivative == type)
    {
        d[1]=d[1]-a[1]*f1;
        d[sample_count_-2]=d[sample_count_-2]-c[sample_count_-2]*f2;

        bt[1]=c[1]/b[1];
        for(int i=2;i<sample_count_-2;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[1]=d[1]/b[1];
        for(int i=2;i<=sample_count_-2;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        M_[sample_count_-2]=gm[sample_count_-2];
        for(int i=sample_count_-3;i>=1;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];

        M_[0]=f1;
        M_[sample_count_-1]=f2;
    }
    else
        return false;

    delete a;
    delete b;
    delete c;
    delete d;
    delete gm;
    delete bt;
    delete f;
    delete h;

    return true;
}
/* 得到速度和加速度数组 */
bool cubicSpline::getYbyX(double &x_in, double &y_out)
{
    int klo,khi,k;
    klo=0;
    khi=sample_count_-1;
    double hh,bb,aa;

    //  二分法查找x所在区间段
    while(khi-klo>1)
    {
        k=(khi+klo)>>1;
        if(x_sample_[k]>x_in)
            khi=k;
        else
            klo=k;
    }
    hh=x_sample_[khi]-x_sample_[klo];

    aa=(x_sample_[khi]-x_in)/hh;
    bb=(x_in-x_sample_[klo])/hh;

    y_out=aa*y_sample_[klo]+bb*y_sample_[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;

    //////test
    acc = (M_[klo]*(x_sample_[khi]-x_in) + M_[khi]*(x_in - x_sample_[klo])) / hh;
    vel = M_[khi]*(x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh)
          - M_[klo]*(x_sample_[khi]-x_in) * (x_sample_[khi]-x_in) / (2 * hh)
          + (y_sample_[khi] - y_sample_[klo])/hh
          - hh*(M_[khi] - M_[klo])/6;
    //printf("[---位置、速度、加速度---]");
    //printf("%0.9f, %0.9f, %0.9f\n",y_out, vel, acc);
    //////test end

    return true;
}

/* SOCKET服务器端处理客户端连接函数 */
void *fun_thrAcceptHandler(void *socketListen)
{
    while(1)
    {
        /* accept函数主要用于服务器端，建立好连接后，它返回的一个新的套接字
         * 此后，服务器端即可使用这个新的套接字与该客户端进行通信，而原本的套接字则继续用于监听其他客户端的连接请求。 */
        int sockaddr_in_size = sizeof(struct sockaddr_in);
        struct sockaddr_in client_addr;
        int _socketListen = *((int *)socketListen);
        int socketCon = accept(_socketListen, (struct sockaddr *)(&client_addr), (socklen_t *)(&sockaddr_in_size));
        if(socketCon < 0){
            printf("连接失败\n");
        }else{
            printf("连接成功 ip: %s:%d\n",inet_ntoa(client_addr.sin_addr),client_addr.sin_port);
        }
        printf("连接套接字为：%d\n",socketCon);

        /* 开启新的通讯线程，负责同连接上来的客户端进行通讯 */
        _MySocketInfo socketInfo;                   // 用于保存客户端套接字的信息
        socketInfo.socketCon = socketCon;
        socketInfo.ipaddr = inet_ntoa(client_addr.sin_addr);
        socketInfo.port = client_addr.sin_port;
        arrConSocket[conClientCount] = socketInfo;
        conClientCount++;
        printf("连接了%d个用户\n",conClientCount);

        //让进程休息1秒
        sleep(1);
    }
}

/* 判断线程是否被杀死 */
int checkThrIsKill(pthread_t thr)
{
    /* 传递的pthread_kill的signal参数为0时，用这个保留的信号测试线程是否存在 */
    int res = 1;
    int res_kill = pthread_kill(thr,0);
    if(res_kill == 0){
        res = 0;
    }
    return res;
}

/* 收到action的goal后调用的回调函数 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {

    /* move_group规划的路径包含的路点个数 */
    point_num = goal->trajectory.points.size();
    ROS_INFO("First Move_group give us %d points",point_num);

    /* 各个关节位置 */
    double p_lumbar[point_num];
    double p_big_arm[point_num];
    double p_small_arm[point_num];
    double p_wrist[point_num];
    double p_hand[point_num];

    /* 各个关节速度 */
    double v_lumbar[point_num];
    double v_big_arm[point_num];
    double v_small_arm[point_num];
    double v_wrist[point_num];
    double v_hand[point_num];

    /* 各个关节加速度 */
    double a_lumbar[point_num];
    double a_big_arm[point_num];
    double a_small_arm[point_num];
    double a_wrist[point_num];
    double a_hand[point_num];

    /* 时间数组 */
    double time_from_start[point_num];

    for (int i = 0; i < point_num; i++) {
        p_lumbar[i] = goal->trajectory.points[i].positions[0];
        p_big_arm[i] = goal->trajectory.points[i].positions[1];
        p_small_arm[i] = goal->trajectory.points[i].positions[2];
        p_wrist[i] = goal->trajectory.points[i].positions[3];
        p_hand[i] = goal->trajectory.points[i].positions[4];

        v_lumbar[i] = goal->trajectory.points[i].velocities[0];
        v_big_arm[i] = goal->trajectory.points[i].velocities[1];
        v_small_arm[i] = goal->trajectory.points[i].velocities[2];
        v_wrist[i] = goal->trajectory.points[i].velocities[3];
        v_hand[i] = goal->trajectory.points[i].velocities[4];

        a_lumbar[i] = goal->trajectory.points[i].accelerations[0];
        a_big_arm[i] = goal->trajectory.points[i].accelerations[1];
        a_small_arm[i] = goal->trajectory.points[i].accelerations[2];
        a_wrist[i] = goal->trajectory.points[i].accelerations[3];
        a_hand[i] = goal->trajectory.points[i].accelerations[4];

        time_from_start[i] = goal->trajectory.points[i].time_from_start.toSec();
    }

    // 实例化样条
    cubicSpline spline;

    // lumbar
    spline.loadData(time_from_start, p_lumbar, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    time_from_start_.clear();
    p_lumbar_.clear();
    v_lumbar_.clear();
    a_lumbar_.clear();
    x_out = 0;
    double rate = (time_from_start[point_num-1] - time_from_start[0])/(point_num*4);
    for (int k = 0; k <= point_num*4 ; k++) {
        spline.getYbyX(x_out, y_out);
        time_from_start_.push_back(x_out);
        p_lumbar_.push_back(y_out);
        v_lumbar_.push_back(vel);
        a_lumbar_.push_back(acc);
        x_out += rate;
    }

    // big_arm
    spline.loadData(time_from_start, p_big_arm, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_big_arm_.clear();
    v_big_arm_.clear();
    a_big_arm_.clear();
    x_out = 0;
    for (int k = 0; k <= point_num*4 ; k++) {
        spline.getYbyX(x_out, y_out);
        p_big_arm_.push_back(y_out);
        v_big_arm_.push_back(vel);
        a_big_arm_.push_back(acc);
        x_out += rate;
    }

    // small_arm
    spline.loadData(time_from_start, p_small_arm, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_small_arm_.clear();
    v_small_arm_.clear();
    a_small_arm_.clear();
    x_out = 0;
    for (int k = 0; k <= point_num*4 ; k++) {
        spline.getYbyX(x_out, y_out);
        p_small_arm_.push_back(y_out);
        v_small_arm_.push_back(vel);
        a_small_arm_.push_back(acc);
        x_out += rate;
    }

    // wrist
    spline.loadData(time_from_start, p_wrist, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_wrist_.clear();
    v_wrist_.clear();
    a_wrist_.clear();
    x_out = 0;
    for (int k = 0; k <= point_num*4 ; k++) {
        spline.getYbyX(x_out, y_out);
        p_wrist_.push_back(y_out);
        v_wrist_.push_back(vel);
        a_wrist_.push_back(acc);
        x_out += rate;
    }

    // hand
    spline.loadData(time_from_start, p_hand, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    p_hand_.clear();
    v_hand_.clear();
    a_hand_.clear();
    x_out = 0;
    for (int k = 0; k <= point_num*4 ; k++) {
        spline.getYbyX(x_out, y_out);
        p_hand_.push_back(y_out);
        v_hand_.push_back(vel);
        a_hand_.push_back(acc);
        x_out += rate;
    }

    //control_msgs::FollowJointTrajectoryFeedback feedback;
    //feedback = NULL;
    //as->publishFeedback(feedback);
    ROS_INFO("Now We get all joints P,V,A,T!");
    point_changed = true;
    as->setSucceeded();
}

/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "redwall_arm_control");
    ros::NodeHandle nh;
    // 定义一个服务器
    Server server(nh, "redwall_arm/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
    // 服务器开始运行
    server.start();

    printf("开始socket\n");
    /* 创建TCP连接的Socket套接字 */
    int socketListen = socket(AF_INET, SOCK_STREAM, 0);
    if(socketListen < 0)
    {
        printf("创建TCP套接字失败\n");
        exit(-1);
    }
    else{
        printf("创建套接字成功\n");
    }

    /* 绑定服务器端口地址信息 */
    struct sockaddr_in server_addr;                 // struct sockaddr_in是已经声明了的结构名
    bzero(&server_addr,sizeof(struct sockaddr_in)); // 等价于memset(server_addr,0,sizeof(struct sockaddr_in));清零操作
    server_addr.sin_family=AF_INET;
    server_addr.sin_addr.s_addr=htonl(INADDR_ANY);  // 这里地址使用全0，即所有可能的地址
    server_addr.sin_port=htons(PORT);               // htons一般是转换端口号为整数
    if(bind(socketListen, (struct sockaddr *)&server_addr,sizeof(struct sockaddr)) != 0)
    {
        perror("绑定ip地址,端口号失败\n");
        exit(-1);
    }
    else{
        printf("绑定ip地址,端口号成功\n");
    }

    /* 开始监听相应的端口,最大不超过10个连接 */
    if(listen(socketListen, 10) != 0){
        printf("开启监听失败\n");
        exit(-1);
    }else{
        printf("开启监听成功\n");
    }

    /* 创建一个子线程用于接受连接客户端连接 */
    pthread_t thrAccept;
    pthread_create(&thrAccept,NULL,fun_thrAcceptHandler,&socketListen);

    /* 实时发送数据 */
    while(ros :: ok())
    {
        ros::spinOnce();

        if( point_changed )
        {
            /***** 判断客户端连接和数据发送是否成功 *****/
            if(conClientCount <= 0)
            {
                printf("没有客户端连接\n");
                exit(0);
            }
            else {
                for (int i = 0; i < conClientCount; i++) {
                    for(int k = 0; k <= point_num*4 ; k++)
                    {
                        p2.time_from_begin = time_from_start_.at(k);
                        p2.lumbar_pos = p_lumbar_.at(k);
                        p2.big_arm_pos = p_big_arm_.at(k);
                        p2.small_arm_pos = p_small_arm_.at(k);
                        p2.wrist_pos = p_wrist_.at(k);
                        p2.hand_pos = p_hand_.at(k);
                        p2.lumbar_vel = v_lumbar_.at(k);
                        p2.big_arm_vel = v_big_arm_.at(k);
                        p2.small_arm_vel = v_small_arm_.at(k);
                        p2.wrist_vel = v_wrist_.at(k);
                        p2.hand_vel = v_hand_.at(k);
                        p2.lumbar_acc = a_lumbar_.at(k);
                        p2.big_arm_acc = a_big_arm_.at(k);
                        p2.small_arm_acc = a_small_arm_.at(k);
                        p2.wrist_acc = a_wrist_.at(k);
                        p2.hand_acc = a_hand_.at(k);

                        bzero(writebuf, sizeof(writebuf));   // 清零writebuf
                        memcpy(writebuf, &p2, sizeof(p2));    // 复制p2数据到writebuf
                        unsigned int sendMsg_len = write(arrConSocket[i].socketCon, writebuf,sizeof(p2)); //返回值：写入文档的字节数（成功）；-1（出错)
                        if (sendMsg_len > 0) {
                            //ROS_INFO("Send Msg_len %d successful!",sendMsg_len);
                        } else {
                            printf("向%s:%d发送失败\n", arrConSocket[i].ipaddr, arrConSocket[i].port);
                        }
                    }
                }

            }
            /****** 只有当有数据更新的时候才发送 *******/
            point_changed = false;
        }
        /***** 判断线程存活多少 ******/
        for(int i=0;i<thrReceiveClientCount;i++)
        {
            if(checkThrIsKill(arrThrReceiveClient[i]) == 1)
            {
                printf("有个线程被杀了\n");
                thrReceiveClientCount--;
            }
        }
        //printf("当前有接受数据线程多少个：%d\n",thrReceiveClientCount);
    }

    /* 关闭原始套接字 */
    close(socketListen);
    return 0;
}

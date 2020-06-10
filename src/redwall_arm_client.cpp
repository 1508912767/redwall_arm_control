#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <sys/epoll.h>
#include <iostream>

/* 使用变长数组 */
#include <vector>
#include <algorithm>

#define p813 "/sys/devices/ocp.3/pwm_test_P8_13.12/"
#define p914 "/sys/devices/ocp.3/pwm_test_P9_14.13/"
#define p921 "/sys/devices/ocp.3/pwm_test_P9_21.14/"
#define GPIO_DIR "/sys/class/gpio/"
#define pwm_path "/sys/devices/bone_capemgr.9/slots"

#define PORT 7788
#define zero 0
#define one 1
#define ADDR "127.0.0.1"


using namespace std;
vector<double> time_from_start;
vector<double> p_lumbar;
vector<double> p_big_arm;
vector<double> p_small_arm;
vector<double> p_wrist;
vector<double> p_hand;
vector<double> v_lumbar;
vector<double> v_big_arm;
vector<double> v_small_arm;
vector<double> v_wrist;
vector<double> v_hand;
vector<double> a_lumbar;
vector<double> a_big_arm;
vector<double> a_small_arm;
vector<double> a_wrist;
vector<double> a_hand;

/* 存储的结构体 p1*/
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

struct vel_data p1;
char recvbuf[sizeof(p1)];

typedef struct MySocketInfo
{
    int socketCon;
    unsigned long ipaddr;
    unsigned short port;
}_MySocketInfo;

/*SOCKET客户端处理接受数据函数*/
void *fun_thrReceiveHandler(void *socketCon)
{
    while(1)
    {
        /* 保存目标套接字信息 */
        int _socketCon = *((int *)socketCon);
        bzero(recvbuf, sizeof(p1));
        unsigned int buffer_length = read(_socketCon,recvbuf,sizeof(p1));
        if(buffer_length == 0){
            printf("服务器端异常关闭\n");
            exit(-1);
        }else if(buffer_length < 0){
            printf("接受客户端数据失败\n");
            break;
        }
        //printf("Receive buffer length %d\n",buffer_length);
        /* 将接收到的速度控制信息以p1结构体格式解码*/
        memcpy(&p1,recvbuf,sizeof(recvbuf));
        //point_num = p1.point_num_;
        time_from_start.push_back(p1.time_from_begin);
        p_lumbar.push_back(p1.lumbar_pos);
        p_big_arm.push_back(p1.big_arm_pos);
        p_small_arm.push_back(p1.small_arm_pos);
        p_wrist.push_back(p1.wrist_pos);
        p_hand.push_back(p1.hand_pos);
        v_lumbar.push_back(p1.lumbar_vel);
        v_big_arm.push_back(p1.big_arm_vel);
        v_small_arm.push_back(p1.small_arm_vel);
        v_wrist.push_back(p1.wrist_vel);
        v_hand.push_back(p1.hand_vel);
        a_lumbar.push_back(p1.lumbar_acc);
        a_big_arm.push_back(p1.big_arm_acc);
        a_small_arm.push_back(p1.small_arm_acc);
        a_wrist.push_back(p1.wrist_acc);
        a_hand.push_back(p1.hand_acc);
    }
}
void *lumbar_motor(void *)
{
    while(1)
    {
        usleep(1000);
        if(time_from_start.size() != 0)
        {
            vector<double>::iterator pStart = time_from_start.begin();
            vector<double>::iterator pEnd = time_from_start.end();
            while (pStart != pEnd){
                cout << *pStart <<endl;
                pStart++;
            }
            time_from_start.clear();
            p_lumbar.clear();
            v_lumbar.clear();
            a_lumbar.clear();
        }
    }

}

/* 设置gpio函数 */
int set_gpio(void)
{
    FILE *stream1 = fopen(GPIO_DIR"gpio44/direction","r+");
    /*如果打开文件失败则先加载*/
    if(stream1==NULL)
    {
        stream1=fopen(GPIO_DIR"export","w");
        fwrite("44",sizeof(int),2,stream1);
        fclose(stream1);
        stream1=fopen(GPIO_DIR"gpio44/direction","r+");
    }
    /* P8.12端口为输出*/
    fwrite("out",sizeof(char),3,stream1);
    fclose(stream1);

    FILE *stream2=fopen(GPIO_DIR"gpio45/direction","r+");
    /*如果打开文件失败则打开相应端口*/
    if (stream2==NULL)
    {
        stream2=fopen(GPIO_DIR"export","w");
        fwrite("45",sizeof(int),2,stream2);
        fclose(stream2);
        stream2=fopen(GPIO_DIR"gpio45/direction","r+");
    }
    /* P8.11端口为输出*/
    fwrite("out",sizeof(char),3,stream2);
    fclose(stream2);

    /* 设置GPIO70-->P8-45为输入模式 */
    FILE *stream3 = fopen(GPIO_DIR"gpio26/direction","r+");
    if (stream3==NULL)
    {
        stream3 = fopen(GPIO_DIR"export","w");
        fwrite("26",sizeof(int),2,stream3);
        fclose(stream3);
        stream3 = fopen(GPIO_DIR"gpio26/direction","r+");
    }
    fwrite("out",sizeof(char),3,stream3);
    fclose(stream3);

    FILE *stream4= fopen(p921"polarity","r+");
    if(!stream4)
    {
        printf("p921的polarity打开失败\n");
    }
    char buf3[7];
    /* 舵机不需要方向信号,所以不必写入gpio26，但是必须给定默认polarity=0才行 */
    sprintf(buf3,"%d",zero);
    fwrite(buf3,sizeof(int),3,stream4);
    fclose(stream4);

    return 0;
}

/* 加载设备树函数 */
int write_tree(void)
{
    int fd = open(pwm_path,O_WRONLY);
    if(fd < 0){
        printf("打开slots失败\n");
    }
    write(fd,"am33xx_pwm",10);
    write(fd,"bone_pwm_P8_13",14);
    write(fd,"bone_pwm_P9_14",14);
    write(fd,"bone_pwm_P9_21",14);
    close(fd);

    return 0;
}

/* 主函数完成接受和发送数据 */
int main(int argc, char **argv)
{
    /***** 加载设备树 *****/
    //write_tree();

    /***** 配置GPIO用于控制电机转向 *****/
    //set_gpio();

    /***** 开始套接字通信 *****/
    printf("开始socket\n");
    int socketCon = socket(AF_INET, SOCK_STREAM, 0);
    if(socketCon < 0)
    {
        printf("创建TCP连接套接字失败\n");
        exit(-1);
    }

    /***** 绑定服务器端口地址信息 *****/
    struct sockaddr_in server_addr;
    bzero(&server_addr,sizeof(struct sockaddr_in));
    server_addr.sin_family=AF_INET;
    server_addr.sin_port=htons(PORT);
    if(inet_pton(AF_INET, ADDR, &server_addr.sin_addr) !=1)
    {
        printf("ipv4地址转换失败");
    }

    /***** 连接服务器 *****/
    int res_con = connect(socketCon,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr));
    if(res_con != 0)
    {
        printf("连接服务器失败\n");
        exit(-1);
    }
    printf("连接成功,连接结果为：%d\n",res_con);

    /***** 开启新的实时接受数据线程 *****/
    pthread_t thrReceive;
    pthread_create(&thrReceive,NULL,fun_thrReceiveHandler,&socketCon);

    /***** 开启处理电机数据线程 *****/
    pthread_t id1;
    pthread_create(&id1,NULL,lumbar_motor,NULL);
//    pthread_t id2;
//    pthread_create(&id2,NULL,bigarm_motor,NULL);
//    pthread_t id3;
//    pthread_create(&id3,NULL,smallarm_motor,NULL);
//    pthread_t id4;
//    pthread_create(&id4,NULL,wrist_motor,NULL);
//    pthread_t id5;
//    pthread_create(&id5,NULL,hand_motor,NULL);

    pthread_detach(id1);
//    pthread_detach(id2);
//    pthread_detach(id3);
//    pthread_detach(id4);
//    pthread_detach(id5);
    pthread_join(thrReceive,NULL);

    /***** 关闭套接字 *****/
    close(socketCon);
    return 0;
}

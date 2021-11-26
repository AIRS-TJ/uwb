/*******************************/
#include <cstring>
#include <iostream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cfloat>

#include <sys/time.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <strings.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
/*******************************/
//主线程函数：void* udp_read(void* pstatu)
//关键设定：本机IP（sin_addr）/端口号（sin_port）   该配置与PYTHON端需要一致
//示例：addr.sin_addr.s_addr = inet_addr("192.168.2.101");
//		addr.sin_port =htons(8000);

/*
开始线程
thread_fun线程函数
pthread线程函数所在pthread变量
par线程函数参数
COM_STATU线程函数状态控制变量1:运行0:退出
*/
  ros::Publisher odom_pub;

int start_thread_func(void *(*func)(void *), pthread_t *pthread, void *par, int *COM_STATU)
{
    *COM_STATU = 1;
    memset(pthread, 0, sizeof(pthread_t));
    int temp;
    /*创建线程*/
    if ((temp = pthread_create(pthread, NULL, func, par)) != 0)
        printf("线程创建失败!\n");
    else
    {
        int id = pthread_self();
        printf("线程%lu被创建\n", *pthread);
    }
    return temp;
}

/*
结束线程
pthread线程函数所在pthread变量
COM_STATU线程函数状态控制变量1:运行0:退出
*/

int stop_thread_func(pthread_t *pthread, int *COM_STATU)
{
    printf("preparestopthread%lu/n", *pthread);
    *COM_STATU = 0;
    if (*pthread != 0)
    {
        pthread_join(*pthread, NULL);
    }
    printf("线程%d退出!/n", *COM_STATU);
}

//打印线程ID号
void printtid(void)
{
    int id = pthread_self();
    printf("inthread%u/n", id);
}

//返回系统时间的秒位
double sys_sec()
{
    struct timeval tv2;
    gettimeofday(&tv2, NULL);
    return tv2.tv_sec;
}
//返回系统时间的豪秒位
double sys_millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

//返回整体系统时间-换算成秒
double get_systime_sec()
{
    double sys_time;
    struct timeval tnow;
    gettimeofday(&tnow, NULL);
    sys_time = tnow.tv_sec + tnow.tv_usec / 1000000.0;
    return sys_time;
}

//记录程序启动时刻的整体系统时间（豪秒）
static double robot_sys_millis = 0;
void init_robot_sys_millis()
{
    robot_sys_millis = sys_millis();
}

//记录程序执行时间（豪秒）
double get_robot_sys_millis()
{
    double sys_millis_now = sys_millis();
    return sys_millis_now - robot_sys_millis;
}
//记录程序执行时间（秒）
double get_time()
{
    return get_robot_sys_millis() / 1000.0;
}

static long long pkg_count_pos = 0;              //记录包数量
void ANO_DT_Data_Receive_Anl(unsigned char *buf) //解析函数
{
    static double now = 0;
    static double last = 0;

    int date_count = 0;
    float udp_pos[3];
    memcpy(&udp_pos[0], buf + 2 + (date_count++) * sizeof(float), sizeof(float)); //复制UDP-buf值到udp_pos使用
    memcpy(&udp_pos[1], buf + 2 + (date_count++) * sizeof(float), sizeof(float)); //复制UDP-buf值到udp_pos使用
    memcpy(&udp_pos[2], buf + 2 + (date_count++) * sizeof(float), sizeof(float)); //复制UDP-buf值到udp_pos使用
    
    //ros topic
    ros::Time current_time = ros::Time::now();
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x= udp_pos[0];
    odom.pose.pose.position.y= udp_pos[1];
    odom.pose.pose.position.z= udp_pos[2];
    //publish the message
    odom_pub.publish(odom);
    //

    pkg_count_pos++;
    last = now;
    now = get_systime_sec();

    if ((now - last) > 0.1)
        printf("badt:  %lf    udp_pos:%lf %lf %lf\n", now - last, udp_pos[0], udp_pos[1], udp_pos[2]); //打印传输延时高的包
}

//数据预处理函数
//数据格式： 0XAA(帧头1) + 0xAF(帧头2) + float1 + float2 + float3 + 0x66(帧尾)
void ANO_DT_Data_Receive_Prepare(unsigned char data)
{
    static int _data_len_set = sizeof(float) * 3 + 1; //
    static unsigned char RxBuffer[1000];
    static int _data_len = sizeof(float) * 3 + 1; //
    static int _data_cnt = 0;                     //_data_len   2+  104 +  +1
    static unsigned char state = 0;

    if (state == 0 && data == 0xAA) //帧头1
    {
        state = 1;
        RxBuffer[0] = data;
        //printf(" state0 \n");
    }
    else if (state == 1 && data == 0xAF) //帧头2
    {
        state = 2;
        RxBuffer[1] = data;
        _data_cnt = 0;
        _data_len = _data_len_set;
        //printf(" state1 \n");
    }
    else if (state == 2)
    {
        _data_len--;
        RxBuffer[2 + _data_cnt++] = data;

        if (_data_len == 0 && data == 0x66)
        {
            ANO_DT_Data_Receive_Anl(RxBuffer);
            state = 0;
        }

        //state = 3;
        //printf(" state2 \n");
    }
    else
    {
        state = 0;
        //printf(" no: %x  \n",data);
    }
}

//设置非阻塞
static void setnonblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0)
    {
        printf("fcntl F_GETFL fail\n");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0)
    {
        printf("fcntl F_SETFL fail\n");
    }
}

//读取UDP数据线程
void *udp_read(void *pstatu)
{
    //创建socket对象
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    setnonblocking(sockfd);
    printf("sockfd --- %d......\n", sockfd);
    //创建网络通信对象
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(1325);                       //port
    addr.sin_addr.s_addr = inet_addr("192.168.2.73"); //laikago_pro ip_set

    //绑定socket对象与通信链接
    usleep(2000);
    int ret = -1;
    ret = bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)); //绑定socket对象与通信链接
    printf("bind---%d\n", ret);
    if (ret < 0)
    {
        printf("bind err\n");
        //return -1;
    }
    else
        printf(" bind ok\n");

    system("netstat -an | grep 8888"); // 查看连接状态

    struct sockaddr_in cli;
    socklen_t len = sizeof(cli);
    long long counter_pos_msg = 0;
    long stop_count = 0; //读取失败流程计数
    while (1)
    {
        counter_pos_msg += 1; //流程计数
        //printf("UDO_POS --- runing......%d\n",counter_pos_msg);
        unsigned char buf[1000] = {0, 0, 0, 0, 0, 0, 0, 0};
        int num = 0;                                                                                 //读取字符个数
        num = recvfrom(sockfd, &buf, sizeof(unsigned char) * 100, 0, (struct sockaddr *)&cli, &len); //从缓冲区读取UDP数据
        //printf("NUM:%d\n",num);
        if (num > 0)
        {
            for (int i = 0; i < num; i++)
                ANO_DT_Data_Receive_Prepare(buf[i]);
            stop_count = 0;
        } //数据解析
        else
            stop_count++;
        if (stop_count >= 5000)
            printf("\nUDO_POS --- NO DATA......\n");
        usleep(1000); //1ms
        //printf("UDO_POS --- runing......\n");

        //ROS_INFO("robot_state:%f\n",robot_state);
    }
    close(sockfd);
}

//Demo
int main(int argc, char** argv)
{
    ros::init(argc, argv, "optitrack_odom");
    ros::NodeHandle n;
    //TASK_PTHREAD  position
    pthread_t thread[50];
    const int UDP_THREAD_ID = 1;
    int UDP_READ_STATU = 0;

    odom_pub = n.advertise<nav_msgs::Odometry>("optitrack_odom", 50);

    if (start_thread_func(udp_read, &thread[UDP_THREAD_ID], &UDP_READ_STATU, &UDP_READ_STATU) != 0) //启动udp_read线程
    {
        printf("UDP_POS error to leave/n");
        return -1;
    }
    struct sched_param sched, my_param;
    sched.sched_priority = 99;                                      //准备设定线程优先级 99 数据
    pthread_setschedparam(thread[UDP_THREAD_ID], SCHED_RR, &sched); //设定线程优先级 99
    usleep(1000 * 1000);
    printf("UDO_POS --- INIT OK!!!\n");
    while (/* condition */1)
    {
       usleep(1000); /* code */
    }
    

}
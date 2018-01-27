#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/serial.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>

enum port_states
{
	st_server,            
	st_client,      
};

typedef struct
{
	uint	tx_lens;//发送数据个数
	uint	rx_lens;//接收数据个数
	uint 	tx_pkgs;//发送数据包个数
	uint	rx_pkgs;//接受数据包个数
	uint	err_lens;//错误的数据包个数
	uint	err_pkgs;//错误的数据包个数
	uint	lost_pkgs;//丢失的数据包个人数
	uint	lost_lens;//丢失的数据包个人数
	enum port_states	states;
	uint		rx_timeout;
	float	accuracy;
	int		ufd;
	char	st_exit;//通知进程结束
	pthread_t id;
}tds_Ch_Data;

#define FALSE 0
#define TRUE 1

#define max_port	5

int RX_TIME_OUT_MS;//收报超时时间
int test_buad;//测试用比特率
int start_run = 0;
tds_Ch_Data ch_data[max_port];

//打开对应通道的串口
int open_uart(int port, int tty, int bond)
{
	char str[32];
	int fd;
	struct termios newtio;
	
	sprintf(str, "/dev/ttySAC%d", tty);//内部串口1～2
		
	//fd = open(str, O_RDWR | O_NOCTTY ); //柱塞读
	fd = open(str, O_RDWR | FNDELAY ); //非柱塞读
    if (fd <0) 
    {
		printf("open %s Err\n", str);
		perror(str); 
		return(-1); 
	}

    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag |= bond;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    newtio.c_cflag &= ~CRTSCTS;

    newtio.c_lflag = 0;
    newtio.c_oflag = 0;

    newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    newtio.c_oflag  &= ~OPOST;   /*Output*/

    newtio.c_cc[VMIN] =  1; 
    newtio.c_cc[VTIME] = 0;

    newtio.c_iflag &= ~(IXON|IXOFF|IXANY);

    cfsetispeed(&newtio, bond);
    cfsetospeed(&newtio, bond);
 
    /*
      now clean the modem line and activate the settings for the port
    */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    
    ch_data[port].ufd = fd;
    
    return 1; 
}

//关闭对应通道的串口
int close_uart(int port)
{
	close(ch_data[port].ufd);
	ch_data[port].ufd = 0;
}

void timer_thread(union sigval v)
{  
	int i;
    for(i=0;i<max_port;i++)
    {
		//ch_data[i].rx_timeout ++;
		__sync_fetch_and_add(&(ch_data[i].rx_timeout), 1);
	}
	//printf("time ok");
}

void uart_client(int port, int tty, int boud)
{
	int nread,i,a;
	char rx_buf[1024];
	char tx_buf[1024];
	
	//ch_data[port].states = st_client;
	
	open_uart(port, tty, boud);//打开对应串口
	
	nread = read(ch_data[port].ufd, rx_buf, 256);//读空接收缓存
	
	while(1)
	{
		if(ch_data[port].st_exit)
			break;
			
		nread = read(ch_data[port].ufd, rx_buf, 256);//读取255字节的串口数据到buf
		if(nread)
		{
			for(i=0;i<nread;i++)
			{
				if(rx_buf[i]==0x00)//找到帧头
					a=0;
				if(a < sizeof(tx_buf))
					tx_buf[a]=rx_buf[i];
				else
					a = 0;
				a++;
				ch_data[port].rx_lens++;

				if(rx_buf[i]==0xff)//找到帧尾
				{
					/*printf("\nGet Dada:\n");
					for(i=0;i<256;i++)
						printf("0x%02x ", tx_buf[i]);*/
					ch_data[port].rx_pkgs++;
					write(ch_data[port].ufd,tx_buf,a);//发送数组
					ch_data[port].tx_pkgs++;
					ch_data[port].tx_lens += a;
					a = 0;
				}
			}
		}
		usleep(3000);
	}
	
	close_uart(port);//关闭对应串口
	exit(1);
}

void uart_server(int port, int tty, int boud)
{
	int nread,i,a,n;
	char rx_buf[1024];
	char tx_buf[1024];
	char tmp_buf[1024];
	
	//ch_data[port].states = st_server;
	
	open_uart(port, tty, boud);//打开对应串口
	
	for(i=0;i<256;i++)//初始化发送数据
		tmp_buf[i] = i;
		
	nread = read(ch_data[port].ufd, rx_buf, 256);//读空接收缓存
	
	while(start_run != 1)
		usleep(1000);
		
	while(1)
	{
txdata:	if(ch_data[port].st_exit)
			break;
		
		write(ch_data[port].ufd,tmp_buf,256);//发送数组
		//ch_data[port].rx_timeout = 0;
		__sync_lock_test_and_set(&(ch_data[port].rx_timeout), 0);
		ch_data[port].tx_pkgs++;
		ch_data[port].tx_lens += 256;
		
		while(1)
		{
			if(ch_data[port].rx_timeout > RX_TIME_OUT_MS)
			{
				ch_data[port].lost_pkgs ++;
				ch_data[port].lost_lens += 256;
				a = 0;
				//printf("port %d rx time out!\n", port);
				goto txdata;
			}
			
			memset(rx_buf, 0x55, sizeof(rx_buf));
			nread = read(ch_data[port].ufd, rx_buf, 256);//读取255字节的串口数据到buf
			if(nread)
			{
				for(i=0;i<nread;i++)
				{
					if(rx_buf[i]==0x00)//找到帧头
						a=0;
					if(a < sizeof(tx_buf))
						tx_buf[a]=rx_buf[i];
					else
						a = 0;
					a++;
					ch_data[port].rx_lens++;

					if(rx_buf[i]==0xff)//找到帧尾
					{
						ch_data[port].rx_pkgs++;
						a=0;
						for(n=0;n<256;n++)
						{
							if(tx_buf[n] != n)
							{
								ch_data[port].err_lens ++;
								a = 1;
							}
						}
						if(a)
						{
							ch_data[port].err_pkgs ++;
							/*printf("\nGet Dada:\n");
							for(i=0;i<256;i++)
								printf("0x%02x ", tx_buf[i]);*/
						}
						else	
							goto txdata;
					}
				}
			}
			usleep(3000);
		}
		usleep(3000);
	}
	
	close_uart(port);//关闭对应串口
	exit(1);
}

void thread1(void)
{
	if(ch_data[0].states == st_server)
		uart_server(0, 1, test_buad);
	else
		uart_client(0, 1, test_buad);
}
void thread2(void)
{
	if(ch_data[1].states == st_server)
		uart_server(1, 2, test_buad);
	else
		uart_client(1, 2, test_buad);
}
void thread3(void)
{
	if(ch_data[2].states == st_server)
		uart_server(2, 3, test_buad);
	else
		uart_client(2, 3, test_buad);
}

void thread4(void)
{
	if(ch_data[3].states == st_server)
		uart_server(3, 4, test_buad);
	else
		uart_client(3, 4, test_buad);
}
void thread5(void)
{
	if(ch_data[4].states == st_server)
		uart_server(4, 5, test_buad);
	else
		uart_client(4, 5, test_buad);
}


void my_handler_ctrl_c(int s)//捕捉 ctrl + c
{
	char i;
	for(i=0;i<max_port;i++)
	{
		ch_data[i].st_exit = 1;
		//pthread_exit(ch_data[i].id, NULL);
	}
	printf("Caught signal %d\n",s);
} 

#define CLOCKID CLOCK_REALTIME

int main(int argc, char *argv[])
{   
	char i;
	int input_buad;
	timer_t timerid;
	struct sigevent evp;  
	time_t start_time;
	time_t now_time;
    struct tm * timeinfo;
    memset(&evp, 0, sizeof(struct sigevent));       //清零初始化  
    char tmp_printf[4096];
    char tmp_str[512];
    
    if (argc == 2)
    {
		ch_data[0].states = st_client;
		ch_data[1].states = st_server;
		ch_data[2].states = st_server;
		ch_data[3].states = st_server;
		ch_data[4].states = st_client;
	}
	else if(argc == 3)
	{
		if(argv[2][1] == 's')
		{
			ch_data[0].states = st_server;
			ch_data[1].states = st_server;
			ch_data[2].states = st_server;
			ch_data[3].states = st_server;
			ch_data[4].states = st_server;
		}
		else
		{
			ch_data[0].states = st_client;
			ch_data[1].states = st_client;
			ch_data[2].states = st_client;
			ch_data[3].states = st_client;
			ch_data[4].states = st_client;
		}
	}
    else
    {
        printf("\n\rusage1: %s [baud] \n\r", argv[0]);
        printf("\n\rusage2: %s [baud] [-s -c]\n\r", argv[0]);
        exit(-1);
    }
    
    input_buad = atoi(argv[1]);
    switch(input_buad)
    {
		case 300:
		test_buad = B300;break;
		case 1200:
		test_buad = B1200;break;
		case 2400:
		test_buad = B2400;break;
		case 4800:
		test_buad = B4800;break;
		case 9600:
		test_buad = B9600;break;
		case 19200:
		test_buad = B19200;break;
		case 38400:
		test_buad = B38400;break;
		case 57600:
		test_buad = B57600;break;
		case 115200:
		test_buad = B115200;break;
		default:
		printf("Does not support the baud rate\n\r");
		exit(1);
	}
    //RX_TIME_OUT_MS = 1*256*10*4*1000/input_buad;//计算超时时间
    RX_TIME_OUT_MS = 1*256*10*4*1000/input_buad;//计算超时时间
  
    evp.sigev_value.sival_int = 111;            //也是标识定时器的，这和timerid有什么区别？回调函数可以获得  
    evp.sigev_notify = SIGEV_THREAD;            //线程通知的方式，派驻新线程
    evp.sigev_notify_function = timer_thread;   //线程函数地址
  
    if (timer_create(CLOCKID, &evp, &timerid) == -1)  
    {  
        perror("fail to timer_create");  
        exit(-1);  
    }  
  
    // XXX int timer_settime(timer_t timerid, int flags, const struct itimerspec *new_value,struct itimerspec *old_value);  
    // timerid--定时器标识  
    // flags--0表示相对时间，1表示绝对时间  
    // new_value--定时器的新初始值和间隔，如下面的it  
    // old_value--取值通常为0，即第四个参数常为NULL,若不为NULL，则返回定时器的前一个值  
      
    //第一次间隔it.it_value这么长,以后每次都是it.it_interval这么长,就是说it.it_value变0的时候会装载it.it_interval的值  
    struct itimerspec it;  
    it.it_interval.tv_sec = 0;  
    it.it_interval.tv_nsec = 1*1000*1000;//1mS
    it.it_value.tv_sec = 0;  
    it.it_value.tv_nsec = 1*1000*1000;//1mS  
  
    if (timer_settime(timerid, 0, &it, NULL) == -1)  
    {  
        perror("fail to timer_settime");  
        exit(-1);  
    }

	struct sigaction sigIntHandler;  //捕捉 ctrl + c
	sigIntHandler.sa_handler = my_handler_ctrl_c;  
	sigemptyset(&sigIntHandler.sa_mask);  
	sigIntHandler.sa_flags = 0;  
	sigaction(SIGINT, &sigIntHandler, NULL);  
    
    
    int ret=pthread_create(&ch_data[0].id,NULL,(void *) thread1,NULL);
	if(ret!=0)
	{
		printf ("Create pthread error!\n");
		exit (1);
	}
    ret=pthread_create(&ch_data[1].id,NULL,(void *) thread2,NULL);
	if(ret!=0)
	{
		printf ("Create pthread error!\n");
		exit (1);
	}
	ret=pthread_create(&ch_data[2].id,NULL,(void *) thread3,NULL);
	if(ret!=0)
	{
		printf ("Create pthread error!\n");
		exit (1);
	}
	ret=pthread_create(&ch_data[3].id,NULL,(void *) thread4,NULL);
	if(ret!=0)
	{
		printf ("Create pthread error!\n");
		exit (1);
	}
	ret=pthread_create(&ch_data[4].id,NULL,(void *) thread5,NULL);
	if(ret!=0)
	{
		printf ("Create pthread error!\n");
		exit (1);
	}
	
	sleep(1);
	start_run = 1;
	time ( &start_time);
	while(1)
	{
		printf("\033[H""\033[J");//清屏
		time ( &now_time);
		//system("true > /tmp/log");
		
		
		sprintf(tmp_printf, "Test App Speed %dbps time out %dmS Test time %dmin %dSec\n\r", input_buad, RX_TIME_OUT_MS,
			  (now_time - start_time)/60, (now_time - start_time)%60);
		timeinfo = localtime ( &start_time );
		sprintf(tmp_str, "App start time -- %s", asctime (timeinfo));
		strcat(tmp_printf, tmp_str);//合并字符串
		timeinfo = localtime ( &now_time );
		sprintf(tmp_str, "Now time -- %s", asctime (timeinfo));
		strcat(tmp_printf, tmp_str);//合并字符串
			  
		sprintf(tmp_str, "PORT  states    tx_lens     rx_lens     tx_pkgs     rx_pkgs     err_pkgs     err_lens    lost_pkgs    lost_lens  accuracy\n\r");
		strcat(tmp_printf, tmp_str);//合并字符串
		for(i=0;i<max_port;i++)
		{
			if(ch_data[i].tx_lens != 0)//计算串口合格率
				ch_data[i].accuracy = (float)(ch_data[i].tx_lens - ch_data[i].err_lens - ch_data[i].lost_lens)
									  /(float)ch_data[i].tx_lens*100.0;
				
			sprintf(tmp_str, "%2d    %s ", i,(ch_data[i].states == st_server) ? 
				   "\033[1;32;40mserver\033[0m" : "\033[1;36;40mclient\033[0m");
			strcat(tmp_printf, tmp_str);//合并字符串
			if(ch_data[i].states == st_server)
			{
				if(ch_data[i].accuracy < 100)
					sprintf(tmp_str, "%10d  %10d  %10d  %10d   %10d   %10d   %10d   %10d   \033[1;31;40m%.2f%%\033[0m\n\r",
					ch_data[i].tx_lens, ch_data[i].rx_lens, 
					ch_data[i].tx_pkgs, ch_data[i].rx_pkgs, ch_data[i].err_pkgs, ch_data[i].err_lens, ch_data[i].lost_pkgs,
					ch_data[i].lost_lens, ch_data[i].accuracy);
				else
					sprintf(tmp_str, "%10d  %10d  %10d  %10d   %10d   %10d   %10d   %10d   \033[1;32;40m%.2f%%\033[0m\n\r",
					ch_data[i].tx_lens, ch_data[i].rx_lens, 
					ch_data[i].tx_pkgs, ch_data[i].rx_pkgs, ch_data[i].err_pkgs, ch_data[i].err_lens, ch_data[i].lost_pkgs,
					ch_data[i].lost_lens, ch_data[i].accuracy);
					
			}
			else
				sprintf(tmp_str, "%10d  %10d  %10d  %10d\n\r",ch_data[i].tx_lens, ch_data[i].rx_lens, 
				ch_data[i].tx_pkgs, ch_data[i].rx_pkgs);
			strcat(tmp_printf, tmp_str);//合并字符串
			
		}
		strcat(tmp_printf, "\0");
		printf("%s", tmp_printf);
		
		sleep(5);
		
		ret = 1;
		for(i=0;i<max_port;i++)
		{
			if(ch_data[i].ufd != 0)
				ret = 0;
		}
		if(ret)
		{
			printf("app exit\n\r");
			exit(1);
		}
	}
	
	return 0;
}

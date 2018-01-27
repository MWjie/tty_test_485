#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/serial.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>

//20171207 mwj 485串口测试程序 根据tty_test.c修改

enum port_status{
    st_server,
    st_client
};

typedef struct{
	uint	tx_lens;//发送数据个数
	uint	rx_lens;//接收数据个数
	uint 	tx_pkgs;//发送数据包个数
	uint	rx_pkgs;//接受数据包个数
	uint	err_lens;//错误的数据包个数
	uint	err_pkgs;//错误的数据包个数
	uint	lost_pkgs;//丢失的数据包个人数
	uint	lost_lens;//丢失的数据包个人数
	enum    port_status	states;
	uint	rx_timeout;
	float	accuracy;
	int		ufd;
	char	st_exit;//通知进程结束
	pthread_t id;
}tds_Ch_Data;

#define MAX_PORT 		( 3 )//最大串口数
#define CLOCKID			( CLOCK_REALTIME )

tds_Ch_Data ch_data[MAX_PORT];
int RX_TIME_OUT_MS;//收报超时时间
_Bool start_run = 0;//串口线程运行标志

//打开对应通道的串口
int open_uart(int port, int tty, int baudrate){
	char str[32];
	int fd;
	struct termios newtio;
	
	sprintf(str, "/dev/ttySAC%d", tty);//内部串口1～2
		
	//fd = open(str, O_RDWR | O_NOCTTY ); //柱塞读
	fd = open(str, O_RDWR | FNDELAY ); //非柱塞读
    if (fd <0){
		printf("open %s Err\n", str);
		perror(str); 
		return(-1); 
	}

    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag |= baudrate;
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

    cfsetispeed(&newtio, baudrate);
    cfsetospeed(&newtio, baudrate);
 
    /*
      now clean the modem line and activate the settings for the port
    */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    
    ch_data[port].ufd = fd;
    
    return 1; 
}

//打开串口
void openSerail(int baudrate){
	unsigned char port;
	for(port=0; port<MAX_PORT; port++){
		open_uart(port, port+1, baudrate);
	}
}

//关闭串口
void closeSerail(void){
	unsigned char port;
	for(port=0; port<MAX_PORT; port++){
		close(ch_data[port].ufd);
		ch_data[port].ufd = 0;
		ch_data[port].st_exit = 1;
	}	
}

//串口接收
void uart_client(int port){
    ssize_t nread;
	ushort i, count;
    _Bool is_frame, is_errframe;
	char rx_buf[1024];

	nread = read(ch_data[port].ufd, rx_buf, 256);//读空接收缓存

    while(start_run != 1){
		usleep(1000);
	}

	while(1){
		count = 0;		
		is_frame = 0;
		is_errframe = 0;

		if(ch_data[port].st_exit){
			exit(1);
		}	
		if(ch_data[port].states == st_server){
			break;
		}	
		if(ch_data[port].rx_timeout > RX_TIME_OUT_MS)
		{
			ch_data[port].lost_lens += 256;
			is_errframe = 1;
			continue;
		}	
		nread = read(ch_data[port].ufd, rx_buf, 256);//读取255字节的串口数据到buf
		if(nread){			
			__sync_lock_test_and_set(&(ch_data[port].rx_timeout), 0);
			for(i=0; i<nread; i++){
				if(rx_buf[i] == 0x00){//找到帧头
					count = 0;
                    is_frame = 1;
                }
				if(is_frame && count < sizeof(rx_buf)){
                    if(rx_buf[i] != count){//是否存在错误帧                   
                        ch_data[port].err_lens++;
                        is_errframe = 1;
                    }
                    count++;				    
                }
				ch_data[port].rx_lens++;
				if(rx_buf[i] == 0xff){//找到帧尾
					ch_data[port].rx_pkgs++;
				}
                if(is_errframe){//存在错误帧
                    ch_data[port].err_pkgs++;
				}
			}
		}
		usleep(1000);
	}
}

//串口发送
void uart_server(int port){
    ssize_t nread;
	char rx_buf[1024];
	char tx_buf[1024];
	unsigned short i;
	unsigned char  p;

	for(i=0; i<256; i++){//初始化发送数据
		tx_buf[i] = i;
	}	
	nread = read(ch_data[port].ufd, rx_buf, 256);//读空接收缓存
	
	while(start_run != 1){
		usleep(1000);
	}	

	while(1){
		if(ch_data[port].st_exit){
			exit(1);
		}
		if(ch_data[port].states == st_client){
			break;
		}
		write(ch_data[port].ufd, tx_buf, 256);//发送数组
		for(p=0; p<MAX_PORT; p++){
			__sync_lock_test_and_set(&(ch_data[p].rx_timeout), 0);
		}
		ch_data[port].tx_pkgs++;
		ch_data[port].tx_lens += 256;

		sleep(1);
	}
}

void client2server(void){
	unsigned char port;
	for(port=0; port<MAX_PORT; port++){
		if(ch_data[port].states == st_server){
			ch_data[port].states = st_client;
			if(port == MAX_PORT-1){
				ch_data[0].states = st_server;
			}else{
				ch_data[port+1].states = st_server;
			}
			break;
		}
	}
}

void thread1(void){
	while(1){
		if(ch_data[0].states == st_server)
			uart_server(0);
		else
			uart_client(0);
	}	
}
void thread2(void){
	while(1){
		if(ch_data[1].states == st_server)
			uart_server(1);
		else
			uart_client(1);
	}
}
void thread3(void){
	while(1){
		if(ch_data[2].states == st_server)
			uart_server(2);
		else
			uart_client(2);
	}
}

void timer_thread(union sigval v)
{  
	unsigned char port;
    for(port=0; port<MAX_PORT; port++)
    {
		__sync_fetch_and_add(&(ch_data[port].rx_timeout), 1);
	}
}

void my_handler_ctrl_c(int s){//捕捉 ctrl + c
	closeSerail();
	printf("Caught signal %d\n",s);
} 

int main(int argc, char **argv){
    int test_buad;//波特率
	int serverSendlens;
	int rxbefore[MAX_PORT] = {0};
	int txbefore[MAX_PORT] = {0};
	unsigned char port;
	timer_t timerid;	
	time_t start_time;
	time_t now_time;
    struct tm * timeinfo;
	struct sigevent evp;  
    memset(&evp, 0, sizeof(struct sigevent));//清零初始化  
	char tmp_printf[4096];
    char tmp_str[512];

    if(argc > 2){
        printf("\n\rusage1: %s [baud] \n\r", argv[0]);
        printf("\n\rusage2: %s [baud] [-s -c]\n\r", argv[0]);
        exit(-1);
    }
    
    switch(atoi(argv[1])){
        case 300:   test_buad = B300;   break;
        case 1200:  test_buad = B1200;  break;
        case 2400:  test_buad = B2400;  break;
        case 4800:  test_buad = B4800;  break;
        case 9600:  test_buad = B9600;  break;
        case 19200: test_buad = B19200; break;
        case 38400: test_buad = B38400; break;
        case 57600: test_buad = B57600; break;
        case 115200:test_buad = B115200;break;
        default:
            printf("Does not support the baud rate\n\r");
            exit(1);
    }
	openSerail(test_buad);
	RX_TIME_OUT_MS = 1*256*10*4*1000/test_buad;//计算超时时间

	for(port=0; port<MAX_PORT; port++){
		ch_data[port].states = (port == 0) ? st_server : st_client;
	}

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

    int ret = pthread_create(&ch_data[0].id, NULL, (void *)thread1, NULL);
	if(ret != 0){
		printf("Create pthread error!\n");
		exit(1);
	}
    ret = pthread_create(&ch_data[1].id, NULL, (void *)thread2, NULL);
	if(ret != 0){
		printf("Create pthread error!\n");
		exit(1);
	}
	ret = pthread_create(&ch_data[2].id, NULL, (void *)thread3, NULL);
	if(ret != 0){
		printf("Create pthread error!\n");
		exit(1);
	}		
    start_run = 1;//线程运行标志
    time(&start_time);
	sleep(1);

	while(1){
		printf("\033[H""\033[J");//清屏
		time(&now_time);

		sprintf(tmp_printf, "Test App Speed %dbps time out %dmS Test time %ldmin %ldSec\n\r", test_buad, RX_TIME_OUT_MS,
			  (now_time - start_time)/60, (now_time - start_time)%60);
		timeinfo = localtime(&start_time);
		sprintf(tmp_str, "App start time -- %s", asctime(timeinfo));
		strcat(tmp_printf, tmp_str);//合并字符串
		timeinfo = localtime(&now_time);
		sprintf(tmp_str, "Now time -- %s", asctime(timeinfo));
		strcat(tmp_printf, tmp_str);//合并字符串			  
		sprintf(tmp_str, "PORT  states    tx_lens     rx_lens     tx_pkgs     rx_pkgs     err_pkgs     err_lens    lost_pkgs    lost_lens  accuracy\n\r");
		strcat(tmp_printf, tmp_str);//合并字符串

		for(port=0; port<MAX_PORT; port++){
			if(ch_data[port].states == st_server){
				serverSendlens = ch_data[port].tx_lens - txbefore[port];
				txbefore[port] = ch_data[port].tx_lens;
			}
		}

		for(port=0; port<MAX_PORT; port++){
			if(ch_data[port].ufd == 0){
				printf("app exit\n\r");
				exit(1);
			}
			if(ch_data[port].states == st_client){//计算串口合格率
				ch_data[port].accuracy = (float)(ch_data[port].rx_lens - rxbefore[port] - ch_data[port].err_lens - ch_data[port].lost_lens)
									  /(float)serverSendlens*100.0;
				rxbefore[port] = ch_data[port].rx_lens;
			}
			sprintf(tmp_str, "%2d    %s ", port,(ch_data[port].states == st_server) ? 
				   "\033[1;32;40mserver\033[0m" : "\033[1;36;40mclient\033[0m");
			strcat(tmp_printf, tmp_str);//合并字符串
			if(ch_data[port].states == st_client){
				if(ch_data[port].accuracy < 100)
					sprintf(tmp_str, "%10d  %10d  %10d  %10d   %10d   %10d   %10d   %10d   \033[1;31;40m%.2f%%\033[0m\n\r",
						ch_data[port].tx_lens, ch_data[port].rx_lens, 
						ch_data[port].tx_pkgs, ch_data[port].rx_pkgs, ch_data[port].err_pkgs, ch_data[port].err_lens, ch_data[port].lost_pkgs,
						ch_data[port].lost_lens, ch_data[port].accuracy);
				else
					sprintf(tmp_str, "%10d  %10d  %10d  %10d   %10d   %10d   %10d   %10d   \033[1;32;40m%.2f%%\033[0m\n\r",
						ch_data[port].tx_lens, ch_data[port].rx_lens, 
						ch_data[port].tx_pkgs, ch_data[port].rx_pkgs, ch_data[port].err_pkgs, ch_data[port].err_lens, ch_data[port].lost_pkgs,
						ch_data[port].lost_lens, ch_data[port].accuracy);					
			}
			else
				sprintf(tmp_str, "%10d  %10d  %10d  %10d\n\r",ch_data[port].tx_lens, ch_data[port].rx_lens, 
					ch_data[port].tx_pkgs, ch_data[port].rx_pkgs);
			strcat(tmp_printf, tmp_str);//合并字符串
		}
		strcat(tmp_printf, "\0");
		printf("%s", tmp_printf);
		client2server();//交换主从
		sleep(1);
	}
}
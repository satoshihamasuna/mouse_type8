/*
 * myshell.cpp
 *
 *  Created on: 2024/03/09
 *      Author: sato1
 */

#include "core/ntshell.h"
#include "core/ntlibc.h"
#include "util/ntopt.h"
#include "../Inc/myshell.h"
#include "communicate.h"
#include "../../Module/Inc/log_data.h"
#include "typedef.h"
#include "../../Task/Inc/sensing_task.h"
#include "../../Task/Inc/ctrl_task.h"
#include "../../Params/run_param.h"

#include <stdio.h>

typedef int (*USRCMDFUNC)(int argc, char **argv);

static int user_callback(const char *text, void *extobj);

static int usrcmd_execute(const char *text);
static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);

//コマンド一覧
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);
static int usrcmd_disp(int argc, char **argv);
static int usrcmd_end(int argc, char **argv);
static int usrcmd_debug(int argc, char **argv);
static int usrcmd_log(int argc, char **argv);
typedef struct {
	const char *cmd;
	const char *desc;
    USRCMDFUNC func;
} cmd_table_t;

static const cmd_table_t cmdlist[] = {
    { "help", "This is a description text string for help command.", usrcmd_help },
    { "info", "This is a description text string for info command.", usrcmd_info },
	{ "disp", "This is a description text string for disp command.", usrcmd_disp },
	{ "end",  "This is a description text string for end command.", usrcmd_end },
	{ "debug","This is a description text string for debug command.", usrcmd_debug },
	{ "log"  ,"This is a description text string for debug command.", usrcmd_log }
};

static ntshell_t nts;

t_bool	shell_end_flag;

/* ---------------------------------------------------------------
	help と info
--------------------------------------------------------------- */
static int usrcmd_help(int argc, char **argv)
{
    const cmd_table_t *p = &cmdlist[0];
    for (uint16_t i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        printf("  %s", p->cmd);
        printf("\t:");
        printf("  %s", p->desc);
        printf("\r\n");
        p++;
    }
    return 0;
}

static int usrcmd_info(int argc, char **argv)
{
    if (argc != 2) {
    	printf("info sys\r\n");
    	printf("info ver\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "sys") == 0) {
    	printf("prototype7\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "ver") == 0) {
    	printf("Version 0.0.0\r\n");
        return 0;
    }
    printf("Unknown sub command found\r\n");
    return -1;
}


static int usrcmd_disp(int argc, char **argv)
{
    if (argc != 2) {
    	printf("disp maze\r\n");
    	printf("disp log\r\n");
    	return 0;
    }
    if (ntlibc_strcmp(argv[1], "maze") == 0) {
    	printf("prototype7\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "log") == 0) {
    	LogData::getInstance().indicate_data();
        return 0;
    }
    printf("Unknown sub command found\r\n");
    return -1;
}

static int usrcmd_end(int argc, char **argv)
{
    if (argc != 2) {
    	printf("end exe\r\n");
    	return 0;
    }
    if (ntlibc_strcmp(argv[1], "exe") == 0) {
    	shell_end_flag = True;
    	printf("shell_end!\r\n");
        return 0;
    }

    printf("Unknown sub command found\r\n");
    return -1;
}

static int usrcmd_log(int argc, char **argv)
{
    if (argc != 2) {
    	printf("end exe\r\n");
    	return 0;
    }
    if (ntlibc_strcmp(argv[1], "mode0") == 0) {
    	LogData::getInstance().set_logmode(0);
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "mode1") == 0) {
        	LogData::getInstance().set_logmode(1);
            return 0;
    }

    printf("Unknown sub command found\r\n");
    return -1;
}


int shell_debug_straight(int argc, char **argv)
{
	static t_pid_gain debug_sp_gain = {4.0,0.05,0.00};
	static t_pid_gain debug_om_gain = {0.1f,0.01f,0.00f};

	Motion *motion = &(CtrlTask_type7::getInstance());
	IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

	if (ntlibc_strcmp(argv[2], "om_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_om_gain.Kp,debug_om_gain.Ki,debug_om_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_om_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_om_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_om_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_om_gain.Kp,debug_om_gain.Ki,debug_om_gain.Kd);
		}
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "sp_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_sp_gain.Kp,debug_sp_gain.Ki,debug_sp_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_sp_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_sp_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_sp_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_sp_gain.Kp,debug_sp_gain.Ki,debug_sp_gain.Kd);
		}
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "exe") == 0) {
		printf("execute!\r\n");
		for(int i = 0;irsens->IrSensor_Avg() < 2000;i++)
		{
			(i%2 == 0) ? Indicate_LED(0xff):Indicate_LED(0x00|0x00);
			HAL_Delay(200);
		}

		for(int i = 0;i < 21;i++)
		{
			(i%2 == 0) ? Indicate_LED(0xff):Indicate_LED(0x00|0x00);
			HAL_Delay(50);
		}
		motion->Motion_start();
		LogData::getInstance().data_count = 0;
		LogData::getInstance().log_enable = True;
		motion->Init_Motion_straight(90.0*4.0,6.5,0.7,0.0,&debug_sp_gain,&debug_om_gain);
		motion->execute_Motion();

		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		HAL_Delay(500);
		return 0;
	}
	printf("Unknown sub command found\r\n");
	return -1;
}

int shell_debug_diagonal(int argc, char **argv)
{
	static t_pid_gain debug_sp_gain = {12.0,0.04,0.0};
	static t_pid_gain debug_om_gain = {0.60f, 0.01f, 0.00f};

	Motion *motion = &(CtrlTask_type7::getInstance());
	IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

	if (ntlibc_strcmp(argv[2], "om_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_om_gain.Kp,debug_om_gain.Ki,debug_om_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_om_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_om_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_om_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_om_gain.Kp,debug_om_gain.Ki,debug_om_gain.Kd);
		}
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "sp_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_sp_gain.Kp,debug_sp_gain.Ki,debug_sp_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_sp_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_sp_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_sp_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_sp_gain.Kp,debug_sp_gain.Ki,debug_sp_gain.Kd);
		}
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "exe") == 0) {
		printf("execute!\r\n");
		for(int i = 0;irsens->IrSensor_Avg() < 2000;i++)
		{
			(i%2 == 0) ? Indicate_LED(0xff):Indicate_LED(0x00|0x00);
			HAL_Delay(200);
		}

		for(int i = 0;i < 21;i++)
		{
			(i%2 == 0) ? Indicate_LED(0xff):Indicate_LED(0x00|0x00);
			HAL_Delay(50);
		}
		motion->Motion_start();
		LogData::getInstance().data_count = 0;
		LogData::getInstance().log_enable = True;
		motion->Init_Motion_diagonal(63.63*6.0,6.5,0.7,0.0,&debug_sp_gain,&debug_om_gain);
		motion->execute_Motion();

		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		HAL_Delay(500);
		return 0;
	}
	printf("Unknown sub command found\r\n");
	return -1;
}

int shell_debug_turn(int argc, char **argv)
{
	static t_pid_gain debug_sp_gain = {12.0,0.04,0.0};
	static t_pid_gain debug_om_gain = {0.60f, 0.01f, 0.00f};

	static t_pid_gain debug_turn_sp_gain = {12.0,0.04,0.0};
	static t_pid_gain debug_turn_om_gain = {0.60f, 0.01f, 0.00f};


	Motion *motion = &(CtrlTask_type7::getInstance());
	IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

	if (ntlibc_strcmp(argv[2], "om_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_om_gain.Kp,debug_om_gain.Ki,debug_om_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_om_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_om_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_om_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_om_gain.Kp,debug_om_gain.Ki,debug_om_gain.Kd);
		}
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "sp_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_sp_gain.Kp,debug_sp_gain.Ki,debug_sp_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_sp_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_sp_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_sp_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_sp_gain.Kp,debug_sp_gain.Ki,debug_sp_gain.Kd);
		}
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "turn_om_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_turn_om_gain.Kp,debug_turn_om_gain.Ki,debug_turn_om_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_turn_om_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_turn_om_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_turn_om_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_turn_om_gain.Kp,debug_turn_om_gain.Ki,debug_turn_om_gain.Kd);
		}
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "turn_sp_params") == 0)
	{
		printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_turn_sp_gain.Kp,debug_turn_sp_gain.Ki,debug_turn_sp_gain.Kd);
		printf("y/n?->");
		unsigned char c = 'n';
		scanf("%c",&c);
		printf("%c\n",c);
		if(c == 'y')
		{
			float tmp;
			printf("Kp->");scanf("%f",&tmp);debug_turn_sp_gain.Kp = tmp;printf("%.3f\n",tmp);
			printf("Ki->");scanf("%f",&tmp);debug_turn_sp_gain.Ki = tmp;printf("%.3f\n",tmp);
			printf("Kd->");scanf("%f",&tmp);debug_turn_sp_gain.Kd = tmp;printf("%.3f\n",tmp);
			printf("Kp->%.3lf,Ki->%.3lf,Kd->%.3lf\n",debug_turn_sp_gain.Kp,debug_turn_sp_gain.Ki,debug_turn_sp_gain.Kd);
		}
		return 0;
	}

	if (ntlibc_strcmp(argv[2], "exe") == 0) {
		printf("execute!\r\n");
		for(int i = 0;irsens->IrSensor_Avg() < 2000;i++)
		{
			(i%2 == 0) ? Indicate_LED(0xff):Indicate_LED(0x00|0x00);
			HAL_Delay(200);
		}

		for(int i = 0;i < 21;i++)
		{
			(i%2 == 0) ? Indicate_LED(0xff):Indicate_LED(0x00|0x00);
			HAL_Delay(50);
		}
		motion->Motion_start();
		LogData::getInstance().data_count = 0;
		LogData::getInstance().log_enable = True;
		//motion->Init_Motion_straight(90.0*4.0,6.5,0.7,0.0,&debug_sp_gain,&debug_om_gain);
		//motion->execute_Motion();

		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		HAL_Delay(500);
		return 0;
	}
	printf("Unknown sub command found\r\n");
	return -1;
}

static int usrcmd_debug(int argc, char **argv)
{
    if (argc != 3) {
    	printf("debug straight exe\r\n");
    	printf("debug straight om_params\r\n");
    	printf("debug straight sp_params\r\n");
    	printf("debug diagonal exe\r\n");
    	printf("debug diagonal om_params\r\n");
    	printf("debug diagonal sp_params\r\n");
    	return 0;
    }
	if (ntlibc_strcmp(argv[1], "straight") == 0)
	{
		return shell_debug_straight(argc,argv);
	}
	if (ntlibc_strcmp(argv[1], "diagonal") == 0)
	{
		return shell_debug_diagonal(argc,argv);
	}
    printf("Unknown sub command found\r\n");
    return -1;
}

/* ---------------------------------------------------------------
	送受信用ローカル関数
--------------------------------------------------------------- */
static int func_read(char *buf, int cnt, void *extobj)
{
	  for (int16_t i = 0; i < cnt;i ++)
	  {
		  buf[i] = (char) Communicate_RxPopData();
	  }
	  return cnt;
}

static int func_write(const char *buf, int cnt, void *extobj)
{

	  for (int16_t i = 0; i < cnt;i ++)
	  {
		  Communicate_TxPushData((int8_t)(buf[i]));
	  }
	return cnt;
}

/* ---------------------------------------------------------------
	コールバック関数
--------------------------------------------------------------- */
static int user_callback(const char *text, void *extobj)
{

	usrcmd_execute(text);
	return 0;
}

static int usrcmd_execute(const char *text)
{
	return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
{
    if (argc == 0) {
        return 0;
    }
    const cmd_table_t *p = &cmdlist[0];
    for (uint16_t i = 0; i < (sizeof(cmdlist) / sizeof(cmdlist[0])); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    printf("Unknown command found.\r\n");
    return 0;
}

/* ---------------------------------------------------------------
	初期設定関数
--------------------------------------------------------------- */

void Myshell_Initialize( void )
{
	void *extobj = 0;

	ntshell_init(&nts, func_read, func_write, user_callback, extobj);
}

void Myshell_Execute( void )
{

	shell_end_flag = False;
	while(shell_end_flag != True)
	{
		if( (&nts)->initcode != 0x4367 ) {
			return;
		} else;

		unsigned char ch;
		func_read((char *)&ch, sizeof(ch), (&nts)->extobj);
		vtrecv_execute(&((&nts)->vtrecv), &ch, sizeof(ch));

	}
}


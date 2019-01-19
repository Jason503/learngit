#ifndef __TBT_H
#define __TBT_H
#include "s32k_conf.h"
#include "stdio.h"
#define success 0
#define len_error  1
#define head_error 2
#define type_error 3
#define cmd_error  4
#define data_error 5

//命令
#define head 							0xB0//头字节
#define type_tbt 					0x56//TBT类型字节
#define cmd_navi_type     0X00//导航显示类型
#define cmd_ve_dir        0X01//自车方向
#define cmd_cur_name      0X02//当前道路名
#define cmd_nc_nam			  0X03//下一道路名
#define cmd_nt_icon			  0X04//下个路口转向图标
#define cmd_nnt_icon			0X05//下个路口距离-下下个路口剩余距离转向图标
#define cmd_Re_dis_bar		0X06//剩余距离进度条（到下一个交叉点）
#define cmd_Re_dis				0X07//当前导航段剩余距离
#define cmd_ve_icon				0X08//自车图标
#define cmd_tl_icon				0X09//左转
#define cmd_tr_icon				0X0A//右转
#define cmd_lf_icon				0X0B//左前方转
#define cmd_rf_icon				0X0C//右前方转
#define cmd_lb_icon				0X0D//左后方转
#define cmd_rb_icon				0X0E//右后方转
#define cmd_lt_icon				0X0F//左掉头
#define cmd_gs_icon				0X10//直行
#define cmd_wp_icon				0X11//到达途经点
#define cmd_enr_icon			0X12//进入环岛
#define cmd_exr_icon			0X13//驶出环岛
#define cmd_sa_icon				0X14//到达服务区
#define cmd_ts_icon				0X15//到达收费站
#define cmd_des_icon			0X16//到达目的地
#define cmd_tu_icon				0X17//进入隧道
#define cmd_ant_icon			0X18//顺行

#define can_num			0U
#define can_id  		0x0a
#define can_mailbox 0x00

uint8_t TBT_Handler(const uint8_t*data,const uint8_t len);//TBT的数据处理



//数据存储的buffer num 
#define Storing_buf_num0 0
#define Storing_buf_num1 1
#define Storing_buf_num2 2
#define Storing_buf_num3 3
#define Storing_buf_num4 4
#define Storing_buf_num5 5
#define Storing_buf_num6 6
#define Storing_buf_num7 7
#define Storing_buf_num8 8
#define Storing_buf_num9 9

//箭头颜色
#define Guide_arrow_color 3

//导航箭头方向
#define Gui_arr_direction_ve 	0    //无对应信息 自车图标
#define Gui_arr_direction_tl 	12		//左转
#define Gui_arr_direction_tr 	4			//右转
#define Gui_arr_direction_lf 	15		//左前方转
#define Gui_arr_direction_rf 	1 		//右前方转
#define Gui_arr_direction_lb 	10		//左后方转
#define Gui_arr_direction_rb 	6			//右后方转
#define Gui_arr_direction_lt 	0  	 //左掉头 （可取 * ，故取值为0）
#define Gui_arr_direction_gs 	0			//直行
#define Gui_arr_direction_wp 	0  	 //到达途经点（ 可取 * ，故取值为0）
#define Gui_arr_direction_enr 0			//进入环岛
#define Gui_arr_direction_exr 0			//驶出环岛
#define Gui_arr_direction_sa  0 	 //到达服务区 （无对应信息）
#define Gui_arr_direction_ts  0 	 //到达收费站 （可取 * ，故取值为0）
#define Gui_arr_direction_des 0 	 //到达目的地 （可取 * ，故取值为0）
#define Gui_arr_direction_tu  0		 //进入隧道 （无对应信息）
#define Gui_arr_direction_ant 0		 //顺行    （无对应信息）


//线路
#define Connec_road__ve	 			//自车图标
#define Connec_road__tl	  3 	//左转
#define Connec_road__tr	  3		//右转
#define Connec_road__lf	  0		//左前方转
#define Connec_road__rf	  6		//右前方转
#define Connec_road__lb	  5		//左后方转
#define Connec_road__rb	  1		//右后方转
#define Connec_road__lt	  7		//左掉头
#define Connec_road__gs	  7		//直行
#define Connec_road__wp				//到达途经点
#define Connec_road__enr	7		//进入环岛
#define Connec_road__exr	7		//驶出环岛
#define Connec_road__sa				//到达服务区
#define Connec_road__ts	 			//到达收费站
#define Connec_road__des	 		//到达目的地 
#define Connec_road__tu				//进入隧道
#define Connec_road__ant			//顺行  



#endif 


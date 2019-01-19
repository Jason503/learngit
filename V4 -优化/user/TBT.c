#include "TBT.h"

uint8_t tbt_cmd =0xff;

uint8_t cur_Bam_data[4];//起始帧
uint8_t gui_Bam_data[4];//起始帧

uint8_t cur_data[7][8];
uint8_t gui_data[9][8];


/*******************函数申明*****************************************************/
static  uint8_t TBT_check(const uint8_t * data);

static uint8_t send_tbt_cur_data(uint8_t dt_num);			//发送当前道路名的CAN数据
static uint8_t send_tbt_gui_data(uint8_t dt_num);			//发送导航点的CAN数据
static uint8_t send_de_data(void);										//Demand 的数据

static uint8_t  send_Navi_type(const uint8_t*data); 	//导航显示类型
static uint8_t send_ve_dir(const uint8_t*data);			 	//自车方向
static uint8_t send_cur_name(const uint8_t*data);			//当前道路名
static uint8_t send_nc_name(const uint8_t*data);			//下一道路名
static uint8_t send_nt_icon(const uint8_t*data);			//下个路口转向图标
static uint8_t send_nnt_icon(const uint8_t*data);			//下个路口距离-下下个路口剩余距离转向图标
static uint8_t send_Re_dis_bar(const uint8_t*data);		//剩余距离进度条（到下一个交叉点）
static uint8_t send_Re_dis(const uint8_t*data);				//当前导航段剩余距离
static uint8_t send_ve_icon(const uint8_t*data); 			//自车图标
static uint8_t send_tl_icon(const uint8_t*data); 			//左转
static uint8_t send_tr_icon(const uint8_t*data);			//右转
static uint8_t send_lf_icon(const uint8_t*data);			//左前方转
static uint8_t send_rf_icon(const uint8_t*data);			//右前方转
static uint8_t send_lb_icon(const uint8_t*data); 			//左后方转
static uint8_t send_rb_icon(const uint8_t*data);			//右后方转
static uint8_t send_lt_icon(const uint8_t*data); 			//左掉头
static uint8_t send_gs_icon(const uint8_t*data); 			//直行
static uint8_t send_wp_icon(const uint8_t*data); 			//到达途经点
static uint8_t send_enr_icon(const uint8_t*data); 		//进入环岛
static uint8_t send_exr_icon(const uint8_t*data); 		//驶出环岛
static uint8_t send_sa_icon(const uint8_t*data); 			//到达服务区
static uint8_t send_ts_icon(const uint8_t*data); 			//到达收费站
static uint8_t send_des_icon(const uint8_t*data); 		//到达目的地
static uint8_t send_tu_icon(const uint8_t*data); 			//进入隧道
static uint8_t send_ant_icon(const uint8_t*data); 		//顺行


/////////////////////////////////////////////////////////////////////


uint8_t TBT_Handler(const uint8_t*data,const uint8_t len)
{
	uint8_t res = 0 ;
	cur_Bam_data[3] = 3;
	gui_Bam_data[3] = 3;
	
	/*
	* 接收的BUFFER的字节数要大于2 
	* 接收数据格式：head+type+(tbt)cmd+(tbt)value 
	* value可以没有
	*/
	
	if(len<3)
	{
		return len_error;
	}

//验证数据格式是否正确	

	
	/*校验头字节 0XB0*/
		if(data[0]!=head)
		{
			return head_error;
		}

		
		/*校验TBT类型字节 0X56*/
		if(data[1]!=type_tbt)
		{
			return type_error;
		}

	tbt_cmd = data[2];//get the tbt cmd	
	
//第一步：开始数据整合

	switch(tbt_cmd)
	{
		case cmd_navi_type: res=send_Navi_type(data); 		break ;//导航显示类型
		case cmd_ve_dir:		res=send_ve_dir(data); 				break ;//自车方向
		case cmd_cur_name:  res=send_cur_name(data);			break ;//当前道路名
		case cmd_nc_nam:    res=send_nc_name(data); 			break ;//下一道路名
		case cmd_nt_icon:   res=send_nt_icon(data);				break ;//下个路口转向图标
		case cmd_nnt_icon:  res=send_nnt_icon(data);			break ;//下个路口距离-下下个路口剩余距离转向图标
		case cmd_Re_dis_bar:res=send_Re_dis_bar(data); 		break ;//剩余距离进度条（到下一个交叉点）
		case cmd_Re_dis: 		res=send_Re_dis(data);				break ;//当前导航段剩余距离
		case cmd_ve_icon:		res=send_ve_icon(data); 			break ;//自车图标
		case cmd_tl_icon:		res=send_tl_icon(data); 			break ;//左转
		case cmd_tr_icon:		res=send_tr_icon(data); 			break ;//右转
		case cmd_lf_icon:		res=send_lf_icon(data); 			break ;//左前方转
		case cmd_rf_icon:		res=send_rf_icon(data); 			break ;//右前方转
		case cmd_lb_icon:		res=send_lb_icon(data); 			break ;//左后方转
		case cmd_rb_icon:		res=send_rb_icon(data); 			break ;//右后方转
		case cmd_lt_icon:		res=send_lt_icon(data); 			break ;//左掉头
		case cmd_gs_icon:		res=send_gs_icon(data); 			break ;//直行
		case cmd_wp_icon:		res=send_wp_icon(data); 			break ;//到达途经点
		case cmd_enr_icon:	res=send_enr_icon(data); 			break ;//进入环岛
		case cmd_exr_icon:	res=send_exr_icon(data); 			break ;//驶出环岛
		case cmd_sa_icon:		res=send_sa_icon(data); 			break ;//到达服务区
		case cmd_ts_icon:		res=send_ts_icon(data); 			break ;//到达收费站
		case cmd_des_icon:	res=send_des_icon(data); 			break ;//到达目的地
		case cmd_tu_icon:		res=send_tu_icon(data); 			break ;//进入隧道
		case cmd_ant_icon:	res=send_ant_icon(data); 			break ;//顺行	
		default : return data_error; 									//	break ;
		
	}
	
	if(res)
	{
		return data_error;
	}
	

	//第二步：整合数据完成，准备发送多包数据
	
	//准备好BAM的数据
	
	/*data[4]代表着是字节数。得到需要多少个DTn.当前道路名从DT2开始所以加1 */
	if((tbt_cmd==cmd_cur_name)&&(len>5))
	{
		cur_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+1;
	}
	
	/*data[4]代表着是字节数。得到需要多少个DTn.下一个道路名从DT4开始所以加3*/
	if((tbt_cmd==cmd_nc_nam)&&(len>5))
	{
		gui_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+3;
	}

		
	//发送CAN报文
	if(tbt_cmd==cmd_cur_name)
	{
	
		send_tbt_cur_data(cur_Bam_data[3]);
	}
	else
	{
	
		send_tbt_gui_data(gui_Bam_data[3]);
	}
   
	//第三步：多包数据发送完成，准备发送Demand的报文，需要延时100ms再发。
	
	
	res=send_de_data();
	if(res)
	{
		printf("data error\n");
	}
	
	
	return success;//正常
	
}

static uint8_t send_tbt_cur_data(uint8_t dt_num)
{
		uint8_t i;
		cur_Bam_data[0] = 0x20;
	  cur_data[0][0] = 0x01;
	
	//需要知道发送多少个DTn
	
	
	for(i=0;i<dt_num;i++)
	{
		cur_data[i][7]=TBT_check(cur_data[i]);	
	}
			
	SendCANData(can_num,can_mailbox,can_id, cur_Bam_data, 4);//发送BAM数据
	
	for(i=0;i<dt_num;i++)
	{
		SendCANData(can_num,can_mailbox,can_id, cur_data[i], 8);//发送DTn数据
		printf("cur_data[%d][0]=%x\n",i,cur_data[i][0]);
		printf("cur_data[%d][1]=%x\n",i,cur_data[i][1]);
		printf("cur_data[%d][2]=%x\n",i,cur_data[i][2]);
	  printf("cur_data[%d][3]=%x\n",i,cur_data[i][3]); 
		printf("cur_data[%d][4]=%x\n",i,cur_data[i][4]);
		printf("cur_data[%d][5]=%x\n",i,cur_data[i][5]);
		printf("cur_data[%d][6]=%x\n",i,cur_data[i][6]);
		printf("cur_data[%d][7]=%x\n",i,cur_data[i][7]);
	}
	
	return success;
}
static uint8_t send_tbt_gui_data( uint8_t dt_num)
{
	
	uint8_t i;
	/*********固定的值**********/
	gui_Bam_data[0] = 0x20;//固定为0x20
	

	gui_data[0][0] = 0x01;//字节的头部数据被固定 
	gui_data[1][0] = 0x02;
	gui_data[2][0] = 0x03;
	
	gui_data[0][1] = Storing_buf_num0; //使用num 0 store buffer
	gui_data[0][1]&= 0x0F;

	
	gui_data[0][2]&= ~(3<<6);//高2位清零
	gui_data[0][2]|= Guide_arrow_color<<6;
	
	
	//需要知道发送多少个DTn

		for(i=0;i<dt_num;i++)
	{
		gui_data[i][7]=TBT_check(gui_data[i]);	
	}
			
	SendCANData(can_num,can_mailbox,can_id, gui_Bam_data, 4);//发送BAM数据
	
	for(i=0;i<dt_num;i++)
	{
		SendCANData(can_num,can_mailbox,can_id, gui_data[i], 8);//发送DTn数据
		printf("gui_data[%d][0]=%x\n",i,gui_data[i][0]);
		printf("gui_data[%d][1]=%x\n",i,gui_data[i][1]);
		printf("gui_data[%d][2]=%x\n",i,gui_data[i][2]);
	  printf("gui_data[%d][3]=%x\n",i,gui_data[i][3]); 
		printf("gui_data[%d][4]=%x\n",i,gui_data[i][4]);
		printf("gui_data[%d][5]=%x\n",i,gui_data[i][5]);
		printf("gui_data[%d][6]=%x\n",i,gui_data[i][6]);
		printf("gui_data[%d][7]=%x\n",i,gui_data[i][7]);
	}
	
	return success;
}

//发送demand的数据报文
static uint8_t send_de_data()
{
	
	return success;	
}


//导航显示类型
static uint8_t  send_Navi_type(const uint8_t*data) 
{
	
	return data_error;//未得到相应信息，属于demand信息
}
//自车方向
static uint8_t send_ve_dir(const uint8_t*data) 
{
	
	return data_error;//未得到相应信息，属于demand信息
}
//当前道路名
static uint8_t send_cur_name(const uint8_t*data)
{
	
	uint8_t i;
	cur_Bam_data[0] = 0x20;
	cur_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+1;//data[4]代表着是字节数。得到需要多少个DTn.当前道路名从DT2开始所以加1


	//字节编码方式 
	cur_data[0][1]&= ~(7<<5);//清高三位
	cur_data[0][1]|= data[3]<<5;
	
	//字节数
	cur_data[0][1]&= ~(31<<0);//清低5位
	cur_data[0][1]|= data[4]<<0;
  
  
	if(cur_Bam_data[3]-1>0)//存在字符，需要传送的数据包在 DT1 以上
	{
				cur_data[1][0] = 0x02;
				
				if(data[4]<7)//字符小于或等于6字节
				{
						for(i=0;i<data[4];i++)
						{
							cur_data[1][i+1]=data[5+i];
						}
				}
				else
				{
						for(i=0;i<6;i++)//字符大于于6字节
						{
							cur_data[1][i+1]=data[5+i];
						}
				}		
		
		if((cur_Bam_data[3]-2>0)||(cur_Bam_data[3]-2==0))//存在字符，需要传送的数据包在 DT2 以上
		{
					cur_data[2][0] = 0x03;
					
					if((data[4]-6)<7)//字符小于或等于12字节
					{
							for(i=0;i<(data[4]-6);i++)
							{
								cur_data[2][i+1]=data[5+i+6];
								
							}
					}
					else
					{
							for(i=0;i<6;i++)//字符大于于12字节
							{
								cur_data[2][i+1]=data[5+i+6];
							}
					}			

		
				if(cur_Bam_data[3]-3>0)//存在字符，需要传送的数据包在 DT3 以上
				{
							cur_data[3][0] = 0x04;
							
							if((data[4]-12)<7)//字符小于或等于18字节
							{
									for(i=0;i<(data[4]-12);i++)
									{
										cur_data[3][i+1]=data[5+i+12];
									}
							}
							else
							{
									for(i=0;i<6;i++)//字符大于于18字节
									{
										cur_data[3][i+1]=data[5+i+12];
									}
							}

					if(cur_Bam_data[3]-4>0)//存在字符，需要传送的数据包在 DT4 以上
					{
								cur_data[4][0] = 0x05;
								if((data[4]-18)<7)//字符小于或等于24字节
								{
										for(i=0;i<(data[4]-18);i++)
										{
											cur_data[4][i+1]=data[5+i+18];
										}
								}
								else//字符大于于24字节
								{
										for(i=0;i<6;i++)
										{
											cur_data[4][i+1]=data[5+i+18];
										}
								}
							
					
						if(cur_Bam_data[3]-5>0)//存在字符，需要传送的数据包在 DT5 以上
						{
									cur_data[5][0] = 0x06;				
									if((data[4]-24)<7)//字符小于或等于30字节
									{
											for(i=0;i<(data[4]-24);i++)
											{
												cur_data[5][i+1]=data[5+i+24];
											}
									}
									else//字符大于于30字节
									{
											for(i=0;i<6;i++)
											{
												cur_data[5][i+1]=data[5+i+24];
											}
									}				
									
							 if(cur_Bam_data[3]-6>0)//存在字符，需要传送的数据包在 DT6以上
							 {
										cur_data[6][0] = 0x07;					 
										if((data[4]-30)<7)//字符小于或等于36字节
										{
												for(i=0;i<(data[4]-30);i++)
												{
													cur_data[6][i+1]=data[5+i+30];
												}
										}
										else//字符大于于36字节 只保留前36个字节
										{
												for(i=0;i<6;i++)
												{
													cur_data[6][i+1]=data[5+i+30];
												}												
										}						 
								 
									}	
							}
						}
					}	
	
			}
	}
	return success;
}
//下一道路名
static uint8_t send_nc_name(const uint8_t*data)
{
	uint8_t i;
	gui_Bam_data[0] = 0x20;//固定为0x20
	gui_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+3;//data[4]代表着是字节数。得到需要多少个DTn.下一个道路名从DT4开始所以加3
	
	//字节编码
	gui_data[0][6]&= ~(7<<5);
	gui_data[0][6] = data[3]<<5;//发送数据格式 ： 0xB0+0x56+0x03+字符编码+字符数+字符数据
	
	//字节数
	gui_data[0][6]&= ~(63<<0);
	gui_data[0][6]|= data[4]<<0;
	
	if(gui_Bam_data[3]-3>0)//存在字符，需要传送的数据包在 DT3 以上
	{
				gui_data[3][0] = 0x04;
				
				if(data[4]<7)//字符小于或等于6字节
				{
						for(i=0;i<data[4];i++)
						{
							gui_data[3][i+1]=data[5+i];
						}
				}
				else
				{
						for(i=0;i<6;i++)//字符大于于6字节
						{
							gui_data[3][i+1]=data[5+i];
						}
				}

		if(gui_Bam_data[3]-4>0)//存在字符，需要传送的数据包在 DT4 以上
		{
					gui_data[4][0] = 0x05;
					if((data[4]-6)<7)//字符小于或等于12字节
					{
							for(i=0;i<(data[4]-6);i++)
							{
								gui_data[4][i+1]=data[5+i+6];
							}
					}
					else//字符大于于12字节
					{
							for(i=0;i<6;i++)
							{
								gui_data[4][i+1]=data[5+i+6];
							}
					}
				
		
			if(gui_Bam_data[3]-5>0)//存在字符，需要传送的数据包在 DT5 以上
			{
				  	gui_data[5][0] = 0x06;				
						if((data[4]-12)<7)//字符小于或等于18字节
						{
								for(i=0;i<(data[4]-12);i++)
								{
									gui_data[5][i+1]=data[5+i+12];
								}
						}
						else//字符大于于18字节
						{
								for(i=0;i<6;i++)
								{
									gui_data[5][i+1]=data[5+i+12];
								}
						}				
						
				 if(gui_Bam_data[3]-6>0)//存在字符，需要传送的数据包在 DT6以上
				 {
							gui_data[6][0] = 0x07;					 
							if((data[4]-18)<7)//字符小于或等于24字节
							{
									for(i=0;i<(data[4]-18);i++)
									{
										gui_data[6][i+1]=data[5+i+18];
									}
							}
							else//字符大于于24字节
							{
									for(i=0;i<6;i++)
									{
										gui_data[6][i+1]=data[5+i+18];
									}
							}						 
					 
						if(gui_Bam_data[3]-7>0)//存在字符，需要传送的数据包在 DT7 以上
						{
							      gui_data[7][0] = 0x08;								
										if((data[4]-24)<7)//字符小于或等于30字节
										{
												for(i=0;i<(data[4]-24);i++)
												{
													gui_data[7][i+1]=data[5+i+24];
												}
										}
										else//字符大于于30字节
										{
												for(i=0;i<6;i++)
												{
													gui_data[7][i+1]=data[5+i+24];
												}
										}						 
														
							
									if(gui_Bam_data[3]-8>0)//存在字符，需要传送的数据包在 DT8 以上
										{
													gui_data[8][0] = 0x09;	
													if((data[4]-30)<7)//字符小于或等于36字节
													{
															for(i=0;i<(data[4]-30);i++)
															{
																gui_data[8][i+1]=data[5+i+30];
															}
													}
													else//字符大于于36字节
													{
															for(i=0;i<6;i++)
															{
																gui_data[8][i+1]=data[5+i+30];
															}	
													}
										}									
				
								}
						}	
				}
			}
		}	
    return success;

}
//下个路口转向图标
static uint8_t send_nt_icon(const uint8_t*data)
{
	printf("send_nt_icon_error\n");
	return data_error;//未得到相应信息，属于demand信息
}
//下个路口距离-下下个路口剩余距离转向图标
static uint8_t send_nnt_icon(const uint8_t*data)
{
		printf("send_nnt_icon_error\n");	
	return data_error;//未得到相应信息，属于demand信息
}
//剩余距离进度条（到下一个交叉点）
static uint8_t send_Re_dis_bar(const uint8_t*data)
{
		printf("send_Re_dis_bar_error\n");	
	return data_error;//未得到相应信息，属于demand信息
}
//当前导航段剩余距离
static uint8_t send_Re_dis(const uint8_t*data)
{
		printf("send_Re_dis_error\n");			
	return data_error;//未得到相应信息，属于demand信息
}
//自车图标
static uint8_t send_ve_icon(const uint8_t*data) 
{
	printf("send_ve_icon_error\n");	
	return data_error;//无对应信息
	
}

//左转
static uint8_t send_tl_icon(const uint8_t*data) 
{
	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(0<<0);
	


	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_tl<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__tl);	

	return success;
}

//右转
static uint8_t send_tr_icon(const uint8_t*data)
{
	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(0<<0);
	
		
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_tr<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__tr);	
	
	return success;
}

//左前方转
static uint8_t send_lf_icon(const uint8_t*data)
{
	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_lf<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__lf);	
	
	return success;	
}

//右前方转
static uint8_t send_rf_icon(const uint8_t*data) 
{
	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_rf<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__rf);		
	
	return success;	
}

//左后方转
static uint8_t send_lb_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_lb<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__lb);		
	
	return success;		
}

//右后方转
static uint8_t send_rb_icon(const uint8_t*data)
{
	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_rb<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__rb);	
	
	return success;
}

//左掉头
static uint8_t send_lt_icon(const uint8_t*data) 
{
	
	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(6<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_lt<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__lt);		
	
	return success;
}

//直行
static uint8_t send_gs_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_gs<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	
	return success;
}

//到达途经点
static uint8_t send_wp_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(8<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_wp<<0);	
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	
	return success;	
}

//进入环岛
static uint8_t send_enr_icon(const uint8_t*data) 
{
	

	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(1<<0);

	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_enr<<0);	
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__enr);

	return success;	
}

//驶出环岛
static uint8_t send_exr_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(2<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_exr<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__enr);	
	
	return success;
}

//到达服务区
static uint8_t send_sa_icon(const uint8_t*data) 
{
	return data_error;//无信息对应
}

//到达收费站
static uint8_t send_ts_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(10<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_ts<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;	
	
	return success;
}

//到达目的地
static uint8_t send_des_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//低6位清零
	gui_data[0][2]|=	(9<<0);
	
	gui_data[0][3]&= ~(31<<0);//低5位清零
	gui_data[0][3]|=	(Gui_arr_direction_des<<0);	
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	
	return success;
}

//进入隧道
static uint8_t send_tu_icon(const uint8_t*data) 
{
	printf("send_tu_icon_error\n");
	return data_error;//无信息对应
}

//顺行
static uint8_t send_ant_icon(const uint8_t*data) 
{
	printf("send_ant_icon_error\n");	
  return data_error;//无信息对应
}


/*
* 函数名：TBT_check
* 功能描述:对数据校验，算出检验值
* 参数：uint8_t * data
* TBT的校验和采用半字节的相加，
* 把前7个字节和第8个字节的高4位相加，得到校验值。
*/
static  uint8_t TBT_check(const uint8_t * data)
{
	uint8_t i;
	uint8_t temp  = 0;
	uint8_t value = 0;
	for(i=0;i<7;i++)
	{
		temp =((data[i]>>4)+(data[i]&0X0F));
		temp&=0X0F;
		value+= temp;
		value&=0X0F;	
	}
	temp &=data[7]>>4;
	temp&=0x0F;
	value+= temp;
	value&=0X0F;
	
	return value;
}











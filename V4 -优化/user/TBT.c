#include "TBT.h"

uint8_t tbt_cmd =0xff;

uint8_t cur_Bam_data[4];//��ʼ֡
uint8_t gui_Bam_data[4];//��ʼ֡

uint8_t cur_data[7][8];
uint8_t gui_data[9][8];


/*******************��������*****************************************************/
static  uint8_t TBT_check(const uint8_t * data);

static uint8_t send_tbt_cur_data(uint8_t dt_num);			//���͵�ǰ��·����CAN����
static uint8_t send_tbt_gui_data(uint8_t dt_num);			//���͵������CAN����
static uint8_t send_de_data(void);										//Demand ������

static uint8_t  send_Navi_type(const uint8_t*data); 	//������ʾ����
static uint8_t send_ve_dir(const uint8_t*data);			 	//�Գ�����
static uint8_t send_cur_name(const uint8_t*data);			//��ǰ��·��
static uint8_t send_nc_name(const uint8_t*data);			//��һ��·��
static uint8_t send_nt_icon(const uint8_t*data);			//�¸�·��ת��ͼ��
static uint8_t send_nnt_icon(const uint8_t*data);			//�¸�·�ھ���-���¸�·��ʣ�����ת��ͼ��
static uint8_t send_Re_dis_bar(const uint8_t*data);		//ʣ����������������һ������㣩
static uint8_t send_Re_dis(const uint8_t*data);				//��ǰ������ʣ�����
static uint8_t send_ve_icon(const uint8_t*data); 			//�Գ�ͼ��
static uint8_t send_tl_icon(const uint8_t*data); 			//��ת
static uint8_t send_tr_icon(const uint8_t*data);			//��ת
static uint8_t send_lf_icon(const uint8_t*data);			//��ǰ��ת
static uint8_t send_rf_icon(const uint8_t*data);			//��ǰ��ת
static uint8_t send_lb_icon(const uint8_t*data); 			//���ת
static uint8_t send_rb_icon(const uint8_t*data);			//�Һ�ת
static uint8_t send_lt_icon(const uint8_t*data); 			//���ͷ
static uint8_t send_gs_icon(const uint8_t*data); 			//ֱ��
static uint8_t send_wp_icon(const uint8_t*data); 			//����;����
static uint8_t send_enr_icon(const uint8_t*data); 		//���뻷��
static uint8_t send_exr_icon(const uint8_t*data); 		//ʻ������
static uint8_t send_sa_icon(const uint8_t*data); 			//���������
static uint8_t send_ts_icon(const uint8_t*data); 			//�����շ�վ
static uint8_t send_des_icon(const uint8_t*data); 		//����Ŀ�ĵ�
static uint8_t send_tu_icon(const uint8_t*data); 			//�������
static uint8_t send_ant_icon(const uint8_t*data); 		//˳��


/////////////////////////////////////////////////////////////////////


uint8_t TBT_Handler(const uint8_t*data,const uint8_t len)
{
	uint8_t res = 0 ;
	cur_Bam_data[3] = 3;
	gui_Bam_data[3] = 3;
	
	/*
	* ���յ�BUFFER���ֽ���Ҫ����2 
	* �������ݸ�ʽ��head+type+(tbt)cmd+(tbt)value 
	* value����û��
	*/
	
	if(len<3)
	{
		return len_error;
	}

//��֤���ݸ�ʽ�Ƿ���ȷ	

	
	/*У��ͷ�ֽ� 0XB0*/
		if(data[0]!=head)
		{
			return head_error;
		}

		
		/*У��TBT�����ֽ� 0X56*/
		if(data[1]!=type_tbt)
		{
			return type_error;
		}

	tbt_cmd = data[2];//get the tbt cmd	
	
//��һ������ʼ��������

	switch(tbt_cmd)
	{
		case cmd_navi_type: res=send_Navi_type(data); 		break ;//������ʾ����
		case cmd_ve_dir:		res=send_ve_dir(data); 				break ;//�Գ�����
		case cmd_cur_name:  res=send_cur_name(data);			break ;//��ǰ��·��
		case cmd_nc_nam:    res=send_nc_name(data); 			break ;//��һ��·��
		case cmd_nt_icon:   res=send_nt_icon(data);				break ;//�¸�·��ת��ͼ��
		case cmd_nnt_icon:  res=send_nnt_icon(data);			break ;//�¸�·�ھ���-���¸�·��ʣ�����ת��ͼ��
		case cmd_Re_dis_bar:res=send_Re_dis_bar(data); 		break ;//ʣ����������������һ������㣩
		case cmd_Re_dis: 		res=send_Re_dis(data);				break ;//��ǰ������ʣ�����
		case cmd_ve_icon:		res=send_ve_icon(data); 			break ;//�Գ�ͼ��
		case cmd_tl_icon:		res=send_tl_icon(data); 			break ;//��ת
		case cmd_tr_icon:		res=send_tr_icon(data); 			break ;//��ת
		case cmd_lf_icon:		res=send_lf_icon(data); 			break ;//��ǰ��ת
		case cmd_rf_icon:		res=send_rf_icon(data); 			break ;//��ǰ��ת
		case cmd_lb_icon:		res=send_lb_icon(data); 			break ;//���ת
		case cmd_rb_icon:		res=send_rb_icon(data); 			break ;//�Һ�ת
		case cmd_lt_icon:		res=send_lt_icon(data); 			break ;//���ͷ
		case cmd_gs_icon:		res=send_gs_icon(data); 			break ;//ֱ��
		case cmd_wp_icon:		res=send_wp_icon(data); 			break ;//����;����
		case cmd_enr_icon:	res=send_enr_icon(data); 			break ;//���뻷��
		case cmd_exr_icon:	res=send_exr_icon(data); 			break ;//ʻ������
		case cmd_sa_icon:		res=send_sa_icon(data); 			break ;//���������
		case cmd_ts_icon:		res=send_ts_icon(data); 			break ;//�����շ�վ
		case cmd_des_icon:	res=send_des_icon(data); 			break ;//����Ŀ�ĵ�
		case cmd_tu_icon:		res=send_tu_icon(data); 			break ;//�������
		case cmd_ant_icon:	res=send_ant_icon(data); 			break ;//˳��	
		default : return data_error; 									//	break ;
		
	}
	
	if(res)
	{
		return data_error;
	}
	

	//�ڶ���������������ɣ�׼�����Ͷ������
	
	//׼����BAM������
	
	/*data[4]���������ֽ������õ���Ҫ���ٸ�DTn.��ǰ��·����DT2��ʼ���Լ�1 */
	if((tbt_cmd==cmd_cur_name)&&(len>5))
	{
		cur_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+1;
	}
	
	/*data[4]���������ֽ������õ���Ҫ���ٸ�DTn.��һ����·����DT4��ʼ���Լ�3*/
	if((tbt_cmd==cmd_nc_nam)&&(len>5))
	{
		gui_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+3;
	}

		
	//����CAN����
	if(tbt_cmd==cmd_cur_name)
	{
	
		send_tbt_cur_data(cur_Bam_data[3]);
	}
	else
	{
	
		send_tbt_gui_data(gui_Bam_data[3]);
	}
   
	//��������������ݷ�����ɣ�׼������Demand�ı��ģ���Ҫ��ʱ100ms�ٷ���
	
	
	res=send_de_data();
	if(res)
	{
		printf("data error\n");
	}
	
	
	return success;//����
	
}

static uint8_t send_tbt_cur_data(uint8_t dt_num)
{
		uint8_t i;
		cur_Bam_data[0] = 0x20;
	  cur_data[0][0] = 0x01;
	
	//��Ҫ֪�����Ͷ��ٸ�DTn
	
	
	for(i=0;i<dt_num;i++)
	{
		cur_data[i][7]=TBT_check(cur_data[i]);	
	}
			
	SendCANData(can_num,can_mailbox,can_id, cur_Bam_data, 4);//����BAM����
	
	for(i=0;i<dt_num;i++)
	{
		SendCANData(can_num,can_mailbox,can_id, cur_data[i], 8);//����DTn����
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
	/*********�̶���ֵ**********/
	gui_Bam_data[0] = 0x20;//�̶�Ϊ0x20
	

	gui_data[0][0] = 0x01;//�ֽڵ�ͷ�����ݱ��̶� 
	gui_data[1][0] = 0x02;
	gui_data[2][0] = 0x03;
	
	gui_data[0][1] = Storing_buf_num0; //ʹ��num 0 store buffer
	gui_data[0][1]&= 0x0F;

	
	gui_data[0][2]&= ~(3<<6);//��2λ����
	gui_data[0][2]|= Guide_arrow_color<<6;
	
	
	//��Ҫ֪�����Ͷ��ٸ�DTn

		for(i=0;i<dt_num;i++)
	{
		gui_data[i][7]=TBT_check(gui_data[i]);	
	}
			
	SendCANData(can_num,can_mailbox,can_id, gui_Bam_data, 4);//����BAM����
	
	for(i=0;i<dt_num;i++)
	{
		SendCANData(can_num,can_mailbox,can_id, gui_data[i], 8);//����DTn����
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

//����demand�����ݱ���
static uint8_t send_de_data()
{
	
	return success;	
}


//������ʾ����
static uint8_t  send_Navi_type(const uint8_t*data) 
{
	
	return data_error;//δ�õ���Ӧ��Ϣ������demand��Ϣ
}
//�Գ�����
static uint8_t send_ve_dir(const uint8_t*data) 
{
	
	return data_error;//δ�õ���Ӧ��Ϣ������demand��Ϣ
}
//��ǰ��·��
static uint8_t send_cur_name(const uint8_t*data)
{
	
	uint8_t i;
	cur_Bam_data[0] = 0x20;
	cur_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+1;//data[4]���������ֽ������õ���Ҫ���ٸ�DTn.��ǰ��·����DT2��ʼ���Լ�1


	//�ֽڱ��뷽ʽ 
	cur_data[0][1]&= ~(7<<5);//�����λ
	cur_data[0][1]|= data[3]<<5;
	
	//�ֽ���
	cur_data[0][1]&= ~(31<<0);//���5λ
	cur_data[0][1]|= data[4]<<0;
  
  
	if(cur_Bam_data[3]-1>0)//�����ַ�����Ҫ���͵����ݰ��� DT1 ����
	{
				cur_data[1][0] = 0x02;
				
				if(data[4]<7)//�ַ�С�ڻ����6�ֽ�
				{
						for(i=0;i<data[4];i++)
						{
							cur_data[1][i+1]=data[5+i];
						}
				}
				else
				{
						for(i=0;i<6;i++)//�ַ�������6�ֽ�
						{
							cur_data[1][i+1]=data[5+i];
						}
				}		
		
		if((cur_Bam_data[3]-2>0)||(cur_Bam_data[3]-2==0))//�����ַ�����Ҫ���͵����ݰ��� DT2 ����
		{
					cur_data[2][0] = 0x03;
					
					if((data[4]-6)<7)//�ַ�С�ڻ����12�ֽ�
					{
							for(i=0;i<(data[4]-6);i++)
							{
								cur_data[2][i+1]=data[5+i+6];
								
							}
					}
					else
					{
							for(i=0;i<6;i++)//�ַ�������12�ֽ�
							{
								cur_data[2][i+1]=data[5+i+6];
							}
					}			

		
				if(cur_Bam_data[3]-3>0)//�����ַ�����Ҫ���͵����ݰ��� DT3 ����
				{
							cur_data[3][0] = 0x04;
							
							if((data[4]-12)<7)//�ַ�С�ڻ����18�ֽ�
							{
									for(i=0;i<(data[4]-12);i++)
									{
										cur_data[3][i+1]=data[5+i+12];
									}
							}
							else
							{
									for(i=0;i<6;i++)//�ַ�������18�ֽ�
									{
										cur_data[3][i+1]=data[5+i+12];
									}
							}

					if(cur_Bam_data[3]-4>0)//�����ַ�����Ҫ���͵����ݰ��� DT4 ����
					{
								cur_data[4][0] = 0x05;
								if((data[4]-18)<7)//�ַ�С�ڻ����24�ֽ�
								{
										for(i=0;i<(data[4]-18);i++)
										{
											cur_data[4][i+1]=data[5+i+18];
										}
								}
								else//�ַ�������24�ֽ�
								{
										for(i=0;i<6;i++)
										{
											cur_data[4][i+1]=data[5+i+18];
										}
								}
							
					
						if(cur_Bam_data[3]-5>0)//�����ַ�����Ҫ���͵����ݰ��� DT5 ����
						{
									cur_data[5][0] = 0x06;				
									if((data[4]-24)<7)//�ַ�С�ڻ����30�ֽ�
									{
											for(i=0;i<(data[4]-24);i++)
											{
												cur_data[5][i+1]=data[5+i+24];
											}
									}
									else//�ַ�������30�ֽ�
									{
											for(i=0;i<6;i++)
											{
												cur_data[5][i+1]=data[5+i+24];
											}
									}				
									
							 if(cur_Bam_data[3]-6>0)//�����ַ�����Ҫ���͵����ݰ��� DT6����
							 {
										cur_data[6][0] = 0x07;					 
										if((data[4]-30)<7)//�ַ�С�ڻ����36�ֽ�
										{
												for(i=0;i<(data[4]-30);i++)
												{
													cur_data[6][i+1]=data[5+i+30];
												}
										}
										else//�ַ�������36�ֽ� ֻ����ǰ36���ֽ�
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
//��һ��·��
static uint8_t send_nc_name(const uint8_t*data)
{
	uint8_t i;
	gui_Bam_data[0] = 0x20;//�̶�Ϊ0x20
	gui_Bam_data[3]=(data[4]/6)+((data[4]%6) && (uint8_t*)0xff)+3;//data[4]���������ֽ������õ���Ҫ���ٸ�DTn.��һ����·����DT4��ʼ���Լ�3
	
	//�ֽڱ���
	gui_data[0][6]&= ~(7<<5);
	gui_data[0][6] = data[3]<<5;//�������ݸ�ʽ �� 0xB0+0x56+0x03+�ַ�����+�ַ���+�ַ�����
	
	//�ֽ���
	gui_data[0][6]&= ~(63<<0);
	gui_data[0][6]|= data[4]<<0;
	
	if(gui_Bam_data[3]-3>0)//�����ַ�����Ҫ���͵����ݰ��� DT3 ����
	{
				gui_data[3][0] = 0x04;
				
				if(data[4]<7)//�ַ�С�ڻ����6�ֽ�
				{
						for(i=0;i<data[4];i++)
						{
							gui_data[3][i+1]=data[5+i];
						}
				}
				else
				{
						for(i=0;i<6;i++)//�ַ�������6�ֽ�
						{
							gui_data[3][i+1]=data[5+i];
						}
				}

		if(gui_Bam_data[3]-4>0)//�����ַ�����Ҫ���͵����ݰ��� DT4 ����
		{
					gui_data[4][0] = 0x05;
					if((data[4]-6)<7)//�ַ�С�ڻ����12�ֽ�
					{
							for(i=0;i<(data[4]-6);i++)
							{
								gui_data[4][i+1]=data[5+i+6];
							}
					}
					else//�ַ�������12�ֽ�
					{
							for(i=0;i<6;i++)
							{
								gui_data[4][i+1]=data[5+i+6];
							}
					}
				
		
			if(gui_Bam_data[3]-5>0)//�����ַ�����Ҫ���͵����ݰ��� DT5 ����
			{
				  	gui_data[5][0] = 0x06;				
						if((data[4]-12)<7)//�ַ�С�ڻ����18�ֽ�
						{
								for(i=0;i<(data[4]-12);i++)
								{
									gui_data[5][i+1]=data[5+i+12];
								}
						}
						else//�ַ�������18�ֽ�
						{
								for(i=0;i<6;i++)
								{
									gui_data[5][i+1]=data[5+i+12];
								}
						}				
						
				 if(gui_Bam_data[3]-6>0)//�����ַ�����Ҫ���͵����ݰ��� DT6����
				 {
							gui_data[6][0] = 0x07;					 
							if((data[4]-18)<7)//�ַ�С�ڻ����24�ֽ�
							{
									for(i=0;i<(data[4]-18);i++)
									{
										gui_data[6][i+1]=data[5+i+18];
									}
							}
							else//�ַ�������24�ֽ�
							{
									for(i=0;i<6;i++)
									{
										gui_data[6][i+1]=data[5+i+18];
									}
							}						 
					 
						if(gui_Bam_data[3]-7>0)//�����ַ�����Ҫ���͵����ݰ��� DT7 ����
						{
							      gui_data[7][0] = 0x08;								
										if((data[4]-24)<7)//�ַ�С�ڻ����30�ֽ�
										{
												for(i=0;i<(data[4]-24);i++)
												{
													gui_data[7][i+1]=data[5+i+24];
												}
										}
										else//�ַ�������30�ֽ�
										{
												for(i=0;i<6;i++)
												{
													gui_data[7][i+1]=data[5+i+24];
												}
										}						 
														
							
									if(gui_Bam_data[3]-8>0)//�����ַ�����Ҫ���͵����ݰ��� DT8 ����
										{
													gui_data[8][0] = 0x09;	
													if((data[4]-30)<7)//�ַ�С�ڻ����36�ֽ�
													{
															for(i=0;i<(data[4]-30);i++)
															{
																gui_data[8][i+1]=data[5+i+30];
															}
													}
													else//�ַ�������36�ֽ�
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
//�¸�·��ת��ͼ��
static uint8_t send_nt_icon(const uint8_t*data)
{
	printf("send_nt_icon_error\n");
	return data_error;//δ�õ���Ӧ��Ϣ������demand��Ϣ
}
//�¸�·�ھ���-���¸�·��ʣ�����ת��ͼ��
static uint8_t send_nnt_icon(const uint8_t*data)
{
		printf("send_nnt_icon_error\n");	
	return data_error;//δ�õ���Ӧ��Ϣ������demand��Ϣ
}
//ʣ����������������һ������㣩
static uint8_t send_Re_dis_bar(const uint8_t*data)
{
		printf("send_Re_dis_bar_error\n");	
	return data_error;//δ�õ���Ӧ��Ϣ������demand��Ϣ
}
//��ǰ������ʣ�����
static uint8_t send_Re_dis(const uint8_t*data)
{
		printf("send_Re_dis_error\n");			
	return data_error;//δ�õ���Ӧ��Ϣ������demand��Ϣ
}
//�Գ�ͼ��
static uint8_t send_ve_icon(const uint8_t*data) 
{
	printf("send_ve_icon_error\n");	
	return data_error;//�޶�Ӧ��Ϣ
	
}

//��ת
static uint8_t send_tl_icon(const uint8_t*data) 
{
	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(0<<0);
	


	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_tl<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__tl);	

	return success;
}

//��ת
static uint8_t send_tr_icon(const uint8_t*data)
{
	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(0<<0);
	
		
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_tr<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__tr);	
	
	return success;
}

//��ǰ��ת
static uint8_t send_lf_icon(const uint8_t*data)
{
	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_lf<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__lf);	
	
	return success;	
}

//��ǰ��ת
static uint8_t send_rf_icon(const uint8_t*data) 
{
	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_rf<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__rf);		
	
	return success;	
}

//���ת
static uint8_t send_lb_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_lb<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__lb);		
	
	return success;		
}

//�Һ�ת
static uint8_t send_rb_icon(const uint8_t*data)
{
	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_rb<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__rb);	
	
	return success;
}

//���ͷ
static uint8_t send_lt_icon(const uint8_t*data) 
{
	
	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(6<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_lt<<0);
		
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][5]|= (1<<Connec_road__lt);		
	
	return success;
}

//ֱ��
static uint8_t send_gs_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(0<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_gs<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	
	return success;
}

//����;����
static uint8_t send_wp_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(8<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_wp<<0);	
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	
	return success;	
}

//���뻷��
static uint8_t send_enr_icon(const uint8_t*data) 
{
	

	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(1<<0);

	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_enr<<0);	
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__enr);

	return success;	
}

//ʻ������
static uint8_t send_exr_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(2<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_exr<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	gui_data[0][4]|= (1<<Connec_road__enr);	
	
	return success;
}

//���������
static uint8_t send_sa_icon(const uint8_t*data) 
{
	return data_error;//����Ϣ��Ӧ
}

//�����շ�վ
static uint8_t send_ts_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(10<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_ts<<0);
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;	
	
	return success;
}

//����Ŀ�ĵ�
static uint8_t send_des_icon(const uint8_t*data) 
{

	gui_data[0][2]&= ~(63<<0);//��6λ����
	gui_data[0][2]|=	(9<<0);
	
	gui_data[0][3]&= ~(31<<0);//��5λ����
	gui_data[0][3]|=	(Gui_arr_direction_des<<0);	
	
	gui_data[0][4] = 0;
	gui_data[0][5] = 0;
	
	return success;
}

//�������
static uint8_t send_tu_icon(const uint8_t*data) 
{
	printf("send_tu_icon_error\n");
	return data_error;//����Ϣ��Ӧ
}

//˳��
static uint8_t send_ant_icon(const uint8_t*data) 
{
	printf("send_ant_icon_error\n");	
  return data_error;//����Ϣ��Ӧ
}


/*
* ��������TBT_check
* ��������:������У�飬�������ֵ
* ������uint8_t * data
* TBT��У��Ͳ��ð��ֽڵ���ӣ�
* ��ǰ7���ֽں͵�8���ֽڵĸ�4λ��ӣ��õ�У��ֵ��
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











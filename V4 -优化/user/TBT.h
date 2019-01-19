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

//����
#define head 							0xB0//ͷ�ֽ�
#define type_tbt 					0x56//TBT�����ֽ�
#define cmd_navi_type     0X00//������ʾ����
#define cmd_ve_dir        0X01//�Գ�����
#define cmd_cur_name      0X02//��ǰ��·��
#define cmd_nc_nam			  0X03//��һ��·��
#define cmd_nt_icon			  0X04//�¸�·��ת��ͼ��
#define cmd_nnt_icon			0X05//�¸�·�ھ���-���¸�·��ʣ�����ת��ͼ��
#define cmd_Re_dis_bar		0X06//ʣ����������������һ������㣩
#define cmd_Re_dis				0X07//��ǰ������ʣ�����
#define cmd_ve_icon				0X08//�Գ�ͼ��
#define cmd_tl_icon				0X09//��ת
#define cmd_tr_icon				0X0A//��ת
#define cmd_lf_icon				0X0B//��ǰ��ת
#define cmd_rf_icon				0X0C//��ǰ��ת
#define cmd_lb_icon				0X0D//���ת
#define cmd_rb_icon				0X0E//�Һ�ת
#define cmd_lt_icon				0X0F//���ͷ
#define cmd_gs_icon				0X10//ֱ��
#define cmd_wp_icon				0X11//����;����
#define cmd_enr_icon			0X12//���뻷��
#define cmd_exr_icon			0X13//ʻ������
#define cmd_sa_icon				0X14//���������
#define cmd_ts_icon				0X15//�����շ�վ
#define cmd_des_icon			0X16//����Ŀ�ĵ�
#define cmd_tu_icon				0X17//�������
#define cmd_ant_icon			0X18//˳��

#define can_num			0U
#define can_id  		0x0a
#define can_mailbox 0x00

uint8_t TBT_Handler(const uint8_t*data,const uint8_t len);//TBT�����ݴ���



//���ݴ洢��buffer num 
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

//��ͷ��ɫ
#define Guide_arrow_color 3

//������ͷ����
#define Gui_arr_direction_ve 	0    //�޶�Ӧ��Ϣ �Գ�ͼ��
#define Gui_arr_direction_tl 	12		//��ת
#define Gui_arr_direction_tr 	4			//��ת
#define Gui_arr_direction_lf 	15		//��ǰ��ת
#define Gui_arr_direction_rf 	1 		//��ǰ��ת
#define Gui_arr_direction_lb 	10		//���ת
#define Gui_arr_direction_rb 	6			//�Һ�ת
#define Gui_arr_direction_lt 	0  	 //���ͷ ����ȡ * ����ȡֵΪ0��
#define Gui_arr_direction_gs 	0			//ֱ��
#define Gui_arr_direction_wp 	0  	 //����;���㣨 ��ȡ * ����ȡֵΪ0��
#define Gui_arr_direction_enr 0			//���뻷��
#define Gui_arr_direction_exr 0			//ʻ������
#define Gui_arr_direction_sa  0 	 //��������� ���޶�Ӧ��Ϣ��
#define Gui_arr_direction_ts  0 	 //�����շ�վ ����ȡ * ����ȡֵΪ0��
#define Gui_arr_direction_des 0 	 //����Ŀ�ĵ� ����ȡ * ����ȡֵΪ0��
#define Gui_arr_direction_tu  0		 //������� ���޶�Ӧ��Ϣ��
#define Gui_arr_direction_ant 0		 //˳��    ���޶�Ӧ��Ϣ��


//��·
#define Connec_road__ve	 			//�Գ�ͼ��
#define Connec_road__tl	  3 	//��ת
#define Connec_road__tr	  3		//��ת
#define Connec_road__lf	  0		//��ǰ��ת
#define Connec_road__rf	  6		//��ǰ��ת
#define Connec_road__lb	  5		//���ת
#define Connec_road__rb	  1		//�Һ�ת
#define Connec_road__lt	  7		//���ͷ
#define Connec_road__gs	  7		//ֱ��
#define Connec_road__wp				//����;����
#define Connec_road__enr	7		//���뻷��
#define Connec_road__exr	7		//ʻ������
#define Connec_road__sa				//���������
#define Connec_road__ts	 			//�����շ�վ
#define Connec_road__des	 		//����Ŀ�ĵ� 
#define Connec_road__tu				//�������
#define Connec_road__ant			//˳��  



#endif 


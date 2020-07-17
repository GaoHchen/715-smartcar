/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ctime
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8G2K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/


#include "zf_tim.h"
#include "board.h"



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ����ʼ����Ϊ�ⲿ����
//  @param      tim_n           ѡ��ģ��
//  @return     void
//  @since      v1.0
//  Sample usage:               ctimer_count_init(CTIM0_P34);		//��ʼ����ʱ��0���ⲿ����ΪP3.4����
//  @note                       ����1ʹ�ö�ʱ��1��Ϊ�����ʷ�������
//								����2ʹ�ö�ʱ��2��Ϊ�����ʷ�������
//								����3ʹ�ö�ʱ��3��Ϊ�����ʷ�������
//								����4ʹ�ö�ʱ��4��Ϊ�����ʷ�������
//                              STC8G���ж�ʱ��0-��ʱ��4����5����ʱ����
//								�������ɼ�����Ҳ��Ҫ��ʱ����Ϊ�ⲿ������
//-------------------------------------------------------------------------------------------------------------------
void ctimer_count_init(CTIMN_enum tim_n)
{
	switch(tim_n)
	{
		case CTIM0_P34:
		{
			TL0 = 0; 
			TH0 = 0; 
			TMOD |= 0x04; //�ⲿ����ģʽ
			TR0 = 1; //������ʱ��
			break;
		}
		
		case CTIM1_P35:
		{
			TL1 = 0x00;
			TH1 = 0x00;
			TMOD |= 0x40; // �ⲿ����ģʽ
			TR1 = 1; // ������ʱ��
			break;
		}
		
		case CTIM2_P12:
		{
			T2L = 0x00;
			T2H = 0x00;
			AUXR |= 0x18; // �����ⲿ����ģʽ��������ʱ��
			break;
		}
		
		case CTIM3_P04:
		{
			T3L = 0; 
			T3H = 0;
			T4T3M |= 0x0c; // �����ⲿ����ģʽ��������ʱ��
			break;
		}
		
		case CTIM4_P06:
		{
			T4L = 0;
			T4H = 0;
			T4T3M |= 0xc0; // �����ⲿ����ģʽ��������ʱ��
			break;
		}
	
	}	
} 

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ������ֵ
//  @param      countch     ����ͨ���ż�����
//  @return     uint32      ���ؼ���ֵ
//  Sample usage:           num = ctimer_count_read(CTIM0_P34);     
//-------------------------------------------------------------------------------------------------------------------
uint16 ctimer_count_read(CTIMN_enum tim_n)
{
	uint16 dat;
	
	switch(tim_n)
	{
		case CTIM0_P34:
		{
			dat = (uint8)TH0 << 8;
			dat = ((uint8)TL0) | dat;
			break;
		}
		case CTIM1_P35:
		{
			dat = (uint8)TH1 << 8;
			dat = ((uint8)TL1) | dat;
			break;
		}
		case CTIM2_P12:
		{
			dat = (uint8)T2H << 8;
			dat = ((uint8)T2L) | dat;
			break;
		}
		case CTIM3_P04:
		{
			dat = (uint8)T3H << 8;
			dat = ((uint8)T3L) | dat;	
			break;
		}
		case CTIM4_P06:
		{
			dat = (uint8)T4H << 8;
			dat = ((uint8)T4L) | dat;
			break;
		}
		
	}


	return dat;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���������ֵ
//  @param      countch     ����ͨ���ż�����
//  @return     void      
//  Sample usage:           ctimer_count_clean(CTIM0_P34);     
//-------------------------------------------------------------------------------------------------------------------
void ctimer_count_clean(CTIMN_enum tim_n)
{	
	switch(tim_n)
	{
		case CTIM0_P34:
		{
			TR0 = 0;
			TH0 = 0;
			TL0 = 0;
			TR0 = 1;
			break;
		}
		case CTIM1_P35:
		{
			TR1 = 0;
			TH1 = 0;
			TL1 = 0;
			TR1 = 1;
			break;
		}
		case CTIM2_P12:
		{
			AUXR &= ~(1<<4);
			T2H = 0;
			T2L = 0;
			AUXR |= 1<<4;
			break;
		}
		case CTIM3_P04:
		{
			T4T3M &= ~(1<<3);
			T3H = 0;
			T3L = 0;
			T4T3M |= (1<<3);
			break;
		}
		case CTIM4_P06:
		{
			T4T3M &= ~(1<<7);
			T4H = 0;
			T4L = 0;
			T4T3M |= (1<<7);
			break;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ�������ж�
//  @param      tim_n      ��ʱ��ͨ����
//  @param      time_ms    ʱ��(ms)
//  @return     void      
//  Sample usage:          pit_timer_ms(TIM_0, 10)
//						   ʹ�ö�ʱ��0�������жϣ�ʱ��10msһ�Ρ�
//-------------------------------------------------------------------------------------------------------------------
void pit_timer_ms(TIMN_enum tim_n,uint16 time_ms)
{
	uint16 temp;
	temp = 65536 - (sys_clk / (12 * (1000 / time_ms)));
	
	if(TIM_0 == tim_n)
	{
		TMOD |= 0x00; 	// ģʽ 0
		TL0 = temp; 	
		TH0 = temp >> 8;
		TR0 = 1; 		// ������ʱ��
		ET0 = 1; 		// ʹ�ܶ�ʱ���ж�
	}
	else if(TIM_1 == tim_n)
	{
		TMOD |= 0x00; // ģʽ 0
		TL1 = temp; 	
		TH1 = temp >> 8;
		TR1 = 1; // ������ʱ��
		ET1 = 1; // ʹ�ܶ�ʱ���ж�
	}
	else if(TIM_2 == tim_n)
	{
		T2L = temp; 	
		T2H = temp >> 8;
		AUXR |= 0x10; // ������ʱ��
		IE2 |= 0x04; // ʹ�ܶ�ʱ���ж�
	}
	else if(TIM_3 == tim_n)
	{
		T3L = temp; 	
		T3H = temp >> 8;
		T4T3M |= 0x08; // ������ʱ��
		IE2 |= 0x20; // ʹ�ܶ�ʱ���ж�
	}
	else if(TIM_4 == tim_n)
	{
		T4L = temp; 	
		T4H = temp >> 8;
		T4T3M |= 0x80; // ������ʱ��
		IE2 |= 0x40; // ʹ�ܶ�ʱ���ж�
	}
}


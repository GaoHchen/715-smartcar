/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8G2K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
#include "headfile.h"

//UART1�ж�
void UartIsr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��SBUF
        res = SBUF;
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
				//������λ��ISP��س�������ʵ���Զ�����
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
		
    }
}

//UART2�ж�
void Uart2Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S2BUF

	}
}


//UART3�ж�
void Uart3Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S3BUF

	}
}


//UART4�ж�

void Uart4Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;
		//�������ݼĴ���Ϊ��S4BUF;

	}
}


////PCA���ڶ�ʱ���ж�
//void PCA_Isr() interrupt 7
//{
//	if(PCA0_GET_FLAG)
//	{
//		//����жϱ�־λ
//		PCA_CLEAR_FLAG(PCA_0);	

//		//���ؼ�����
//		pca_reload_counter(PCA_0);
//		
//		//����CCD�ɼ�����
//		ccd_collect();
//		//�ɼ���ɱ�־λ��1
//		tsl1401_finish_flag = 1;
//		
//		
//	}
//	
//	if(PCA1_GET_FLAG)
//	{
//		//����жϱ�־λ
//		PCA_CLEAR_FLAG(PCA_1);	
//		//���ؼ�����
//		pca_reload_counter(PCA_1);

//	}
//	//	PCA_2������ʱ
//	if(PCA2_GET_FLAG)
//	{
//		//����жϱ�־λ
//		PCA_CLEAR_FLAG(PCA_2);	
//		//���ؼ�����
//		//pca_reload_counter(PCA_2);

//	}
//	
//}



void INT0_Isr() interrupt 0
{
	//�˱�־λ��Ӳ���Զ����㡣
	//INT0_CLEAR_FLAG;
	
}
void INT1_Isr() interrupt 2
{
	//�˱�־λ��Ӳ���Զ����㡣
	//INT1_CLEAR_FLAG;
	
}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;
	
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;
	
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;

}


//void INT0_Routine(void) interrupt 0
//void TM0_Rountine(void) interrupt 1
//void INT1_Routine(void) interrupt 2
//void TM1_Rountine(void) interrupt 3
//void UART1_Routine(void) interrupt 4
//void ADC_Routine(void) interrupt 5
//void LVD_Routine(void) interrupt 6
//void PCA_Routine(void) interrupt 7
//void UART2_Routine(void) interrupt 8
//void SPI_Routine(void) interrupt 9
//void INT2_Routine(void) interrupt 10
//void INT3_Routine(void) interrupt 11
//void TM2_Routine(void) interrupt 12
//void INT4_Routine(void) interrupt 16
//void UART3_Routine(void) interrupt 17
//void UART4_Routine(void) interrupt 18
//void TM3_Routine(void) interrupt 19
//void TM4_Routine(void) interrupt 20
//void CMP_Routine(void) interrupt 21
//void PWM0_Routine(void) interrupt 22
//void PWM0FD_Routine(void) interrupt 23
//void I2C_Routine(void) interrupt 24
//void PWM1_Routine(void) interrupt 28
//void PWM2_Routine(void) interrupt 29
//void PWM3_Routine(void) interrupt 30
//void PWM4_Routine(void) interrupt 31
//void PWM5_Routine(void) interrupt 32
//void PWM2FD_Routine(void) interrupt 33
//void PWM4FD_Routine(void) interrupt 34
//void TKSU_Routine(void) interrupt 35

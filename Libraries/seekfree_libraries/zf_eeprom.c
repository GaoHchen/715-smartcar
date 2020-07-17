/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		eeprom
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8G2K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/
#include "zf_eeprom.h"
#include "board.h"
#include "intrins.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��EEPROM
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_init(void)
{
	IAP_CONTR |= 1<<7;	 	//ʹ��EEPROM����
	iap_set_tps();			//���ò����ȴ�ʱ��
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ر�EEPROM
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_idle(void)
{
	IAP_CONTR &= ~(1<<7);//ʧ��EEPROM����
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡEEPROM����ʧ��״̬λ����Ҫ��������
//  @param      NULL
//  @return     void
//  Sample usage:           
//								����ʧ�ܷ���1;
//-------------------------------------------------------------------------------------------------------------------
uint8 iap_get_cmd_state(void)
{
	return ((IAP_CONTR&0x01) == 0x01);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����IAP�ȴ�ʱ��
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_set_tps(void)
{
	uint8 write_time;
	write_time = (sys_clk / 1000000) + 1;
	IAP_TPS = write_time;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM��ȡ����ֽ�
//  @param      addr			��Ҫ��ȡ��eeprom��ַ
//  @param      *buf			��Ҫ��ȡ�����ݵ�ַ
//  @param      len				��Ҫ��ȡ�����ݳ���
//  @return     void
//  Sample usage:               uint8 str[10];
//								iap_read_bytes(0x00,str,10);
//								��0x00-0x0A��ַ�е����ݣ���ȡ��str�С�
//-------------------------------------------------------------------------------------------------------------------
void iap_read_bytes(uint16 addr, uint8 *buf, uint8 len)
{
	IAP_CMD = 1; 				//���� IAP ������	

	while(len--)
	{
		IAP_ADDRL = addr; 		//���� IAP �͵�ַ
		IAP_ADDRH = addr >> 8; 	//���� IAP �ߵ�ַ
		IAP_TRIG = 0x5a; 		//д��������(0x5a)
		IAP_TRIG = 0xa5; 		//д��������(0xa5)	
		*buf++ = IAP_DATA; 		//�� IAP ����
		addr++;
		_nop_(); 
	}
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROMд����ֽ�
//  @param      addr			��Ҫд��eeprom��ַ
//  @param      *buf			��Ҫд�����ݵ�ַ
//  @param      len				��Ҫд�����ݳ���
//  @return     void
//  Sample usage:       		iap_write_bytes(0x00,(uint8 *)"0123456789";,10);
//								��"0123456789"д��0x00-0x0A��ַ��;
//-------------------------------------------------------------------------------------------------------------------
void iap_write_bytes(uint16 addr, uint8 *buf, uint8 len)
{
	IAP_CMD = 2; 				//���� IAP ������	
	
	while(len--)
	{
		IAP_ADDRL = addr; 		//���� IAP �͵�ַ
		IAP_ADDRH = addr >> 8; 	//���� IAP �ߵ�ַ
		IAP_DATA = *buf++; 		//д IAP ����
		addr++;

		IAP_TRIG = 0x5a; 		//д��������(0x5a)
		IAP_TRIG = 0xa5; 		//д��������(0xa5)
		_nop_(); 
	}
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM����Ŀ���ַ���ڵ�һҳ��1����/512�ֽڣ�
//  @param      addr			��Ҫд��eeprom��ַ
//  @return     void
//  Sample usage:       		iap_erase(0x20);
//								����0x00-0x200������
//-------------------------------------------------------------------------------------------------------------------
void iap_erase_page(uint16 addr) 
{ 
	IAP_CMD = 3; 				//���� IAP ��������
	IAP_ADDRL = addr; 			//���� IAP �͵�ַ
	IAP_ADDRH = addr >> 8;  	//���� IAP �ߵ�ַ
	IAP_TRIG = 0x5a; 			//д��������(0x5a)
	IAP_TRIG = 0xa5; 			//д��������(0xa5)
	_nop_(); 
}




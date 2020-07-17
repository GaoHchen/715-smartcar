/*********************************************************************************************************************

 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK5.27
 * @Target core		STC8G8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/

#include "headfile.h"
#define PI  3.14

#define  down			P43
#define  right    P42
#define  left     P37
#define  mid    	P36
#define  up    		P41

#define SPEEDL_PIN   CTIM3_P04
#define SPEEDR_PIN   CTIM4_P06

float K1 =0.15; 
float angle = 0, angle_dot; 	
float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
float Gyro_Balance,Angle_Balance,Gyro_Turn,Acceleration_Z;
float AngleControlOut,GyroControlOut=0;
float Gyro_error=0;
float Last_error=0;
float Gyro_Ingter;
float Prev_error=0;
//直立参数
float Angle_Kp=0,Angle_Kd=0;
float Gyro_Kp=-0.4,Gyro_Kd=-0.03,Gyro_Ki=0;
//
float balance_Kp=0,balance_Kd=0;  //-2.6 -0.15 
float balanceControlOut=0;
//
//速度环初始化参数  
int16 temp_l=0,temp_r=0;
int32 g_nLeftSpeed = 0,g_nRighSpeed = 0;
float g_fLeftRealSpeed;
float g_fRighRealSpeed;
float Ratio_Encoder_Left = 157/(1658*0.1);// 左轮速度=counter*左轮周长(mm)/(左轮转一圈对应的脉冲数*程序周期)
float Ratio_Encoder_Righ = 157/(1658*0.1);//  右轮速度=counter*右轮周长(mm)/(右轮转一圈对应的脉冲数*程序周期)
float g_fRealSpeed = 0;
float g_fLeftRealSpeed;
float g_fRighRealSpeed;
float g_fSpeedFilter = 0;
float g_fSpeedError;
float g_fSpeedErrorTemp[5] = {0};
int32 g_fExpectSpeed = 400;
float fSpeedErrorInteg = 0;
float g_fSpeedControlOutOld = 0,g_fSpeedControlOutNew = 0;
float g_fSpeedControlOut = 0;
uint8 Flag_SpeedControl = 0;
int32  g_fSpeedF;
float g_fSpeed=0;
//清华速度控制环
float g_speedControl_P = 50;//纯P速度控制
float g_speedControl_I = 0;	
float g_speedControl_D = 0;//0.3;
float g_PWMOut=0;
int16  speedl_power;
int16  speedr_power;
int32 g_nLeftPWM, g_nRighPWM;
uint8 GP[1];
uint8 GD[1];
uint8 AP[1];
uint8 AD[1];
uint8 DP[1];
uint8 DD[1];
//方向环初始化参数
uint16 zuoAD[10],youAD[10],zhongAD[10],g_ValueOfAD[10];
uint16  zuozong_last[2]={0},youzong_last[2]={0}, zuozong_shu_last[2]={0},youzong_shu_last[2]={0},zhongzong_last[2]={0};
uint16 temp,t,zuoADmax,youADmax,dangqian_chazhi,dangqian_chazhi_shu,you_shuzong,zuo_shuzong;
int16 zuozong,youzong,zhongzong,KEE,shu_KE;
uint8 i,c,Direction_falg=0;
int16 g_fLeftVoltageSigma = 0;
int16 g_fRightVoltageSigma = 0;
uint16 D_last,D_last2,index,pid_Ki=1;
uint16 dangqian_he;
int16 fLeftRightAdd,fLeftRightSub,fLeftRightSub_last;
int16   KEE,KE,KE_LAST=0,shu_KE,shu_KE_last,KEEE;
uint16 canshu[10]={0} ,D_1,D_2,D_3,D_now ;
int16 fValue,fValue_last;
int16 nLeft_last,nRight_last,zuo_shuzong_last,you_shuzong_last;
//方向环PID
float  DIR_CONTROL_P=0;
float  DIR_CONTROL_D=0;
int16 g_fDirectionControlOut=0;
uint16 a[];
/**

  * @example  裂开代码
  *
  * @date     2020/6/25 星期四
*/
void AD_get()
{
     for(i=0;i<5;i++)
     {
         zuoAD[i]=adc_once(ADC_P12, ADC_10BIT);
         youAD[i]=adc_once(ADC_P11, ADC_10BIT);
     }
	zuozong=(zuoAD[4]+zuoAD[3]+zuoAD[2]+zuoAD[1]+zuoAD[0])/5;
  
  youzong=(youAD[4]+youAD[3]+youAD[2]+youAD[1]+youAD[0])/5;

}
void AD_chuli(void)
{
  zuozong=110+((zuozong-zuoADmax)*200)/zuoADmax;       //归一化
  youzong=110+((youzong-youADmax)*200)/youADmax;
  
  if(zuozong>=800) zuozong=800;
  if(youzong>=800) youzong=800;
  if(youzong<=0) youzong=0;
  if(zuozong<=0) zuozong=0;
  
  if(youzong<zuozong)
  {  //判断左右方向                          
    // dangqian_chazhi=(( uint16)(zuozong-youzong)*200)/(zuozong+youzong);          //差比和        
    dangqian_chazhi=zuozong-youzong;
    c=0;//车在跑道右侧，右轮加速
  }else if(youzong>zuozong)
  {
    //  dangqian_chazhi=(( uint16)(youzong-zuozong)*200)/(zuozong+youzong);                  
    c=1;//车在跑道左侧，左轮加速
    dangqian_chazhi=youzong-zuozong;
  }
  
//  if(you_shuzong<zuo_shuzong)
//  {  //判断左右方向                          
//    // dangqian_chazhi=(( uint16)(zuozong-youzong)*200)/(zuozong+youzong);          //差比和        
//    dangqian_chazhi_shu=zuo_shuzong-you_shuzong;
//    //  shu_c=0;//车在跑道右侧，右轮加速
//  }
//  else  
//  {
//    //  dangqian_chazhi=(( uint16)(youzong-zuozong)*200)/(zuozong+youzong);                  
//    //  shu_c=1;//车在跑道左侧，左轮加速
//    dangqian_chazhi_shu=you_shuzong-zuo_shuzong;
//  }
  KEE=(dangqian_chazhi*200)/(youzong+zuozong);
//  shu_KE=(dangqian_chazhi_shu*200)/(you_shuzong+zuo_shuzong);
  //KEEE=10*dangqian_chazhi_shu/dangqian_chazhi;
//  if(shu_KE>=200) shu_KE=200;
  if(KEE>=200) KEE=200;
}

void Deeprom()
{
	//从eeprom读取数据
   iap_read_bytes(0x00,GP,2);
   Gyro_Kp=GP[0];
	 iap_read_bytes(0x02,GD,2);
   Gyro_Kd=GD[0];
	 iap_read_bytes(0x04,AP,2);
   Angle_Kp=AP[0];
	 iap_read_bytes(0x06,AD,2);
   Angle_Kd=AD[0];
	 iap_read_bytes(0x08,DP,2);
   DIR_CONTROL_P=DP[0];
	 iap_read_bytes(0x10,DD,2);
   DIR_CONTROL_D=DD[0];
	
}
void Xeeprom()
{
	//写入数据到eeprom
	 GP[0]=Gyro_Kp;
	 iap_write_bytes(0x00,GP,2);
	 GD[0]=Gyro_Kd;
	 iap_write_bytes(0x02,GD,2);
	 AP[0]=Angle_Kp;
	 iap_write_bytes(0x04,AP,2);
	 AD[0]=Angle_Kd;
	 iap_write_bytes(0x06,AD,2);
	 DP[0]=DIR_CONTROL_P;
	 iap_write_bytes(0x08,DP,2);
	 DD[0]=DIR_CONTROL_D;
	 iap_write_bytes(0x10,DD,2);
	
}



/*****按键检测及去抖*****/
void key_press(bit key)
{
	oled_fill(0x00);
	if(key==0)								//按键按下会有6~10ms抖动电平，20ms稳定时间
	{													//检测是否按下，10ms后再次检测
		pca_delay_ms(10);
//		if(key==0)
//		{	
// 		}
		while(!key)					 //按键松开检测
		{
			oled_uint16(40, 2, key);
			oled_uint16(40, 3, up);
		}
	}
}





void AD_max_min()
{
	oled_fill(0x00);
/**********初始界面**********/
	  while(up)			
  {   
    AD_get();
    //将结果显示到TFT上
		oled_p6x8str(0,1,"star");
	  oled_p6x8str(0,1,"left:");oled_uint16(40, 1, zuozong);
		oled_p6x8str(0,2,"right:");oled_uint16(40, 2, youzong);
		oled_p6x8str(0,3,"right:");oled_uint16(40, 3, youzong);
  }
   
	key_press(up);
	oled_fill(0x00);
	
/**********获取电感基准值**********/	
	while(up)
	{
		oled_p6x8str(20,1,"if enter menu");
		oled_p6x8str(30,4,"yes");
		oled_p6x8str(80,4,"no");
		if(left==0)
		{
			break;
		}
		if(right==0)
		{
			oled_fill(0x00);
			while(up)
			{   
				AD_get();
				//将结果显示到TFT上
				oled_uint16(0, 2, zuoADmax);
				oled_uint16(0, 3, youADmax);
		

				youADmax=youzong;
				zuoADmax=zuozong;

			}
			pca_delay_ms(10);
			while(!up);					 //按键松开检测
			oled_fill(0x00);



			while(up)
			{  
				if(left==0)
				{
					Gyro_Kp-=1;   
				}
				else if(right==0)
				{
					Gyro_Kp+=1; 
				}
				//oled_p6x8str(0,1,"Gp");oled_printf_float(30,1,Gyro_Kp,2,3);
				oled_p6x8str(0,1,"Gp");oled_int16(30,1,Gyro_Kp);
				pca_delay_ms(100);
			}
			pca_delay_ms(10);
			while(!up);					 //按键松开检测
			
			while(up)
			{   
				if(left==0)
				{
					Gyro_Kd-=1;   
				}
				else if(right==0)
				{
					Gyro_Kd+=1; 
				}
				//oled_p6x8str(0,2,"Gd");oled_printf_float(30,2,Gyro_Kd,2,3);
				oled_p6x8str(0,2,"Gd");oled_int16(30,2,Gyro_Kd);
				pca_delay_ms(100);
			}
			pca_delay_ms(10);
			while(!up);					 //按键松开检测
			
			while(up)
			{   
				if(left==0)
				{
					Angle_Kp-=1;   
				}
				else if(right==0)
				{
					Angle_Kp+=1; 
				}
				//oled_p6x8str(0,3,"Ap");oled_printf_float(30,3,Angle_Kp,2,3);
				oled_p6x8str(0,3,"Ap");oled_int16(30,3,Angle_Kp);
				pca_delay_ms(100);
			}
			pca_delay_ms(10);
			while(!up);					 //按键松开检测
    
			while(up)
			{   
				if(left==0)
				{
					Angle_Kd-=2;   
				}
				else if(right==0)
				{
					Angle_Kd+=2; 
				}
				//oled_p6x8str(0,4,"Ad");oled_printf_float(30,4,Angle_Kd,2,3);
				oled_p6x8str(0,4,"Ad");oled_int16(30,4,Angle_Kd);
				pca_delay_ms(100);
			}
			pca_delay_ms(10);
			while(!up);					 //按键松开检测
			
			while(up)
			{  			
				if(left==0)
				{
					DIR_CONTROL_P-=1;   
				}
				else if(right==0)
				{
					DIR_CONTROL_P+=1; 
				}
				//oled_p6x8str(0,1,"Gp");oled_printf_float(30,1,Gyro_Kp,2,3);
				oled_p6x8str(0,5,"DP");oled_int16(30,5,DIR_CONTROL_P);
				pca_delay_ms(100);
			}
			pca_delay_ms(10);
			while(!up);					 //按键松开检测
			
			
			while(up)
			{   
				if(left==0)
				{
					DIR_CONTROL_D-=1;   
				}
				else if(right==0)
				{
					DIR_CONTROL_D+=1; 
				}
				//oled_p6x8str(0,2,"Gd");oled_printf_float(30,2,Gyro_Kd,2,3);
				oled_p6x8str(0,6,"DD");oled_int16(30,6,DIR_CONTROL_D);
				pca_delay_ms(100);
			}
			pca_delay_ms(10);
			while(!up);					 //按键松开检测
			iap_erase_page(0X20);
			Xeeprom();
			pca_delay_ms(100);
			oled_printf_int32(30,5,3,5);
			pca_delay_ms(1000);
			oled_printf_int32(30,5,2,5);
			pca_delay_ms(1000);
			oled_printf_int32(30,5,1,5);
			pca_delay_ms(1000);
			oled_printf_int32(30,5,0,5);
			oled_fill(0x00);
		}
	}
	oled_fill(0x00);
}
void get_mpu6050()
{
  mpu6050_get_accdata();
  mpu6050_get_gyro();  
}
void Yijielvbo(float angle_m, float gyro_m)
{
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * 0.005);
}

void Get_Angle()
{ 

  Gyro_Y=mpu_gyro_y;
  Gyro_Z=mpu_gyro_z;
  Accel_X=mpu_acc_x;
  Accel_Z=mpu_acc_z;
  //Gyro_Z  = Gyro[2] ;
  //Accel_X = Accel[0];
  //Accel_Z = Accel[2];
  if(Gyro_Y>32768)  Gyro_Y-=65536;                       //
  if(Gyro_Z>32768)  Gyro_Z-=65536;                       //
  if(Accel_X>32768) Accel_X-=65536;                      //
  if(Accel_Z>32768) Accel_Z-=65536;                      //
  Gyro_Balance=-Gyro_Y;                                  //
  Accel_Y=(atan2(Accel_X,Accel_Z)*180/PI);               //
  Gyro_Y=Gyro_Y/16.4;                                    //	
  Gyro_Z=Gyro_Z/16.4;
   Yijielvbo(Accel_Y,-Gyro_Y);
  Angle_Balance=angle;  
  Gyro_Turn=Gyro_Z;                                      //
  Acceleration_Z=Accel_Z; //
}

void AngleControl()
{
  static float fValue=0;
  fValue = angle-12.5;//  看屏幕上的一二行改变车身角度此参数也要改变
  AngleControlOut = (fValue)*Angle_Kp + -Gyro_Y*Angle_Kd/1000; 
}
void GyroControl()
{
  Gyro_error = -Gyro_Y+AngleControlOut;
  Gyro_Ingter += Gyro_error; 
  if(Gyro_Ingter>1000)    Gyro_Ingter=1000;
  if(Gyro_Ingter<-1000)   Gyro_Ingter=-1000;  
  GyroControlOut = Gyro_Kp/-100*Gyro_error + Gyro_Ki*Gyro_Ingter + Gyro_Kd/-100*(Gyro_error-Last_error); 
  Prev_error = Last_error;
  Last_error = Gyro_error;
}
void CalSpeedError(void)
{
  static float fSpeedOld = 0, fSpeedNew = 0;
  int16 temp_l=0,temp_r=0;
 
  //读取采集到的编码器脉冲数
  temp_l = ctimer_count_read(SPEEDL_PIN);
  temp_r = ctimer_count_read(SPEEDR_PIN);
    
  //计数器清零
  ctimer_count_clean(SPEEDL_PIN);
  ctimer_count_clean(SPEEDR_PIN);
  
  //采集方向信息
//  if(1==SPEEDL_DIR)          
	g_nLeftSpeed = temp_l;
  //else                        g_nLeftSpeed = -temp_l;
//  if(1==SPEEDR_DIR)           
	g_nRighSpeed = temp_r;
//  else                        g_nRighSpeed = -temp_r;
  
  g_fLeftRealSpeed = g_nLeftSpeed*Ratio_Encoder_Left;
  g_fLeftRealSpeed = (g_fLeftRealSpeed>4000?4000:g_fLeftRealSpeed);		//滤左编码器的噪声
  g_fRighRealSpeed = g_nRighSpeed*Ratio_Encoder_Righ;
  g_fRighRealSpeed = (g_fRighRealSpeed>4000?4000:g_fRighRealSpeed);		//滤右编码器的噪声
  
  g_fRealSpeed = (g_fLeftRealSpeed + g_fRighRealSpeed)*0.5;				//真实速度
  //速度采集梯度平滑，每次采集最大变化200
  fSpeedOld = g_fSpeedFilter;
  //g_fSpeedFilter = g_fRealSpeed;
  fSpeedNew = g_fRealSpeed;
  
  if(fSpeedNew>=fSpeedOld)
    g_fSpeedFilter = ((fSpeedNew-fSpeedOld)>400?(fSpeedOld+400):fSpeedNew);
  else
    g_fSpeedFilter = ((fSpeedNew-fSpeedOld)<-400?(fSpeedOld-400):fSpeedNew);
  
  g_fSpeedError = g_fExpectSpeed - g_fSpeedFilter;
  g_fSpeedErrorTemp[4] = g_fSpeedErrorTemp[3];
  g_fSpeedErrorTemp[3] = g_fSpeedErrorTemp[2];
  g_fSpeedErrorTemp[2] = g_fSpeedErrorTemp[1];
  g_fSpeedErrorTemp[1] = g_fSpeedErrorTemp[0];
  g_fSpeedErrorTemp[0] = g_fSpeedError;
  
  //        lcd_showstr(0,1,"LS:");  lcd_showint32(3*8,1,g_nLeftSpeed,9);
  //        lcd_showstr(0,2,"RS:");  lcd_showint32(3*8,2,g_nRighSpeed,9);
  //	lcd_showstr(0,3,"LrS:");  lcd_showint32(3*8,3,g_fLeftRealSpeed,9);
  //        lcd_showstr(0,4,"RrS:");  lcd_showint32(3*8,4,g_fRighRealSpeed,9);
  //        lcd_showstr(0,5,"RS:");  lcd_showint32(3*8,5,g_fRealSpeed,9);
  //        lcd_showstr(0,1,"LS:");  lcd_showint32(3*8,6,g_nLeftSpeed,9);
//	oled_printf_int32(0,1,g_nLeftSpeed,5);
//	oled_printf_int32(0,2,g_nRighSpeed,5);	
}
void SpeedControl(void)
{
  //	static float fSpeedErrorDot = 0;
  CalSpeedError();
  // g_fSpeedError = (g_fSpeedError>800?800:g_fSpeedError);
  //	if(Flag_OpenStart == 1){g_fSpeedError = (g_fSpeedError>1300?1300:g_fSpeedError);}//起跑速度偏差限幅
  //	else g_fSpeedError = (g_fSpeedError>800?800:g_fSpeedError);//速度偏差限幅
  //	//动态期望速度
  //	g_fExpectSpeed = 2200 + g_fSpeedError/6;					//超2.9减速
  //	g_fExpectSpeed = (g_fExpectSpeed>2800?2800:g_fExpectSpeed);
  //	g_fExpectSpeed = (g_fExpectSpeed<1800?1800:g_fExpectSpeed);
  
  fSpeedErrorInteg += (g_speedControl_I/100.0) * g_fSpeedError;
  //if(Flag_Stop==1|Flag_Speed==1)fSpeedErrorInteg = 0;		//停车积分清零
  
  fSpeedErrorInteg = (fSpeedErrorInteg < 0.0? 0.0: fSpeedErrorInteg);//积分下限
  fSpeedErrorInteg = (fSpeedErrorInteg > 50.0? 50.0: fSpeedErrorInteg);//积分上限
  
  g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = (g_speedControl_P/100.0)*g_fSpeedError +fSpeedErrorInteg+ g_speedControl_D*(g_fSpeedErrorTemp[0]-g_fSpeedErrorTemp[1]);
  // g_fSpeedControlOutNew = (g_fSpeedControlOutNew>800?800:g_fSpeedControlOutNew);
}
//速度平滑输出
void SpeedControlOut(void)
{
  g_fSpeedControlOut = (g_fSpeedControlOutNew - g_fSpeedControlOutOld)*Flag_SpeedControl/20 + g_fSpeedControlOutOld;  
}
//确定机械直立角度
//void balance()
//{
//	  static float fValue=0;
//  fValue = angle-135;//  fValue减小 向前
//  balanceControlOut = (fValue)*balance_Kp + angle_dot*balance_Kd; 
//}
//结束
void DirectionControl(void) 
{
  int16 nLeft, nRight;
  // static int16 Ke[5];
  AD_chuli();
  g_fLeftVoltageSigma = zuozong;
  g_fRightVoltageSigma = youzong;
  
  nLeft = (int16)(g_fLeftVoltageSigma );     
  nRight = (int16)(g_fRightVoltageSigma );
  
  dangqian_he=nLeft+ nRight;
  
  g_fLeftVoltageSigma = 0;
  g_fRightVoltageSigma = 0;
  
  
  if(( nLeft>10||nRight>10))
  {
    fLeftRightAdd = nLeft+ nRight;
    fLeftRightSub = dangqian_chazhi;
    
    if( canshu[2]==1)
    { 
      if((nLeft>=40)&&( nRight>=40))
      {
        canshu[2]=0;
      }
      if(fLeftRightAdd==0)fLeftRightAdd+=1;
      KE= ( canshu[1] *200 )/ fLeftRightAdd;
      
    }
    else if(fLeftRightSub<fLeftRightAdd )  
    { 
      KE= fLeftRightSub;
      canshu[0] =c;
      canshu[1] =fLeftRightSub;
      
    } 
    else
    {
      canshu[2] =1;
      
      KE= ( canshu[1] *200 )/ fLeftRightAdd+1;
    }
    if(KE>=200) KE=200;
    if(KE<=0) KE=0;
    
    if(KE<300) 
    {
      D_1=KE/10;
      D_3=D_2+D_1;
    }
    
    
    D_now= D_last- KE;
    
    if(c==0)
    {
      fValue = KE * DIR_CONTROL_P+ DIR_CONTROL_D* D_now;
      fValue_last=fValue;
    }
    else
    {
      
      fValue = KE * DIR_CONTROL_P+DIR_CONTROL_D* D_now;
      fValue_last=fValue;
      fValue=- fValue;
    }  
    
    D_last2=shu_KE;
    D_last=KE;
    D_2=D_1;
    nLeft_last=nLeft;
    nRight_last=nRight;
    zuo_shuzong_last=zuo_shuzong;
    you_shuzong_last=you_shuzong;
    
    fLeftRightSub_last =fLeftRightSub;
    if(fValue > 9000) fValue = 9000;
    if(fValue < -9000) fValue = -9000;
    
//    if((zuozong<=50||youzong<=50)) 
//    {
//       Dir2P=70;
//       DIR_CONTROL_P=Dir2P;
//    }
  } 
  
  g_fDirectionControlOut =fValue;
  
}
void PWMOut(void)
{
	float g_nLeftPWM, g_nRighPWM;
	  g_fSpeed=(g_nLeftSpeed+g_nRighSpeed)/2;
    g_fSpeedF+=g_fSpeed;
	g_PWMOut = 45*GyroControlOut;	
	 if(Direction_falg==0)
    {
      
      if(g_fDirectionControlOut>=0)
      {
        g_nLeftPWM = 10*g_PWMOut - 3*g_fDirectionControlOut;
        g_nRighPWM = 10*g_PWMOut + 1*g_fDirectionControlOut; 
      }
      if(g_fDirectionControlOut<0)
      {
        g_nLeftPWM = 10*g_PWMOut - 1*g_fDirectionControlOut;
        g_nRighPWM =10*g_PWMOut + 3*g_fDirectionControlOut;
      }
      
    }

  speedl_power=g_nLeftPWM;
  speedr_power=g_nRighPWM;
  if(0<=speedl_power) //左电机   前转 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
    {
      speedl_power = (speedl_power > 10000? 10000: speedl_power);
      pwm_duty(PWM2_P26, speedl_power);
      pwm_duty(PWM2_P27, 0);
    }
  else                //左电机   后转
    {
      speedl_power=0-speedl_power;
      speedl_power = (speedl_power > 10000? 10000: speedl_power);
      pwm_duty(PWM2_P26, 0);
      pwm_duty(PWM2_P27, speedl_power);
      
    }
    
  if(0<=speedr_power) //右电机   前转
    {
      speedr_power = (speedr_power > 10000? 10000: speedr_power);
      pwm_duty(PWM4_P45, 0);
      pwm_duty(PWM4_P46, speedr_power);
    }
    else                //右电机   后转
    {
      speedr_power=0-speedr_power;
      speedr_power = (speedr_power > 10000? 10000: speedr_power);
      pwm_duty(PWM4_P45, speedr_power);
      pwm_duty(PWM4_P46, 0);
    }	
}
void PCA_Isr() interrupt 7
{
	static uint8 flag_getmpu6050;
	static uint8 flag_balance;
	static uint8 flag_speed;
	static uint8 flag_DirectionControl;
	if(PCA0_GET_FLAG)
	{
		//清除中断标志位
		PCA_CLEAR_FLAG(PCA_0);	

		//重载计数器
		pca_reload_counter(PCA_0);
   
   if(++flag_getmpu6050>=1)
   {
    get_mpu6050();
    Get_Angle();
		GyroControl();
	 }
   if(++flag_balance>=2)
	 {
    AngleControl();
   // balance();
     PWMOut();
		 flag_balance=0;
	 }
   if(++flag_speed>=3)
	 {
    Flag_SpeedControl++;
    if(Flag_SpeedControl >= 20)
    {
      SpeedControl();
      Flag_SpeedControl = 0;
    }
    SpeedControlOut();
		flag_speed=0;

	 }
		if(++flag_DirectionControl>=4)
		{
    AD_get();
  //  DirectionControl(); 
			flag_DirectionControl=0;
		}
	
}
		}

void main()
{

	DisableGlobalIRQ();	//关闭总中断

	board_init();		//初始化内部寄存器
  NVIC_SetPriority(UART3_IRQn, 3);
	NVIC_SetPriority(PWM2_IRQn, 0);
	NVIC_SetPriority(IIC_IRQn, 2);
	NVIC_SetPriority(CCP_PCA_PWM_IRQn,1);
	adc_init(ADC_P11, 0);	//初始化ADC,P1.0通道 ，ADC时钟频率：SYSclk/2/1
	adc_init(ADC_P12, 0);	//初始化ADC,P1.1通道 ，ADC时钟频率：SYSclk/2/1
	
	oled_init();
	iap_init();

  ctimer_count_init(CTIM3_P04);	
	ctimer_count_init(CTIM4_P06);

	AD_max_min();

	
	pwm_init(PWM2_P26, 15000, 0);//左前
	pwm_init(PWM2_P27, 15000, 0);//左后
	pwm_init(PWM4_P46, 15000, 0);//右前
	pwm_init(PWM4_P45, 15000, 0);//右后
	simiic_init();
	mpu6050_init();
  pca_delay_ms(10);
  pca_init_interrupt_ms(PCA_0,1);	//使用PCA_0作为周期中断，时间1ms一次
	EnableGlobalIRQ();	//开启总中断
    while(1)
	{       
   P40=0;

		oled_p6x8str(30,0,"car");
		
    oled_printf_int32(0,1,Accel_Y,5);
		oled_printf_int32(0,2,angle,5);
		oled_printf_int32(0,3,zuozong,5);
		oled_printf_int32(0,4,youzong,5);
		oled_printf_int32(0,5,speedl_power,5);
		
    oled_printf_int32(0,6,speedr_power,5);
    oled_printf_int32(0,7,g_fDirectionControlOut,5);


		pca_delay_ms(10);
		if(mid==0)
		{
			DisableGlobalIRQ();	//关闭总中断
			AD_max_min();
			EnableGlobalIRQ();	//开启总中断
		}

   }


	}
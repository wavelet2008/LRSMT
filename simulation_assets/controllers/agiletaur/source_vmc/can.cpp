#include "include.h"
#include "can.h"
#include "gait_math.h"
#include "locomotion_header.h"
#if !RUN_WEBOTS
#include "delay.h"
#include "led_fc.h"
#endif
_LEG_MOTOR leg_motor[4];
char reset_err_flag = 0;
#if !RUN_WEBOTS
uint32_t can1_rx_id;
int can_rx_over[5];
int can_rx_cnt[5];
u8 canbuft1[8];
u8 canbufr1[8];
#if CAN_NART_SEL== DISABLE || CAN_FB_SYNC
int  CAN_SAFE_DELAY=0;//us  ��֤���ڵ���ճ�ʱС��11
#else
int  CAN_SAFE_DELAY=130;//us  ��֤���ڵ���ճ�ʱС��11
#endif
#define USE_ID_CHECK1 0

u32  slave_id1 = 99 ; 
float cnt_rst1=0;
int can1_rx_cnt;

static float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

static float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

static float Bytes2Float(unsigned char *bytes,int num)
{
    unsigned char cByte[24];
    int i;
    for (i=0;i<num;i++)
    {
		 cByte[num-1-i] = bytes[i];
    }   
    float pfValue=*(float*)&cByte;
    return  pfValue;
}

static void Float2Bytes(float pfValue,unsigned char* bytes)
{
  char* pchar=(char*)&pfValue;
  for(int i=0;i<sizeof(float);i++)
  {
    *bytes=*pchar;
     pchar++;
     bytes++;  
  }
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

static float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

		
void CAN_motor_init(void){
	char i;
	
	for(i=0;i<4;i++){
		leg_motor[i].connect=0;
		leg_motor[i].motor_en=0;
		//leg_motor[i].motor_mode=MOTOR_MODE_CURRENT;	
		leg_motor[i].motor_mode=MOTOR_MODE_T;	//  Ŀǰ��������ģʽ
		
		reset_current_cmd(i);
		
		leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=3;
		leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=25;
		
		#if defined(MOCO_ML_LST)
			leg_motor[0].q_reset[1]=2;//�����ƫ��λ�Ƕ�  +-
			leg_motor[0].q_reset[0]=70;//                0~360

			leg_motor[1].q_reset[1]=120;//�����ƫ��λ�Ƕ�
			leg_motor[1].q_reset[0]=-175;

			leg_motor[2].q_reset[1]=2;//�����ƫ��λ�Ƕ�
			leg_motor[2].q_reset[0]=70;

			leg_motor[3].q_reset[1]=120;//�����ƫ��λ�Ƕ�
			leg_motor[3].q_reset[0]=-175;					
		
		#else
			leg_motor[0].q_reset[1]=350;//�����ƫ��λ�Ƕ�  +-
			leg_motor[0].q_reset[0]=66;//                0~360

			leg_motor[1].q_reset[1]=131;//�����ƫ��λ�Ƕ�g
			leg_motor[1].q_reset[0]=-166;

			leg_motor[2].q_reset[1]=350;//�����ƫ��λ�Ƕ�
			leg_motor[2].q_reset[0]=66;

			leg_motor[3].q_reset[1]=131;//�����ƫ��λ�Ƕ�g
			leg_motor[3].q_reset[0]=-166;			
		#endif			

	}
}

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	  
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
		#if !CAN_ABOM_E
		CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
		#else
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//����Զ����߹���	  
		#endif
  	CAN_InitStructure.CAN_AWUM=ENABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=CAN_NART_SEL;//DISABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
	//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
 	#if CAN_ABOM_E
		CAN_ITConfig(CAN1,CAN_IT_ERR,DISABLE);
	#endif
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//������жϱ�־
	CAN_ClearITPendingBit(CAN1, CAN_IT_TME);//�巢���жϱ�־  
	return 0;
}   

float can_dt[4]={0};
void CAN1_RX0_IRQHandler(void)
{
	static int cnt_rx,led_flag;
	char can_node_id=0;
	char temp;
	CanRxMsg RxMessage;
	int i=0,id=0;
	CAN_Receive(CAN1, 0, &RxMessage);
	cnt_rst1=0;
	if(cnt_rx++>500&&1){cnt_rx=0;
	led_flag=!led_flag;
	//LEDRGB_RED(led_flag);
	}
	for(i=0;i<8;i++)
		canbufr1[i]=0x00;
	for(i=0;i<RxMessage.DLC;i++)
		canbufr1[i]=RxMessage.Data[i];
	can1_rx_id=RxMessage.StdId;	
	
	if(RxMessage.DLC==8){
			 //LEG FR
		 if ( RxMessage.StdId == CAN_FB_POS_HEAD+0 ) 
			{
				can_dt[0] = Get_Cycle_T(10); 
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				can_rx_cnt[id]=*(canbufr1+3);//leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				can_rx_over[id]=*(canbufr1+7);//leg_motor[id].temp[1]=*(canbufr1+7);
				leg_motor[id].can_bus_id=1;
			}
			//LEG HR
			else if ( RxMessage.StdId == CAN_FB_POS_HEAD+1 ) 
			{ can_dt[1] = Get_Cycle_T(11); 
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}			
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				can_rx_cnt[id]=*(canbufr1+3);//leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				can_rx_over[id]=*(canbufr1+7);//leg_motor[id].temp[1]=*(canbufr1+7);
				leg_motor[id].can_bus_id=1;
			}
				 //LEG FL
		 else if ( RxMessage.StdId == CAN_FB_POS_HEAD+2 ) 
			{can_dt[2] = Get_Cycle_T(12); 
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}			
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				can_rx_cnt[id]=*(canbufr1+3);//leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				can_rx_over[id]=*(canbufr1+7);//leg_motor[id].temp[1]=*(canbufr1+7);
				leg_motor[id].can_bus_id=1;
			}
			//LEG HL
			else if ( RxMessage.StdId == CAN_FB_POS_HEAD+3) 
			{can_dt[3] = Get_Cycle_T(13); 
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+3 ) 
			{
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr1+0)<<8)|*(canbufr1+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr1+2)<<8)|*(canbufr1+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr1+4)<<8)|*(canbufr1+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr1+6)<<8)|*(canbufr1+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=1;
			}			
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+3 ) 
			{
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr1+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr1+1);
				leg_motor[id].bat_v[0]=*(canbufr1+2);
				can_rx_cnt[id]=*(canbufr1+3);//leg_motor[id].temp[0]=*(canbufr1+3);
				temp=*(canbufr1+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr1+5);
				leg_motor[id].bat_v[1]=*(canbufr1+6);
				can_rx_over[id]=*(canbufr1+7);//leg_motor[id].temp[1]=*(canbufr1+7);
				leg_motor[id].can_bus_id=1;
			}
		}
	can1_rx_cnt++;
		
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//������жϱ�־
	CAN_ClearITPendingBit(CAN1, CAN_IT_TME);//�巢���жϱ�־  
}


u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x00;//0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
			return 1;
  return 0;		//good

}

u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	static int cnt_rx,led_flag;
	char can_node_id=0;
	char temp;
	int id=0;
	CAN_Receive(CAN1, 0, &RxMessage);
	cnt_rst1=0;
	
	if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i]; 
	can1_rx_id=RxMessage.StdId;	
	

	can1_rx_cnt++;
	return RxMessage.DLC;	
}


uint32_t can2_rx_id;
u8 canbuft2[8];
u8 canbufr2[8];

#define USE_ID_CHECK2 0
u32  slave_id2 = 99 ; 
float cnt_rst2;
int can2_rx_cnt;
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
		#if !CAN_ABOM_E
		CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
		#else
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//����Զ����߹���	  
		#endif
  	CAN_InitStructure.CAN_AWUM=ENABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=CAN_NART_SEL;//DISABLE	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN1 
    
	//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN2_RX0_INT_ENABLE
	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

	#if CAN_ABOM_E
		CAN_ITConfig(CAN2,CAN_IT_ERR,DISABLE);
	#endif
	return 0;
}   
 
#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN2_RX0_IRQHandler(void)
{
	static int cnt_rx,led_flag,id=0;
	CanRxMsg RxMessage;
	int i=0;
	char temp;
	CAN_Receive(CAN2, 0, &RxMessage);
	cnt_rst2=0;
	if(cnt_rx++>500&&1){cnt_rx=0;
	led_flag=!led_flag;
	//LEDRGB_BLUE(led_flag);
	}
	for(i=0;i<8;i++)
		canbufr2[i]=0x00;
	for(i=0;i<RxMessage.DLC;i++)
		canbufr2[i]=RxMessage.Data[i];
	can2_rx_id=RxMessage.StdId;	
	if(RxMessage.DLC==8){
				 //LEG FR
		 if ( RxMessage.StdId == CAN_FB_POS_HEAD+0 ) 
			{
				can_dt[0] = Get_Cycle_T(10); 	
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+0 ) 
			{
				id=0;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				can_rx_cnt[id]=*(canbufr2+3);//leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				can_rx_over[id]=*(canbufr2+7);//leg_motor[id].temp[1]=*(canbufr2+7);
				leg_motor[id].can_bus_id=2;
			}
			//LEG HR
			else if ( RxMessage.StdId == CAN_FB_POS_HEAD+1 ) 
			{
				can_dt[1] = Get_Cycle_T(11); 
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+1 ) 
			{
				id=1;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				can_rx_cnt[id]=*(canbufr2+3);//leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				can_rx_over[id]=*(canbufr2+7);//leg_motor[id].temp[1]=*(canbufr2+7);
				leg_motor[id].can_bus_id=2;
			}
				 //LEG FL
		 else if ( RxMessage.StdId == CAN_FB_POS_HEAD+2 ) 
			{
				can_dt[2] = Get_Cycle_T(12); 
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+2 ) 
			{
				id=2;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				can_rx_cnt[id]=*(canbufr2+3);//leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				can_rx_over[id]=*(canbufr2+7);//leg_motor[id].temp[1]=*(canbufr2+7);
				leg_motor[id].can_bus_id=2;
			}
			//LEG HL
			else if ( RxMessage.StdId == CAN_FB_POS_HEAD+3) 
			{
				can_dt[3] = Get_Cycle_T(13); 
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].q_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_POS_DIV;
				leg_motor[id].q_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_POS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}
			else if ( RxMessage.StdId == CAN_FB_SPD_HEAD+3 ) 
			{	
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				leg_motor[id].qd_now[0]=(float)((int16_t)(*(canbufr2+0)<<8)|*(canbufr2+1))/CAN_DPOS_DIV;
				leg_motor[id].qd_now[1]=(float)((int16_t)(*(canbufr2+2)<<8)|*(canbufr2+3))/CAN_DPOS_DIV;
				leg_motor[id].t_now[0]=(float)((int16_t)(*(canbufr2+4)<<8)|*(canbufr2+5))/CAN_T_DIV;
				leg_motor[id].t_now[1]=(float)((int16_t)(*(canbufr2+6)<<8)|*(canbufr2+7))/CAN_T_DIV;
				leg_motor[id].can_bus_id=2;
			}			
			else if ( RxMessage.StdId == CAN_FB_STATE_HEAD+3 ) 
			{
				id=3;
				leg_motor[id].connect=1;
				leg_motor[id].loss_cnt=0;
				temp=*(canbufr2+0);
				leg_motor[id].connect_motor[0]=temp%10;
				leg_motor[id].ready[0]=temp/10;
				leg_motor[id].err_flag[0]=*(canbufr2+1);
				leg_motor[id].bat_v[0]=*(canbufr2+2);
				can_rx_cnt[id]=*(canbufr2+3);//leg_motor[id].temp[0]=*(canbufr2+3);
				temp=*(canbufr2+4);
				leg_motor[id].connect_motor[1]=temp%10;
				leg_motor[id].ready[1]=temp/10;
				leg_motor[id].err_flag[1]=*(canbufr2+5);
				leg_motor[id].bat_v[1]=*(canbufr2+6);
				can_rx_over[id]=*(canbufr2+7);//leg_motor[id].temp[1]=*(canbufr2+7);
				leg_motor[id].can_bus_id=2;
			}
	}
	can2_rx_cnt++;
}
#endif


u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x00;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
			return 1;
  return 0;		//good

}

u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	  can2_rx_id=RxMessage.StdId;	
	return RxMessage.DLC;	
}
//----------------------------------------------------------------------------------------------------------------------
void reset_current_cmd(char id)//�������
{
	leg_motor[id].set_t[0]=leg_motor[id].set_t[1]=leg_motor[id].set_t[2]=0;
	leg_motor[id].set_i[0]=leg_motor[id].set_i[1]=leg_motor[id].set_i[2]=0;
}


void CAN_set_pos_force_mit(char id){//mit �汾 ���ͽǶ�
	char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	leg_motor[id].set_t[0]=LIMIT(leg_motor[id].set_t[0],-leg_motor[id].max_t[0],leg_motor[id].max_t[0]);
	leg_motor[id].set_t[1]=LIMIT(leg_motor[id].set_t[1],-leg_motor[id].max_t[1],leg_motor[id].max_t[1]);
	
	_temp=To_180_degrees(robotwb.Leg[id].tar_sita[0])*CAN_POS_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	if(leg_motor[id].ready[0])
	_temp=leg_motor[id].set_t[0]*CAN_T_DIV;
	else
	_temp=0;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	
	_temp=To_180_degrees(robotwb.Leg[id].tar_sita[1])*CAN_POS_DIV;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	if(leg_motor[id].ready[1])
	_temp=leg_motor[id].set_t[1]*CAN_T_DIV;
	else
	_temp=0;		
	canbuft1[6]=BYTE1(_temp);
	canbuft1[7]=BYTE0(_temp);
	
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,CAN_SD_MIT_HEAD+id);
	else
	res=CAN2_Send_Msg(canbuft1,8,CAN_SD_MIT_HEAD+id);		
	
}

void CAN_set_pos_force_mit_zero(char id){//mit �汾 0����
	char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	
	_temp=To_180_degrees(robotwb.Leg[id].tar_sita[0])*CAN_POS_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=0;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=To_180_degrees(robotwb.Leg[id].tar_sita[1])*CAN_POS_DIV;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	_temp=0;
	canbuft1[6]=BYTE1(_temp);
	canbuft1[7]=BYTE0(_temp);
	
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,CAN_SD_MIT_HEAD+id);
	else
	res=CAN2_Send_Msg(canbuft1,8,CAN_SD_MIT_HEAD+id);
}


void CAN_set_torque(char id){//����Ť�� Ŀǰʹ��
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	leg_motor[id].set_t[0]=LIMIT(leg_motor[id].set_t[0],-leg_motor[id].max_t[0],leg_motor[id].max_t[0]);
	leg_motor[id].set_t[1]=LIMIT(leg_motor[id].set_t[1],-leg_motor[id].max_t[1],leg_motor[id].max_t[1]);

	_temp=leg_motor[id].set_t[0]*CAN_T_DIV*leg_motor[id].ready[0];
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].set_t[1]*CAN_T_DIV*leg_motor[id].ready[1];
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];
	canbuft1[5]=leg_motor[id].motor_en;
	canbuft1[6]=leg_motor[id].reset_q;
	canbuft1[7]=reset_err_flag;
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,CAN_SD_T_HEAD+id);
	else
	res=CAN2_Send_Msg(canbuft1,8,CAN_SD_T_HEAD+id);		
}

void CAN_set_torque_param(char id){//����Ť��ϵ�� PD�ն�
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};

	_temp=leg_motor[id].t_to_i[0]*CAN_T_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	
	if(leg_motor[id].motor_en&&leg_motor[id].ready[0]&&leg_motor[id].ready[1])
		_temp=robotwb.Leg[id].q_pid.kp*CAN_GAIN_DIV_P;
	else
		_temp=0;	
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	
	if(leg_motor[id].motor_en&&leg_motor[id].ready[0]&&leg_motor[id].ready[1])
		_temp=robotwb.Leg[id].q_pid.kd*CAN_GAIN_DIV_D;
	else
		_temp=0;	
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	
	canbuft1[6]=leg_motor[id].max_i[0];
	canbuft1[7]=leg_motor[id].motor_en*100+leg_motor[id].reset_q*10+reset_err_flag;

	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,CAN_SD_PARAM_T_HEAD+id);
	else
	res=CAN2_Send_Msg(canbuft1,8,CAN_SD_PARAM_T_HEAD+id);		
}

void CAN_set_zero_off(char id){//������λ
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	
	_temp=leg_motor[id].q_reset[0]*CAN_POS_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_reset[1]*CAN_POS_DIV;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];
	canbuft1[5]=leg_motor[id].motor_en;//en
	canbuft1[6]=leg_motor[id].reset_q;
	canbuft1[7]=reset_err_flag;
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,CAN_SD_PARAM_SYS_HEAD+id);
	else
	res=CAN2_Send_Msg(canbuft1,8,CAN_SD_PARAM_SYS_HEAD+id);		
}

void CAN_set_current(char id){//��������
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
	leg_motor[id].set_i[0]=LIMIT(leg_motor[id].set_i[0],-leg_motor[id].max_i[0],leg_motor[id].max_i[0]);
	leg_motor[id].set_i[1]=LIMIT(leg_motor[id].set_i[1],-leg_motor[id].max_i[1],leg_motor[id].max_i[1]);

	_temp=leg_motor[id].set_i[0]*CAN_I_DIV*leg_motor[id].ready[0];
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].set_i[1]*CAN_I_DIV*leg_motor[id].ready[1];
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];
	canbuft1[5]=leg_motor[id].motor_en;
	canbuft1[6]=leg_motor[id].reset_q;
	canbuft1[7]=0;
	if(leg_motor[id].can_bus_id==1)
	res=CAN1_Send_Msg(canbuft1,8,CAN_SD_I_HEAD+id);	
	else
	res=CAN2_Send_Msg(canbuft1,8,CAN_SD_I_HEAD+id);	
}

void CAN_set_current_zero(char id){//�����ط���
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};

	_temp=0;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=0;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);

	canbuft1[4]=leg_motor[id].max_i[0];
	canbuft1[5]=leg_motor[id].motor_en;
	canbuft1[6]=leg_motor[id].reset_q;
	canbuft1[7]=0;

	res=CAN2_Send_Msg(canbuft1,8,CAN_SD_T_HEAD+id);	
}

void CAN_reset_q(char id){//������λ�Ƕ�
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].q_reset[0]*CAN_POS_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_reset[1]*CAN_POS_DIV;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].q_reset[2]*CAN_POS_DIV;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA3);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB3);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA3);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB3);	
	break;
	}
}

void CAN_Broad(char id){//�㲥
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].q_set[0]*CAN_POS_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[1]*CAN_POS_DIV;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[2]*CAN_POS_DIV;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA6);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB6);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA6);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB6);	
	break;
	}
}

void CAN_set_q(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].q_set[0]*CAN_POS_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[1]*CAN_POS_DIV;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].q_set[2]*CAN_POS_DIV;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA2);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB2);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA2);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB2);	
	break;
	}
}


void CAN_set_force(char id){
  char res=0;
	vs16 _temp;
	u8 canbuft1[8]={0};
  
	_temp=leg_motor[id].f_set[0]*CAN_F_DIV;
	canbuft1[0]=BYTE1(_temp);
	canbuft1[1]=BYTE0(_temp);
	_temp=leg_motor[id].f_set[1]*CAN_F_DIV;
	canbuft1[2]=BYTE1(_temp);
	canbuft1[3]=BYTE0(_temp);
	_temp=leg_motor[id].f_set[2]*CAN_F_DIV;
	canbuft1[4]=BYTE1(_temp);
	canbuft1[5]=BYTE0(_temp);
	canbuft1[6]=0;
	canbuft1[7]=leg_motor[id].motor_en;
	switch(id){
	case 0:
	res=CAN1_Send_Msg(canbuft1,8,0xA4);	
	break;
	case 1:
	res=CAN1_Send_Msg(canbuft1,8,0xB4);	
	break;
	case 2:
	res=CAN2_Send_Msg(canbuft1,8,0xA4);	
	break;
	case 3:
	res=CAN2_Send_Msg(canbuft1,8,0xB4);	
	break;
	}
}


u8 CAN1_Send_Msg_Board(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x00;//0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=0;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=0;				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
			return 1;
  return 0;		//good
}

u8 CAN2_Send_Msg_Board(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x00;//0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=0;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=0;				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
			return 1;
  return 0;		//good
}

void CAN_get_fb(char can_sel){//ң�ط�����ȡ
  char res=0;
	vs16 _temp;
	switch(can_sel){
	case 1:
	res=CAN1_Send_Msg_Board(canbuft1,1,CAN_FB_REMOTE_FB1);	
	break;
	case 2:
	res=CAN2_Send_Msg_Board(canbuft2,1,CAN_FB_REMOTE_FB1);	
	break;
	}
}

char id_send[4]={0,0,0,0};
char flag_delay_pass[4]={1,0,1,0};
	
void CAN_motor_sm(float dt)
{
	static int cnt_[4]={0};
	static float timer_[4]={0};
	static int state_[4]={0};
	static int send_flag[4]={0};
	char i=0;

	char id_cnt=0;
	
	#if CAN_FB_SYNC//��ֵ
	
	
	#endif
	
	for(i=0;i<4;i++)
	{
		id_send[i]=0;
		//flag_delay_pass[i]=0;
	}
	
	//1
	for(i=0;i<4;i++)
	{
		if(leg_motor[i].can_bus_id==1){
				//flag_delay_pass[id_cnt]=1;
				id_send[id_cnt++]=i;	
				break;
		}
	}
	//1
	for(i=0;i<4;i++)
	{
		if(leg_motor[i].can_bus_id==2){
				//flag_delay_pass[id_cnt]=0;
				id_send[id_cnt++]=i;	
				break;
		}
	}
	//2
	for(i=0;i<4;i++)
	{
		if(leg_motor[i].can_bus_id==1&&i!=id_send[0]){
				//flag_delay_pass[id_cnt]=1;
				id_send[id_cnt++]=i;	
				break;
		}
	}	
	//3
	for(i=0;i<4;i++)
	{
		if(leg_motor[i].can_bus_id==2&&i!=id_send[1]){
				//flag_delay_pass[id_cnt]=0;
				id_send[id_cnt++]=i;	
				break;
		}
	}	
	
	
	for(i=0;i<4;i++){
	switch(state_[id_send[i]]){
		case 0:
		#if defined(CAN_ANL_MIT_MODE)//debug mit

				timer_[id_send[i]]+=dt;
				if(timer_[id_send[i]]>0.3){//����Ť��ϵ�� ʹ�ܸ�λ
					timer_[id_send[i]]=0;
					if(send_flag[id_send[i]]==1){
						send_flag[id_send[i]]=0;
						CAN_set_torque_param(id_send[i]);
					}
					else{
						send_flag[id_send[i]]=1;
						CAN_set_zero_off(id_send[i]);//���͸�λ�Ƕ�
					}
				}else
					CAN_set_pos_force_mit_zero(id_send[i]);

				
				if(leg_motor[id_send[i]].connect)
				{
					cnt_[id_send[i]]=0;
					state_[id_send[i]]++;
				}
				
		#else
			timer_[id_send[i]]+=dt;
			if(timer_[id_send[i]]>0.1){//����Ť��ϵ�� ʹ�ܸ�λ
				timer_[id_send[i]]=0;
				if(send_flag[id_send[i]]==1){send_flag[id_send[i]]=0;
				CAN_set_torque_param(id_send[i]);}
				else{send_flag[id_send[i]]=1;
				CAN_set_zero_off(id_send[i]);//���͸�λ�Ƕ�
				}
			}else
				CAN_set_current_zero(id_send[i]);
				
			if(leg_motor[id_send[i]].connect)
			{
				cnt_[id_send[i]]=0;
				state_[id_send[i]]++;
			}
		#endif
		if(flag_delay_pass[id_send[i]]==1)
			delay_us(0);
		else
			delay_us(CAN_SAFE_DELAY);
		break;	
		case 1:	//------------------------------
			if(!leg_motor[id_send[i]].connect)
				state_[id_send[i]]=0;
			else{
				timer_[id_send[i]]+=dt;
				if(timer_[id_send[i]]>0.3){//����Ť��ϵ�� ʹ�ܸ�λ
					timer_[id_send[i]]=0;
					if(send_flag[id_send[i]]==1){
						send_flag[id_send[i]]=0;
						CAN_set_torque_param(id_send[i]);
					}
					else{
						send_flag[id_send[i]]=1;
						CAN_set_zero_off(id_send[i]);//���͸�λ�Ƕ�
					}
				}else{			
					if(leg_motor[id_send[i]].motor_mode==MOTOR_MODE_T){
					#if defined(CAN_ANL_MIT_MODE)
						CAN_set_pos_force_mit(id_send[i]);
					#else
						CAN_set_torque(id_send[i]);
					#endif
					}
					else if(leg_motor[id_send[i]].motor_mode==MOTOR_MODE_CURRENT)
						CAN_set_current(id_send[i]);
					else if(leg_motor[id_send[i]].motor_mode==MOTOR_MODE_POS)
						CAN_set_q(id_send[i]);		
				}
			}
						
			if(leg_motor[id_send[i]].motor_mode!=leg_motor[id_send[i]].motor_mode_reg&&1)
			{
				state_[id_send[i]]=0;
				leg_motor[id_send[i]].motor_en=0;
			}
		if(flag_delay_pass[id_send[i]]==1)
			delay_us(0);
		else
			delay_us(CAN_SAFE_DELAY);
		break;
		default:
			state_[id_send[i]]=0;
		break;
	}

	leg_motor[id_send[i]].motor_en_reg=leg_motor[id_send[i]].motor_en;
	leg_motor[id_send[i]].motor_mode_reg=leg_motor[id_send[i]].motor_mode;
	}
	
	#if CAN_FB_SYNC//ң�ػ�ȡ����
		CAN_get_fb(1);
		CAN_get_fb(2);
	#endif	
}
#endif




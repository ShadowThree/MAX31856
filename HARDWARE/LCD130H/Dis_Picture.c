

#include "Dis_Picture.h" 
										    

//////////////////////////////////////////////////////////////////////////////////	 
/******************************************************************************/	 
//ͼƬ��ʾ ��������	

//DevEBox  ��Խ����
//�Ա����̣�mcudev.taobao.com
//�Ա����̣�shop389957290.taobao.com	
/******************************************************************************/							  


//16λ ��ֱɨ��  �ҵ���  ��λ��ǰ
void Show_Picture(void)
{
	u32 i,j,k=0;
	
	u16 picH,picL;
	
	LCD_Clear(WHITE);//����
	
	Draw_Font16B(24,16,BLUE,"3: ͼƬ��ʾ����");	
	
	delay_ms(1000);    //��ʱ��ʾ
	
	LCD_Clear(WHITE);  //����
	
//	#if USE_HORIZONTAL==1	//ʹ�ú���	

//	{
//	
//		LCD_Set_Window(0,0,lcddev.width,lcddev.height);//����һ���Զ�������ʾ���򴰿�

//		LCD_WriteRAM_Prepare();     	//��ʼд��GRAM	
//						
//		for(i=0;i<lcddev.width;i++)
//		for(j=0;j<lcddev.height;j++)
//		{
//			picH=gImage_LCD280[k++];
//			picL=gImage_LCD280[k++];
//			
//			LCD_WR_DATA8(picH);  //д8λ��ʾ����
//			LCD_WR_DATA8(picL);
//			
//		}			
//	}	

//		
//	#else	
	
	{
		
		LCD_Set_Window(0,0,lcddev.width,lcddev.height);//����һ���Զ�������ʾ���򴰿�

		LCD_WriteRAM_Prepare();     	//��ʼд��GRAM	
						
		for(i=0;i<lcddev.height;i++)
		for(j=0;j<lcddev.width;j++)
		{
			picH=gImage_LCD280[k++];
			picL=gImage_LCD280[k++];
			
			LCD_WR_DATA8(picH);  //д8λ��ʾ����
			LCD_WR_DATA8(picL);
		}	
	}

	
//	#endif
	
	
	 delay_ms(1500);              	//��ʱ
	
} 





/*******************************************************************************/

//DevEBox  ��Խ����

//�Ա����̣�mcudev.taobao.com
//�Ա����̣�shop389957290.taobao.com
/*******************************************************************************/

















		  







#include "nrf24l01.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//NRF24L01 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/16 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
 
	 

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
struct spi_transaction nrf24l01_trans;
uint8_t nrf240_send_packet[256];
uint8_t nrf240_received_packet[256];
#define debugMsg printf
//初始化24L01的IO口
void NRF24L01_Init(void)
{
	spi1_init();
	spi_init_slaves();
	nrf24l01_trans.slave_idx = NRF24L01_SLAVE_IDX;
	nrf24l01_trans.cpol = SPICpolIdleHigh;
	nrf24l01_trans.cpha = SPICphaEdge2;
	nrf24l01_trans.dss = SPIDss8bit;
	nrf24l01_trans.select = SPISelectUnselect;
	nrf24l01_trans.output_buf = nrf240_send_packet;
	nrf24l01_trans.input_buf = nrf240_received_packet;
	nrf24l01_trans.status = SPITransDone;

	/*
extern bool_t spi_submit(struct spi_periph *p, struct spi_transaction *t);
extern void spi_slave_select(uint8_t slave);
extern void spi_slave_unselect(uint8_t slave);
extern bool_t spi_lock(struct spi_periph *p, uint8_t slave);
extern bool_t spi_resume(struct spi_periph *p, uint8_t slave);
*/
/*
	RCC->APB2ENR|=1<<2;    //使能PORTA口时钟 
	RCC->APB2ENR|=1<<4;    //使能PORTC口时钟 
	GPIOA->CRL&=0XFFF000FF;//PA4输出
	GPIOA->CRL|=0X00033300; 
	GPIOA->ODR|=7<<2;	   //PA2.3.4 输出1		 
	GPIOC->CRL&=0XFF00FFFF;//PC4输出 PC5输出
	GPIOC->CRL|=0X00830000; 
	GPIOC->ODR|=3<<4;	   //上拉	 
	SPIx_Init();    //初始化SPI
	NRF24L01_CE=0; 	//使能24L01
	NRF24L01_CSN=1;	//SPI片选取消		  		 		  
*/

}

uint8_t is_spi_busy()
{
	if( nrf24l01_trans.status <= SPITransRunning )
		return 1;
	return 0;
}
uint8_t wait_spi_no_busy()
{
	int ms=0;
	while( is_spi_busy() ){
		delay_ms(1);
		debugMsg("spi busy , wait %d ms",++ms);	
	}
	return 1;
}
 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	debugMsg("write reg start\n\r");
	wait_spi_no_busy();

	nrf24l01_trans.output_length = 2;
	nrf24l01_trans.input_length = 0;
	nrf24l01_trans.output_buf[0] = reg;
	nrf24l01_trans.output_buf[1] = value;

  	spi_submit(&NRF24L01_SPI_DEV, &nrf24l01_trans);

	debugMsg("write reg end\n\r");
	wait_spi_no_busy();

	return nrf24l01_trans.status;

	/*
	uint8_t status;	
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status =SPIx_ReadWriteByte(reg);//发送寄存器号 
  	SPIx_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN=1;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
	*/
}
//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
	debugMsg("read reg start\n\r");
	wait_spi_no_busy();

	nrf24l01_trans.output_length = 1;
	nrf24l01_trans.input_length = 1;
	nrf24l01_trans.output_buf[0] = reg;

  	spi_submit(&NRF24L01_SPI_DEV, &nrf24l01_trans);

	debugMsg("read reg end\n\r");
	wait_spi_no_busy();
	debugMsg("read over\n\r");
	//return nrf24l01_trans.input_buf[0];
	return nrf24l01_trans.status;

/*
	uint8_t reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
  	SPIx_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=SPIx_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN = 1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
*/
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t reg_val;	    
	debugMsg("read reg start\n\r");
	wait_spi_no_busy();

	nrf24l01_trans.output_length = 1;
	nrf24l01_trans.input_length = len;
	nrf24l01_trans.output_buf[0] = reg;

  	spi_submit(&NRF24L01_SPI_DEV, &nrf24l01_trans);

	debugMsg("read reg end\n\r");
	wait_spi_no_busy();

	debugMsg("read over\n\r");
	for( int i = 0 ; i< len; i++) pBuf[i]= nrf24l01_trans.input_buf[i];
	return nrf24l01_trans.status;
	/*
	uint8_t status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //使能SPI传输
  	status=SPIx_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPIx_ReadWriteByte(0XFF);//读出数据
  	NRF24L01_CSN=1;       //关闭SPI传输
  	return status;        //返回读到的状态值
	*/
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t reg_val;	    
	debugMsg("read reg start\n\r");
	wait_spi_no_busy();

	nrf24l01_trans.output_length = len+1;
	nrf24l01_trans.input_length = 0;
	nrf24l01_trans.output_buf[0] = reg;

	for( int i=0 ; i< len; i++ )
		nrf24l01_trans.output_buf[i+1] = pBuf[i];

  	spi_submit(&NRF24L01_SPI_DEV, &nrf24l01_trans);

	debugMsg("read reg end\n\r");
	wait_spi_no_busy();

	debugMsg("read over\n\r");

	return nrf24l01_trans.status;
	/*
	uint8_t status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
  	status = SPIx_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPIx_ReadWriteByte(*pBuf++); //写入数据	 
  	NRF24L01_CSN = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
	*/
}				   


//检测24L01是否存在
//返回值:0，成功;1，失败	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t out_buf[5]={0};
	uint8_t i;

	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	debugMsg( "write buf result=%d\n\r",nrf24l01_trans.status); 
	NRF24L01_Read_Buf(TX_ADDR,out_buf,5); //读出写入的地址  
	debugMsg( "read buf result=%d\n\r",nrf24l01_trans.status); 
	debugMsg("check value:");
	for( i=0; i<5; i++) debugMsg("%02x,",out_buf[i]);
	debugMsg("\n\r");
	/*
	SPIx_SetSpeed(SPI_SPEED_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
	*/
}	
/*
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
 	SPIx_SetSpeed(SPI_SPEED_8);//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	NRF24L01_CE=0;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE=1;//启动发送	   
	while(NRF24L01_IRQ!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							   
	SPIx_SetSpeed(SPI_SPEED_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}					    
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void RX_Mode(void)
{
	NRF24L01_CE=0;	  
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  	NRF24L01_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	NRF24L01_CE = 1; //CE为高,进入接收模式 

	spi_slave_select(NRF24L01_SLAVE_IDX);
}						 
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void TX_Mode(void)
{														 
	NRF24L01_CE=0;	    
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);       //设置RF通道为40
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_CE=1;//CE为高,10us后启动发送

	spi_slave_select(NRF24L01_SLAVE_IDX);
}		  
*/




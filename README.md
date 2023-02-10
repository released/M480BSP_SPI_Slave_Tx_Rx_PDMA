# M480BSP_SPI_Slave_Tx_Rx_PDMA
 M480BSP_SPI_Slave_Tx_Rx_PDMA


update @ 2023/02/10

1. use M487 ETM EVB , to test SPI0/2 slave PDMA RX / TX function , use SPI1 as SPI master , to send data/clock

2. Initial SPI0 or SPI2 ( check define : ENABLE_SLAVE_SPI0 , ENABLE_SLAVE_SPI2 ) act as SPI slave 

	- SPI slave SPI0 : PA0(MOSI)/PA1(MISO)/PA2(CLK)/PA3(SS)
		
	- SPI slave SPI2 : PA8(MOSI)/PA9(MISO)/PA10(CLK)/PA11(SS)
		
	- SPI master SPI1 : PC0(SS)/PC1(CLK)/PC2(MOSI)/PC3(MISO)

3. SPI slave when detect CS pin level from SPI master , and trigger PDMA RX / TX function 

4. use terminal , digit 1 , to send SPI MASTER data

5. below is log message with MASTER send 16 bytes ( TX , RX ) , SLAVE receive (RX)

slave use previous receive data , for next TX data

![image](https://github.com/released/M480BSP_SPI_Slave_Tx_Rx_PDMA/blob/main/log_16bytes.jpg)
	
below is log message with MASTER send 64 bytes

![image](https://github.com/released/M480BSP_SPI_Slave_Tx_Rx_PDMA/blob/main/log_64bytes.jpg)	

5. below is LA capture 

![image](https://github.com/released/M480BSP_SPI_Slave_Tx_Rx_PDMA/blob/main/LA_1.jpg)	


![image](https://github.com/released/M480BSP_SPI_Slave_Tx_Rx_PDMA/blob/main/LA_2.jpg)	



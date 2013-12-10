/*
 * spiDriver.c
 *
 *  Created on: Dec 3, 2013
 *      Author: tsbiberdorf
 */




/*
*******************************************************************************
*                                                                             
*  Module Name : hal_spi0_k10.c       
*  Description : Provide common UART routines for serial IO
*		    				   
*******************************************************************************
*/


/*
*******************************************************************************
*                                FILE HISTORY                                   
*******************************************************************************
|     Date    |  By  |                 Description                            |     
*******************************************************************************      
*  28/05/2013   NGS   Created File.  
*									    
*******************************************************************************
*/

/*
*******************************************************************************
*                                INCLUDE FILES                                   
*******************************************************************************
*/

#include  "IO_Map.h"
#include "spiDriver.h"


/*
*******************************************************************************
*                          LOCAL DEFINES & CONSTANTS                                
*******************************************************************************
*/
	
#define BAUD_RATE_PRESCALER_2					(0U)
#define BAUD_RATE_PRESCALER_3					(1U)
#define BAUD_RATE_PRESCALER_5					(2U)
#define BAUD_RATE_PRESCALER_7					(3U)

#define DOUBLE_BAUD_RATE_0						(0U)
#define DOUBLE_BAUD_RATE_1						(1U)

#define BAUD_RATE_SCALER_2						(0U)
#define BAUD_RATE_SCALER_4						(1U)
#define BAUD_RATE_SCALER_8						(2U)
#define BAUD_RATE_SCALER_16						(3U)
#define BAUD_RATE_SCALER_32						(4U)
#define BAUD_RATE_SCALER_64						(5U)
#define BAUD_RATE_SCALER_128					(6U)
#define BAUD_RATE_SCALER_512					(7U)
#define BAUD_RATE_SCALER_1024					(8U)


/*
*******************************************************************************
*                         LOCAL VARIABLE DECLARATION
*******************************************************************************
*/

//None


/*
*******************************************************************************
*                         LOCAL FUNCTION DECLARATION
*******************************************************************************
*/


/*
*******************************************************************************
* void SpiClockEnable(SPI_MemMapPtr base)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base    		
* Output        : void
* Description   : Enable clock to respective SPI module
*******************************************************************************
*/
void SpiClockEnable(SPI_MemMapPtr base)
{
	if(SPI0_BASE_PTR == base)
	{
		//-->Enable clock to SPI0 module
		SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK;
	}
	if(SPI1_BASE_PTR == base)
	{
		//-->Enable clock to SPI0 module
		SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;
	}
	if(SPI2_BASE_PTR == base)
	{
		//
		//Enable clock to SPI0 module
		//
		SIM_SCGC3 |= SIM_SCGC3_SPI2_MASK;
	}
	
}//End of  SpiClockEnable

/*
*******************************************************************************
* void SpiNVICEnable(SPI_MemMapPtr base)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base    		
* Output        : void
* Description   : Enable NVIC interrupt for respective SPI module
*******************************************************************************
*/
void SpiNVICEnable(SPI_MemMapPtr base)
{
	if(SPI0_BASE_PTR == base)
	{
		set_irq_priority (INT_SPI0-16, 3);
		enable_irq(INT_SPI0-16) ;
	}
	if(SPI1_BASE_PTR == base)
	{
		set_irq_priority (INT_SPI1-16, 3);
		enable_irq(INT_SPI1-16) ;
	}
	if(SPI2_BASE_PTR == base)
	{
		set_irq_priority (INT_SPI2-16, 3);
		enable_irq(INT_SPI2-16) ;
	}
	
}//End of  SpiClockEnable


/*
*******************************************************************************
* void SpiSetMode( SPI_MemMapPtr base, uint8_t mode)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  mode : 1=> Master
* 				  		 0=> Slave    		
* Output        : void
* Description   : Enable Master or Slave mode for respective SPI module
*******************************************************************************
*/
void SpiSetMode( SPI_MemMapPtr base , uint8_t mode)
{
	if(SPI_MASTER_MODE == mode)
	{
		//-->SPI in Master mode 
		SPI_MCR_REG(base) |= (SPI_MASTER_MODE <<SPI_MCR_MSTR_SHIFT );
	}
	else
	{
		//-->SPI in Slave mode 
		SPI_MCR_REG(base) |= (SPI_SLAVE_MODE <<SPI_MCR_MSTR_SHIFT );
	}
	
}// End of SpiSetMode


/*
*******************************************************************************
* void SpiEnableReceiver( SPI_MemMapPtr base)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base    		
* Output        : void
* Description   : Enable Receiver for respective SPI module
*******************************************************************************
*/
void SpiEnableReceiver( SPI_MemMapPtr base)
{

	SPI_MCR_REG(base) |= (SPI_RECV_ENABLE <<SPI_MCR_ROOE_SHIFT );

}//End of SpiEnableREeceiver


/*
*******************************************************************************
* void SpiSetRxFifo( SPI_MemMapPtr base, uint8_t rx_fifo_enable)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 
* 				  rx_fifo_enable: 1=> FIFO disabled
* 				  				  0=> FIFO Enabled		
* Output        : void
* Description   : Enable or  disable Rx Fifo for respective SPI module
*******************************************************************************
*/
void SpiSetRxFifo( SPI_MemMapPtr base, uint8_t rx_fifo_enable)
{
	if(SPI_DISABLE_RXFIFO == rx_fifo_enable)
	{
		//--> Disable Rx FIFO
		SPI_MCR_REG(base) |=(SPI_DISABLE_RXFIFO <<SPI_MCR_DIS_RXF_SHIFT);
	}
	else
	{
		//--> Disable Rx FIFO
		SPI_MCR_REG(base) |=(SPI_ENABLE_RXFIFO <<SPI_MCR_DIS_RXF_SHIFT);
	}
		
}//End of SpiSetRxFifo

/*
*******************************************************************************
* void SpiSetTxFifo( SPI_MemMapPtr base, uint8_t tx_fifo_enable)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 
* 				  tx_fifo_enable: 1=> FIFO disabled
* 				  				  0=> FIFO Enabled		
* Output        : void
* Description   : Enable  or disable Tx Fifo for respective SPI module
*******************************************************************************
*/
void SpiSetTxFifo( SPI_MemMapPtr base, uint8_t tx_fifo_enable)
{
	if(SPI_DISABLE_RXFIFO == tx_fifo_enable)
	{
		//--> Disable Tx FIFO
		SPI_MCR_REG(base) |=(SPI_DISABLE_TXFIFO <<SPI_MCR_DIS_TXF_SHIFT);
	}
	else
	{
		//--> Disable Tx FIFO
		SPI_MCR_REG(base) |=(SPI_ENABLE_TXFIFO <<SPI_MCR_DIS_TXF_SHIFT);
	}
		
}//End of SpiSetTxFifo

/*
*******************************************************************************
* void SpiClrFifoCounter( SPI_MemMapPtr base)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* Output        : void
* Description   : Clear RX and TX FIFO counters for respective SPI module
*******************************************************************************
*/
void SpiClrFifoCounter( SPI_MemMapPtr base)
{
	//-->Clear RX and TX FIFO counters
	SPI_MCR_REG(base) |=(SPI_CLR_TXFIFO<<SPI_MCR_CLR_TXF_SHIFT);
	SPI_MCR_REG(base) |=(SPI_CLR_RXFIFO<<SPI_MCR_CLR_RXF_SHIFT);
	
}// End of  SpiClrFifoCounter


/*
*******************************************************************************
* void SpiSetPCS( SPI_MemMapPtr base , uint8_t peri_chip_sel)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
*				  peri_chip_sel: 0=> Low
*				  				 1=> High
* Output        : void
* Description   : Set Peripheral Chip Select for respective SPI module
*******************************************************************************
*/
void SpiSetPCS( SPI_MemMapPtr base, uint8_t peri_chip_sel)
{
	//-->Peripheral Chip Select is Inactive Low
	SPI_MCR_REG(base) |=SPI_MCR_PCSIS(peri_chip_sel);
	
}// End of SpiSetPCS

/*
*******************************************************************************
* void SpiDisable( SPI_MemMapPtr base );
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* Output        : void
* Description   : Disable respective SPI module
*******************************************************************************
*/
void SpiDisable( SPI_MemMapPtr base)
{
	//-->Disable SPI
	SPI_MCR_REG(base) |=SPI_DISABLE;
	
	SPI_MCR_REG(base) &= (~SPI_MCR_MDIS_MASK);

}// End of SpiDisable


/*
*******************************************************************************
* void SpiSetBaudRate(SPI_MemMapPtr base, uint8_t baudrate)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  baudrate : select one of 3 possible values i.e
* 				  			 500 kHZ, 250 kHz, 125Khz
* Output        : void
* Description   : SCK baud rate = (bus or peripheral clck /PBR) x [(1+DBR)/BR]
* 				  Note : The values have been defined for a bus clock of 48Mhz
* 				  		 If the bus clock changes the prescaler values will 
* 				  		 also be affected.
*******************************************************************************
*/
void SpiSetBaudRate(SPI_MemMapPtr base, uint8_t baudrate)
{

	switch(baudrate)
	{
		case BAUDRATE_500KHZ:
			SPI_CTAR_REG(base, 0)  |=  SPI_CTAR_PBR(BAUD_RATE_PRESCALER_3);
			SPI_CTAR_REG(base, 0)  |=  SPI_CTAR_BR(BAUD_RATE_SCALER_32);
			SPI_CTAR_REG(base, 0)  |= (DOUBLE_BAUD_RATE_0 << SPI_CTAR_DBR_SHIFT); 
			break;
			
		case BAUDRATE_250KHZ:
			SPI_CTAR_REG(base, 0)  |=  SPI_CTAR_PBR(BAUD_RATE_PRESCALER_3);
			SPI_CTAR_REG(base, 0)  |=  SPI_CTAR_BR(BAUD_RATE_SCALER_64);
			SPI_CTAR_REG(base, 0)  |= (DOUBLE_BAUD_RATE_0 << SPI_CTAR_DBR_SHIFT); 
			break;
			
		case BAUDRATE_125KHZ:
			SPI_CTAR_REG(base, 0)  |=  SPI_CTAR_PBR(BAUD_RATE_PRESCALER_3);
			SPI_CTAR_REG(base, 0)  |=  SPI_CTAR_BR(BAUD_RATE_SCALER_128);
			SPI_CTAR_REG(base, 0)  |= (DOUBLE_BAUD_RATE_0 << SPI_CTAR_DBR_SHIFT); 
			break;
	}
}// End of  SpiSetBaudRate


/*
*******************************************************************************
* void SpiSetClockPhase( SPI_MemMapPtr base , uint8_t clock_phase )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  clock_phase : 0=>Data is captured on the leading edge of SCK 
* 				                  and changed on the following edge.
								1=>Data is changed on the leading edge of SCK
								 and captured on the following edge.
* Output        : void
* Description   : Set Clock Phase for SPI
*******************************************************************************
*/
void SpiSetClockPhase( SPI_MemMapPtr base , uint8_t clock_phase)
{
    SPI_CTAR_REG(base,0) |= (clock_phase <<SPI_CTAR_CPHA_SHIFT );
	
}//End of SpiSetClockPhase


/*
*******************************************************************************
* void SpiSetClockPolarity( SPI_MemMapPtr base , uint8_t clock_polarity )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  clock_polarity: 0=> The inactive state value of SCK is low.
								  1=> The inactive state value of SCK is high.
* Output        : void
* Description   : Set Clock Polarity for SPI
*******************************************************************************
*/
void SpiSetClockPolarity( SPI_MemMapPtr base , uint8_t clock_polarity)
{
	SPI_CTAR_REG(base,0) |= (clock_polarity <<SPI_CTAR_CPOL_SHIFT );

}//End of SpiSetClockPolarity


/*
*******************************************************************************
* void SpiSetShiftPriority( SPI_MemMapPtr base , uint8_t shift_priority)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  shift_priority: 0=> Data is transferred MSB first.
								  1=>Data is transferred LSB first.
* Output        : void
* Description   : Set Shift priority for SPI
*******************************************************************************
*/
void SpiSetShiftPriority( SPI_MemMapPtr base , uint8_t shift_priority)
{
	SPI_CTAR_REG(base,0) |= (shift_priority<<SPI_CTAR_LSBFE_SHIFT );

}//End of SpiSetShiftPriority


/*
*******************************************************************************
* void SpiSetFrameSize( SPI_MemMapPtr base , uint8_t frame_size)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  frame_size : Number of bits transmitted -1
* Output        : void
* Description   : Set Frame Size for SPI
*******************************************************************************
*/
void SpiSetFrameSize( SPI_MemMapPtr base , uint8_t frame_size)
{
	SPI_CTAR_REG(base,0)= SPI_CTAR_FMSZ(frame_size);

}//End of SpiSetFrameSize

/*
*******************************************************************************
* void SpiSetDelayAfterTX( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  delay : Selects the scaler value for the After Transfer Delay.
* Output        : void
* Description   : Set After Transfer Delay for SPI
*******************************************************************************
*/
void SpiSetDelayAfterTX( SPI_MemMapPtr base , uint8_t delay)
{
	SPI_CTAR_REG(base,0) |= SPI_CTAR_DT(delay);

}//End of SpiSetDelayAfterTX


/*
*******************************************************************************
* void SpiSetDelayAfterSCK( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  delay : Selects the scaler value for the After SCK Delay.
* Output        : void
* Description   : Set After SCK Delay for SPI
*******************************************************************************
*/
void SpiSetDelayAfterSCK( SPI_MemMapPtr base , uint8_t delay)
{
	SPI_CTAR_REG(base,0) |= SPI_CTAR_ASC(delay);

}//End of SpiSetDelayAfterSCK

/*
*******************************************************************************
* void SpiSetDelayAfterCS( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  delay : Selects the scaler value for the Delay after CS.
* Output        : void
* Description   : Set After CS Delay for SPI
*******************************************************************************
*/
void SpiSetDelayAfterCS( SPI_MemMapPtr base , uint8_t delay)
{
	SPI_CTAR_REG(base,0) |= SPI_CTAR_CSSCK(delay);

}//End of SpiSetDelayAfterPCS

/*
*******************************************************************************
* void SpiSetDelayAfterPCSSCK( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  delay : Selects the prescaler value for the delay between 
* 				  		  assertion of PCS and the first edge of the SCK.
* Output        : void
* Description   : Set After PCS and SCK Delay for SPI
*******************************************************************************
*/
void SpiSetDelayAfterPCSSCK( SPI_MemMapPtr base , uint8_t delay)
{
	SPI_CTAR_REG(base,0) |= SPI_CTAR_PCSSCK(delay);

}//End of SpiSetDelayAfterPCSSCK


/*
*******************************************************************************
* void SpiSetDelayAfterPASC( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  delay : Selects the prescaler value for the delay between 
* 				  		  the last edge of SCK and the negation of PCS.
* Output        : void
* Description   : Set After SCK and PCS Delay for SPI
*******************************************************************************
*/
void SpiSetDelayAfterPASC( SPI_MemMapPtr base , uint8_t delay)
{
	SPI_CTAR_REG(base,0) |= SPI_CTAR_PASC(delay);

}//End of SpiSetDelayAfterPASC

/*
*******************************************************************************
* void SpiSetDelayAfterPDT( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* 				  delay : Selects the Delay after Transfer Prescaler value
* Output        : void
* Description   : Set delay after transfer prescaler value for SPI
*******************************************************************************
*/
void SpiSetDelayAfterPDT( SPI_MemMapPtr base , uint8_t delay)
{
	SPI_CTAR_REG(base,0) |= SPI_CTAR_PDT(delay);

}//End of SpiSetDelayAfterPDT


/*
*******************************************************************************
* void SpiEnable( SPI_MemMapPtr base )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* Output        : void
* Description   : Enable The SPI module
*******************************************************************************
*/
void SpiEnable( SPI_MemMapPtr base )
{
	SPI_MCR_REG(base) &= SPI_ENABLE; 

}//End of SpiEnable



/*
*******************************************************************************
*uint8_t Spi0MasteTxRxByte( SPI_MemMapPtr base, uint8_t tx_byte)
*******************************************************************************
* Input         : character to send
* Output        : received byte
* Description   : Wait for space in the SPI0 Tx FIFO and then send a character
* 				  Check if character is available in RX FIFO and return it
 ********************************************************************************
*/
uint8_t SpiMasterTxRxByte( SPI_MemMapPtr base, uint8_t tx_byte)
{
	uint8_t recv_byte = 0;


	//wait till TX FIFO is Empty.
	while((SPI_SR_REG(base)  & SPI_SR_TFFF_MASK) != TX_FIFO_EMPTY);
		
	// Transmit Byte on SPI 
	SPI_PUSHR_REG(base) = (uint32_t)tx_byte;

	//--> Wait till transmit complete
	while (((SPI_SR_REG(base) & SPI_SR_TCF_MASK)) != SPI_SR_TCF_MASK);

	//--> Clear Transmit Flag.
	SPI_SR_REG(base) |= (uint32_t)SPI_SR_TFFF_MASK;
	
	
	//wait till RX_FIFO is full
	while((SPI_SR_REG(base) & SPI_SR_RFDF_MASK) != RX_FIFO_EMPTY);

	//Read character from receiver
	recv_byte = (uint8_t)SPI_POPR_REG(base); 
	
	//-->Clear the RX FIFO Drain Flag
	SPI_SR_REG(base) |= (uint32_t)SPI_SR_RFDF_MASK;
	
	return(recv_byte);
}// End of Spi0MasteTxRxByte


/*
*******************************************************************************
* void SpiEnableTxInterrupt( SPI_MemMapPtr base )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* Output        : void
* Description   : Enable The SPI module TX interrupt
*******************************************************************************
*/
void SpiEnableTxInterrupt( SPI_MemMapPtr base )
{
	SPI_RSER_REG(base) |= SPI_RSER_TFFF_RE_MASK; 
	
}//End of SpiEnableTxInterrupt


/*
*******************************************************************************
* void SpiDisableTxInterrupt( SPI_MemMapPtr base )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* Output        : void
* Description   : Disable The SPI module TX interrupt
*******************************************************************************
*/
void SpiDisableTxInterrupt( SPI_MemMapPtr base )
{
	SPI_RSER_REG(base) &= (~SPI_RSER_TFFF_RE_MASK); 

}//End of SpiDisableTxInterrupt


/*
*******************************************************************************
* void SpiEnableRxInterrupt( SPI_MemMapPtr base )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* Output        : void
* Description   : Enable The SPI module RX interrupt
*******************************************************************************
*/
void SpiEnableRxInterrupt( SPI_MemMapPtr base )
{
	SPI_RSER_REG(base) |= SPI_RSER_RFDF_RE_MASK; 

}//End of SpiEnableRxInterrupt

/*
*******************************************************************************
* void SpiDisableRxInterrupt( SPI_MemMapPtr base )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base 	
* Output        : void
* Description   : Disable The SPI module RX interrupt
*******************************************************************************
*/
void SpiDisableRxInterrupt( SPI_MemMapPtr base )
{
	SPI_RSER_REG(base) &= (~SPI_RSER_RFDF_RE_MASK); 

}//End of SpiEnableRxInterrupt

/*-------------------------------END OF FILE----------------------------------*/

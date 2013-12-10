/*
 * spiDriver.h
 *
 *  Created on: Dec 3, 2013
 *      Author: tsbiberdorf
 */

#ifndef SPIDRIVER_H_
#define SPIDRIVER_H_




/*
*******************************************************************************
*                                                                             
*  Module Name : driver_spi_k10.h                                          
*										         
*  Description : Header file for driver_spi_k10.c
*						    				   
*******************************************************************************
*/


/*
*******************************************************************************
*                                FILE HISTORY                                   
*******************************************************************************
|     Date    |  By  |                 Description                            |     
*******************************************************************************      
*  	28/05/2013   NGS   Created File.  
*									    
*******************************************************************************
*/



/*
*******************************************************************************
*                                INCLUDE FILES                                   
*******************************************************************************
*/

#include "arm_cm4.h"
#include  "IO_Map.h"


#include "stdlib.h"
//#include "std_data_types.h"
//
//#include "port_cntrl.h"


/*
*******************************************************************************
*                          GLOBAL DEFINES & CONSTANTS                                
*******************************************************************************
*/

#define SPI_ENABLE			                    (0xFFFFBFFEU)
#define SPI_DISABLE			                    (1U)

#define SPI_CLR_RXFIFO		                    (1U)
#define SPI_CLR_TXFIFO		                    (1U)

#define SPI_DISABLE_RXFIFO	                    (1U)
#define SPI_DISABLE_TXFIFO  	                (1U)

#define SPI_ENABLE_RXFIFO	                    (0U)
#define SPI_ENABLE_TXFIFO  	                    (0U)

#define SPI_PERI_CHIP_SEL_INACTV_LOW            (0U)
#define SPI_PERI_CHIP_SEL_INACTV_HIGH           (1U)

#define SPI_RECV_ENABLE                         (1U)
#define SPI_RECV_DISABLE                        (0U)

#define SPI_MASTER_MODE		                    (1U)
#define SPI_SLAVE_MODE							(0U)

#define FRAME_SIZE8								(7U)
#define FRAME_SIZE16							(15U)

#define LSB_FIRST	           					(1U)
#define MSB_FIRST	           					(0U)

#define CLK_PHASE0           					(0U)
#define CLK_PHASE1            					(1U)

#define CLK_POLARITY0         					(0U)
#define CLK_POLARITY1         					(1U)

#define PCSSCK_PRESCALER_1			 			(0U)
#define PCSSCK_PRESCALER_3						(1U)
#define PCSSCK_PRESCALER_5						(2U)
#define PCSSCK_PRESCALER_7						(3U)

#define PASC_PRESCALER_1						(0U)
#define PASC_PRESCALER_3						(1U)
#define PASC_PRESCALER_5						(2U)
#define PASC_PRESCALER_7						(3U)

#define PDT_PRESCALER_1							(0U)
#define PDT_PRESCALER_3							(1U)
#define PDT_PRESCALER_5							(2U)
#define PDT_PRESCALER_7							(3U)

#define ASC_SCALER_1							(1U)
#define ASC_SCALER_2							(2U)
#define ASC_SCALER_3							(3U)
#define ASC_SCALER_4							(4U)
#define ASC_SCALER_5							(5U)
#define ASC_SCALER_6							(6U)

#define DT_SCALER_1								(1U)
#define DT_SCALER_2								(2U)
#define DT_SCALER_3								(3U)
#define DT_SCALER_4								(4U)
#define DT_SCALER_5								(5U)
#define DT_SCALER_6								(6U)
	
#define PCS_PRESCALER_2							(0U)
#define PCS_PRESCALER_4							(1U)
#define PCS_PRESCALER_8							(2U)
#define PCS_PRESCALER_16						(3U)
#define PCS_PRESCALER_32						(4U)
#define PCS_PRESCALER_64						(5U)


#define SPI_DATA_RECEIVED						(0x20000U)
#define SPI_TX_READY							(0x2000000U)

#define TX_FIFO_EMPTY					(0x2000000U)
#define RX_FIFO_EMPTY					(0x20000U)

#define SPI1_TX_TRANSMIT_OVER 0
#define SPI1_TX_TRANSMIT_ON 1

/*
*******************************************************************************
*                         GLOBAL VARIABLE DECLARATION
*******************************************************************************
*/

typedef struct
{
	uint8_t *data_buff_ptr;
	uint8_t *rxdata_buff_ptr;
	uint8_t data_len;
	uint8_t txdata_len;
	uint8_t rxdata_len;
	uint8_t data_buff_status;
	
}spi_tx_data_t;

typedef enum
{
	BAUDRATE_500KHZ,
	BAUDRATE_250KHZ,
	BAUDRATE_125KHZ
}SPI_BAUDRATE;

void Spi1MasterInit(SPI_MemMapPtr base,SPI_BAUDRATE sel_baudrate);

/*
*******************************************************************************
*                         GLOBAL FUNCTION DECLARATION
*******************************************************************************
*/
extern void SpiClockEnable(SPI_MemMapPtr base);
extern void SpiNVICEnable(SPI_MemMapPtr base);
extern void SpiSetMode( SPI_MemMapPtr base , uint8_t mode);
extern void SpiEnableReceiver(SPI_MemMapPtr base);
extern void SpiSetRxFifo( SPI_MemMapPtr base, uint8_t tx_fifo_enable);
extern void SpiSetTxFifo( SPI_MemMapPtr base, uint8_t tx_fifo_enable);
extern void SpiClrFifoCounter( SPI_MemMapPtr base);
extern void SpiSetPCS( SPI_MemMapPtr base, uint8_t peri_chip_sel);
extern void SpiDisable( SPI_MemMapPtr base);
extern void SpiSetBaudRate(SPI_MemMapPtr base, uint8_t baudrate);
extern void SpiSetClockPhase( SPI_MemMapPtr base , uint8_t clock_phase);
extern void SpiSetClockPolarity( SPI_MemMapPtr base , uint8_t clock_polarity);
extern void SpiSetShiftPriority( SPI_MemMapPtr base , uint8_t shift_priority);
extern void SpiSetFrameSize( SPI_MemMapPtr base , uint8_t frame_size);
extern void SpiSetDelayAfterTX( SPI_MemMapPtr base , uint8_t delay);
extern void SpiSetDelayAfterSCK( SPI_MemMapPtr base , uint8_t delay);
extern void SpiSetDelayAfterCS( SPI_MemMapPtr base , uint8_t delay);
extern void SpiSetDelayAfterPCSSCK( SPI_MemMapPtr base , uint8_t delay);
extern void SpiSetDelayAfterPASC( SPI_MemMapPtr base , uint8_t delay);
extern void SpiSetDelayAfterPDT( SPI_MemMapPtr base , uint8_t delay);
extern void SpiEnable( SPI_MemMapPtr base );

extern void SpiEnableTxInterrupt( SPI_MemMapPtr base );
extern void SpiDisableTxInterrupt( SPI_MemMapPtr base );
extern void SpiEnableRxInterrupt( SPI_MemMapPtr base );
extern void SpiDisableRxInterrupt( SPI_MemMapPtr base );

extern uint8_t SpiMasterTxRxByte( SPI_MemMapPtr base, uint8_t tx_byte);

/*-------------------------------END OF FILE----------------------------------*/

#endif /* SPIDRIVER_H_ */

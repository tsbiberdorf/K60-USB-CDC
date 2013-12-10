/*
 * Initialize.c
 *
 *  Created on: Nov 12, 2013
 *      Author: tsbiberdorf
 */

#include  "IO_Map.h"
#include "spiDriver.h"

#define SPI1_TX_BUFFER_EMPTY			(0U)
#define RX_CNTR_MASK	(0x000000F0) // RX counter mask is bits 7:4 of SPIx_SR register

/*
*******************************************************************************
*                         LOCAL VARIABLE DECLARATION
*******************************************************************************
*/

volatile static spi_tx_data_t spi2_tx_data_s = {0};

static void init_clock(void)
{
    SIM_SCGC3 |= SIM_SCGC3_SPI2_MASK;
}
static void init_io(void)
{
    PORTD_PCR11 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR11 |= PORT_PCR_MUX(2);
    PORTD_PCR12 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR12 |= PORT_PCR_MUX(2);
    PORTD_PCR13 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR13 |= PORT_PCR_MUX(2);
    PORTD_PCR14 &= ~PORT_PCR_MUX_MASK;
    PORTD_PCR14 |= PORT_PCR_MUX(2);
}
static void init_set_master(void)
{
    SPI2_MCR |= SPI_MCR_MSTR_MASK;
}
static void init_fifo(void)
{
    SPI2_MCR &= ~SPI_MCR_MDIS_MASK;

//    SPI2_MCR |= SPI_MCR_DIS_RXF_MASK |
//                SPI_MCR_DIS_TXF_MASK |
//                SPI_MCR_CLR_RXF_MASK |
//                SPI_MCR_CLR_TXF_MASK;

    SPI2_MCR |= SPI_MCR_CLR_RXF_MASK |
                SPI_MCR_CLR_TXF_MASK;
}
static void init_inactive_cs(void)
{
    SPI2_MCR |= SPI_MCR_PCSIS(1<<0);
}
static void init_inactive_clock(void)
{
    // must be low
}
static void init_frame_size(void)
{
    SPI2_CTAR0 &= ~SPI_CTAR_FMSZ_MASK;
    SPI2_CTAR0 |= SPI_CTAR_FMSZ(7);
}
static void init_clock_phase(void)
{
    // defalut = capture data on rising edge
}
static void init_baudrate(void)
{
    // default = sys clock/2/2 = 48/4 = 12M
}
static void init_msb_first(void)
{
    // default is msb first
}
static void spiTransferStart(void)
{
    SPI2_MCR &= ~SPI_MCR_HALT_MASK;
}
static void spiTransferStop(void)
{
    SPI2_MCR |= SPI_MCR_HALT_MASK;
}

uint8_t hal_spi_transfer_one_byte(uint8_t v, uint8_t end)
{
    if(end)
        SPI2_PUSHR = //SPI_PUSHR_CONT_MASK |
                     SPI_PUSHR_EOQ_MASK  |
                     SPI_PUSHR_PCS(1<<0) |
                     (v);
    else
        SPI2_PUSHR = SPI_PUSHR_CONT_MASK |
                     SPI_PUSHR_PCS(1<<0) |
                     (v);

    while((SPI2_SR & SPI_SR_TCF_MASK)==0)
        ;
    SPI2_SR |= SPI_SR_TCF_MASK;
    return SPI2_POPR&0xff;
}


void spiInit(void)
{
    init_clock();
    init_io();
    init_set_master();
    init_fifo();
    init_inactive_cs();
    init_inactive_clock();
    init_frame_size();
    init_clock_phase();
    init_msb_first();
    init_baudrate();
}



void Spi1MasterInit(SPI_MemMapPtr base,SPI_BAUDRATE sel_baudrate)
{		
	SpiClockEnable(base);
	
	SpiNVICEnable(base);
	
	SpiDisable(base);
	
	SpiSetMode(base, SPI_MASTER_MODE);
	
	SpiEnableReceiver(base);
	
	SpiSetRxFifo(base, SPI_DISABLE_RXFIFO);
	
	SpiSetTxFifo(base, SPI_DISABLE_TXFIFO);
	
	SpiClrFifoCounter(base);
	
//	SpiSetPCS(base,SPI_PERI_CHIP_SEL_INACTV_HIGH );
//	
//	SpiSetFrameSize(base,FRAME_SIZE8);
//		
//	SpiSetBaudRate(base, sel_baudrate);
//	
//	SpiSetClockPhase(base, CLK_PHASE0);
//	
//	SpiSetClockPolarity(base, CLK_POLARITY0);
//
//	SpiSetShiftPriority(base,MSB_FIRST);
//	
//	SpiSetDelayAfterTX(base, DT_SCALER_5);
//	
//	SpiSetDelayAfterSCK(base , ASC_SCALER_5);
//	
//	SpiSetDelayAfterCS(base , PCS_PRESCALER_64);
//	
//	SpiSetDelayAfterPCSSCK(base, PCSSCK_PRESCALER_1);
//	
//	SpiSetDelayAfterPASC(base, PASC_PRESCALER_1);
//	
//	SpiSetDelayAfterPDT(base , PDT_PRESCALER_1);
	
	SpiEnable(base);
	
	SpiEnableRxInterrupt(base);
//	
//	Spi1RxPrcsFp = RxPrcsFp;
	
}// End of Spi1MasterInit


/*
*******************************************************************************
*void Spi1MasterTransmitByte(uint8 tx_byte)
*******************************************************************************
* Input         : character to send
* Output        : void
* Description   : Send a byte on SPI1
* 				  This routine is for transfer on polling basis
 ********************************************************************************
*/
void Spi1MasterTransmitByte(uint8_t tx_byte)
{
	(void)SpiMasterTxRxByte(SPI2_BASE_PTR, tx_byte);
	
}//End of Spi1MasterTransmitByte

/*
*******************************************************************************
*uint8 Spi1MasterReceiveByte(void)
*******************************************************************************
* Input         : void
* Output        : Received byte
* Description   : read a byte from SPI1
* 				  This routine is for reception on polling basis
 ********************************************************************************
*/
uint8_t Spi1MasterReceiveByte(void)
{
	uint8_t recv_byte = 0;
	uint8_t tx_dummy_byte = 0x00;

	recv_byte = SpiMasterTxRxByte( SPI2_BASE_PTR, tx_dummy_byte);
	
	return recv_byte;
	
}//End of Spi1MasterReceiveByte


/*
*******************************************************************************
*void Spi2MasterTx(uint8 *tx_buff_ptr,uint8_t tx_count)
*******************************************************************************
* Input         : tx_buff_ptr: Pointer Data Buffer to be transmitted
* 				  tx_count: Number of bytes to be transmitted
* Output        : void
* Description   : Transmit data on SPI1
 ********************************************************************************
*/
void Spi2MasterTx(uint8_t *tx_buff_ptr,uint8_t *rx_buff_ptr,uint8_t tx_count)
{
	
	spi2_tx_data_s.data_buff_ptr = tx_buff_ptr;
	spi2_tx_data_s.rxdata_buff_ptr = rx_buff_ptr;
	spi2_tx_data_s.data_len = tx_count;
	spi2_tx_data_s.txdata_len = tx_count;
	spi2_tx_data_s.rxdata_len = 0;
	spi2_tx_data_s.data_buff_status = SPI1_TX_TRANSMIT_ON;
	spiTransferStart();
	SpiEnableTxInterrupt(SPI2_BASE_PTR);
	
}//End of Spi2MasterTx


/*
*******************************************************************************
*uint8 Spi1TxRxIsrHandler(void)
*******************************************************************************
* Input         : void
* Output        : void
* Description   : Interrupt handler for SPI1
* 				  1.) Transmit interrupt transmits the data till it is available
* 				  		in transmit buffer
* 				  2.) Receive interrupt calls the RxPrcs from the API
 ********************************************************************************
*/

//NOTE: add this function's declaration in kinetis_sysinit file 
//for vector/IRQ no 43
void Spi2TxRxIsrHandler(void)
{
	volatile uint8_t spi_rx_data = 0;
	uint32_t stat_reg = 0;
	
	//--> Read status register	
	stat_reg = (SPI_SR_REG(SPI2_BASE_PTR) & (SPI_SR_TFFF_MASK | SPI_SR_RFDF_MASK ));

	//-->If TX FIFO is empty.
	if((SPI_SR_REG(SPI2_BASE_PTR)   & SPI_SR_TFFF_MASK) == SPI_TX_READY)
	{
		if( SPI1_TX_BUFFER_EMPTY == spi2_tx_data_s.data_len)
		{
			//-->Set Tx Over Flag
			spi2_tx_data_s.data_buff_status = SPI1_TX_TRANSMIT_OVER;
			
			//-->Disable UART0 TX interrupt
			SpiDisableTxInterrupt(SPI2_BASE_PTR);
					
		}
		
		else
		{
			//--> Transmit Byte on SPI 
			while((SPI_SR_REG(SPI2_BASE_PTR) & SPI_SR_TFFF_MASK))
			{
				if(spi2_tx_data_s.data_len > 1 )
				{
					// as long as bit it set, we have room in our fifo
					SPI_PUSHR_REG(SPI2_BASE_PTR) = SPI_PUSHR_CONT_MASK |
							SPI_PUSHR_PCS(1<<0) | *spi2_tx_data_s.data_buff_ptr;

					spi2_tx_data_s.data_buff_ptr++;
					spi2_tx_data_s.data_len--;
				}
				else
				{
					// last byte in our tx buffer
					// as long as bit it set, we have room in our fifo
					SPI_PUSHR_REG(SPI2_BASE_PTR) = 
							SPI_PUSHR_PCS(1<<0) | *spi2_tx_data_s.data_buff_ptr;

					spi2_tx_data_s.data_buff_ptr++;
					spi2_tx_data_s.data_len--;
					break;
					
				}
			}
			//-->Clear Transmit FIFO Flag.
			SPI_SR_REG(SPI2_BASE_PTR) |= SPI_SR_TFFF_MASK;
			SpiEnableRxInterrupt( SPI2_BASE_PTR );
		}
	}
	
	//-->If RX FIFO is not empty
	if(SPI_SR_REG(SPI2_BASE_PTR) & SPI_SR_RFDF_MASK)
	{
		if(SPI_SR_REG(SPI2_BASE_PTR) & RX_CNTR_MASK )
		{
			//Read character from receiver
			while((SPI_SR_REG(SPI2_BASE_PTR) & SPI_SR_RFDF_MASK))
			{
				// read each byte from the fifo
				spi_rx_data = SPI_POPR_REG(SPI2_BASE_PTR) ; 
				if(spi2_tx_data_s.rxdata_len < spi2_tx_data_s.txdata_len)
				{
					spi2_tx_data_s.rxdata_buff_ptr[spi2_tx_data_s.rxdata_len++] = spi_rx_data;
				}
				else
				{
					SpiDisableRxInterrupt(SPI2_BASE_PTR);
					spiTransferStop();
					break;
				}
			}
		}
		
		//--> Copy data received on SPI1 to application buffer
		//--> by calling the application function

		/** @todo need to save data somewhere */
//		if(Spi1RxPrcsFp != NULL)
//		{
//			Spi1RxPrcsFp(spi_rx_data);	
//		}
	}
	
}//End of Spi1TxRxIsrHandler

/*
*******************************************************************************
*bool_t IsSpi1TxOver(void)
*******************************************************************************
* Input         : void 		
* Output        : void 
* Description   : return Spi1 Transmit Over status
* 				  0=>Transmit is completed
* 				  1=>Transmit is On
********************************************************************************
*/
uint8_t IsSpi1TxOver(void)
{
	return (spi2_tx_data_s.data_buff_status); 

}//End of IsSpi1TxOver


void PDB0Init(void)
{
	/* SIM_SCGC6: PDB=1 */
	SIM_SCGC6 |= SIM_SCGC6_PDB_MASK;                                   

	/* PDB0_SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,LDMOD=0,PDBEIE=0,SWTRIG=0,DMAEN=0,PRESCALER=3,TRGSEL=0x0F,PDBEN=0,PDBIF=0,PDBIE=1,??=0,MULT=1,CONT=1,LDOK=0 */
	PDB0_SC = PDB_SC_LDMOD(0x00) |
			PDB_SC_PRESCALER(0x00) |
			PDB_SC_TRGSEL(0x0F) |
			PDB_SC_PDBIE_MASK |
			PDB_SC_MULT(0x01) |
			PDB_SC_CONT_MASK;       
	/* PDB0_MOD: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,MOD=0x0D */
	PDB0_MOD = PDB_MOD_MOD(0x289);                                   
	/* PDB0_IDLY: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,IDLY=0 */
	PDB0_IDLY = PDB_IDLY_IDLY(0x00);                                   
	/* PDB0_CH0C1: BB&=~3,TOS&=~2,TOS|=1,EN&=~2,EN|=1 */
	PDB0_CH0C1 = (uint32_t)((PDB0_CH0C1 & (uint32_t)~(uint32_t)(
			PDB_C1_BB(0x03) |
			PDB_C1_TOS(0x02) |
			PDB_C1_EN(0x02)
	)) | (uint32_t)(
			PDB_C1_TOS(0x01) |
			PDB_C1_EN(0x01)
	));                                  
	/* PDB0_CH0S: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CF=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ERR=0xFF */
	PDB0_CH0S = (PDB_S_CF(0x00) | PDB_S_ERR(0xFF));                                   
	/* PDB0_CH0DLY0: DLY=0 */
	PDB0_CH0DLY0 &= (uint32_t)~(uint32_t)(PDB_DLY_DLY(0xFFFF));                                   
	/* PDB0_CH0DLY1: DLY=0 */
	PDB0_CH0DLY1 &= (uint32_t)~(uint32_t)(PDB_DLY_DLY(0xFFFF));                                   
	/* PDB0_CH0C1: BB&=~3,TOS&=~2,TOS|=1,EN&=~2,EN|=1 */
	PDB0_CH0C1 = (uint32_t)((PDB0_CH0C1 & (uint32_t)~(uint32_t)(
			PDB_C1_BB(0x03) |
			PDB_C1_TOS(0x02) |
			PDB_C1_EN(0x02)
	)) | (uint32_t)(
			PDB_C1_TOS(0x01) |
			PDB_C1_EN(0x01)
	));                                  
	/* PDB0_CH1S: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CF=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ERR=0xFF */
	PDB0_CH1S = (PDB_S_CF(0x00) | PDB_S_ERR(0xFF));                                   
	/* PDB0_CH1DLY0: DLY=0 */
	PDB0_CH1DLY0 &= (uint32_t)~(uint32_t)(PDB_DLY_DLY(0xFFFF));                                   
	/* PDB0_CH1DLY1: DLY=0 */
	PDB0_CH1DLY1 &= (uint32_t)~(uint32_t)(PDB_DLY_DLY(0xFFFF));                                   
	/* PDB0_CH2C1: BB&=~3,TOS&=~3,EN&=~3 */
	/* PDB0_DACINT0: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,INT=0 */
	PDB0_DACINT0 = PDB_INT_INT(0x00);                                   
	/* PDB0_DACINT1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,INT=0 */
	PDB0_DACINT1 = PDB_INT_INT(0x00);                                   
	/* PDB0_DACINTC0: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,EXT=0,TOE=0 */
	PDB0_DACINTC0 = 0x00U;                                   
	/* PDB0_DACINTC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,EXT=0,TOE=0 */
	PDB0_DACINTC1 = 0x00U;                                   
	/* PDB0_PO0DLY: DLY1=0,DLY2=0 */
	PDB0_PO0DLY = (PDB_PODLY_DLY1(0x00) | PDB_PODLY_DLY2(0x00));                                   
	/* PDB0_PO1DLY: DLY1=0,DLY2=0 */
	PDB0_PO1DLY = (PDB_PODLY_DLY1(0x00) | PDB_PODLY_DLY2(0x00));                                   
	/* PDB0_PO2DLY: DLY1=0,DLY2=0 */
	PDB0_PO2DLY = (PDB_PODLY_DLY1(0x00) | PDB_PODLY_DLY2(0x00));                                   
	/* PDB0_POEN: POEN&=~0x0F */
	PDB0_POEN &= (uint32_t)~(uint32_t)(PDB_POEN_POEN(0x0F));                                   
	/* PDB0_SC: PDBEN=1,LDOK=1 */
	PDB0_SC |= (PDB_SC_PDBEN_MASK | PDB_SC_LDOK_MASK);                                   
}

/*
 ** ===================================================================
 **     Method      :  ADC0_Init (component Init_ADC)
 **     Description :
 **         This method initializes registers of the ADC module
 **         according to the Peripheral Initialization settings.
 **         Call this method in user code to initialize the module. By
 **         default, the method is called by PE automatically; see "Call
 **         Init method" property of the component for more details.
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */
void ADC0Init(void)
{
	/* SIM_SCGC6: ADC0=1 */
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;                                   
	/* ADC0_CFG1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADLPC=0,ADIV=3,ADLSMP=0,MODE=3,ADICLK=0 */
	ADC0_CFG1 = ADC_CFG1_ADIV(0x03) |
			ADC_CFG1_MODE(0x03) |
			ADC_CFG1_ADICLK(0x00);       
	/* ADC0_CFG2: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADACKEN=0,ADHSC=0,ADLSTS=0 */
	ADC0_CFG2 &= (uint32_t)~(uint32_t)(
			ADC_CFG2_ADACKEN_MASK |
			ADC_CFG2_ADHSC_MASK |
			ADC_CFG2_ADLSTS(0x03) |
			0xFFFFFFE0U
	);                                   
	/* ADC0_CV1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CV=0 */
	ADC0_CV1 = ADC_CV1_CV(0x00);                                   
	/* ADC0_CV2: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CV=0 */
	ADC0_CV2 = ADC_CV2_CV(0x00);                                   
	/* ADC0_OFS: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OFS=4 */
	ADC0_OFS = ADC_OFS_OFS(0x04);                                   
	/* ADC0_SC2: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADACT=0,ADTRG=1,ACFE=0,ACFGT=0,ACREN=0,DMAEN=0,REFSEL=0 */
	ADC0_SC2 = (ADC_SC2_ADTRG_MASK | ADC_SC2_REFSEL(0x00));                                   
	/* ADC0_SC3: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CAL=0,CALF=0,??=0,??=0,ADCO=0,AVGE=1,AVGS=0 */
	ADC0_SC3 = (ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0x00));                                   
	/* ADC0_SC1A: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,DIFF=0,ADCH=7 */
	ADC0_SC1A = (ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0x07));                                   
	/* ADC0_SC1B: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=0,DIFF=0,ADCH=0x1F */
	ADC0_SC1B = ADC_SC1_ADCH(0x1F);                                   
}

#define YELLOW_LED_BASE (PTA_BASE_PTR)
#define YELLOW_LED_BIT (0x10000000U)
#define ORANGE_LED_BASE (PTA_BASE_PTR)
#define ORANGE_LED_BIT (0x00000800U)

#define PERFORMACE_BASE	(PTD_BASE_PTR)
#define PTD12_BIT		(0x1000U)
#define PTD13_BIT		(0x2000U)
#define PTD14_BIT		(0x4000U)
#define PTD15_BIT		(0x8000U)


/** @brief Set Yellow LED for output operation
 * 
 */
void YellowLEDInit(void)
{
	/* Configure pin 28 as output */
	/* GPIOA_PDDR: PDD|=0x10000000 */
	GPIOA_PDDR |= GPIO_PDDR_PDD(YELLOW_LED_BIT);                                   
	  
	/* Initialization of Port Control register */
	/* PORTA_PCR28: MUX=1 */
	PORTA_PCR28 = (uint32_t)(PORT_PCR_MUX(0x01));  
}

/** @brief Set Orange LED for output operation
 * 
 */
void OrangeLEDInit(void)
{
	/* Configure pin 11 as output */
	/* GPIOA_PDDR: PDD|=0x10000000 */
	GPIOA_PDDR |= GPIO_PDDR_PDD(ORANGE_LED_BIT);                                   
	  
	/* Initialization of Port Control register */
	/* PORTA_PCR11: MUX=1 */
	PORTA_PCR11 = (uint32_t)(PORT_PCR_MUX(0x01));  
}

void SetYellowLED(void)
{
	GPIO_PSOR_REG(YELLOW_LED_BASE) = YELLOW_LED_BIT;
}

void SetOrangeLED(void)
{
	GPIO_PSOR_REG(ORANGE_LED_BASE) = ORANGE_LED_BIT;
}

void ClrYellowLED(void)
{
	GPIO_PCOR_REG(YELLOW_LED_BASE) = YELLOW_LED_BIT;
}

void ClrOrangeLED(void)
{
	GPIO_PCOR_REG(ORANGE_LED_BASE) = ORANGE_LED_BIT;
}

void NegYellowLED(void)
{
	GPIO_PTOR_REG(YELLOW_LED_BASE) = YELLOW_LED_BIT;
}

void NegOrangeLED(void)
{
	GPIO_PTOR_REG(ORANGE_LED_BASE) = ORANGE_LED_BIT;
}

void PerformanceGPIOInit(void)
{
	/* Configure Port D base pins 12,13,14,15 as output */
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;	/* turnon clocks for PORTD */
	GPIOD_PDDR |= GPIO_PDDR_PDD(PTD12_BIT) |GPIO_PDDR_PDD(PTD13_BIT) | GPIO_PDDR_PDD(PTD14_BIT) |GPIO_PDDR_PDD(PTD15_BIT);                                   
	  
	/* Initialization of Port Control register */
	/* PORTD_PCR12: MUX=1 */
	PORTD_PCR12 = (uint32_t)(PORT_PCR_MUX(0x01));  
	/* PORTD_PCR13: MUX=1 */
	PORTD_PCR13 = (uint32_t)(PORT_PCR_MUX(0x01));  
	/* PORTD_PCR14: MUX=1 */
	PORTD_PCR14 = (uint32_t)(PORT_PCR_MUX(0x01));  
	/* PORTD_PCR15: MUX=1 */
	PORTD_PCR15 = (uint32_t)(PORT_PCR_MUX(0x01));  	
}

void SetPTD12(void)
{
	GPIO_PSOR_REG(PERFORMACE_BASE) = PTD12_BIT;
}

void ClrPTD12(void)
{
	GPIO_PCOR_REG(PERFORMACE_BASE) = PTD12_BIT;
}

void NegPTD12(void)
{
	GPIO_PTOR_REG(PERFORMACE_BASE) = PTD12_BIT;
}

void SetPTD13(void)
{
	GPIO_PSOR_REG(PERFORMACE_BASE) = PTD13_BIT;
}

void ClrPTD13(void)
{
	GPIO_PCOR_REG(PERFORMACE_BASE) = PTD13_BIT;
}

void NegPTD13(void)
{
	GPIO_PTOR_REG(PERFORMACE_BASE) = PTD13_BIT;
}

void SetPTD14(void)
{
	GPIO_PSOR_REG(PERFORMACE_BASE) = PTD14_BIT;
}

void ClrPTD14(void)
{
	GPIO_PCOR_REG(PERFORMACE_BASE) = PTD14_BIT;
}

void NegPTD14(void)
{
	GPIO_PTOR_REG(PERFORMACE_BASE) = PTD14_BIT;
}

void SetPTD15(void)
{
	GPIO_PSOR_REG(PERFORMACE_BASE) = PTD15_BIT;
}

void ClrPTD15(void)
{
	GPIO_PCOR_REG(PERFORMACE_BASE) = PTD15_BIT;
}

void NegPTD15(void)
{
	GPIO_PTOR_REG(PERFORMACE_BASE) = PTD15_BIT;
}

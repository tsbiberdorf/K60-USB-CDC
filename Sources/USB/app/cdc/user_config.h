/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2009 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************//*!
 *
 * @file user_config.h
 *
 * @author
 *
 * @version
 *
 * @date May-28-2009
 *
 * @brief The file contains User Modifiable Macros for Virtual COM Loopback
 *        Application
 *
 *****************************************************************************/
#define  KEY_PRESS_SIM_TMR_INTERVAL      (1000)        /* 2s between simulated key press events */
/*
   Below two MACROS are required for Virtual COM Loopback
   Application to execute
*/
#define LONG_SEND_TRANSACTION       /* support to send large data pkts */
#define LONG_RECIEVE_TRANSACTION    /* support to receive large data pkts */

/* User Defined MACRO to set number of Timer Objects */
#define MAX_TIMER_OBJECTS           5

#if MAX_TIMER_OBJECTS
/* When Enabled Timer Callback is invoked with an argument */
#define TIMER_CALLBACK_ARG
#undef TIMER_CALLBACK_ARG
#endif

#define USB_PACKET_SIZE  uint_16 /* support 16/32 bit packet size */
/* Use double buffered endpoints 5 & 6. To be only used with S08 cores */
#define DOUBLE_BUFFERING_USED

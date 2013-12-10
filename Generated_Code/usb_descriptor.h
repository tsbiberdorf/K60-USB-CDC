/** ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : usb_descriptor.h
**     Project   : ProcessorExpert
**     Processor : MK60DN512LQ10
**     Component : USB_CDC_CLASS
**     Version   : Driver 01.00, CPU db: 3.00.000
**     Compiler  : CodeWarrior ARM C Compiler
**     Date/Time : 2013-12-09, 14:17, # CodeGen: 4
**     Abstract  :
**     Settings  :
**     (c) Copyright <company/user-name>, 2011
**     http      : www.<company>.com
**     mail      : info@<company>.com
** ###################################################################*/

#ifndef _USB_DESCRIPTOR_H
#define _USB_DESCRIPTOR_H

#include "usb_class.h"
#include "usb_framework.h"
/******************************************************************************
 * Macro's
 *****************************************************************************/
#define BCD_USB_VERSION                  (0x0200)
#define REMOTE_WAKEUP_SUPPORT            (FALSE)
#define IMPLEMENT_QUEUING                (TRUE)

/* Communication Class SubClass Codes */
#define DIRECT_LINE_CONTROL_MODEL           (0x01)
#define ABSTRACT_CONTROL_MODEL              (0x02)
#define TELEPHONE_CONTROL_MODEL             (0x03)
#define MULTI_CHANNEL_CONTROL_MODEL         (0x04)
#define CAPI_CONTROL_MOPDEL                 (0x05)
#define ETHERNET_NETWORKING_CONTROL_MODEL   (0x06)
#define ATM_NETWORKING_CONTROL_MODEL        (0x07)
#define WIRELESS_HANDSET_CONTROL_MODEL      (0x08)
#define DEVICE_MANAGEMENT                   (0x09)
#define MOBILE_DIRECT_LINE_MODEL            (0x0A)
#define OBEX                                (0x0B)
#define ETHERNET_EMULATION_MODEL            (0x0C)

/* Communication Class Protocol Codes */
#define NO_CLASS_SPECIFIC_PROTOCOL  (0x00)
#define AT_250_PROTOCOL             (0x01)
#define AT_PCCA_101_PROTOCOL        (0x02)
#define AT_PCCA_101_ANNEX_O         (0x03)
#define AT_GSM_7_07                 (0x04)
#define AT_3GPP_27_007              (0x05)
#define AT_TIA_CDMA                 (0x06)
#define ETHERNET_EMULATION_PROTOCOL (0x07)
#define EXTERNAL_PROTOCOL           (0xFE)
#define VENDOR_SPECIFIC             (0xFF)

/* Data Class Protocol Codes */
/* #define NO_CLASS_SPECIFIC_PROTOCOL  (0x00) */
#define PYHSICAL_INTERFACE_PROTOCOL    (0x30)
#define HDLC_PROTOCOL                  (0x31)
#define TRANSPARENT_PROTOCOL           (0x32)
#define MANAGEMENT_PROTOCOL            (0x50)
#define DATA_LINK_Q931_PROTOCOL        (0x51)
#define DATA_LINK_Q921_PROTOCOL        (0x52)
#define DATA_COMPRESSION_V42BIS        (0x90)
#define EURO_ISDN_PROTOCOL             (0x91)
#define RATE_ADAPTION_ISDN_V24         (0x92)
#define CAPI_COMMANDS                  (0x93)
#define HOST_BASED_DRIVER              (0xFD)
#define CDC_UNIT_FUNCTIONAL            (0xFE)
/* #define VENDOR_SPECIFIC             (0xFF) */

/* Descriptor SubType in Communications Class Functional Descriptors */
#define HEADER_FUNC_DESC              (0x00)
#define CALL_MANAGEMENT_FUNC_DESC     (0x01)
#define ABSTRACT_CONTROL_FUNC_DESC    (0x02)
#define DIRECT_LINE_FUNC_DESC         (0x03)
#define TELEPHONE_RINGER_FUNC_DESC    (0x04)
#define TELEPHONE_REPORT_FUNC_DESC    (0x05)
#define UNION_FUNC_DESC               (0x06)
#define COUNTRY_SELECT_FUNC_DESC      (0x07)
#define TELEPHONE_MODES_FUNC_DESC     (0x08)
#define USB_TERMINAL_FUNC_DESC        (0x09)
#define NETWORK_CHANNEL_FUNC_DESC     (0x0A)
#define PROTOCOL_UNIT_FUNC_DESC       (0x0B)
#define EXTENSION_UNIT_FUNC_DESC      (0x0C)
#define MULTI_CHANNEL_FUNC_DESC       (0x0D)
#define CAPI_CONTROL_FUNC_DESC        (0x0E)
#define ETHERNET_NETWORKING_FUNC_DESC (0x0F)
#define ATM_NETWORKING_FUNC_DESC      (0x10)
#define WIRELESS_CONTROL_FUNC_DESC    (0x11)
#define MOBILE_DIRECT_LINE_FUNC_DESC  (0x12)
#define MDLM_DETAIL_FUNC_DESC         (0x13)
#define DEVICE_MANAGEMENT_FUNC_DESC   (0x14)
#define OBEX_FUNC_DESC                (0x15)
#define COMMAND_SET_FUNC_DESC         (0x16)
#define COMMAND_SET_DETAIL_FUNC_DESC  (0x17)
#define TELEPHONE_CONTROL_FUNC_DESC   (0x18)
#define OBEX_SERVICE_ID_FUNC_DESC     (0x19)


#define CIC_SUBCLASS_CODE                (2)
#define CIC_PROTOCOL_CODE                (0x00)
#define CIC_NOTIF_ELEM_SUPPORT           (1)
#define CIC_ENDP_COUNT                   (1)
#define CIC_NOTIF_ENDPOINT               (3)
#define CIC_NOTIF_ENDP_PACKET_SIZE       (16)

#define DATA_CLASS_SUPPORT               (1)
#define DIC_PROTOCOL_CODE                (0x00)
#define DIC_ENDP_COUNT                   (2)
#define DIC_BULK_IN_ENDPOINT             (1)
#define DIC_BULK_IN_ENDP_PACKET_SIZE     (16)/* max supported is 64 */
#define DIC_BULK_OUT_ENDPOINT            (2)
#define DIC_BULK_OUT_ENDP_PACKET_SIZE    (16)/* max supported is 64 */
#define REMOTE_WAKEUP_SHIFT              (5)

/* Various descriptor sizes */
#define DEVICE_DESCRIPTOR_SIZE            (18)
#define IFACE_ONLY_DESC_SIZE              (9)
#define CONFIG_ONLY_DESC_SIZE             (9)
#define CONFIG_DESC_SIZE                  (CONFIG_ONLY_DESC_SIZE + IFACE_ONLY_DESC_SIZE + 19 +   \
                                          (CIC_NOTIF_ELEM_SUPPORT & 0x01) \
                                          * 7 + DATA_CLASS_SUPPORT * 23)
#define DEVICE_QUALIFIER_DESCRIPTOR_SIZE  (10)

#define ENDP_ONLY_DESC_SIZE               (7)

/* Max descriptors provided by the Application */
#define USB_MAX_STD_DESCRIPTORS           (7)

/* Max configuration supported by the Application */
#define USB_MAX_CONFIG_SUPPORTED          (1)

/* Max string descriptors supported by the Application */
#define USB_MAX_STRING_DESCRIPTORS        (4)

/* Max language codes supported by the USB */
#define USB_MAX_LANGUAGES_SUPPORTED       (1)

/* string descriptors sizes */
#define USB_STR_DESC_SIZE (2)
#define USB_STR_0_SIZE  (2)
#define USB_STR_1_SIZE  (56)
#define USB_STR_2_SIZE  (46)
#define USB_STR_n_SIZE  (32)

/* descriptors codes */
#define USB_DEVICE_DESCRIPTOR     (1)
#define USB_CONFIG_DESCRIPTOR     (2)
#define USB_STRING_DESCRIPTOR     (3)
#define USB_IFACE_DESCRIPTOR      (4)
#define USB_ENDPOINT_DESCRIPTOR   (5)
#define USB_DEVQUAL_DESCRIPTOR    (6)
#define USB_CS_INTERFACE          (0x24)
#define USB_CS_ENDPOINT           (0x25)

#define USB_MAX_SUPPORTED_INTERFACES     (2)

/* Implementation Specific Macros */
#define LINE_CODING_SIZE              (0x07)
#define COMM_FEATURE_DATA_SIZE        (0x02)

#define LINE_CODE_DTERATE_IFACE0      (115200) /*e.g 9600 is 0x00002580 */
#define LINE_CODE_CHARFORMAT_IFACE0   (0x00)   /* 1 stop bit */
#define LINE_CODE_PARITYTYPE_IFACE0   (0x00)   /* No Parity */
#define LINE_CODE_DATABITS_IFACE0     (0x08)   /* Data Bits Format */

#define LINE_CODE_DTERATE_IFACE1      (9600)   /*e.g. 115200 is 0x0001C200*/
#define LINE_CODE_CHARFORMAT_IFACE1   (0x00)   /* 1 stop bit */
#define LINE_CODE_PARITYTYPE_IFACE1   (0x00)   /* No Parity */
#define LINE_CODE_DATABITS_IFACE1     (0x08)   /* Data Bits Format */

#define STATUS_ABSTRACT_STATE_IFACE0  (0x0000) /* Disable Multiplexing
                                                  ENDP in this interface will
                                                 continue to accept/offer data*/
#define STATUS_ABSTRACT_STATE_IFACE1  (0x0000) /* Disable Multiplexing
                                                  ENDP in this interface will
                                                 continue to accept/offer data*/
#define COUNTRY_SETTING_IFACE0        (0x0000) /* Country Code in the format as
                                                  defined in [ISO3166] */
#define COUNTRY_SETTING_IFACE1        (0x0000) /* Country Code in the format as
                                                  defined in [ISO3166] */

/* Notifications Support */
#define PSTN_SUBCLASS_NOTIF_SUPPORT     (TRUE)
#define WMC_SUBCLASS_NOTIF_SUPPORT      (FALSE)
#define CDC_CLASS_NOTIF_SUPPORT         (FALSE)

#define CDC_DESC_ENDPOINT_COUNT         (3)

/* Add your custom macro here*/

/******************************************************************************
 * Types
 *****************************************************************************/
typedef const struct _USB_LANGUAGE
{
    uint_16 const language_id;      /* Language ID */
    uint_8 const ** lang_desc;      /* Language Descriptor String */
    uint_8 const * lang_desc_size;  /* Language Descriptor Size */
} USB_LANGUAGE;

typedef const struct _USB_ALL_LANGUAGES
{
    /* Pointer to Supported Language String */
    uint_8 const *languages_supported_string;
    /* Size of Supported Language String */
    uint_8 const languages_supported_size;
    /* Array of Supported Languages */
    USB_LANGUAGE usb_language[USB_MAX_SUPPORTED_INTERFACES];
}USB_ALL_LANGUAGES;

typedef const struct _USB_ENDPOINTS
{
    /* Number of non control Endpoints */
    uint_8 count;
    /* Array of Endpoints Structures */
    USB_EP_STRUCT ep[CDC_DESC_ENDPOINT_COUNT];
}USB_ENDPOINTS;



/******************************************************************************
* Global Functions
*****************************************************************************/
uint_8 USB_Desc_Get_Descriptor(uint_8 controller_ID, uint_8 type, uint_8 str_num, uint_16 index, uint_8_ptr *descriptor, USB_PACKET_SIZE *size);
/*
** ===================================================================
**     Method      :  USB_Desc_Get_Descriptor (component USB_CDC_CLASS)
**     Description :
**         The function returns the correponding descriptor
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN]  Controller ID
**         type            - [IN]  Type of descriptor requested
**         str_num         - [IN]  String index for string
**                           descriptor
**         index           - [IN]  String descriptor language Id
**       * descriptor      - [OUT] Output descriptor
**                           pointer
**       * size            - [OUT] Size of descriptor returned
**     Returns     :
**         ---             - USB_OK                         when
**                           Successfull
**                           USBERR_INVALID_REQ_TYPE        when Error
** ===================================================================
*/

uint_8 USB_Desc_Get_Interface(uint_8 controller_ID, uint_8 interface, uint_8_ptr alt_interface);
/*
** ===================================================================
**     Method      :  USB_Desc_Get_Interface (component USB_CDC_CLASS)
**     Description :
**         The function returns the alternate interface
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface number
**         alt_interface   - [OUT] Output alternate
**                           interface
**     Returns     :
**         ---             - USB_OK                         when
**                           Successfull
**                           USBERR_INVALID_REQ_TYPE        when Error
** ===================================================================
*/

boolean USB_Desc_Valid_Configation(uint_8 controller_ID, uint_16 config_val);
/*
** ===================================================================
**     Method      :  USB_Desc_Valid_Configation (component USB_CDC_CLASS)
**     Description :
**         This function checks whether the configuration is valid or
**         not
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         config_val      - [IN] Configuration value
**     Returns     :
**         ---             - TRUE           When Valid
**                           FALSE          When Error
** ===================================================================
*/
uint_8 USB_Desc_Set_Interface(uint_8 controller_ID, uint_8 interface, uint_8 alt_interface);
/*
** ===================================================================
**     Method      :  USB_Desc_Set_Interface (component USB_CDC_CLASS)
**     Description :
**         The function sets the alternate interface
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface number
**         alt_interface   - [IN] Input alternate
**                           interface
**     Returns     :
**         ---             - USB_OK                         when
**                           Successfull
**                           USBERR_INVALID_REQ_TYPE        when Error
** ===================================================================
*/
boolean USB_Desc_Valid_Interface(uint_8 controller_ID, uint_8 interface);
/*
** ===================================================================
**     Method      :  USB_Desc_Valid_Interface (component USB_CDC_CLASS)
**     Description :
**         This function checks whether the interface is valid or not
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Target interface
**     Returns     :
**         ---             - TRUE           When Valid
**                           FALSE          When Error
** ===================================================================
*/

boolean USB_Desc_Remote_Wakeup(uint_8 controller_ID);
/*
** ===================================================================
**     Method      :  USB_Desc_Remote_Wakeup (component USB_CDC_CLASS)
**     Description :
**         This function returns remote wakeup is supported or not
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**     Returns     :
**         ---             - REMOTE_WAKEUP_SUPPORT (TRUE) - If remote
**                           wakeup supported
** ===================================================================
*/

void* USB_Desc_Get_Endpoints(uint_8 controller_ID);
/*
** ===================================================================
**     Method      :  USB_Desc_Get_Endpoints (component USB_CDC_CLASS)
**     Description :
**         This function returns the information about all the non
**         control endpoints implemented
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**     Returns     :
**         ---             - pointer to USB_ENDPOINTS
** ===================================================================
*/

uint_8 USB_Desc_Get_Line_Coding(uint_8 controller_ID, uint_8 interface, uint_8_ptr *coding_data);
/*
** ===================================================================
**     Method      :  USB_Desc_Get_Line_Coding (component USB_CDC_CLASS)
**     Description :
**         The function returns the Line Coding/Configuraion
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface Number
**       * coding_data     - [OUT] Line Coding Data
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint_8 USB_Desc_Set_Line_Coding(uint_8 controller_ID, uint_8 interface, uint_8_ptr *coding_data);
/*
** ===================================================================
**     Method      :  USB_Desc_Set_Line_Coding (component USB_CDC_CLASS)
**     Description :
**         The function sets the Line Coding/Configuraion
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface Number
**       * coding_data     - [IN] Line Coding Data
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint_8 USB_Desc_Get_Abstract_State(uint_8 controller_ID, uint_8 interface, uint_8_ptr *feature_data);
/*
** ===================================================================
**     Method      :  USB_Desc_Get_Abstract_State (component USB_CDC_CLASS)
**     Description :
**         The function gets the current setting for communication
**         feature
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface Number
**       * feature_data    - [OUT] Output Comm
**                           Feature Data
**     Returns     :
**         ---             - Error code
** ===================================================================

*/

uint_8 USB_Desc_Get_Country_Setting(uint_8 controller_ID, uint_8 interface, uint_8_ptr *feature_data);
/*
** ===================================================================
**     Method      :  USB_Desc_Get_Country_Setting (component USB_CDC_CLASS)
**     Description :
**         The function gets the current setting for communication
**         feature (COUNTRY_CODE)
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface Number
**       * coding_data     - [OUT] Output Comm Feature
**                           Data
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint_8 USB_Desc_Set_Abstract_State(uint_8 controller_ID, uint_8 interface, uint_8_ptr *feature_data);
/*
** ===================================================================
**     Method      :  USB_Desc_Set_Abstract_State (component USB_CDC_CLASS)
**     Description :
**         The function sets the current setting for communication
**         feature (ABSTRACT_STATE)
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface Number
**       * coding_data     - [IN] Output Comm Feature
**                           Data
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint_8 USB_Desc_Set_Country_Setting(uint_8 controller_ID, uint_8 interface, uint_8_ptr *feature_data);
/*
** ===================================================================
**     Method      :  USB_Desc_Set_Country_Setting (component USB_CDC_CLASS)
**     Description :
**         The function sets the current setting for communication
**         feature (COUNTRY_CODE)
**     Parameters  :
**         NAME            - DESCRIPTION
**         controller_ID   - [IN] Controller ID
**         interface       - [IN] Interface Number
**       * coding_data     - [IN] Output Comm Feature
**                           Data
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

#endif
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.2 [05.07]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/

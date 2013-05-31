/*
 * PIC24 ECAN library
 * (used with PIC24HJ128GPX02)
 * Operating in Register Indirect mode.
 *
 * IMPORTANT:
 * From silicon errata sheet for PIC24HJ128GPX02:
 * "Do not use the DMA with ECAN in Peripheral Indirect mode."
 *
 * In Register Indirect mode, the ECAN/DMA
 * cannot differentiate between buffers. The
 * application must keep track of where the
 * received message was stored.
 *
 * basic steps to use this library:
 * 
 * 1  setup io pins. remap as needed.
 * 2  #include "ecan_pic24_register_indirect.h"
 *      and add ecan_pic24_register_indirect.c source file in project/makefile
 * 3  add the following extern variable declaraionts to main() file:
 *      // ECAN variables
 *      extern volatile int rxBufferIndex;
 *      extern int rxIndex;
 *      extern int txBufferIndex;
 *      extern unsigned int ECAN_transmit_buffer[7] __attribute__((space(dma))); // CAN message transmit buffer size is seven bytes
 *      extern unsigned int ECAN_receive_buffer[NUM_OF_RX_BUFFERS][8] __attribute__((space(dma))); // CAN receive message buffer size is eight bytes
 *      extern volatile int received;
 *
 * 4  call ECAN1_initialize();
 * 5  set up message receive filters
 * 6  create and send a message using the ECAN1_send_standard_message() function
 * 7  receive a message by monitoring the received variable: if (received == 1){} from within main()
 *
 */

#ifndef _ECAN_PIC24_REGISTER_INDIRECT_H_
#define _ECAN_PIC24_REGISTER_INDIRECT_H_

/*
 * Define the number of ECAN RECEIVE buffers.
 * Only one TX buffer is used. This
 * is explained in the readme file along with
 * this code example.
 */
#define NUM_OF_RX_BUFFERS 8

#define ECAN_BITRATE 500000   // CAN bus baud rate (bits per second)
#define ECAN_FCY     40000000 // Fosc divided by 2

// ECAN hardware peripheral modes
#define ECAN_MODE_NORMAL 0x0
#define ECAN_MODE_DISABLE 0x1
#define ECAN_MODE_LOOPBACK 0x2
#define ECAN_MODE_LISTEN_ONLY 0x3
#define ECAN_MODE_CONFIGURATION 0x4
#define ECAN_MODE_RESERVED1 0x5 // unimplemented
#define ECAN_MODE_RESERVED2 0x6 // unimplemented
#define ECAN_MODE_LISTEN_ALL_MESSAGES 0x7

// data structure to store message filter settings
// TODO MAKE THESE DARN FILTERS WORK PROPERLY !!!!!
typedef struct _ecan_receive_filter{
    unsigned int filter_number; // 0 to 15
    unsigned int receive_sid;
    unsigned long receive_eid;
    unsigned int receive_use_extended;  // 0 or 1
    unsigned int mask_number; // 0 to 2
    unsigned int filter_mask;
    
} ecan_filter;

// data structure to store raw message data
typedef struct _raw_ecan_message_data{
    unsigned int word0;
    unsigned int word1;
    unsigned int word2;
    unsigned int word3;
} raw_ecan_message_data;

int ECAN1_initialize(unsigned int receive_normal_standard_id);

void ECAN1_config_DMA(int txBuffer,int rxBuffer, int numOfRxBuffers);
void ECAN1_config_clock(void);
void ECAN1_config_interrupts(void);
void ECAN1_config_transmit_buffers(void);
int ECAN1_set_operating_mode(unsigned int);

unsigned int ECAN1_get_filter_register_value(unsigned int standard_id, unsigned long extended_id, unsigned int use_extended);
void ECAN1_config_receive_filter(unsigned int receive_standard_id);
int ECAN1_disable_receive_filter(unsigned int filter_number);
void ECAN1_disable_all_filters(void);
void ECAN1_disable_all_masks(void);

void ECAN_create_standard_message(raw_ecan_message_data * message_data, int standard_id, unsigned int * output);
void ECAN_create_extended_message(raw_ecan_message_data * message_data,int standard_id, long extended_id, unsigned int * output);
int ECAN1_send_standard_message(raw_ecan_message_data * message_data, int standard_id, unsigned int * output);

unsigned int ECAN_get_standard_ID_Filter(int standard_id);
unsigned int ECAN_get_extended_ID_Filter(int standard_id, unsigned long extended_id, int exid_enable);

#endif /* _ECAN_PIC24_REGISTER_INDIRECT_H_ */

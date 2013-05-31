/*
 * From the data sheets and family reference manualas,
 * the proper steps for setting up the ECAN module are:
 *
 * Step 1: Request Configuration Mode from the ECAN Module
 * Step 2: Select ECAN Clock and Bit Timing
 * Step 3: Assign Number of Buffers Used by ECAN Module in DMA Memory Space
 * Step 4: Set Up Filters and Masks
 * Step 5: Put the ECAN Module in Normal Mode
 * Step 6: Set Up the Transmit/Receive Buffers
 */

#include <xc.h>
#include "h/ecan_pic24_register_indirect.h"

// ECAN variables
volatile int rxBufferIndex = -1;
int rxIndex;
int txBufferIndex = 0;
unsigned int ECAN_transmit_buffer[7] __attribute__((space(dma))); // CAN message transmit buffer size is seven bytes
unsigned int ECAN_receive_buffer[NUM_OF_RX_BUFFERS][8] __attribute__((space(dma))); // CAN receive message buffer size is eight bytes
volatile int received = 0; // written to in the ECAN interrupt service routine. 1 = a message has been received and is in ECAN_receive_buffer
volatile int ECAN_ready_to_transmit = 0; // 1 = the ECNA is ready and available to transmit

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    /* Only the Transmit Message, Receive
     * Message Events and Error flags are processed. You can
     * check for the other events as well.*/

    _C1IF = 0;

    if (C1INTFbits.TBIF == 1) {
        /* Transmit done. Set the flag so that
         * the main application loop knows that
         * the message was transmitted */
        ECAN_ready_to_transmit = 1;

        C1INTFbits.TBIF = 0;

        DMA0CONbits.CHEN = 0; /* Uncomment these lines if the TX data payload */
        DMA0CONbits.CHEN = 1; /* length is not fixed. */
    }

    if (C1INTFbits.RBIF == 1) {
        /* Received a packet. Check if the
         * rxBufferIndex is at the boundary
         * and reset to initial condition
         * if needed.

         * You can check the RXFUL flags
          if needed. */

        if (rxBufferIndex == (NUM_OF_RX_BUFFERS - 1))
            rxBufferIndex = -1;

        C1INTFbits.RBIF = 0;
        C1RXFUL1 = 0;
        received = 1;
        rxBufferIndex++;
    }

    //int errVal;

    //if (C1INTFbits.ERRIF == 1) {
    //errVal = C1EC;
    //C1INTFbits.ERRIF = 0;
    //}
}

int ECAN1_initialize(unsigned int receive_normal_standard_id) {

    int retval = 0;

    retval = ECAN1_set_operating_mode(ECAN_MODE_CONFIGURATION);

    ECAN1_config_DMA(__builtin_dmaoffset(ECAN_transmit_buffer), __builtin_dmaoffset(ECAN_receive_buffer), NUM_OF_RX_BUFFERS);
    ECAN1_config_clock();
    ECAN1_config_interrupts();
    ECAN1_config_receive_filter(receive_normal_standard_id);
    ECAN1_config_transmit_buffers();
    //ECAN1_disable_all_masks();
    //ECAN1_disable_all_filters();
    retval = ECAN1_set_operating_mode(ECAN_MODE_NORMAL);
    ECAN_ready_to_transmit = 1;

    return retval;
}

unsigned int ECAN_get_standard_ID_Filter(int standard_id) {
    /* This function returns the value to be
     * stored in CxFnSID register.
     * SID - The Standard ID
     * EID - The Extended ID. For Standard messages
     * 		 EID should be zero.
     * EXIDE - 0 for Standard messages. 1 otherwise
     */

    unsigned int returnVal;

    returnVal = standard_id << 5;

    //if(exide == 1)
    //	returnVal |= 0x8;
    //else
    returnVal &= 0xFFFE;
    //returnVal |= (int)(eid >> 16);
    return (returnVal);
}

unsigned int ECAN_get_extended_ID_Filter(int standard_id, unsigned long eid, int exide) {
    /* This function returns the value to be
     * stored in CxFnSID register.
     * SID - The Standard ID
     * EID - The Extended ID. For Standard messages
     *       EID should be zero.
     * EXIDE - 0 for Standard messages. 1 otherwise
     */

    unsigned int returnVal;

    returnVal = standard_id << 5;

    if (exide == 1) {
        returnVal |= 0x8;
    }
    else {
        returnVal &= 0xFFFE;
    }
        returnVal |= (int) (eid >> 16);
    return (returnVal);
}

/*
unsigned int ECANFilterEID(unsigned long eid) {
     // This function will return value to be
     // stored on CxFnEID register.
     // EID - this is the extended ID

    unsigned int returnVal;

    returnVal = (int) (eid & 0xFFFF);

    return (returnVal);
}
 */

void ECAN1_config_DMA(int txBufferOffset, int rxBufferOffset, int numOfRxBuffers) {
    /* Set up DMA Channel 0 to copy data from
     * DMA RAM to ECAN module 1. The DMA mode is
     * Register Indirect with Post Increment.
     * The length of each ECAN message is 6 words maximum.
     * Additionally the Continuous mode with
     * ping pong disabled is used. For ease of error
     * handling while using register indirect mode, only
     * one TX buffer is used. */

    DMA0CON = 0x2000;
    DMA0PAD = (int) (&C1TXD);
    DMA0CNT = 5;
    DMA0REQ = 0x0046;
    DMA0STA = txBufferOffset;
    _DMA0IE = 0;
    _DMA0IF = 0;
    DMA0CONbits.CHEN = 1;

    /* Set up DMA Channel 1 to copy data from
     * ECAN module 1 to DMA RAM. The Receive
     * memory is treated like a FIFO. The ECAN
     * module cannot differentiate between buffers
     * when the DMA is in register indirect mode.
     * Note the size of each received message is
     * eight words. Continuous with ping pong disabled
     * is used.*/

    DMA1CON = 0x0000;
    DMA1PAD = (int) (&C1RXD);
    DMA1CNT = (numOfRxBuffers * 8) - 1;
    DMA1REQ = 0x0022;
    DMA1STA = rxBufferOffset;
    _DMA1IE = 0;
    _DMA1IF = 0;
    DMA1CONbits.CHEN = 1;

}

void ECAN1_config_clock(void) {
    /* ECAN_FCY and ECAN_BITRATE are defined in
     * ECAN1Drv.h. The total time quanta per
     * bit is 8. Refer to ECAN FRM section for
     * more details on setting the CAN bit rate */

    C1CTRL1bits.CANCKS = 1;
    C1CFG1 = ((ECAN_FCY / 16) / ECAN_BITRATE) - 1;
    C1CFG2 = 0x0290;

    /* CAN Baud Rate Configuration  other method from microchip example    
    #define FCAN            40000000 
    #define BITRATE         1000000  
    #define NTQ             20      // 20 Time Quanta in a Bit Time
    #define BRP_VAL         ((FCAN/(2*NTQ*BITRATE))-1)
     */

}

void ECAN1_config_interrupts(void) {
    /* Only the C1 Event Interrupt is used
     * in this code example. All the status
     * flags are cleared before the module is
     * enabled */

    C1INTF = 0;
    _C1IF = 0;
    _C1TXIF = 0;
    _C1RXIF = 0;
    _C1IE = 1;
    _C1TXIE = 0;
    _C1RXIE = 0;
    C1INTE = 0x00FF;
    C1RXFUL1 = 0;
    C1RXFUL2 = 0;
    C1RXOVF1 = 0;
    C1RXOVF2 = 0;
}

void ECAN1_config_transmit_buffers(void) {
    /* Configure only one TX buffer and enable
     * four DMA buffers. No need to configure
     * RX buffers. */

    C1CTRL1bits.WIN = 0;
    C1TR01CONbits.TXEN0 = 1;

}

void ECAN1_config_receive_filter(unsigned int receive_standard_id) {
    /* Enable filters for received messages.
     * 
     * microchip comments:
     *  You may not want to set up the CxBUFPNTn bits to point to
     * FIFO. This way you can avoid the FIFO interrupts.
     * Just point to an arbitrary RX buffer */


    C1CTRL1bits.WIN = 1;
    C1FEN1 = 0x11;
    C1RXF0SID = ECAN_get_standard_ID_Filter(receive_standard_id);
    C1RXF4SID = ECAN_get_standard_ID_Filter(receive_standard_id);
    C1RXM0SID = 0xFFEB; /* Configure MASK 0 - All bits used in comparison*/
    C1RXM0EID = 0xFFFF;
    C1FMSKSEL1bits.F0MSK = 0x0; /* User MASK 0 for all filters */
    C1FMSKSEL1bits.F4MSK = 0x0; /* User MASK 0 for all filters */
    C1BUFPNT1bits.F0BP = 0x1; /* Set the destination buffers to be any thing but */
    C1BUFPNT2bits.F4BP = 0x1; /* configured transmit buffers */

}

int ECAN1_disable_receive_filter(unsigned int filter_number) {

    int retval = 0;
    unsigned int i = 0;
    unsigned int and_mask = 0x01;

    C1CTRL1bits.WIN = 1;

    // Clears the appropriate "filter enable" bit.

    // TODO  verify that this algorithm disables the correct filter enable bit.
    // TODO  if it works, then the big swicth below can be removed.
    if ((filter_number >= 0) && (filter_number <= 15)) {

        for (i = 1; i <= filter_number; i++) {
            and_mask = and_mask << 1;
        }
        C1FEN1 = C1FEN1 & ~and_mask;

    }
    else {
        retval = -1;
    }
    /*
        // disable the filter
        switch (filter_number) {

            case 0:
                C1FEN1bits.FLTEN0 = 0;
                break;

            case 1:
                C1FEN1bits.FLTEN1 = 0;
                break;

            case 2:
                C1FEN1bits.FLTEN2 = 0;
                break;

            case 3:
                C1FEN1bits.FLTEN3 = 0;
                break;

            case 4:
                C1FEN1bits.FLTEN4 = 0;
                break;

            case 5:
                C1FEN1bits.FLTEN5 = 0;
                break;

            case 6:
                C1FEN1bits.FLTEN6 = 0;
                break;

            case 7:
                C1FEN1bits.FLTEN7 = 0;
                break;

            case 8:
                C1FEN1bits.FLTEN8 = 0;
                break;

            case 9:
                C1FEN1bits.FLTEN9 = 0;
                break;

            case 10:
                C1FEN1bits.FLTEN10 = 0;
                break;

            case 11:
                C1FEN1bits.FLTEN11 = 0;
                break;

            case 12:
                C1FEN1bits.FLTEN12 = 0;
                break;

            case 13:
                C1FEN1bits.FLTEN13 = 0;
                break;

            case 14:
                C1FEN1bits.FLTEN14 = 0;
                break;

            case 15:
                C1FEN1bits.FLTEN15 = 0;
                break;
        }
     */
    return retval;

}

void ECAN1_disable_all_filters(void) {

    //  ECAN modules have 16 acceptance masks
    // each is enabled by one bit in the C1FEN1 register

    C1FEN1 = 0x00; // clear all message filters

}

void ECAN1_disable_all_masks(void) {

    //  ECAN modules have 3 acceptance masks
    //  this function configures ECAN to accept all messages

    C1RXM0SID = 0xFFEB; // Configure MASK 0 to use all bits in comparison
    C1RXM0EID = 0xFFFF;

    C1RXM1SID = 0xFFEB; // Configure MASK 1 to use all bits in comparison
    C1RXM1EID = 0xFFFF;

    C1RXM2SID = 0xFFEB; // Configure MASK 2 to use all bits in comparison
    C1RXM2EID = 0xFFFF;

}

int ECAN1_send_message(void) {
    /*
     * The TXREQ bit is set. This will make the
     * ECAN module generate request to the DMA.
     * Note that in Register Indirect Mode the
     * ECAN module is not aware of buffers. So
     * setting the TXREQ bit will cause data to be
     * transferred from the location pointed by
     * the DMA
     */

    /*
     * The DMA Pointer will have to be reset in case
     * arbitration is lost. This can be done by checking
     * the TXLARB bit in the CxTRmnCON register. The DMA
     * Pointer is reset by disabling and enabling
     * the DMA channel.
     */

    unsigned int retval = 0;

    if (ECAN_ready_to_transmit == 1) {
        ECAN_ready_to_transmit = 0;
        C1TR01CONbits.TXREQ0 = 1;
        while (C1TR01CONbits.TXREQ0 == 1) {
            if (C1TR01CONbits.TXLARB0 == 1) {
                /* Arbitration lost. Abort the
                 * message and reset the transmit
                 * buffer DMA */

                C1TR01CONbits.TXREQ0 = 0;
                DMA0CONbits.CHEN = 0;
                DMA0CONbits.CHEN = 1;
                C1TR01CONbits.TXREQ0 = 1;
            }
        }

        /*
         * If the message transmission is aborted the function
         * will return an error code.
         */
        if (C1TR01CONbits.TXABT0 == 1) {
            retval = 2;
        }
    }
    else {
        retval = 1;
    }
    return retval;
}

int ECAN1_send_standard_message(raw_ecan_message_data * message_data, int standard_id, unsigned int * output) {
    int retval = 0;
    if (ECAN_ready_to_transmit == 1) {
        ECAN_create_standard_message(message_data, standard_id, output);
        retval = ECAN1_send_message();
    }
    return retval;
}

int ECAN1_send_extended_message(raw_ecan_message_data * message_data, int standard_id, long extended_id, unsigned int * output) {
    int retval = 0;
    if (ECAN_ready_to_transmit == 1) {
        ECAN_create_extended_message(message_data, standard_id, extended_id, output);
        retval = ECAN1_send_message();
    }
    return retval;
}

int ECAN1_set_operating_mode(unsigned int requested_mode) {
    /*
    000 = Set Normal Operation mode
    001 = Set Disable mode
    010 = Set Loopback mode
    011 = Set Listen Only Mode
    100 = Set Configuration mode
    101 = Reserved
    110 = Reserved
    111 = Set Listen All Messages
     */
    int retval = 0;
    if (((requested_mode >= 0) && (requested_mode <= 4)) || requested_mode == 7) {
        C1CTRL1bits.REQOP = requested_mode; // Request the new mode and wait until it is set
        while (C1CTRL1bits.OPMODE != requested_mode);
    }
    else {
        retval = (int) requested_mode;
    }
    return retval;
}

void ECAN_create_standard_message(raw_ecan_message_data * message_data, int standard_id, unsigned int * output) {
    /* This function will create a standard ID packet
     * DLC is set to 8.
     * data - points to the data payload
     * SID - Standard ID value to be used.
     * output - points to where the packet should be stored
     * which typically is an ECAN TX buffer */

    output[0] = (standard_id << 2) & 0x1FFC; /* SID, SRR = 0 and IDE = 0 */
    output[1] = 0x8;
    output[2] = message_data->word0;
    output[3] = message_data->word1;
    output[4] = message_data->word2;
    output[5] = message_data->word3;

}

void ECAN_create_extended_message(raw_ecan_message_data * message_data, int standard_id, long extended_id, unsigned int * output) {
    /* This function will create a extended ID packet
     * DLC is set to 8.
     * data - points to the data payload
     * sid - standard id
     * eid - extended id
     * output - points to where the packet should be stored
     * which typically is an ECAN TX buffer */


    output[0] = (standard_id << 2) | 0x3;
    output[1] = (int) (extended_id >> 6) & 0x0FFF;
    output[2] = ((int) (extended_id & 0x3F) << 10) | 0x8;
    output[2] = message_data->word0;
    output[3] = message_data->word1;
    output[4] = message_data->word2;
    output[5] = message_data->word3;


}


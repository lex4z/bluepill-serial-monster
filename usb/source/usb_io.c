/*
* MIT License 
* 
* Copyright (c) 2020 Kirill Kotyagin
*/

#if defined (STM32F1)
#include <stm32f1xx.h>
#elif defined(STM32F4)
#include <stm32f4xx.h>
#elif defined(STM32F7)
#include <stm32f7xx.h>
#endif

#include "system_interrupts.h"
#include "status_led.h"
#include "usb_descriptors.h"
#include "usb_core.h"
#include "usb_panic.h"
#include "usb_io.h"

#if !defined(OTG)
#define USB_PMAADDR ((uint32_t)0x40006000)
static volatile usb_btable_entity_t *usb_btable = (usb_btable_entity_t*)USB_PMAADDR;
#elif defined(OTG)
USB_OTG_OUTEndpointTypeDef EndPoint[8];
#define RCC_APB2ENR_OTGFSEN_Pos    (12U)
#define RCC_APB2ENR_OTGFSEN        (1U << RCC_APB2ENR_OTGFSEN_Pos)
#endif

/* USB Initialization After Reset */

void usb_io_reset() {
    #if defined (STM32F103xB)
    uint16_t offset = USB_BTABLE_SIZE;
    for (uint8_t ep_num=0; ep_num<USB_NUM_ENDPOINTS; ep_num++) {
        ep_reg_t ep_type = 0;
        ep_reg_t *ep_reg = ep_regs(ep_num);
        usb_btable[ep_num].tx_offset = offset;
        usb_btable[ep_num].tx_count = 0;
        offset += usb_endpoints[ep_num].tx_size;
        usb_btable[ep_num].rx_offset = offset;
        if (usb_endpoints[ep_num].rx_size > USB_BTABLE_SMALL_BLOCK_SIZE_LIMIT) {
            usb_btable[ep_num].rx_count = ((usb_endpoints[ep_num].rx_size / USB_BTABLE_LARGE_BLOCK_SIZE) - 1) << USB_COUNT0_RX_NUM_BLOCK_Pos;
            usb_btable[ep_num].rx_count |= USB_COUNT0_RX_BLSIZE;
        } else {
            usb_btable[ep_num].rx_count = (usb_endpoints[ep_num].rx_size / USB_BTABLE_SMALL_BLOCK_SIZE) << USB_COUNT0_RX_NUM_BLOCK_Pos;
        }
        offset += usb_endpoints[ep_num].rx_size;
        switch(usb_endpoints[ep_num].type) {
        case usb_endpoint_type_control:
            ep_type = USB_EP_CONTROL;
            break;
        case usb_endpoint_type_isochronous:
            ep_type = USB_EP_ISOCHRONOUS;
            break;
        case usb_endpoint_type_bulk:
            ep_type = USB_EP_BULK;
            break;
        case usb_endpoint_type_interrupt:
            ep_type = USB_EP_INTERRUPT;
            break;
        }
        *ep_reg = USB_EP_RX_VALID | USB_EP_TX_NAK | ep_type | ep_num;
    }
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM | USB_CNTR_SOFM;
    USB->DADDR = USB_DADDR_EF;


    #elif defined(OTG)
    USB_OTG_FS->GINTSTS &= ~0xFFFFFFFF;
    USB_OTG_DEVICE->DAINTMSK = USB_OTG_DAINTMSK_IEPM;
    
    // Очистить FIFO (RX и TX)
    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_RXFFLSH;
    while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);
    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_TXFFLSH;
    while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);

    for (uint8_t ep_num=1; ep_num<USB_NUM_ENDPOINTS; ep_num++) {
        uint32_t ep_type = 0;

        switch(usb_endpoints[ep_num].type) {
        case usb_endpoint_type_control:
            ep_type = USB_EP_CONTROL;
            break;
        case usb_endpoint_type_isochronous:
            ep_type = USB_EP_ISOCHRONOUS;
            break;
        case usb_endpoint_type_bulk:
            ep_type = USB_EP_BULK;
            break;
        case usb_endpoint_type_interrupt:
            ep_type = USB_EP_INTERRUPT;
            break;

        }
       
        
            USB_EP_IN(ep_num)->DIEPCTL |= ep_type | USB_OTG_DIEPCTL_SNAK   |	
                                                USB_OTG_DIEPCTL_USBAEP |
                                                USB_OTG_DOEPCTL_EPENA  |
                                                usb_endpoints[ep_num].tx_size;
        
            USB_EP_OUT(ep_num)->DOEPCTL |= ep_type | USB_OTG_DOEPCTL_SNAK  |
                                                USB_OTG_DIEPCTL_TXFNUM_0 << (ep_num+1) |
                                                USB_OTG_DOEPCTL_EPENA  |
                                                USB_OTG_DIEPCTL_USBAEP |
                                                USB_OTG_DOEPCTL_CNAK   |
                                                usb_endpoints[ep_num].rx_size;
            
        

        USB_OTG_DEVICE->DAINTMSK |=(USB_OTG_DAINTMSK_IEPM << ep_num) | (1U << (USB_OTG_DAINTMSK_OEPM + ep_num));
        //*ep_reg = USB_EP_RX_VALID | USB_EP_TX_NAK | ep_type | ep_num;
    }
    
    ;
    
    // 6. Включить нужные прерывания (RESET, ENUMDNE, RX, TX, SOF)
    USB_OTG_FS->GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
                       USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_IEPINT |
                       USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_SOFM;
    // 7. Включить глобальное прерывание
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    // Сбросить адрес устройства (адрес = 0)
    USB_OTG_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;
    #endif
    
}

void usb_io_init() {
    /* Force USB re-enumeration */
    #if defined (STM32F103xB)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRH &= ~GPIO_CRH_CNF12;
    GPIOA->CRH |= GPIO_CRH_MODE12_1;
    for (int i=0; i<0xFFFF; i++) {
        __NOP();
    }

    GPIOA->CRH &= ~GPIO_CRH_MODE12;
    GPIOA->CRH |= GPIO_CRH_CNF12_0;

    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

    /* Initialize USB */
    if (SystemCoreClock != RCC_MAX_FREQUENCY) {
        RCC->CFGR |= RCC_CFGR_USBPRE;
    }

    RCC->APB1ENR |= RCC_APB1ENR_USBEN;
    
    #elif defined(STM32F105xC)
    RCC->APB2ENR |= RCC_APB2ENR_OTGFSEN;

    GPIOA->CRH &= ~GPIO_CRH_CNF12;
    GPIOA->CRH |= GPIO_CRH_MODE12_1;
    for (int i=0; i<0xFFFF; i++) {
        __NOP();
    }
    GPIOA->CRH &= ~GPIO_CRH_MODE12;
    GPIOA->CRH |= GPIO_CRH_CNF12_0;

    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

    /* Initialize USB */
    if (SystemCoreClock != RCC_MAX_FREQUENCY) {
        RCC->CFGR |= RCC_CFGR_OTGFSPRE;
    }

    RCC->AHBENR |= RCC_AHBENR_OTGFSEN;
    #elif defined(STM32F7)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~GPIO_MODER_MODER12;
    GPIOA->MODER |= GPIO_MODER_MODER12_1; // Set PA12 to output mode
    for (int i=0; i<0xFFFF; i++) {
        __NOP();
    }
    GPIOA->MODER &= ~GPIO_MODER_MODER12; // Set PA12 to input mode
    GPIOA->MODER |= GPIO_MODER_MODER12_0;

    //NVIC_DisableIRQ(OTG_FS_IRQn);

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST){
        __NOP();
    }
    USB_OTG_FS->GINTSTS = 0xFFFFFFFF;
    USB_OTG_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    while ((USB_OTG_FS->GUSBCFG & USB_OTG_GUSBCFG_FDMOD) == 0){
        __NOP();
    }
    USB_OTG_FS->GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_RXFLVLM
    | USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_SOFM | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT;
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
    #endif

    #if defined(OTG)
    USB_OTG_FS->GAHBCFG = USB_OTG_GAHBCFG_GINT; /* Enable Global Interrupt */
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_TXFELVL;
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_PTXFELVL;

	USB_OTG_FS->GINTMSK = USB_OTG_GINTMSK_USBRST |
										//		USB_OTG_GINTMSK_ENUMDNEM |
												USB_OTG_GINTMSK_SOFM   |
												USB_OTG_GINTMSK_OEPINT |
												USB_OTG_GINTMSK_IEPINT |
												USB_OTG_GINTSTS_RXFLVL;
	/* Enable Global Interrupt for Reset, IN, OUT, RX not empty */
    
    USB_OTG_FS->GCCFG = USB_OTG_GCCFG_PWRDWN; /* Power up */
	USB_OTG_DEVICE->DCTL = USB_OTG_DCTL_SDIS;  /* Soft disconnect */
	USB_OTG_PCGCCTL->PCGCCTL = 0;
    USB_OTG_FS->GUSBCFG = USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL; /* Force device mode */

    USB_OTG_FS->GUSBCFG = USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL; /* Force device mode */
	USB_OTG_FS->GUSBCFG &= ~(uint32_t)(0x0FUL << 10UL) ;  /* USB turnaround time (according to AHB and ReferenceManual) */
	USB_OTG_FS->GUSBCFG |= (0x9 << 10);

    USB_EP_OUT(0)->DOEPTSIZ = 0;
	USB_EP_OUT(0)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)); /* This field is decremented to zero after a packet is written into the RxFIFO */
	USB_EP_OUT(0)->DOEPTSIZ |= USB_CDC_MAX_PACKET_SIZE; /* Set in descriptor  */
	USB_EP_OUT(0)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT;  /* STUPCNT==0x11 means, EP can recieve 3 packets. RM says to set STUPCNT = 3*/
	USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA); /* Clear NAK and enable EP0 */
    USB_EP_OUT(0)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT;

    USB_OTG_DEVICE->DCFG |= USB_OTG_DCFG_DSPD_Msk;  /* Device speed - FS */
	USB_OTG_FS->GINTSTS = 0xFFFFFFFF; /* Reset Global Interrupt status */
	USB_OTG_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;   /* Soft connect */
    set_FIFOs_sz();
    #else
    USB->CNTR = USB_CNTR_FRES;
    USB->BTABLE = 0;
    USB->DADDR = 0;
    USB->ISTR = 0;
    USB->CNTR = USB_CNTR_RESETM;
    #endif
}

/* Get Number of RX/TX Bytes Available  */
size_t usb_bytes_available(uint8_t ep_num) {
    #if defined(OTG)
    return (USB_EP_IN(ep_num)->DIEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ) >> USB_OTG_DOEPTSIZ_XFRSIZ_Pos;
    #else
    return usb_btable[ep_num].rx_count & USB_COUNT0_RX_COUNT0_RX;
    #endif
}

size_t usb_space_available(uint8_t ep_num) {
    #if defined(OTG)
        USB_OTG_INEndpointTypeDef *in_ep = USB_EP_IN(ep_num);
        if ((in_ep->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0) {
            return usb_endpoints[ep_num].tx_size;
        }
        return 0;
    #else
        ep_reg_t *ep_reg = ep_regs(ep_num);
        size_t tx_space_available = 0;
        if ((*ep_reg & USB_EPTX_STAT) == USB_EP_TX_NAK) {
            tx_space_available = usb_endpoints[ep_num].tx_size;
        }
        return tx_space_available;
    #endif
}
//fifo
void set_FIFOs_sz(){
	USB_OTG_FS->GRXFSIZ = RX_FIFO_SIZE;											/* all EPs RX FIFO RAM size */
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ((TX_EP0_FIFO_SIZE) << 16) | RX_FIFO_SIZE;						/* EP0 TX FIFO RAM size */
	
    for(uint8_t i = 0; i < (USB_NUM_ENDPOINTS-1)/2; i++){
        USB_OTG_FS->DIEPTXF[i] =  ((TX_EPn_FIFO_SIZE) << 16) | (RX_FIFO_SIZE+TX_EP0_FIFO_SIZE+TX_EPn_FIFO_SIZE*i);
    }
    
	for(uint32_t i = (USB_NUM_ENDPOINTS-1)/2; i < 0x10 ; i++){
		USB_OTG_FS->DIEPTXF[i] = 0;
	}
}

/* Endpoint Read/Write Operations */

int usb_read(uint8_t ep_num, void *buf, size_t buf_size) {
    #if defined(OTG)

    uint32_t rx_count = (USB_OTG_FS->GRXSTSP & USB_OTG_GRXSTSP_BCNT) >> 4;
    if (rx_count > buf_size) return -1;

    uint16_t residue = (rx_count%4==0) ? 0 : 1 ;
	uint32_t block_cnt = (uint32_t)((rx_count/4) + residue);
	uint8_t *tmp_ptr = buf;

	for (uint32_t i = 0; i < block_cnt; i++){
		*(uint32_t *)(void *)buf = USB_OTG_DFIFO(ep_num); // Read 4 bytes from the FIFO
		buf += 4;
	}

	if(ep_num!=0){	
		USB_EP_OUT(ep_num)->DOEPTSIZ = 0;			
		USB_EP_OUT(ep_num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << USB_OTG_DOEPTSIZ_PKTCNT_Pos)); 
		USB_EP_OUT(ep_num)->DOEPTSIZ |= usb_endpoints[ep_num].rx_size; 
		USB_EP_OUT(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}

    buf = tmp_ptr + rx_count;

    return rx_count;
    #else
    ep_reg_t *ep_reg = ep_regs(ep_num);
    usb_pbuffer_data_t *ep_buf = (usb_pbuffer_data_t *)(USB_PMAADDR + (usb_btable[ep_num].rx_offset<<1));
    pb_word_t ep_bytes_count = usb_btable[ep_num].rx_count & USB_COUNT0_RX_COUNT0_RX;
    pb_word_t words_left = ep_bytes_count>>1;
    pb_word_t *buf_p = (pb_word_t*)buf;
    if (ep_bytes_count > buf_size) return -1;
    usb_btable[ep_num].rx_count &= ~USB_COUNT0_RX_COUNT0_RX;
    while(words_left--) *buf_p++ = (ep_buf++)->data;
    if (ep_bytes_count & 0x01) *((uint8_t*)buf_p) = (uint8_t)ep_buf->data;
    *ep_reg = ((*ep_reg ^ USB_EP_RX_VALID) & (USB_EPREG_MASK | USB_EPRX_STAT)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
    return ep_bytes_count;
    #endif
}

size_t usb_send(uint8_t ep_num, const void *buf, size_t count) {
    #if defined(OTG)
    if (ep_num >= USB_NUM_ENDPOINTS) return 0;

    size_t tx_space = usb_endpoints[ep_num].tx_size;
    if (count > tx_space) count = tx_space;

    if(buf == NULL || count == 0){
        USB_EP_IN(ep_num)->DIEPTSIZ = 0;
        USB_EP_IN(ep_num)->DIEPTSIZ = ((USB_OTG_DIEPTSIZ_PKTCNT_Msk & ((1) << USB_OTG_DIEPTSIZ_PKTCNT_Pos))); /* One Packet */
        USB_EP_IN(ep_num)->DIEPTSIZ &= ~USB_OTG_DIEPTSIZ_XFRSIZ_Msk;  /* Zero Length */
        USB_EP_IN(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

        while(USB_EP_IN(ep_num)->DIEPTSIZ!=0){} /* make sure zlp is gone */

        USB_EP_OUT(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
    }else{
        uint32_t* buf_p = (uint32_t*)buf;
        uint16_t residue = (count%4==0) ? 0 : 1;
        uint32_t block_cnt = (uint32_t)((count/4) + residue);
            
        for (uint32_t i = 0; (i < block_cnt) ; i++){
            USB_OTG_DFIFO(ep_num) = *((uint32_t *)(void *)buf_p); // Write 4 bytes to the FIFO
            buf_p+=4;	
        }
    }
    return count;
    #else
    ep_reg_t *ep_reg = ep_regs(ep_num);
    usb_pbuffer_data_t *ep_buf = (usb_pbuffer_data_t *)(USB_PMAADDR + (usb_btable[ep_num].tx_offset<<1));
    pb_word_t *buf_p = (pb_word_t*)buf;
    pb_word_t words_left;
    size_t tx_space_available = usb_endpoints[ep_num].tx_size;
    if (count > tx_space_available) count = tx_space_available;
    words_left = count >> 1;
    while (words_left--) (ep_buf++)->data = *buf_p++;
    if (count & 0x01) (ep_buf)->data = (uint8_t)*buf_p;
    usb_btable[ep_num].tx_count = count;
    *ep_reg = ((*ep_reg ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPTX_STAT)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
    return count;
    #endif
}

size_t usb_circ_buf_read(uint8_t ep_num, circ_buf_t *buf, size_t buf_size) {
    #if defined(OTG)
    uint32_t rx_count = (USB_OTG_FS->GRXSTSP & USB_OTG_GRXSTSP_BCNT) >> 4;
    if (rx_count > buf_size) return -1;

    uint16_t residue = (rx_count%4==0) ? 0 : 1 ;
	uint32_t block_cnt = (uint32_t)((rx_count/4) + residue);
	//uint8_t  *tmp_ptr = buf;

	for (uint32_t i = 0; i < block_cnt; i++){
        buf->data[buf->head] = (uint8_t)(USB_OTG_DFIFO(ep_num));
        buf->head = (buf->head + 1) & (buf_size - 1);
        buf->data[buf->head] = (uint8_t)((USB_OTG_DFIFO(ep_num)) >> 8);
        buf->head = (buf->head + 1) & (buf_size - 1);
        buf->data[buf->head] = (uint8_t)((USB_OTG_DFIFO(ep_num)) >> 16);
        buf->head = (buf->head + 1) & (buf_size - 1);
        buf->data[buf->head] = (uint8_t)((USB_OTG_DFIFO(ep_num)) >> 24);
        buf->head = (buf->head + 1) & (buf_size - 1);
		buf += 4;
	}

    if (residue) {
        buf->data[buf->head] = (uint8_t)(USB_OTG_DFIFO(ep_num));
        buf->head = (buf->head + 1) & (buf_size - 1);
    }

    return rx_count;
    #else
    ep_reg_t *ep_reg = ep_regs(ep_num);
    usb_pbuffer_data_t *ep_buf = (usb_pbuffer_data_t *)(USB_PMAADDR + (usb_btable[ep_num].rx_offset<<1));
    pb_word_t ep_bytes_count = usb_btable[ep_num].rx_count & USB_COUNT0_RX_COUNT0_RX;
    size_t words_left = ep_bytes_count>>1;
    usb_btable[ep_num].rx_count &= ~USB_COUNT0_RX_COUNT0_RX;
    while(words_left--) {
        buf->data[buf->head] = (uint8_t)(ep_buf->data);
        buf->head = (buf->head + 1) & (buf_size - 1);
        buf->data[buf->head] = (uint8_t)(((ep_buf++)->data) >> 8);
        buf->head = (buf->head + 1) & (buf_size - 1);
    }
    if (ep_bytes_count & 0x1) {
        buf->data[buf->head] = (uint8_t)(ep_buf->data);
        buf->head = (buf->head + 1) & (buf_size - 1);
    }
    *ep_reg = ((*ep_reg ^ USB_EP_RX_VALID) & (USB_EPREG_MASK | USB_EPRX_STAT)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
    return ep_bytes_count;
    #endif
}

size_t usb_circ_buf_send(uint8_t ep_num, circ_buf_t *buf, size_t buf_size) {
    #if defined(OTG)
    if (ep_num >= USB_NUM_ENDPOINTS || buf == NULL) return 0;

    size_t count = circ_buf_count(buf->head, buf->tail, buf_size);
    size_t tx_space = usb_endpoints[ep_num].tx_size;
    if (count > tx_space) count = tx_space;

    USB_EP_IN(ep_num)->DIEPTSIZ = (count & USB_OTG_DIEPTSIZ_XFRSIZ_Msk) |
                                  (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
    USB_EP_IN(ep_num)->DIEPINT = USB_EP_IN(ep_num)->DIEPINT;
    USB_EP_IN(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    
    
    __IO uint32_t *fifo = &USB_OTG_DFIFO(ep_num);
    uint32_t word = 0;
    uint8_t byte_count = 0;

    for (size_t i = 0; i < count; i++) {
        word |= ((uint32_t)buf->data[buf->tail]) << (8 * byte_count);
        buf->tail = (buf->tail + 1) & (buf_size - 1);
        byte_count++;
        if (byte_count == 4 || i == count - 1) {
            *fifo = word;
            word = 0;
            byte_count = 0;
        }
    }
    return count;
    #else
    ep_reg_t *ep_reg = ep_regs(ep_num);
    usb_pbuffer_data_t *ep_buf = (usb_pbuffer_data_t *)(USB_PMAADDR + (usb_btable[ep_num].tx_offset<<1));
    size_t count = circ_buf_count(buf->head, buf->tail, buf_size);
    size_t tx_space_available = usb_endpoints[ep_num].tx_size;
    size_t words_left;
    if (count > tx_space_available) count = tx_space_available;
    words_left = count >> 1;
    while (words_left--) {
        pb_word_t pb_word = buf->data[buf->tail];
        buf->tail = (buf->tail + 1) & (buf_size - 1);
        pb_word |= ((uint16_t)buf->data[buf->tail]) << 8;
        buf->tail = (buf->tail + 1) & (buf_size - 1);
        (ep_buf++)->data = pb_word;
    }
    if (count & 0x1) {
        (ep_buf)->data = buf->data[buf->tail];
        buf->tail = (buf->tail + 1) & (buf_size - 1);
    }
    usb_btable[ep_num].tx_count = count;
    *ep_reg = ((*ep_reg ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPTX_STAT)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
    return count;
    #endif
}

/* Endpoint Stall */

void usb_endpoint_set_stall(uint8_t ep_num, usb_endpoint_direction_t ep_direction, uint8_t ep_stall) {
    #if defined(OTG)
    if (ep_direction == usb_endpoint_direction_in) {
        if (ep_stall) {
            USB_EP_IN(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
        } else {
            USB_EP_IN(ep_num)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
            USB_EP_IN(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;
        }
    } else {
        if (ep_stall) {
            USB_EP_OUT(ep_num)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
        } else {
            USB_EP_OUT(ep_num)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
            USB_EP_OUT(ep_num)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;
        }
    }
    #else
    ep_reg_t *ep_reg = ep_regs(ep_num);
    if ((*ep_reg & USB_EP_T_FIELD) != USB_EP_ISOCHRONOUS) {
        if (ep_direction == usb_endpoint_direction_in) {
            if ((*ep_reg & USB_EPTX_STAT) != USB_EP_TX_DIS) {
                if (ep_stall) {
                    *ep_reg = ((*ep_reg ^ USB_EP_TX_STALL) & (USB_EPREG_MASK | USB_EPTX_STAT)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
                } else {
                    *ep_reg = ((*ep_reg ^ USB_EP_TX_NAK) & (USB_EPREG_MASK | USB_EPTX_STAT | USB_EP_DTOG_TX)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
                }
            }
        } else {
            if ((*ep_reg & USB_EPRX_STAT) != USB_EP_RX_DIS) {
                if (ep_stall) {
                    *ep_reg = ((*ep_reg ^ USB_EP_RX_STALL) & (USB_EPREG_MASK | USB_EPRX_STAT)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
                } else {
                    *ep_reg = ((*ep_reg ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPRX_STAT | USB_EP_DTOG_RX)) | (USB_EP_CTR_RX | USB_EP_CTR_TX);
                }
            }
        }
    }
    #endif
}

int usb_endpoint_is_stalled(uint8_t ep_num, usb_endpoint_direction_t ep_direction) {
    #if defined(OTG)
    if (ep_direction == usb_endpoint_direction_in) {
        return (USB_EP_IN(ep_num)->DIEPCTL & USB_OTG_DIEPCTL_STALL) != 0;
    } else {
        return (USB_EP_OUT(ep_num)->DOEPCTL & USB_OTG_DOEPCTL_STALL) != 0;
    }
    #else
    if (ep_direction == usb_endpoint_direction_in) {
        return (*ep_regs(ep_num) & USB_EPTX_STAT) == USB_EP_TX_STALL;
    }
    return (*ep_regs(ep_num) & USB_EPRX_STAT) == USB_EP_RX_STALL;
    #endif
}

/* USB Polling */

static uint8_t usb_transfer_led_timer = 0;

uint16_t istr;

void usb_poll() {
    #if defined(OTG)
    uint32_t gintsts = USB_OTG_FS->GINTSTS;
    uint32_t gintmsk = USB_OTG_FS->GINTMSK;
    uint32_t pending = gintsts & gintmsk;

    if (pending & USB_OTG_GINTSTS_USBRST) {
        USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_USBRST);
        usb_device_handle_reset();
    }

    if (pending & USB_OTG_GINTSTS_USBSUSP) {
        USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_USBSUSP);
        USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
        status_led_set(0);
        usb_device_handle_suspend();
    }

    if (pending & USB_OTG_GINTSTS_WKUINT) {
        USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_WKUINT);
        USB_OTG_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
        usb_device_handle_wakeup();
    }

    if (pending & USB_OTG_GINTSTS_SOF) {
        USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_SOF);
        if (usb_transfer_led_timer) {
            status_led_set(--usb_transfer_led_timer);
        }
        usb_device_handle_frame();
    }

    if (pending & USB_OTG_GINTSTS_IEPINT) {
        for (uint8_t ep = 0; ep < USB_MAX_ENDPOINTS; ep++) {
            if (USB_OTG_DEVICE->DAINT & (1 << ep)) {
                if (usb_endpoints[ep].event_handler) {
                    usb_endpoints[ep].event_handler(ep, usb_endpoint_event_data_sent);
                }
                USB_EP_IN(ep)->DIEPINT = USB_EP_IN(ep)->DIEPINT;
            }
        }
    }

    if (pending & USB_OTG_GINTSTS_OEPINT) {
        for (uint8_t ep = 0; ep < USB_MAX_ENDPOINTS; ep++) {
            if (USB_OTG_DEVICE->DAINT & (1 << (16 + ep))) {
                uint32_t doepint = USB_EP_OUT(ep)->DOEPINT;
                USB_EP_OUT(ep)->DOEPINT = doepint;

                usb_endpoint_event_t evt = (doepint & USB_OTG_DOEPINT_STUP) ?
                    usb_endpoint_event_setup : usb_endpoint_event_data_received;

                if (usb_endpoints[ep].event_handler) {
                    usb_endpoints[ep].event_handler(ep, evt);
                }
            }
        }
    }

    if (pending & USB_OTG_GINTSTS_RXFLVL) {
        uint32_t grxstsp = USB_OTG_FS->GRXSTSP;
        uint8_t ep = grxstsp & USB_OTG_GRXSTSP_EPNUM_Msk;
        uint32_t bcnt = (grxstsp & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;
        uint32_t pktsts = (grxstsp & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;

        if (pktsts == 0x02 || pktsts == 0x06) { // DATA_UPDT or SETUP_UPDT
            EndPoint[ep].DOEPTSIZ = bcnt;
            //read_Fifo(ep, bcnt);
        }
    }
    #else
    istr = USB->ISTR;
    if (istr & USB_ISTR_CTR) {
        uint8_t ep_num = USB->ISTR & USB_ISTR_EP_ID;
        ep_reg_t *ep_reg = ep_regs(ep_num);
        if (*ep_reg & USB_EP_CTR_TX) {
            *ep_reg = ((*ep_reg & (USB_EP_T_FIELD | USB_EP_KIND | USB_EPADDR_FIELD)) | USB_EP_CTR_RX);
            if (usb_endpoints[ep_num].event_handler) {
                usb_endpoints[ep_num].event_handler(ep_num, usb_endpoint_event_data_sent);
            }
        } else {
            usb_endpoint_event_t ep_event = usb_endpoint_event_data_received;
            if (*ep_reg & USB_EP_SETUP) {
                ep_event = usb_endpoint_event_setup;
            }
            *ep_reg = ((*ep_reg & (USB_EP_T_FIELD | USB_EP_KIND | USB_EPADDR_FIELD)) | USB_EP_CTR_TX);
            if (usb_endpoints[ep_num].event_handler) {
                usb_endpoints[ep_num].event_handler(ep_num, ep_event);
            }
        }
        usb_transfer_led_timer = USB_TRANSFER_LED_TIME;
        status_led_set(1);
    } else if (istr & USB_ISTR_RESET) {
        USB->ISTR = (uint16_t)(~USB_ISTR_RESET);
        usb_device_handle_reset();   
    } else if (istr & USB_ISTR_SUSP) {
        USB->ISTR = (uint16_t)(~USB_ISTR_SUSP);
        USB->CNTR |= USB_CNTR_FSUSP;
        status_led_set(0);
        usb_device_handle_suspend();
    } else if (istr & USB_ISTR_WKUP) {
        USB->ISTR = (uint16_t)(~USB_ISTR_WKUP);
        USB->CNTR &= ~USB_CNTR_FSUSP;
        usb_device_handle_wakeup();
    } else if (istr & USB_ISTR_SOF) {
        USB->ISTR = (uint16_t)(~USB_ISTR_SOF);
        if (usb_transfer_led_timer) {
            status_led_set(--usb_transfer_led_timer);
        }
        usb_device_handle_frame();
    }
    #endif
    usb_device_poll();
}

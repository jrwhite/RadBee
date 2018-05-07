#include <stdlib.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dwc/otg_fs.h>

#define REBASE(x)        (MMIO32((x) + (USB_OTG_FS_BASE)))

#define DEBUG

#define RX_FIFO_SIZE 128

#define LED5_OFF (gpio_clear(GPIOD, GPIO14))
#define LED5_ON (gpio_set(GPIOD, GPIO14))
#define LED6_OFF (gpio_clear(GPIOD, GPIO15))
#define LED6_ON (gpio_set(GPIOD, GPIO15))
#define LED4_ON (gpio_set(GPIOD, GPIO12))
#define LED4_OFF (gpio_clear(GPIOD, GPIO12))
#define LED3_OFF (gpio_clear(GPIOD, GPIO13))
#define LED3_ON (gpio_set(GPIOD, GPIO13))

#define LED3_TOGGLE (gpio_toggle(GPIOD, GPIO13))
#define LED4_TOGGLE (gpio_toggle(GPIOD, GPIO12))
#define LED5_TOGGLE (gpio_toggle(GPIOD, GPIO14))
#define LED6_TOGGLE (gpio_toggle(GPIOD, GPIO15))

#define OTG_GOTGCTL_CIDSTS (1 << 16)
#define OTG_HPRT_PCTS (1 << 0)
#define OTG_HNPTXFSIZ (0x028)
#define OTG_FS_HNPTXFSIZ MMIO32(USB_OTG_FS_BASE + OTG_HNPTXFSIZ)

#define _is_host (OTG_FS_GINTSTS & OTG_GINTSTS_CMOD)
#define _is_usba (OTG_FS_GOTGCTL & OTG_GOTGCTL_CIDSTS)
#define _is_dev_connected (OTG_FS_HPRT & OTG_HPRT_PCTS)

typedef struct NptxFifo {
   size_t mem_top;
   uint8_t * buf;  
} NptxFifo;

static NptxFifo nptx_fifo;

typedef struct PtxFifo {
    size_t mem_top;
    uint8_t * buf;
} PtxFifo;

static PtxFifo ptx_fifo;

inline bool _dev_detected(void) {
    if (OTG_FS_HPRT & OTG_HPRT_PCDET) {
        /* clear interrupt */
        OTG_FS_HPRT |= OTG_HPRT_PCDET;
        return true;
    } else {
        return false;
    }
}

inline bool _port_change_detected(void) {
    if (OTG_FS_HPRT & OTG_HPRT_PENCHNG) {
        // OTG_FS_HPRT |= OTG_HPRT_PENCHNG;
        return true;
    } else {
        return false;
    }
}

inline void platform_init(void) {
    /* 120 MHz */
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);

    /* Setup USB pins */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_OTGFS);
	// OTG_FS_GUSBCFG |= OTG_GUSBCFG_PHYSEL;

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO9 | GPIO10 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO10 | GPIO11 | GPIO12);

    /* Setup LEDs */
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO14 | GPIO12 | GPIO13 | GPIO15);
    LED5_ON;

    /* Setup VBUS charge pump */
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0);
    gpio_set(GPIOC, GPIO0);
}

static void usb_otg_ab_test(void) {
    /**
     * This tests the device's ability to distinguish 'A' from 'B'
     * and host-mode operation from device-mode operation.
     * 
     * LED5 On->Off indicating host-mode operation
     * 
     * LED6 Off->On indicating 'A' device
     */
    // while (_is_usba);
    while (!(OTG_FS_GINTSTS & OTG_GINTSTS_CIDSCHG));

    if (_is_usba) LED6_ON;
    if (_is_host) LED5_OFF;

    while (true) {
        if (OTG_FS_GINTSTS & OTG_GINTSTS_CIDSCHG){ 
            LED6_TOGGLE;
            OTG_FS_GINTSTS |= OTG_GINTSTS_CIDSCHG;
        }

        _is_host ? LED5_OFF, LED4_ON : LED5_ON, LED4_OFF;
    }
}

static void usb_otg_connect_test(void) {
    /**
     * Host connect test shows a device connecting to the host.
     * LED5 On->Off when connection is detected
     * LED6 On if device connected, off otherwise
     * LED3 On if in host mode
     */

    while (true) {
        _is_host ? LED3_ON : LED3_OFF;
        _is_dev_connected ? LED6_ON : LED6_OFF;
        if (_dev_detected()) LED5_OFF;
    }

}

inline void usb_otg_core_init(void) {
    /* Wait for AHB idle. */
	// while (!(OTG_FS_GRSTCTL & OTG_GRSTCTL_AHBIDL));

	/* Do core soft reset. */
	// OTG_FS_GRSTCTL |= OTG_GRSTCTL_CSRST;
	// while (OTG_FS_GRSTCTL & OTG_GRSTCTL_CSRST);

    /* USB OTG Core initialization taken from datasheet 34.17.1 */

    /**
     * 1. Program OTG_FS_GAHBCFG register
     * - global interrupt mask GAHMASK = 1
     * - RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
     * - Periodic TxFIFO empty level TXFELVL = 1 (tigger interrupt when buff completely empty)
     */

    OTG_FS_GAHBCFG |= OTG_GAHBCFG_GINT;
    // OTG_FS_GINTSTS = OTG_GINTSTS_MMIS;
    OTG_FS_GAHBCFG |= OTG_GAHBCFG_TXFELVL;

    /**
     * 2. Program the OTG_FS_GUSBCFG register
     * - HNP capable
     * - SRP capable
     * - FS timeout calibration (just 0 for now) 
     * - USB turnaroud time (14.2-15 MHz)
     */

    // OTG_FS_GUSBCFG |= OTG_GUSBCFG_HNPCAP;
    // OTG_FS_GUSBCFG |= OTG_GUSBCFG_SRPCAP;
    OTG_FS_GUSBCFG |= (OTG_GUSBCFG_TOCAL & 0b00);
    OTG_FS_GUSBCFG |= (OTG_GUSBCFG_TRDT_MASK & 0xF);

    /**
     * 3. Unmask interrupts in OTG_FS_GINTMASK
     * - OTG interrupt mask
     * - Mode mismatch interrupt mask
     */

    OTG_FS_GINTMSK |= OTG_GINTMSK_OTGINT;
    OTG_FS_GINTMSK |= OTG_GINTMSK_MMISM;
    OTG_FS_GINTMSK |= OTG_GINTMSK_CIDSCHGM;

    /* Enable VBUS charge pump */
    gpio_clear(GPIOC, GPIO0);
}

void usb_otg_host_init(void) {
    /* Initialize USB Host */

    // while ((_is_host) == 0) {}
    /* Force host mode */
    // OTG_FS_GUSBCFG |= OTG_GUSBCFG_FHMOD;

    /* Unmask HPRTINT interrupt in OTG_FS_GINTMASK */
    OTG_FS_GINTMSK |= OTG_GINTMSK_PRTIM;
    OTG_FS_GINTMSK |= OTG_GINTMSK_HCIM;

    /* Set full-speed host */
    OTG_FS_HCFG |= OTG_HCFG_FSLSS;

	/* Restart the PHY clock. */
	OTG_FS_PCGCCTL = 0;

    /* Drive VBUS */
    OTG_FS_HPRT |= OTG_HPRT_PPWR;

    /* Wait for device to connect */
    while(!_dev_detected());
    LED6_ON;

    /* Reset the device */
    OTG_FS_HPRT ^= OTG_HPRT_PRST;
    for (uint32_t i = 0; i < 500000; i++) { /*500000 for a bit less than 20 ms */
        __asm__("nop");
    }
    OTG_FS_HPRT ^= OTG_HPRT_PRST;
    LED4_ON;

    while(!_port_change_detected());

    LED3_ON;
    /* Specify FIFO size */
    OTG_FS_GRXFSIZ = RX_FIFO_SIZE;
    /* Setup non-periodic Tx FIFO */
    nptx_fifo.mem_top = RX_FIFO_SIZE;
    nptx_fifo.buf = malloc(nptx_fifo.mem_top);
    OTG_FS_HNPTXFSIZ |= (nptx_fifo.mem_top << 16) | ((uint32_t) nptx_fifo.buf);
    /* Setup periodic Tx FIFO */
    ptx_fifo.mem_top = RX_FIFO_SIZE;
    ptx_fifo.buf = malloc(ptx_fifo.mem_top);
    OTG_FS_HPTXFSIZ |= (ptx_fifo.mem_top << 16) | ((uint32_t) ptx_fifo.buf);

}

inline void usb_otg_device_init(void) {
    /**
     * Initialize as a USB device
     * 
     * - Set USB OTG Full-speed device
     * - Set non-zero-length status OUT handshake
     *  (send the received OUT packet to the application)
     */

    OTG_FS_DCFG |= OTG_DCFG_DSPD;

    /**
     * Unmask device interrupts in OTG_FS_GINTMASK
     * - USB reset
     * - Enumeration done
     * - Early suspend
     * - USB suspend
     * - SOF
     */

    OTG_FS_GINTMSK |= OTG_GINTMSK_USBRST;
    OTG_FS_GINTMSK |= OTG_GINTMSK_ENUMDNEM;
    OTG_FS_GINTMSK |= OTG_GINTMSK_ESUSPM;
    OTG_FS_GINTMSK |= OTG_GINTMSK_USBSUSPM;
    OTG_FS_GINTMSK |= OTG_GINTMSK_SOFM;

    /**
     * Enable VBUS line sensing in "B" device mode
     * 
     * - set VBUSBSEN in OTG_FS_GCCFG
     */

    OTG_FS_GCCFG |= OTG_GCCFG_VBUSBSEN;
    
}

int main(void) {

    platform_init();
    
    usb_otg_core_init();

    usb_otg_host_init();

    // usb_otg_ab_test();
    usb_otg_connect_test();


    // usbd_dev = usbd_init(&otgfs_usb_driver, &dev)

	while (1) {
    }
}
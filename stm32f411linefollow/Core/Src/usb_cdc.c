/*
 * usb_cdc.c - Minimal bare-metal USB CDC-ACM for STM32F401 (OTG_FS, device mode)
 *
 * Requirements
 *   - SYSCLK 84 MHz from the 25 MHz HSE, PLLQ giving exactly 48 MHz
 *     (M=25, N=336, P=4, Q=7)
 *   - Leave USB_OTG_FS DISABLED in CubeMX (this file defines OTG_FS_IRQHandler)
 *   - PA11 / PA12 are used for USB
 *
 * Endpoints (same layout as ST's CDC example)
 *   EP0       control
 *   EP1 IN    bulk 64   (device -> PC, sensor data)
 *   EP1 OUT   bulk 64   (PC -> device)
 *   EP2 IN    interrupt 8 (CDC notification, never used but required by the class)
 *
 * Everything runs from the OTG_FS interrupt. The API is non-blocking.
 */
#include "stm32f4xx.h"
#include "usb_cdc.h"
#include <string.h>

/* ---------------------------------------------------------------------
 * Register access
 * ------------------------------------------------------------------- */
#define USB_BASE        ((uint32_t)USB_OTG_FS)
#define USB_G           ((USB_OTG_GlobalTypeDef *)USB_BASE)
#define USB_D           ((USB_OTG_DeviceTypeDef *)(USB_BASE + 0x800U))
#define IN_EP(n)        ((USB_OTG_INEndpointTypeDef  *)(USB_BASE + 0x900U + ((n) * 0x20U)))
#define OUT_EP(n)       ((USB_OTG_OUTEndpointTypeDef *)(USB_BASE + 0xB00U + ((n) * 0x20U)))
#define FIFO_ADDR(n)    ((volatile uint32_t *)(USB_BASE + 0x1000U + ((n) * 0x1000U)))
#define PCGCCTL         (*(volatile uint32_t *)(USB_BASE + 0xE00U))

/* ---------------------------------------------------------------------
 * Bit definitions (positions from RM0368 OTG_FS chapter)
 * ------------------------------------------------------------------- */
#define GAHBCFG_GINT        (1U << 0)

#define GUSBCFG_PHYSEL      (1U << 6)
#define GUSBCFG_TRDT_MASK   (0xFU << 10)
#define GUSBCFG_TRDT_FS     (6U << 10)          /* HCLK >= 32 MHz */
#define GUSBCFG_FHMOD       (1U << 29)
#define GUSBCFG_FDMOD       (1U << 30)

#define GRSTCTL_CSRST       (1U << 0)
#define GRSTCTL_RXFFLSH     (1U << 4)
#define GRSTCTL_TXFFLSH     (1U << 5)
#define GRSTCTL_TXFNUM_ALL  (0x10U << 6)
#define GRSTCTL_AHBIDL      (1U << 31)

#define GCCFG_PWRDWN        (1U << 16)
#define GCCFG_NOVBUSSENS    (1U << 21)

#define GINT_RXFLVL         (1U << 4)
#define GINT_USBRST         (1U << 12)
#define GINT_ENUMDNE        (1U << 13)
#define GINT_IEPINT         (1U << 18)
#define GINT_OEPINT         (1U << 19)

#define DCFG_DSPD_FS        (3U << 0)
#define DCFG_DAD_MASK       (0x7FU << 4)

#define DCTL_SDIS           (1U << 1)
#define DCTL_CGINAK         (1U << 8)

#define EPCTL_MPSIZ_MASK    0x7FFU
#define EPCTL_USBAEP        (1U << 15)
#define EPCTL_STALL         (1U << 21)
#define EPCTL_CNAK          (1U << 26)
#define EPCTL_SNAK          (1U << 27)
#define EPCTL_SD0PID        (1U << 28)
#define EPCTL_EPENA         (1U << 31)
#define EPCTL_EPTYP_BULK    (2U << 18)
#define EPCTL_EPTYP_INTR    (3U << 18)
#define EPCTL_TXFNUM(n)     ((uint32_t)(n) << 22)

#define DIEPINT_XFRC        (1U << 0)
#define DOEPINT_XFRC        (1U << 0)
#define DOEPINT_STUP        (1U << 3)

#define DIEPMSK_XFRCM       (1U << 0)
#define DOEPMSK_XFRCM       (1U << 0)
#define DOEPMSK_STUPM       (1U << 3)

#define PKTSTS_OUT_DATA     2U
#define PKTSTS_SETUP_DATA   6U

/* FIFO layout in 32-bit words (total available on F401 = 320) */
#define RX_FIFO_WORDS       128U
#define TX0_FIFO_WORDS      32U
#define TX1_FIFO_WORDS      64U
#define TX2_FIFO_WORDS      16U

#define EP_MPS              64U
#define CDC_TX_MAX          63U     /* stay below 64 so no ZLP is ever needed */

/* ---------------------------------------------------------------------
 * USB descriptors
 * ------------------------------------------------------------------- */
static const uint8_t dev_desc[18] = {
    18, 0x01,               /* bLength, bDescriptorType = DEVICE */
    0x00, 0x02,             /* bcdUSB 2.00 */
    0x02, 0x00, 0x00,       /* class CDC, subclass 0, protocol 0 */
    64,                     /* EP0 max packet */
    0x83, 0x04,             /* VID 0x0483 (STMicroelectronics) */
    0x40, 0x57,             /* PID 0x5740 (Virtual COM Port) */
    0x00, 0x02,             /* bcdDevice */
    1, 2, 3,                /* iManufacturer, iProduct, iSerialNumber */
    1                       /* bNumConfigurations */
};

static const uint8_t cfg_desc[67] = {
    /* Configuration */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x32,
    /* Interface 0: CDC communication */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* CDC header functional descriptor */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* Call management functional descriptor */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* ACM functional descriptor */
    0x04, 0x24, 0x02, 0x02,
    /* Union functional descriptor */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* EP2 IN, interrupt, 8 bytes, interval 16 ms */
    0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x10,
    /* Interface 1: CDC data */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* EP1 OUT, bulk, 64 bytes */
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
    /* EP1 IN, bulk, 64 bytes */
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00
};

static const uint8_t lang_desc[4] = { 4, 0x03, 0x09, 0x04 };    /* English (US) */

static const char *const str_table[3] = {
    "Line Follower Bot",    /* 1: manufacturer */
    "LF Debug Port",        /* 2: product */
    "000001"                /* 3: serial */
};

/* ---------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------- */
typedef struct {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} setup_t;

static volatile uint8_t  configured;
static volatile uint8_t  dtr;
static volatile uint8_t  tx_busy;

static uint32_t          setup_words[2];
static volatile uint8_t  ep0_out_pending;       /* waiting for an OUT data stage */
static volatile uint16_t ep0_rx_len;
static uint8_t           ep0_rx[EP_MPS];

static uint8_t           line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 }; /* 115200 8N1 */

static uint8_t           rx_buf[EP_MPS];
static volatile uint16_t rx_len;
static volatile uint8_t  rx_ready;

/* ---------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------- */
static void delay_ms(uint32_t ms)
{
    /* Rough busy-wait, good enough for the init delays */
    volatile uint32_t n = (SystemCoreClock / 5000U) * ms;
    while (n--) { }
}

static void fifo_write(uint8_t ep, const uint8_t *src, uint16_t len)
{
    volatile uint32_t *fifo = FIFO_ADDR(ep);
    for (uint16_t i = 0; i < len; i += 4U) {
        uint32_t w = 0;
        for (uint8_t b = 0; b < 4U && (uint16_t)(i + b) < len; b++) {
            w |= (uint32_t)src[i + b] << (8U * b);
        }
        *fifo = w;
    }
}

/* All OUT data is popped from the FIFO 0 address. Copies at most 'cap' bytes
 * but always drains the full 'len' bytes from the FIFO. */
static void fifo_read(uint8_t *dst, uint16_t len, uint16_t cap)
{
    volatile uint32_t *fifo = FIFO_ADDR(0);
    for (uint16_t i = 0; i < len; i += 4U) {
        uint32_t w = *fifo;
        for (uint8_t b = 0; b < 4U && (uint16_t)(i + b) < len; b++) {
            if ((uint16_t)(i + b) < cap) {
                dst[i + b] = (uint8_t)(w >> (8U * b));
            }
        }
    }
}

/* Start an IN transfer and load the data into the TX FIFO */
static void ep_in_send(uint8_t ep, const uint8_t *data, uint16_t len)
{
    uint32_t pkts = (len == 0U) ? 1U : (uint32_t)((len + EP_MPS - 1U) / EP_MPS);

    IN_EP(ep)->DIEPTSIZ = (pkts << 19) | len;
    IN_EP(ep)->DIEPCTL |= EPCTL_EPENA | EPCTL_CNAK;
    if (len) {
        fifo_write(ep, data, len);
    }
}

/* Control IN data stage (or a zero-length status stage when len == 0) */
static void ep0_send(const uint8_t *data, uint16_t len, uint16_t w_length)
{
    if (len > w_length) len = w_length;
    if (len > 127U)     len = 127U;     /* EP0 XFRSIZ field is 7 bits */
    ep_in_send(0, data, len);
}

/* Arm EP0 OUT to receive the next SETUP packet / status / data stage */
static void ep0_arm_out(void)
{
    OUT_EP(0)->DOEPTSIZ = (3U << 29) | (1U << 19) | (3U * 8U);  /* STUPCNT=3, PKTCNT=1, XFRSIZ=24 */
    OUT_EP(0)->DOEPCTL |= EPCTL_EPENA | EPCTL_CNAK;
}

static void ep0_stall(void)
{
    IN_EP(0)->DIEPCTL  |= EPCTL_STALL;
    OUT_EP(0)->DOEPCTL |= EPCTL_STALL;
}

static void ep1_arm_out(void)
{
    rx_len   = 0;
    rx_ready = 0;
    OUT_EP(1)->DOEPTSIZ = (1U << 19) | EP_MPS;
    OUT_EP(1)->DOEPCTL |= EPCTL_EPENA | EPCTL_CNAK;
}

static void cdc_endpoints_open(void)
{
    /* EP1 IN: bulk, uses TX FIFO 1 */
    IN_EP(1)->DIEPCTL = EP_MPS | EPCTL_USBAEP | EPCTL_EPTYP_BULK |
                        EPCTL_TXFNUM(1) | EPCTL_SD0PID | EPCTL_SNAK;

    /* EP2 IN: interrupt (notification), uses TX FIFO 2 */
    IN_EP(2)->DIEPCTL = 8U | EPCTL_USBAEP | EPCTL_EPTYP_INTR |
                        EPCTL_TXFNUM(2) | EPCTL_SD0PID | EPCTL_SNAK;

    /* EP1 OUT: bulk */
    OUT_EP(1)->DOEPCTL = EP_MPS | EPCTL_USBAEP | EPCTL_EPTYP_BULK | EPCTL_SD0PID;
    ep1_arm_out();

    tx_busy = 0;
}

/* ---------------------------------------------------------------------
 * Control request handling
 * ------------------------------------------------------------------- */
static uint16_t build_string(const char *s, uint8_t *out)
{
    uint8_t n = 0;
    while (s[n] && n < 30U) {
        out[2U + 2U * n] = (uint8_t)s[n];
        out[3U + 2U * n] = 0;
        n++;
    }
    out[0] = (uint8_t)(2U + 2U * n);
    out[1] = 0x03;
    return out[0];
}

static void req_get_descriptor(const setup_t *r)
{
    static uint8_t strbuf[64];
    const uint8_t *p = 0;
    uint16_t len = 0;
    uint8_t type = (uint8_t)(r->wValue >> 8);
    uint8_t idx  = (uint8_t)(r->wValue & 0xFFU);

    switch (type) {
    case 0x01:  p = dev_desc;  len = sizeof dev_desc;  break;
    case 0x02:  p = cfg_desc;  len = sizeof cfg_desc;  break;
    case 0x03:
        if (idx == 0) {
            p = lang_desc; len = sizeof lang_desc;
        } else if (idx <= 3U) {
            len = build_string(str_table[idx - 1U], strbuf);
            p = strbuf;
        }
        break;
    default:    /* device qualifier, BOS, etc. -> not supported (full-speed only) */
        break;
    }

    if (p) ep0_send(p, len, r->wLength);
    else   ep0_stall();
}

static void handle_standard_request(const setup_t *r)
{
    static const uint8_t zero2[2] = { 0, 0 };

    switch (r->bRequest) {
    case 0x06:  /* GET_DESCRIPTOR */
        req_get_descriptor(r);
        break;

    case 0x05:  /* SET_ADDRESS */
        USB_D->DCFG = (USB_D->DCFG & ~DCFG_DAD_MASK) | (((uint32_t)r->wValue & 0x7FU) << 4);
        ep0_send(0, 0, 0);
        break;

    case 0x09:  /* SET_CONFIGURATION */
        if ((r->wValue & 0xFFU) == 1U) {
            cdc_endpoints_open();
            configured = 1;
        } else {
            configured = 0;
        }
        ep0_send(0, 0, 0);
        break;

    case 0x08: { /* GET_CONFIGURATION */
        uint8_t c = configured ? 1U : 0U;
        ep0_send(&c, 1, r->wLength);
        break;
    }

    case 0x00:  /* GET_STATUS */
        ep0_send(zero2, 2, r->wLength);
        break;

    case 0x0A: { /* GET_INTERFACE */
        uint8_t alt = 0;
        ep0_send(&alt, 1, r->wLength);
        break;
    }

    case 0x01:  /* CLEAR_FEATURE */
    case 0x03:  /* SET_FEATURE */
    case 0x0B:  /* SET_INTERFACE */
        ep0_send(0, 0, 0);
        break;

    default:
        ep0_stall();
        break;
    }
}

static void handle_class_request(const setup_t *r)
{
    switch (r->bRequest) {
    case 0x20:  /* SET_LINE_CODING: 7-byte OUT data stage, then status */
        ep0_out_pending = 1;
        break;

    case 0x21:  /* GET_LINE_CODING */
        ep0_send(line_coding, sizeof line_coding, r->wLength);
        break;

    case 0x22:  /* SET_CONTROL_LINE_STATE (bit0 = DTR) */
        dtr = (uint8_t)(r->wValue & 1U);
        ep0_send(0, 0, 0);
        break;

    case 0x23:  /* SEND_BREAK */
        ep0_send(0, 0, 0);
        break;

    default:
        ep0_stall();
        break;
    }
}

static void handle_setup(void)
{
    setup_t r;
    memcpy(&r, setup_words, sizeof r);

    ep0_out_pending = 0;
    ep0_rx_len = 0;

    switch (r.bmRequestType & 0x60U) {
    case 0x00:  handle_standard_request(&r); break;
    case 0x20:  handle_class_request(&r);    break;
    default:    ep0_stall();                 break;
    }

    ep0_arm_out();
}

/* EP0 OUT transfer complete: either the OUT data stage (SET_LINE_CODING)
 * or the host's status stage after a control-IN transfer. */
static void ep0_out_done(void)
{
    if (ep0_out_pending && ep0_rx_len) {
        if (ep0_rx_len >= sizeof line_coding) {
            memcpy(line_coding, ep0_rx, sizeof line_coding);
        }
        ep0_out_pending = 0;
        ep0_rx_len = 0;
        ep0_send(0, 0, 0);          /* status stage */
    }
    ep0_arm_out();
}

/* ---------------------------------------------------------------------
 * Interrupt sub-handlers
 * ------------------------------------------------------------------- */
static void usb_reset(void)
{
    uint8_t i;

    USB_G->GRSTCTL = GRSTCTL_TXFFLSH | GRSTCTL_TXFNUM_ALL;
    while (USB_G->GRSTCTL & GRSTCTL_TXFFLSH) { }

    for (i = 0; i < 4U; i++) {
        IN_EP(i)->DIEPINT   = 0xFB7FU;
        IN_EP(i)->DIEPCTL  &= ~EPCTL_STALL;
        IN_EP(i)->DIEPCTL  |= EPCTL_SNAK;
        OUT_EP(i)->DOEPINT  = 0xFB7FU;
        OUT_EP(i)->DOEPCTL &= ~EPCTL_STALL;
        OUT_EP(i)->DOEPCTL |= EPCTL_SNAK;
    }

    /* IN EP0, EP1 and OUT EP0, EP1 */
    USB_D->DAINTMSK = (1U << 0) | (1U << 1) | (1U << 16) | (1U << 17);
    USB_D->DOEPMSK  = DOEPMSK_XFRCM | DOEPMSK_STUPM;
    USB_D->DIEPMSK  = DIEPMSK_XFRCM;

    USB_D->DCFG &= ~DCFG_DAD_MASK;          /* back to address 0 */

    configured = 0;
    dtr = 0;
    tx_busy = 0;
    ep0_out_pending = 0;
    ep0_rx_len = 0;

    ep0_arm_out();
}

static void usb_enum_done(void)
{
    IN_EP(0)->DIEPCTL &= ~EPCTL_MPSIZ_MASK;     /* EP0 max packet = 64 */
    USB_D->DCTL |= DCTL_CGINAK;
    USB_G->GUSBCFG = (USB_G->GUSBCFG & ~GUSBCFG_TRDT_MASK) | GUSBCFG_TRDT_FS;
}

static void usb_rx_pop(void)
{
    while (USB_G->GINTSTS & GINT_RXFLVL) {
        uint32_t st   = USB_G->GRXSTSP;         /* reading pops the entry */
        uint8_t  ep   = (uint8_t)(st & 0xFU);
        uint16_t bcnt = (uint16_t)((st >> 4) & 0x7FFU);
        uint8_t  pkt  = (uint8_t)((st >> 17) & 0xFU);

        if (pkt == PKTSTS_SETUP_DATA) {
            fifo_read((uint8_t *)setup_words, 8, 8);
        } else if (pkt == PKTSTS_OUT_DATA && bcnt) {
            if (ep == 0) {
                fifo_read(ep0_rx, bcnt, sizeof ep0_rx);
                ep0_rx_len = bcnt;
            } else {
                fifo_read(rx_buf, bcnt, sizeof rx_buf);
                rx_len = bcnt;
            }
        }
        /* PKTSTS 3 (OUT complete) and 4 (SETUP complete) carry no data */
    }
}

static void usb_in_irq(void)
{
    uint32_t daint = USB_D->DAINT & USB_D->DAINTMSK;
    uint32_t st;

    if (daint & (1U << 0)) {
        st = IN_EP(0)->DIEPINT;
        IN_EP(0)->DIEPINT = st;
    }
    if (daint & (1U << 1)) {
        st = IN_EP(1)->DIEPINT;
        IN_EP(1)->DIEPINT = st;
        if (st & DIEPINT_XFRC) {
            tx_busy = 0;
        }
    }
}

static void usb_out_irq(void)
{
    uint32_t daint = USB_D->DAINT & USB_D->DAINTMSK;
    uint32_t st;

    if (daint & (1U << 16)) {
        st = OUT_EP(0)->DOEPINT;
        OUT_EP(0)->DOEPINT = st;
        if (st & DOEPINT_STUP) handle_setup();
        if (st & DOEPINT_XFRC) ep0_out_done();
    }
    if (daint & (1U << 17)) {
        st = OUT_EP(1)->DOEPINT;
        OUT_EP(1)->DOEPINT = st;
        if (st & DOEPINT_XFRC) {
            if (rx_len) rx_ready = 1;       /* data waits for cdc_read() */
            else        ep1_arm_out();      /* zero-length packet: re-arm */
        }
    }
}

void OTG_FS_IRQHandler(void)
{
    uint32_t gint = USB_G->GINTSTS & USB_G->GINTMSK;

    if (gint & GINT_USBRST)  { USB_G->GINTSTS = GINT_USBRST;  usb_reset(); }
    if (gint & GINT_ENUMDNE) { USB_G->GINTSTS = GINT_ENUMDNE; usb_enum_done(); }
    if (gint & GINT_RXFLVL)  { usb_rx_pop(); }
    if (gint & GINT_IEPINT)  { usb_in_irq(); }
    if (gint & GINT_OEPINT)  { usb_out_irq(); }
}

/* ---------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */
void cdc_init(void)
{
    /* --- Clocks --- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    (void)RCC->AHB2ENR;

    /* --- PA11 (DM), PA12 (DP): AF10, very high speed, push-pull, no pull --- */
    GPIOA->MODER   = (GPIOA->MODER   & ~((3U << 22) | (3U << 24))) | (2U << 22) | (2U << 24);
    GPIOA->OTYPER &= ~((1U << 11) | (1U << 12));
    GPIOA->OSPEEDR |= (3U << 22) | (3U << 24);
    GPIOA->PUPDR  &= ~((3U << 22) | (3U << 24));
    GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~((0xFU << 12) | (0xFU << 16))) | (10U << 12) | (10U << 16);

    /* --- Core init --- */
    USB_G->GAHBCFG &= ~GAHBCFG_GINT;
    USB_G->GUSBCFG |= GUSBCFG_PHYSEL;               /* embedded full-speed PHY */

    while (!(USB_G->GRSTCTL & GRSTCTL_AHBIDL)) { }
    USB_G->GRSTCTL |= GRSTCTL_CSRST;                /* core soft reset */
    while (USB_G->GRSTCTL & GRSTCTL_CSRST) { }
    delay_ms(3);

    /* Transceiver on, no VBUS sensing (PA9 not needed) */
    USB_G->GCCFG = GCCFG_PWRDWN | GCCFG_NOVBUSSENS;

    /* Force device mode */
    USB_G->GUSBCFG &= ~(GUSBCFG_FHMOD | GUSBCFG_FDMOD);
    USB_G->GUSBCFG |= GUSBCFG_FDMOD;
    delay_ms(50);

    PCGCCTL = 0;                                    /* restart PHY clock */

    /* Device: full speed, hold soft-disconnect until everything is ready */
    USB_D->DCFG = (USB_D->DCFG & ~0x3U) | DCFG_DSPD_FS;
    USB_D->DCTL |= DCTL_SDIS;

    /* --- FIFO layout --- */
    USB_G->GRXFSIZ            = RX_FIFO_WORDS;
    USB_G->DIEPTXF0_HNPTXFSIZ = (TX0_FIFO_WORDS << 16) | RX_FIFO_WORDS;
    USB_G->DIEPTXF[0]         = (TX1_FIFO_WORDS << 16) | (RX_FIFO_WORDS + TX0_FIFO_WORDS);
    USB_G->DIEPTXF[1]         = (TX2_FIFO_WORDS << 16) | (RX_FIFO_WORDS + TX0_FIFO_WORDS + TX1_FIFO_WORDS);

    USB_G->GRSTCTL = GRSTCTL_TXFFLSH | GRSTCTL_TXFNUM_ALL;
    while (USB_G->GRSTCTL & GRSTCTL_TXFFLSH) { }
    USB_G->GRSTCTL = GRSTCTL_RXFFLSH;
    while (USB_G->GRSTCTL & GRSTCTL_RXFFLSH) { }

    /* --- Interrupts --- */
    USB_D->DIEPMSK  = 0;
    USB_D->DOEPMSK  = 0;
    USB_D->DAINTMSK = 0;
    USB_G->GINTSTS  = 0xBFFFFFFFU;                  /* clear anything pending */
    USB_G->GINTMSK  = GINT_RXFLVL | GINT_USBRST | GINT_ENUMDNE | GINT_IEPINT | GINT_OEPINT;
    USB_G->GAHBCFG |= GAHBCFG_GINT;

    NVIC_SetPriority(OTG_FS_IRQn, 2);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    /* --- Connect: the host now sees a new device and starts enumeration --- */
    delay_ms(10);
    USB_D->DCTL &= ~DCTL_SDIS;
}

uint8_t cdc_connected(void)
{
    return configured;
}

uint8_t cdc_dtr(void)
{
    return dtr;
}

uint16_t cdc_write(const uint8_t *data, uint16_t len)
{
    if (!configured || tx_busy || len == 0U) {
        return 0;
    }
    if (len > CDC_TX_MAX) {
        len = CDC_TX_MAX;
    }

    tx_busy = 1;
    ep_in_send(1, data, len);
    return len;
}

uint16_t cdc_read(uint8_t *dst, uint16_t max)
{
    uint16_t n;

    if (!rx_ready) {
        return 0;
    }
    n = rx_len;
    if (n > max) n = max;
    memcpy(dst, rx_buf, n);

    ep1_arm_out();      /* ready for the next packet from the PC */
    return n;
}

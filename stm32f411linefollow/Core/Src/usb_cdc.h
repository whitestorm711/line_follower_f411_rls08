/*
 * usb_cdc.h - Minimal bare-metal USB CDC (virtual COM port) for STM32F401
 *             Uses USB OTG FS on PA11 (DM) / PA12 (DP), CMSIS registers only.
 */
#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

/* Call once after the clock is set to 84 MHz with PLLQ giving exactly 48 MHz. */
void     cdc_init(void);

/* 1 once the PC has enumerated and configured the device. */
uint8_t  cdc_connected(void);

/* 1 if a terminal has opened the port and raised DTR (informational). */
uint8_t  cdc_dtr(void);

/* Non-blocking send of up to 63 bytes. Returns bytes accepted, or 0 if the
 * previous packet is still in flight / host not ready. Never blocks, so it is
 * safe to call from the line-follower loop. */
uint16_t cdc_write(const uint8_t *data, uint16_t len);

/* Non-blocking read of the last packet received from the PC (up to 64 bytes).
 * Returns the number of bytes copied, or 0 if nothing is waiting. */
uint16_t cdc_read(uint8_t *dst, uint16_t max);

#endif /* USB_CDC_H */

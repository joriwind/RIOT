/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_encx24j600
 * @{
 *
 * @file
 * @brief       Internal functions for the ENCX24J600 driver
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include "mutex.h"
#include "encx24j600.h"
#include "encx24j600_internal.h"
#include "encx24j600_defines.h"
#include "vtimer.h"

#include "net/dev_eth.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define ENCX24J600_SPI_SPEED SPI_SPEED_1MHZ

#define ENCX24J600_INIT_DELAY 100000U

#define ENC_BUFFER_START 0x0000
#define ENC_BUFFER_SIZE  0x6000
#define ENC_BUFFER_END   0x5FFF
#define RX_BUFFER_START (0x5340) /* Default value */
#define RX_BUFFER_END   (ENC_BUFFER_END)
#define TX_BUFFER_LEN   (0x2000)
#define TX_BUFFER_END   (RX_BUFFER_START)
#define TX_BUFFER_START (TX_BUFFER_END - TX_BUFFER_LEN)

static void receive_packets(encx24j600_t *dev);
static void cmd(encx24j600_t *dev, char cmd);
static void reg_set(encx24j600_t *dev, uint8_t reg, uint16_t value);
static uint16_t reg_get(encx24j600_t *dev, uint8_t reg);
static void reg_clear_bits(encx24j600_t *dev, uint8_t reg, uint16_t mask);

/* dev_eth interface */
static void _isr(dev_eth_t *dev);
static int _init(dev_eth_t *dev);
static int _send(dev_eth_t *dev, char *pkt, int len);
static void _get_mac_addr(dev_eth_t *dev, uint8_t* buf);
static int _recv(dev_eth_t *encdev, char* buf, int len);

static inline int _get_promiscous(dev_eth_t *dev) {
    return 1;
}

static inline int _set_promiscous(dev_eth_t *dev, int value) {
    return 1;
}

const static eth_driver_t eth_driver_encx24j600 = {
    .init = _init,
    .send = _send,
    .recv = _recv,
    .get_mac_addr = _get_mac_addr,
    .get_promiscous = _get_promiscous,
    .set_promiscous = _set_promiscous,
    .isr = _isr,
};

static inline void lock(encx24j600_t *dev) {
    mutex_lock(&dev->mutex);
}

static inline void unlock(encx24j600_t *dev) {
    mutex_unlock(&dev->mutex);
}

void encx24j600_setup(encx24j600_t *dev, spi_t spi, gpio_t cs, gpio_t int_pin)
{
    dev->ethdev.driver = &eth_driver_encx24j600;
    dev->spi = spi;
    dev->cs = cs;
    dev->int_pin = int_pin;
    dev->rx_next_ptr = RX_BUFFER_START;

    mutex_init(&dev->mutex);
}

static void encx24j600_isr(void *arg)
{
    encx24j600_t *dev = (encx24j600_t *) arg;

    /* disable interrupt line */
    gpio_irq_disable(dev->int_pin);

    /* call dev_eth hook */
    dev_eth_isr((dev_eth_t*) dev);
}

static void _isr(dev_eth_t *encdev)
{
    encx24j600_t *dev = (encx24j600_t *) encdev;

    uint16_t eir;

    lock(dev);
    cmd(dev, CLREIE);

    eir = reg_get(dev, EIR);

    if (eir & PKTIF) {
        receive_packets(dev);
    }

    if (eir & LINKIF) {
        uint16_t estat = reg_get(dev, ESTAT);
        dev_eth_linkstate_handler(encdev, estat & PHYLNK);
    }

    /* drop all flags */
    reg_clear_bits(dev, EIR, LINKIF);

    /* re-enable interrupt */
    gpio_irq_enable(dev->int_pin);
    cmd(dev, SETEIE);

    unlock(dev);
}

static inline void enc_spi_transfer(encx24j600_t *dev, char *out, char *in, int len)
{
    gpio_clear(dev->cs);
    spi_transfer_bytes(dev->spi, out, in, len);
    gpio_set(dev->cs);
}

static inline uint16_t reg_get(encx24j600_t *dev, uint8_t reg)
{
    char cmd[4] = { RCRU, reg, 0, 0 };
    char result[4];

    enc_spi_transfer(dev, cmd, result, 4);

//    DEBUG("encx24j600_reg_get: got 0x%x from register 0x%x\n", (result[2] | (result[3] << 8)), reg);

    return result[2] | (result[3] << 8);
}

static void phy_reg_set(encx24j600_t *dev, uint8_t reg, uint16_t value) {
    reg_set(dev, MIREGADR, reg | (1<<8));
    reg_set(dev, MIWR, value);
}

static void cmd(encx24j600_t *dev, char cmd) {
    gpio_clear(dev->cs);
    spi_transfer_byte(dev->spi, cmd, NULL);
    gpio_set(dev->cs);
}

static void cmdn(encx24j600_t *dev, uint8_t cmd, char *out, char *in, int len) {
    gpio_clear(dev->cs);
    spi_transfer_byte(dev->spi, cmd, NULL);
    spi_transfer_bytes(dev->spi, out, in, len);
    gpio_set(dev->cs);
}

#if 0
static uint8_t reg_getb(encx24j600_t *dev, uint8_t reg)
{
    int banked_addr = reg % 0x20;
    int bank = reg / 0x20;

    char cmd[2] = { RCR | banked_addr };
    char result[2];

    gpio_clear(dev->cs);
    switch_bank(dev, bank);
    spi_transfer_bytes(dev->spi, cmd, result, 2);
    gpio_set(dev->cs);

 //   DEBUG("encx24j600_reg_get8: got 0x%x from register 0x%x\n", (result[2]), reg);

    return result[2] ;
}
#endif

static void reg_set(encx24j600_t *dev, uint8_t reg, uint16_t value)
{
    char cmd[4] = { WCRU, reg, value, value >> 8 };

//    DEBUG("encx24j600_reg_set: setting 0x%x to register 0x%x\n", (cmd[2] | (cmd[3] << 8)), reg);

    enc_spi_transfer(dev, cmd, NULL, 4);
}

static void reg_set_bits(encx24j600_t *dev, uint8_t reg, uint16_t mask)
{
    char cmd[4] = { BFSU, reg, mask, mask >> 8 };

//    DEBUG("encx24j600_reg_set: setting mask 0x%x to register 0x%x\n", (cmd[2] | (cmd[3] << 8)), reg);

    enc_spi_transfer(dev, cmd, NULL, 4);
}

static void reg_clear_bits(encx24j600_t *dev, uint8_t reg, uint16_t mask)
{
    char cmd[4] = { BFCU, reg, mask, mask >> 8 };

//    DEBUG("encx24j600_reg_set: clearing mask 0x%x to register 0x%x\n", (cmd[2] | (cmd[3] << 8)), reg);

    enc_spi_transfer(dev, cmd, NULL, 4);
}

#if 0 /* unused */
/* call with CS down (chip must already be selected) */
static void switch_bank(encx24j600_t *dev, unsigned short bank)
{
    /* only switch bank if needed or possible */
    if (dev->active_bank == bank || bank > 3) {
        return;
    }

    spi_transfer_byte(dev->spi, B0SEL | (bank*2), NULL);

    dev->active_bank = bank;
}
#endif

/*
 * @brief Read/Write to encx24j600's SRAM
 *
 * @param[in] dev   ptr to encx24j600 device handle
 * @param[in] cmd   either WGPDATA, RGPDATA, WRXDATA, RRXDATA, WUDADATA, RUDADATA
 * @param[in] addr  SRAM address to start reading. 0xFFFF means continue from old address
 * @param     ptr   pointer to buffer to read from / write to
 * @param[in] len   nr of bytes to read/write
 */
static void sram_op(encx24j600_t *dev, uint16_t cmd, uint16_t addr, char *ptr, int len)
{
    uint16_t reg;
    char* in = NULL;
    char* out = NULL;

    /* determine pointer addr
     *
     * all SRAM access commands have an
     * offset 0x5e to their pointer registers
     * */
    reg = cmd + 0x5e;

    /* read or write? bit 1 tells us */
    if (reg & 0x2) {
        out = ptr;
    } else {
        in = ptr;
    }

    /* set pointer */
    if (addr != 0xFFFF) {
        reg_set(dev, reg, addr);
    }

    /* copy data */
    cmdn(dev, cmd, in, out, len);
}

static int _init(dev_eth_t *encdev)
{
    encx24j600_t * dev = (encx24j600_t *) encdev;

    DEBUG("encx24j600: starting initialization...\n");

    /* setup IO */
    gpio_init_out(dev->cs, GPIO_PULLUP);
    gpio_set(dev->cs);
    gpio_init_int(dev->int_pin, GPIO_PULLUP, GPIO_FALLING, encx24j600_isr, (void*)dev);

    spi_init_master(dev->spi, SPI_CONF_FIRST_RISING, ENCX24J600_SPI_SPEED);

    lock(dev);

    /* initialization procedure as described in data sheet (39935c.pdf) */
    do {
        do {
            vtimer_usleep(ENCX24J600_INIT_DELAY);
            reg_set(dev, EUDAST, 0x1234);
            vtimer_usleep(ENCX24J600_INIT_DELAY);
        } while (reg_get(dev, EUDAST) != 0x1234);

        while (!(reg_get(dev, ESTAT) & CLKRDY));

        /* issue System Reset */
        cmd(dev, SETETHRST);

        /* make sure initialization finalizes */
        vtimer_usleep(1000);
    } while (!(reg_get(dev, EUDAST) == 0x0000));

    /* configure flow control */
    phy_reg_set(dev, PHANA, 0x05E1);
    reg_set_bits(dev, ECON2, AUTOFC);

    /* setup receive buffer */
    reg_set(dev, ERXST, RX_BUFFER_START);
    reg_set(dev, ERXTAIL, RX_BUFFER_END);
    dev->rx_next_ptr = RX_BUFFER_START;

    /* setup interrupts */
    reg_set_bits(dev, EIE, PKTIE | LINKIE);
    cmd(dev, ENABLERX);
    cmd(dev, SETEIE);

    DEBUG("encx24j600: initialization complete.\n");

    unlock(dev);

    return 0;
}

static int _send(dev_eth_t *encdev, char *pkt, int len) {
    encx24j600_t * dev = (encx24j600_t *) encdev;
    lock(dev);

    /* wait until previous packet has been sent */
    while ((reg_get(dev, ECON1) & TXRTS));

    /* copy packet to SRAM */
    sram_op(dev, WGPDATA, TX_BUFFER_START, pkt, len);

    /* set start of TX packet and length */
    reg_set(dev, ETXST, TX_BUFFER_START);
    reg_set(dev, ETXLEN, len);

    /* initiate sending */
    cmd(dev, SETTXRTS);

    /* wait for sending to complete */
    //while ((reg_get(dev, ECON1) & TXRTS));

    unlock(dev);

    return len;
}

/*static void encx24j600_handle_linkstate(encx24j600_t *dev, int state)
{
    DEBUG("encx24j600: link %s.\n", state ? "UP" : "DOWN");
}*/

static inline int packets_available(encx24j600_t *dev) {
    /* return PKTCNT (low byte of ESTAT) */
    return reg_get(dev, ESTAT) & ~0xFF00;
}

static void _get_mac_addr(dev_eth_t *encdev, uint8_t* buf)
{
    encx24j600_t * dev = (encx24j600_t *) encdev;
    uint16_t *addr = (uint16_t *) buf;

    lock(dev);

    addr[0] = reg_get(dev, MAADR1);
    addr[1] = reg_get(dev, MAADR2);
    addr[2] = reg_get(dev, MAADR3);

    unlock(dev);
}

static int _recv(dev_eth_t *encdev, char* buf, int len)
{
    encx24j600_t * dev = (encx24j600_t *) encdev;
    encx24j600_frame_hdr_t hdr;

    lock(dev);

    /* read frame header */
    sram_op(dev, RRXDATA, dev->rx_next_ptr, (char*)&hdr, sizeof(hdr));

    /* read packet */
    sram_op(dev, RRXDATA, 0xFFFF, buf, hdr.frame_len);

    /* decrement available packet count */
    cmd(dev, SETPKTDEC);

//    DEBUG("encx24j600: Received packet. frame_len=%u next=0x%x, newnext=0x%x ERXTAIL=0x%x\n", (unsigned int)hdr.frame_len, (unsigned int) dev->rx_next_ptr, (unsigned int)hdr.rx_next_ptr, (unsigned int)reg_get(dev, ERXTAIL));

    dev->rx_next_ptr = hdr.rx_next_ptr;

    reg_set(dev, ERXTAIL, dev->rx_next_ptr - 2);

    unlock(dev);

    return hdr.frame_len - 4;
}

static void receive_packets(encx24j600_t *dev)
{
    if (packets_available(dev)) {
        unlock(dev);
        dev_eth_rx_handler((dev_eth_t*)dev);
        lock(dev);
    }

#if 0
    int left;
    int throttle = 10;
    while ((left = packets_available(dev))) {
        while (left--) {
                unlock(dev);
                encx24j600_receive_packet_cb(dev);
                lock(dev);
                if (!throttle--) {
                    return;
                }
        }
    };
#endif
}

/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    driver_encx24j600 ENCX24J600
 * @ingroup     drivers
 * @brief       Driver for the ENCX24J600 Ethernet Adapter
 * @{
 *
 * @file
 * @brief       Interface definition for the ENCX24J600 driver
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef ENCX24J600_H
#define ENCX24J600_H

#include "mutex.h"
#include "kernel_types.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "net/dev_eth.h"

/**
 * @brief encx24j600 dev_eth device
 * @extends dev_eth_t
 */
typedef struct {
    dev_eth_t ethdev;       /**< extended dev_eth structure */
    spi_t spi;              /**< SPI device the enc is connected to*/
    gpio_t cs;              /**< SPI chip select pin */
    gpio_t int_pin;         /**< SPI interrupt pin */
    uint16_t rx_next_ptr;   /**< ptr to next packet whithin devices memory */
    mutex_t mutex;          /**< mutex used to lock SPI access */
} encx24j600_t;

void encx24j600_setup(encx24j600_t *dev, spi_t spi, gpio_t cs_pin, gpio_t int_pin);

#endif /* ENCX24J600_H */
/** @} */

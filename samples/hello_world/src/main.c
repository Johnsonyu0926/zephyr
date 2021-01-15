/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

void main(void)
{
    printk("Hello from zephyr! %s\n", CONFIG_BOARD);

    volatile int i = 0;
    while (1) {
        i++;
    }
 }

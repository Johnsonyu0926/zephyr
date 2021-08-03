/*
 * Copyright (c) 2021-2024 ATL Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SIFIVE_CLIC_H
#define _SIFIVE_CLIC_H

#define CLIC_CTRL_ADDR  0x02000000UL
#define CLIC_HART0_ADDR 0x02800000UL

#define CLIC_MSIP          0x0000
#define CLIC_MSIP_size     0x4
#define CLIC_MTIMECMP      0x4000
#define CLIC_MTIMECMP_size 0x8
#define CLIC_MTIME         0xBFF8
#define CLIC_MTIME_size    0x8

#define CLIC_INTIP  0x000
#define CLIC_INTIE  0x400
#define CLIC_INTCFG 0x800
#define CLIC_CFG    0xc00

#endif /* _SIFIVE_CLIC_H */

/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_SDIO_WRAPPERS_H
#define _SQN_SDIO_WRAPPERS_H

#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/skbuff.h>

struct msmsdcc_host;

int sqn_sdio_dump_net_pkt(int on);

/* HTC:WiMax power ON_OFF function and Card detect function */
extern int mmc_wimax_power(int on);
extern void mmc_wimax_set_carddetect(int val);
extern int mmc_wimax_uart_switch(int uart);
extern int mmc_wimax_set_status(int on);
extern int mmc_wimax_get_hostwakeup_gpio(void);
extern int mmc_wimax_get_netlog_status(void);
extern int mmc_wimax_get_cliam_host_status(void);
extern int mmc_wimax_set_CMD53_timeout_trigger_counter(int counter);
extern int mmc_wimax_get_CMD53_timeout_trigger_counter(void);
extern int mmc_wimax_get_sdio_hw_reset(void);
extern int mmc_wimax_get_packet_filter(void);
/* Wakeup interrupt */
extern void mmc_wimax_enable_host_wakeup(int on);
extern int mmc_wimax_get_netlog_withraw_status(void);
extern int mmc_wimax_get_sdio_interrupt_log(void);
extern int mmc_wimax_get_sdio_wakelock_log(void);
extern int mmc_wimax_get_sdio_wakeup_lite_dump(void);
extern int mmc_wimax_get_hostwakeup_IRQ_ID(void);

extern uint8_t sqn_is_rx_thp_packet(uint8_t  *dest_addr);
extern uint8_t sqn_is_tx_thp_packet(uint8_t  *src_addr);
extern int sqn_is_tx_lsp_packet(const struct sk_buff *skb);
extern int sqn_is_rx_lsp_packet(const struct sk_buff *skb);

extern int mmc_wimax_get_RD_FIFO_LEVEL_ERROR(void);

extern int mmc_wimax_get_wimax_FW_freeze_WK_RX(void);
extern void msmsdcc_reset_and_restore(struct msmsdcc_host *host);

#endif  /* _SQN_SDIO_WRAPPERS_H */

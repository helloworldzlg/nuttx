/****************************************************************************
 * wireless/ieee802154/mac802154_notif.h
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *
 *   Author: Anthony Merlino <anthony@vergeaero.com>
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   The naming and comments for various fields are taken directly
 *   from the IEEE 802.15.4 2011 standard.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __WIRELESS_IEEE802154__MAC802154_NOTIF_H
#define __WIRELESS_IEEE802154__MAC802154_NOTIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Extend the public ieee802154_notif_s to include a private forward link to
 * support a list to handle allocation
 */

struct mac802154_notif_s
{
  struct ieee802154_notif_s pub;        /* Publically visible structure */
  FAR struct mac802154_notif_s *flink;  /* Supports a singly linked list */
};

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

struct ieee802154_privmac_s; /* Forward Reference */

void mac802154_notifpool_init(FAR struct ieee802154_privmac_s *priv);

int mac802154_notif_alloc(FAR struct ieee802154_privmac_s *priv,
                          FAR struct ieee802154_notif_s **notif,
                          bool allow_interrupt);

void mac802154_notify(FAR struct ieee802154_privmac_s *priv,
                      FAR struct ieee802154_notif_s *notif);

#endif /* __WIRELESS_IEEE802154__MAC802154_NOTIF_H */
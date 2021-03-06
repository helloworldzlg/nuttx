/****************************************************************************
 * wireless/iee802154/mac802154_loopback.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/ieee802154/ieee802154_loopback.h>

#include "mac802154.h"

#ifdef CONFIG_IEEE802154_LOOPBACK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We need to have the work queue to handle SPI interrupts */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Worker thread support is required (CONFIG_SCHED_WORKQUEUE)
#else
#  if defined(CONFIG_IEEE802154_LOOPBACK_HPWORK)
#    define LPBKWORK HPWORK
#  elif defined(CONFIG_IEEE802154_LOOPBACK_LPWORK)
#    define LPBKWORK LPWORK
#  else
#    error Neither CONFIG_IEEE802154_LOOPBACK_HPWORK nor CONFIG_IEEE802154_LOOPBACK_LPWORK defined
#  endif
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LO_WDDELAY   (1*CLK_TCK)

/* Fake value for MAC header length */

#define MAC_HDRLEN   9

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lo_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lo_driver_s
{
  bool lo_bifup;               /* true:ifup false:ifdown */
  bool lo_pending;             /* True: TX poll pending */
  uint8_t lo_panid[2];         /* Fake PAN ID for testing */
  WDOG_ID lo_polldog;          /* TX poll timer */
  struct work_s lo_work;       /* For deferring poll work to the work queue */
  FAR struct iob_s *lo_head;   /* Head of IOBs queued for loopback */
  FAR struct iob_s *lo_tail;   /* Tail of IOBs queued for loopback */

  /* This holds the information visible to the NuttX network */

  struct ieee802154_driver_s lo_ieee;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lo_driver_s g_loopback;
static uint8_t g_iobuffer[CONFIG_NET_6LOWPAN_MTU + CONFIG_NET_GUARDSIZE];

static uint8_t g_eaddr[IEEE802154_EADDRSIZE] =
{
  0x0c, 0xfa, 0xde, 0x00, 0xde, 0xad, 0xbe, 0xef
};

static uint8_t g_saddr[IEEE802154_SADDRSIZE] =
{
  0xab, 0xcd
};

static uint8_t g_panid[IEEE802154_PANIDSIZE] =
{
  0xca, 0xfe
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

static void lo_addr2ip(FAR struct net_driver_s *dev);
static inline void lo_netmask(FAR struct net_driver_s *dev);

/* Polling logic */

static int  lo_loopback(FAR struct net_driver_s *dev);
static void lo_loopback_work(FAR void *arg);
static void lo_poll_work(FAR void *arg);
static void lo_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  lo_ifup(FAR struct net_driver_s *dev);
static int  lo_ifdown(FAR struct net_driver_s *dev);
static void lo_txavail_work(FAR void *arg);
static int  lo_txavail(FAR struct net_driver_s *dev);
#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int  lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  lo_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif
static int lo_get_mhrlen(FAR struct ieee802154_driver_s *netdev,
              FAR const struct ieee802154_frame_meta_s *meta);
static int lo_req_data(FAR struct ieee802154_driver_s *netdev,
              FAR const struct ieee802154_frame_meta_s *meta,
              FAR struct iob_s *framelist);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lo_addr2ip
 *
 * Description:
 *   Create a MAC-based IP address from the IEEE 802.15.14 short or extended
 *   address assigned to the node.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
static void lo_addr2ip(FAR struct net_driver_s *dev)
{
  /* Set the MAC address as the eaddr */

  IEEE802154_EADDRCOPY(dev->d_mac.ieee802154.u8, g_eaddr);

  /* Set the IP address based on the eaddr */

  dev->d_ipv6addr[0]  = HTONS(0xfe80);
  dev->d_ipv6addr[1]  = 0;
  dev->d_ipv6addr[2]  = 0;
  dev->d_ipv6addr[3]  = 0;
  dev->d_ipv6addr[4]  = (uint16_t)g_eaddr[0] << 8 | (uint16_t)g_eaddr[1];
  dev->d_ipv6addr[5]  = (uint16_t)g_eaddr[2] << 8 | (uint16_t)g_eaddr[3];
  dev->d_ipv6addr[6]  = (uint16_t)g_eaddr[4] << 8 | (uint16_t)g_eaddr[5];
  dev->d_ipv6addr[7]  = (uint16_t)g_eaddr[6] << 8 | (uint16_t)g_eaddr[7];
  dev->d_ipv6addr[4] ^= 0x200;
}
#else
static void lo_addr2ip(FAR struct net_driver_s *dev)
{
  /* Set the MAC address as the saddr */

  IEEE802154_SADDRCOPY(dev->d_mac.ieee802154.u8, g_saddr);

  /* Set the IP address based on the saddr */

  dev->d_ipv6addr[0]  = HTONS(0xfe80);
  dev->d_ipv6addr[1]  = 0;
  dev->d_ipv6addr[2]  = 0;
  dev->d_ipv6addr[3]  = 0;
  dev->d_ipv6addr[4]  = 0;
  dev->d_ipv6addr[5]  = HTONS(0x00ff);
  dev->d_ipv6addr[6]  = HTONS(0xfe00);
  dev->d_ipv6addr[7]  = (uint16_t)g_saddr[0] << 8 | (uint16_t)g_saddr[1];
  dev->d_ipv6addr[7] ^= 0x200;
}
#endif

/****************************************************************************
 * Name: lo_netmask
 *
 * Description:
 *   Create a netmask of a MAC-based IP address which may be based on either
 *   the IEEE 802.15.14 short or extended address of the MAC.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
 *
 ****************************************************************************/

static inline void lo_netmask(FAR struct net_driver_s *dev)
{
  dev->d_ipv6netmask[0]  = 0xffff;
  dev->d_ipv6netmask[1]  = 0xffff;
  dev->d_ipv6netmask[2]  = 0xffff;
  dev->d_ipv6netmask[3]  = 0xffff;
#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  dev->d_ipv6netmask[4]  = 0;
  dev->d_ipv6netmask[5]  = 0;
  dev->d_ipv6netmask[6]  = 0;
  dev->d_ipv6netmask[7]  = 0;
#else
  dev->d_ipv6netmask[4]  = 0xffff;
  dev->d_ipv6netmask[5]  = 0xffff;
  dev->d_ipv6netmask[6]  = 0xffff;
  dev->d_ipv6netmask[7]  = 0;
#endif
}

/****************************************************************************
 * Name: lo_loopback
 *
 * Description:
 *   Check if the network has any outgoing packets ready to send.  This is
 *   a callback from devif_poll() or devif_timer().  devif_poll() will be
 *   called only during normal TX polling.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int lo_loopback(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;
  struct ieee802154_data_ind_s ind;
  FAR struct iob_s *iob;
  int ret;

  /* Create some fake metadata */

  memset(&ind, 0, sizeof(struct ieee802154_data_ind_s));

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  ind.src.mode  = IEEE802154_ADDRMODE_EXTENDED;
  ind.dest.mode = IEEE802154_ADDRMODE_EXTENDED;
#else
  ind.src.mode  = IEEE802154_ADDRMODE_SHORT;
  ind.dest.mode = IEEE802154_ADDRMODE_SHORT;
#endif

  /* On loopback the local address is both the source and destination. */

  IEEE802154_PANIDCOPY(ind.src.panid, g_panid);
  IEEE802154_SADDRCOPY(ind.src.saddr, g_saddr);
  IEEE802154_EADDRCOPY(ind.src.eaddr, g_eaddr);

  IEEE802154_PANIDCOPY(ind.dest.panid, g_panid);
  IEEE802154_SADDRCOPY(ind.dest.saddr, g_saddr);
  IEEE802154_EADDRCOPY(ind.dest.eaddr, g_eaddr);

  /* Loop while there framelist to be sent, i.e., while the freme list is not
   * emtpy.  Sending, of course, just means relaying back through the network
   * for this driver.
   */

  while (priv->lo_head != NULL)
    {
      ninfo("Looping frame IOB %p\n", iob);

      /* Increment statistics */

      NETDEV_RXPACKETS(&priv->lo_ieee.i_dev);

      /* Remove the IOB from the queue */

      iob           = priv->lo_head;
      priv->lo_head = iob->io_flink;
      iob->io_flink = NULL;

      /* Did the framelist become empty? */

      if (priv->lo_head == NULL)
        {
          priv->lo_tail = NULL;
        }

      /* Return the next frame to the network */

      ninfo("Send frame %p to the network:  Offset=%u Length=%u\n",
            iob, iob->io_offset, iob->io_len);

      ret = sixlowpan_input(&priv->lo_ieee, iob, &ind);

      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->lo_ieee.i_dev);

      if (ret < 0)
        {
          nerr("ERROR: sixlowpan_input returned %d\n", ret);
          NETDEV_TXERRORS(&priv->lo_ieee.i_dev);
          NETDEV_ERRORS(&priv->lo_ieee.i_dev);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: lo_loopback_work
 *
 * Description:
 *   Perform loopback of received framelist.
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void lo_loopback_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  /* Perform the loopback */

  net_lock();
  (void)lo_loopback(&priv->lo_ieee.i_dev);
  net_unlock();
}

/****************************************************************************
 * Name: lo_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void lo_poll_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  /* Perform the poll */

  net_lock();
  (void)devif_timer(&priv->lo_ieee.i_dev, lo_loopback);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->lo_polldog, LO_WDDELAY, lo_poll_expiry, 1, priv);
  net_unlock();
}

/****************************************************************************
 * Name: lo_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lo_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  if (!work_available(&priv->lo_work) || priv->lo_head != NULL)
    {
      nwarn("WARNING: lo_work NOT available\n");
      priv->lo_pending = true;
    }
  else
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      priv->lo_pending = false;
      work_queue(LPBKWORK, &priv->lo_work, lo_poll_work, priv, 0);
    }
}

/****************************************************************************
 * Name: lo_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lo_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("Bringing up: IPv6 %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  ninfo("             Node: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x PANID=%02x:%02x\n",
         dev->d_mac.ieee802154.u8[0], dev->d_mac.ieee802154.u8[1],
         dev->d_mac.ieee802154.u8[2], dev->d_mac.ieee802154.u8[3],
         dev->d_mac.ieee802154.u8[4], dev->d_mac.ieee802154.u8[5],
         dev->d_mac.ieee802154.u8[6], dev->d_mac.ieee802154.u8[7],
         priv->lo_panid[0], priv->lo_panid[1]);
#else
  ninfo("             Node: %02x:%02x PANID=%02x:%02x\n",
         dev->d_mac.ieee802154.u8[0], dev->d_mac.ieee802154.u8[1],
         priv->lo_panid[0], priv->lo_panid[1]);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->lo_polldog, LO_WDDELAY, lo_poll_expiry,
                 1, (wdparm_t)priv);

  priv->lo_bifup = true;
  return OK;
}

/****************************************************************************
 * Name: lo_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lo_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("IP up: %u\n", priv->lo_bifup);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->lo_polldog);

  /* Mark the device "down" */

  priv->lo_bifup = false;
  return OK;
}

/****************************************************************************
 * Name: lo_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void lo_txavail_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  ninfo("TX available work. IP up: %u\n", priv->lo_bifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->lo_bifup)
    {
      /* If so, then poll the network for new XMIT data */

      (void)devif_poll(&priv->lo_ieee.i_dev, lo_loopback);
    }

  net_unlock();
}

/****************************************************************************
 * Name: lo_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int lo_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("Available: %u\n", work_available(&priv->lo_work));

  /* Is our single work structure available?  It may not be if there are
   * pending actions and we will have to ignore the Tx availability
   * action.
   */

  if (!work_available(&priv->lo_work) || priv->lo_head != NULL)
    {
      nwarn("WARNING: lo_work NOT available\n");
      priv->lo_pending = true;
    }
  else
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      priv->lo_pending = false;
      work_queue(LPBKWORK, &priv->lo_work, lo_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: lo_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
#else
  ninfo("MAC: %02x:%02x\n",
         mac[0], mac[1]);
#endif

  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Name: lo_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
#else
  ninfo("MAC: %02x:%02x\n",
         mac[0], mac[1]);
#endif

  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Name: macnet_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int lo_ioctl(FAR struct net_driver_s *dev, int cmd,
                    unsigned long arg)
{
  FAR struct lo_driver_s *priv;

  DEBUGASSERT(dev != NULL && dev->d_private != NULL);
  priv = (FAR struct lo_driver_s *)dev->d_private;

  UNUSED(priv);

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      FAR struct ieee802154_netmac_s *netmac =
        (FAR struct ieee802154_netmac_s *)arg;

      DEBUGASSERT(netmac != NULL);

      if (cmd == MAC802154IOC_MLME_SET_REQUEST)
        {
          FAR struct ieee802154_set_req_s *setreq = &netmac->u.setreq;

          switch (setreq->attr)
            {
              case IEEE802154_ATTR_MAC_PANID:
                IEEE802154_PANIDCOPY(g_panid, setreq->attrval.mac.panid);
                break;

              case IEEE802154_ATTR_MAC_EADDR:
                IEEE802154_EADDRCOPY(g_eaddr, setreq->attrval.mac.eaddr);
#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
                lo_addr2ip(dev);
#endif
                break;

              case IEEE802154_ATTR_MAC_SADDR:
                IEEE802154_SADDRCOPY(g_saddr, setreq->attrval.mac.saddr);
#ifndef CONFIG_NET_6LOWPAN_EXTENDEDADDR
                lo_addr2ip(dev);
#endif
                break;

              default:
                return -ENOTTY;
            }

          return OK;
        }
      else if (cmd == MAC802154IOC_MLME_GET_REQUEST)
        {
          FAR struct ieee802154_get_req_s *getreq = &netmac->u.getreq;

          switch (getreq->attr)
            {
              case IEEE802154_ATTR_MAC_PANID:
                IEEE802154_PANIDCOPY(getreq->attrval.mac.panid, g_panid);
                break;

              case IEEE802154_ATTR_MAC_EADDR:
                IEEE802154_EADDRCOPY(getreq->attrval.mac.eaddr, g_eaddr);
                break;

              case IEEE802154_ATTR_MAC_SADDR:
                IEEE802154_SADDRCOPY(getreq->attrval.mac.saddr, g_saddr);
                break;

              default:
                return -ENOTTY;
            }

          return OK;
        }
      else
        {
          /* Not a supported IEEE 802.15.4 MAC IOCTL command */

          return -ENOTTY;
        }
    }
  else
    {
      /* Not a valid IEEE 802.15.4 MAC IOCTL command */

      return -ENOTTY;
    }
}
#endif

/****************************************************************************
 * Name: lo_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Meta data needed to recreate the MAC header
 *
 * Returned Value:
 *   A non-negative MAC headeer length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int lo_get_mhrlen(FAR struct ieee802154_driver_s *netdev,
                         FAR const struct ieee802154_frame_meta_s *meta)
{
  return MAC_HDRLEN;
}

/****************************************************************************
 * Name: lo_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the MAC.
 *
 * Input parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Meta data needed to recreate the MAC header
 *   framelist - Head of a list of frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int lo_req_data(FAR struct ieee802154_driver_s *netdev,
                       FAR const struct ieee802154_frame_meta_s *meta,
                       FAR struct iob_s *framelist)
{
  FAR struct lo_driver_s *priv;
  FAR struct iob_s *iob;

  DEBUGASSERT(netdev != NULL && netdev->i_dev.d_private != NULL &&
              framelist != NULL);
  priv = (FAR struct lo_driver_s *)netdev->i_dev.d_private;

  /* Add the incoming list of framelist to queue of framelist to loopback */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_RXPACKETS(&priv->lo_ieee.i_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      ninfo("Queuing frame IOB %p\n", iob);

      /* Just zero the MAC header for test purposes */

      DEBUGASSERT(iob->io_offset == MAC_HDRLEN);
      memset(iob->io_data, 0, MAC_HDRLEN);

      /* Add the IOB to the tail of the queue of framelist to be looped back */

      if (priv->lo_tail == NULL)
        {
          priv->lo_head = iob;
        }
      else
        {
          priv->lo_tail->io_flink = iob;
        }

      priv->lo_tail = iob;
    }

  /* Schedule to serialize the poll on the worker thread. */

  work_queue(LPBKWORK, &priv->lo_work, lo_loopback_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee8021514_loopback
 *
 * Description:
 *   Initialize and register the Ieee802.15.4 MAC loopback network driver.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ieee8021514_loopback(void)
{
  FAR struct lo_driver_s *priv;
  FAR struct ieee802154_driver_s *ieee;
  FAR struct net_driver_s *dev;

  ninfo("Initializing\n");

  /* Get the interface structure associated with this interface number. */

  priv = &g_loopback;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lo_driver_s));

  ieee               = &priv->lo_ieee;
  dev                = &ieee->i_dev;
  dev->d_ifup        = lo_ifup;          /* I/F up (new IP address) callback */
  dev->d_ifdown      = lo_ifdown;        /* I/F down callback */
  dev->d_txavail     = lo_txavail;       /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  dev->d_addmac      = lo_addmac;        /* Add multicast MAC address */
  dev->d_rmmac       = lo_rmmac;         /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl       = lo_ioctl;         /* Handle network IOCTL commands */
#endif
  dev->d_buf         = g_iobuffer;       /* Attach the IO buffer */
  dev->d_private     = (FAR void *)priv; /* Used to recover private state from dev */

  /* Set the network mask and advertise our MAC-based IP address */

  lo_netmask(dev);
  lo_addr2ip(dev);

  /* Initialize the Network frame-related callbacks */

  ieee->i_get_mhrlen = lo_get_mhrlen;    /* Get MAC header length */
  ieee->i_req_data   = lo_req_data;      /* Enqueue frame for transmission */

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->lo_polldog  = wd_create();     /* Create periodic poll timer */

  /* Register the loopabck device with the OS so that socket IOCTLs can be
   * performed.
   */

  (void)netdev_register(&priv->lo_ieee.i_dev, NET_LL_IEEE802154);

  /* Put the network in the UP state */

  dev->d_flags = IFF_UP;
  return lo_ifup(&priv->lo_ieee.i_dev);
}

#endif /* CONFIG_IEEE802154_LOOPBACK */

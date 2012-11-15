/*******************************************************************************
*
* eth_frame.h
*
* Steve Holford
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#if !defined(ETH_FRAME_H)
#define ETH_FRAME_H

#include "globalDefs.h"

/*******************************************************************************
* Define basic ethernet frame per IEEE802.3 standard
*******************************************************************************/

/* Ethernet standard lengths in bytes*/
#define ETH_ADDR_LEN    (6)
#define ETH_TYPE_LEN    (2)
#define ETH_CRC_LEN     (4)
#define ETH_MAX_DATA    (1500)
#define ETH_MIN_DATA    (46)
#define ETH_HDR_LEN     (ETH_ADDR_LEN * 2 + ETH_TYPE_LEN)

/* Defined Ethernet Types */
#define ETHTYPE_IPV4    (0x0800)        /* IP V4 RFC-894 (RFC-791) */
#define ETHTYPE_ARP     (0x0806)        /* ARP RFC-826 */
#define ETHTYPE_IPV6    (0x86DD)        /* IP V6 RFC-2464 */

/* Maximum and Minimum Ethernet Frame Sizes */
#define ETH_MAX_FRM     (ETH_HDR_LEN + ETH_MAX_DATA + ETH_CRC_LEN)
#define ETH_MIN_FRM     (ETH_HDR_LEN + ETH_MIN_DATA + ETH_CRC_LEN)
#define ETH_MTU         (ETH_HDR_LEN + ETH_MAX_DATA)

/* Ethernet Frame Header definition */
typedef struct eth_header_s {
    uint8_t    dest[ETH_ADDR_LEN];
    uint8_t    src[ETH_ADDR_LEN];
    uint16_t   type;
} eth_header_t;

/*******************************************************************************
* Define Packet elements for common protocols
*******************************************************************************/

#define ETH_IP4ADDR_LEN   (4)

/* ARP , EthType = 0x0806 */
typedef struct arp_packet_s {
    uint16_t    htype;          /* Network protocol type, Ethernet = 1 */
    uint16_t    ptype;          /* Internetwork protocol, IPV4 = 0x0800 */
    uint8_t     hlen;           /* Length of hardware address in octets = 6 */
    uint8_t     plen;           /* Length of protocol address in octets = 4 */
    uint16_t    oper;           /* Operation request=1, answer=2 */
    uint8_t     sha[ETH_ADDR_LEN]; /* sender hardware address */
    uint8_t     spa[ETH_IP4ADDR_LEN]; /* sender protocol address */
    uint8_t     tha[ETH_ADDR_LEN]; /* target hardware address */
    uint8_t     tpa[ETH_IP4ADDR_LEN]; /* target IP4 address */
} arp_packet_t;

#define ETH_ARP_LEN (28)

/* IPV4, EthType = 0x0800 */
typedef struct ipv4_packet_s {
    uint8_t     ver_ihl;        /* 0-3 version (4), 4-7 header len (5 Min)*/
    uint8_t     dscp_ecn;       /* 0-5 diffentiated services,6-7 congestion */
    uint16_t    len;            /* Total length - min 20bytes, max 65K */
    uint16_t    id;             /* fragement ID */
    uint16_t    flg_off;        /* 0-2 flags, 3-15 fragment offset */
    uint8_t     ttl;            /* time to live counter */
    uint8_t     protocol;       /* Datagram protocol number */
    uint16_t    header_csum;    /* Header checksum */
    uint8_t     spa[ETH_IP4ADDR_LEN]; /* source IPV4 Address */
    uint8_t     tpa[ETH_IP4ADDR_LEN]; /* target IPV4 Address */
    uint8_t*    opt_or_payload; /* Either payload (len=5) or options (rare) */
} ipv4_packet_t;

#define ETH_IPV4_LEN (20)

/* Some IPV4 datagram protocol numbers per ipv4_packet_t.protocol */

#define ETH_PROTO_ICMP      (0x01)
#define ETH_PROTO_TCP       (0x06)
#define ETH_PROTO_UDP       (0x11)

typedef struct icmp_packet_s {
    uint8_t type;
    uint8_t code;
    uint16_t csum;
} icmp_packet_t;

#define ETH_ICMP_ECHO_REQUEST (8)
#define ETH_ICMP_ECHO_REPLY (0)

#define ETH_ICMP_LEN (4)

typedef struct udp_packet_s {
    uint16_t source_port;
    uint16_t dest_port;
    uint16_t len;
    uint16_t csum;
} udp_packet_t;

#define ETH_UDP_LEN (8)

typedef struct tcp_packet_s {
    uint16_t source_port;
    uint16_t dest_port;
    uint32_t tx_sequence;
    uint32_t ack_number;
    uint16_t flags;
    uint16_t window;
    uint16_t csum;
    uint16_t urgent;
    uint32_t options;
} tcp_packet_t;

#define ETH_TCP_LEN (24)

#define TCP_FLG_FIN_MSK (0x0001)
#define TCP_FLG_SYN_MSK (0x0002)
#define TCP_FLG_RST_MSK (0x0004)
#define TCP_FLG_PSH_MSK (0x0008)
#define TCP_FLG_ACK_MSK (0x0010)
#define TCP_FLG_URG_MSK (0x0020)
#define TCP_FLG_ECE_MSK (0x0040)
#define TCP_FLG_CWR_MSK (0x0080)
#define TCP_FLG_NS_MSK  (0x0100)
#define TCP_FLG_DATA_MSK (0xF000)
#define TCP_FLG_DATA_SHIFT (12)

#define TCP_FLG_GET_DATA(data) ((data & TCP_FLG_DATA_MSK) >> TCP_FLG_DATA_SHIFT)
#define TCP_FLG_SET_DATA(data) ((data << TCP_FLG_DATA_SHIFT) & TCP_FLG_DATA_MASK)

#endif  /* !defined(ETH_FRAME_H) */

/*******************************************************************************
*
* demoEnet.c
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "globalDefs.h"
#include "kinetis.h"
#include "hardware.h"
#include "util.h"
#include "eth_frame.h"

/*
 * Transmit and receive buffers, packet sized appropriately
 */

static uint8_t tx_buffer[ETH_MAX_FRM];
static uint8_t rx_buffer[ETH_MAX_FRM];

/*
 * Two useful MAC addresses used in ARP, plus my own fake one 
 */

static uint8_t zero_addr[ETH_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
static uint8_t eff_addr[ETH_ADDR_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t my_addr[ETH_ADDR_LEN] = { 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 };

/*
 * My fake IP Address
 */

static uint8_t my_ip[ETH_IP4ADDR_LEN] = { 192,168,1,13 };

/*
 * Worker function to return TRUE if the two IPV4 addresses match
 */

int ipv4_addr_match (uint8_t* a1, uint8_t* a2)
{   
    int i;

    for (i = 0; i < ETH_IP4ADDR_LEN; i++) {
        if (a1[i] != a2[i]) return FALSE;
    }
    return TRUE;
}

/*
 * Worker function to copy an IPV4 address
 */

void ipv4_addr_copy (uint8_t* dest, uint8_t* src)
{   
    int i;

    for (i = 0; i < ETH_IP4ADDR_LEN; i++) {
        dest[i] = src[i];
    }
}

/*
 * Print out the passed MAC address in human form
 */

void printMacAddr(uint8_t * addr)
{
    int i;
    for ( i = 0; i < 5; i++) {
        iprintf("%02X:",addr[i]);
    }
    iprintf ("%02X", addr[5]);
}

/*
 * Print out the passed IPV4 address in the standard dotted quad format
 */

void printIPAddr(uint8_t * addr)
{
    int i;
    for ( i = 0; i < 3; i++) {
        iprintf("%d.",addr[i]);
    }
    iprintf ("%d", addr[3]);
}

/*
 * Creat the appropriate reply to an incoming ARP request.
 * Per RFC, reflect it back to source with our MAC and IP,
 * Change the operand code to 0x2, ARP reply.
 */

void reply_arp(int fd, arp_packet_t *arp)
{
    eth_header_t *eth_reply = (eth_header_t *) tx_buffer;
    arp_packet_t *reply = (arp_packet_t *)(tx_buffer+ETH_HDR_LEN);
    iprintf("I've Been ARP'D, Forming Reply\n");
    
    eth_hwaddr_copy(eth_reply->dest, arp->sha);
    eth_hwaddr_copy(eth_reply->src, my_addr);
    eth_reply->type = BSWAP16(ETHTYPE_ARP);
    reply->htype = BSWAP16(0x0001);
    reply->ptype = BSWAP16(0x0800);
    reply->hlen = 0x06;
    reply->plen = 0x04;
    reply->oper = BSWAP16(0x0002);  /* ARP reply */
    eth_hwaddr_copy(reply->sha, my_addr);
    ipv4_addr_copy(reply->spa, my_ip);
    eth_hwaddr_copy(reply->tha, arp->sha);
    ipv4_addr_copy(reply->tpa, arp->spa);
    write(fd, tx_buffer, (ETH_HDR_LEN+ETH_ARP_LEN));
}

/*
 * Rip apart the received ARP packet and iprintf it.
 * If the arp is a request and either has a broadcast addr
 * or my MAC addr, and also my IPV4 addr, respond.
 */

void parse_arp (int fd, uint8_t *buf)
{
    arp_packet_t *arp = (arp_packet_t *)buf;
    
    iprintf("***** Processing ARP *****\n");
    iprintf("HType %04X, PType %04X\n", BSWAP16(arp->htype), BSWAP16(arp->ptype));
    iprintf("HLen  %02X, PLen  %02X\n", arp->hlen, arp->plen);
    iprintf("Operation : %s\n", (BSWAP16(arp->oper) == 1 ? ">>> Request >>>" : "<<< Answer <<<"));
    iprintf ("Sender : ");
    printMacAddr(arp->sha);
    iprintf (" => ");
    printIPAddr(arp->spa);
    iprintf ("\n");
    iprintf ("Target : ");
    printMacAddr(arp->tha);
    iprintf (" => ");
    printIPAddr(arp->tpa);
    iprintf ("\n");
    if (BSWAP16(arp->oper) == 1) {
        if ((eth_hwaddr_zero(arp->tha) || eth_hwaddr_match(arp->tha,my_addr)) && ipv4_addr_match(arp->tpa, my_ip)) {
            reply_arp(fd, arp);
        }
    }
}

/*
 * Rip apart and display the received ICMP packet.
 * If it is an echo request (type 8) respond to the requestor.
 * Appropriate response is an ICMP echo reply with the originators
 * payload copied exactly.
 */

void do_icmp (int fd, eth_header_t * eth, ipv4_packet_t * ipv4, uint8_t *buf)
{
    icmp_packet_t *icmp = (icmp_packet_t *)buf;
    eth_header_t *eth_reply = (eth_header_t *) tx_buffer;
    ipv4_packet_t *ipv4_reply = (ipv4_packet_t *)(tx_buffer+ETH_HDR_LEN);
    icmp_packet_t *icmp_reply = (icmp_packet_t *)(tx_buffer+ETH_HDR_LEN+ETH_IPV4_LEN);

    int header_len = ETH_HDR_LEN+ETH_IPV4_LEN+ETH_ICMP_LEN; 
    uint8_t *orig_payload = (buf + ETH_ICMP_LEN);
    uint8_t *new_payload = (tx_buffer + header_len);
    int payload_len = BSWAP16(ipv4->len) - (4 * ((ipv4->ver_ihl)&0x0F)) - ETH_ICMP_LEN;


    iprintf("***** Processing ICMP *****\n");
    iprintf("Type %02X, Code %02X\n", icmp->type, icmp->code);

    if (icmp->type == ETH_ICMP_ECHO_REQUEST) {
        eth_hwaddr_copy(eth_reply->dest, eth->src);
        eth_hwaddr_copy(eth_reply->src, my_addr);
        eth_reply->type = BSWAP16(ETHTYPE_IPV4);
        ipv4_reply->ver_ihl = 0x45;
        ipv4_reply->dscp_ecn = 0x00;
        ipv4_reply->len = ipv4->len;
        ipv4_reply->id = ipv4->id;
        ipv4_reply->flg_off = ipv4->flg_off;
        ipv4_reply->ttl = ipv4->ttl;
        ipv4_reply->protocol = ETH_PROTO_ICMP;
        ipv4_reply->header_csum = 0x0000;   /* MAC will calculate */
        ipv4_addr_copy(ipv4_reply->spa, my_ip);
        ipv4_addr_copy(ipv4_reply->tpa, ipv4->spa);
        icmp_reply->type = ETH_ICMP_ECHO_REPLY;
        icmp_reply->code = 0;
        icmp_reply->csum = 0x0000;  /* MAC will calculate */
        memcpy((void *)new_payload, (void *)orig_payload, payload_len);
        write(fd, tx_buffer, (header_len + payload_len));
    }
}

/*
 * Telnet
 */

typedef struct tcp_con_s {
    uint32_t my_seq;
    uint16_t back_port;
    uint16_t state;
} tcp_con_t;

void do_telnet(int fd, eth_header_t *eth, ipv4_packet_t * ipv4, tcp_packet_t * tcp, uint8_t * buf)
{
    static tcp_con_t connection = { 0x100, 0, 0 };
    uint16_t flags;
    int tx_len;
    int data_len;

    eth_header_t *eth_reply = (eth_header_t *) tx_buffer;
    ipv4_packet_t *ipv4_reply = (ipv4_packet_t *)(tx_buffer+ETH_HDR_LEN);
    tcp_packet_t *tcp_reply = (tcp_packet_t *)(tx_buffer + ETH_HDR_LEN + ETH_IPV4_LEN);
    eth_hwaddr_copy(eth_reply->dest, eth->src);
    eth_hwaddr_copy(eth_reply->src, my_addr);
    eth_reply->type = BSWAP16(ETHTYPE_IPV4);
    ipv4_reply->ver_ihl = 0x45;
    ipv4_reply->dscp_ecn = 0x00;
    ipv4_reply->len = ipv4->len;
    ipv4_reply->id = ipv4->id;
    ipv4_reply->flg_off = ipv4->flg_off;
    ipv4_reply->ttl = ipv4->ttl;
    ipv4_reply->protocol = ETH_PROTO_TCP;
    ipv4_reply->header_csum = 0x0000;   /* MAC will calculate */
    ipv4_addr_copy(ipv4_reply->spa, my_ip);
    ipv4_addr_copy(ipv4_reply->tpa, ipv4->spa);
    tcp_reply->source_port = tcp->dest_port;
    tcp_reply->dest_port = tcp->source_port;
    tcp_reply->window = BSWAP16(0x100);
    tcp_reply->csum = 0x0000;
    tcp_reply->urgent = 0x0000;

    flags = BSWAP16(tcp->flags);
    tx_len = ETH_HDR_LEN + ETH_IPV4_LEN + (TCP_FLG_GET_DATA(flags) * 4);

    data_len = BSWAP16(ipv4->len) - tx_len;
    if (data_len > 0) {
        buf[data_len] = 0;
        iprintf("Telnet Data: %s\n", buf);
    }

    if (flags & TCP_FLG_SYN_MSK) {
        connection.my_seq = 0x100;
        connection.back_port = BSWAP16(tcp->source_port);
        connection.state = 1;
        tcp_reply->ack_number = BSWAP32(BSWAP32(tcp->tx_sequence) + 1);
        tcp_reply->tx_sequence = BSWAP32(connection.my_seq++);
        flags = (flags & TCP_FLG_DATA_MSK) | TCP_FLG_ACK_MSK | TCP_FLG_SYN_MSK;
        tcp_reply->flags = BSWAP16(flags);
        write(fd, tx_buffer, tx_len);
    } else if (flags & TCP_FLG_ACK_MSK) {
        tcp_reply->ack_number = tcp->tx_sequence;
        tcp_reply->tx_sequence = BSWAP32(connection.my_seq);
        flags = (flags & TCP_FLG_DATA_MSK) | TCP_FLG_ACK_MSK;
        tcp_reply->flags = BSWAP16(flags);
        write(fd, tx_buffer, tx_len);
    } else if (flags & TCP_FLG_FIN_MSK) {
        tcp_reply->ack_number = tcp->tx_sequence;
        tcp_reply->tx_sequence = BSWAP32(connection.my_seq);
        connection.state = 0;
        flags = (flags & TCP_FLG_DATA_MSK) | TCP_FLG_ACK_MSK;
        tcp_reply->flags = BSWAP16(flags);
        write(fd, tx_buffer, tx_len);
    } else {
        tcp_reply->ack_number = tcp->tx_sequence;
        tcp_reply->tx_sequence = BSWAP32(connection.my_seq);
        flags = (flags & TCP_FLG_DATA_MSK) | TCP_FLG_ACK_MSK;
        tcp_reply->flags = BSWAP16(flags);
        write(fd, tx_buffer, tx_len);
    }
}

/*
 * You'd play with TCP packets in here
 */

void do_tcp (int fd, eth_header_t * eth, ipv4_packet_t * ipv4, uint8_t *buf)
{
    tcp_packet_t *tcp = (tcp_packet_t *)buf;
    iprintf("***** Processing TCP *****\n");
    iprintf("Source Port : %04X, Dest Port %04X\n", BSWAP16(tcp->source_port), BSWAP16(tcp->dest_port));
    iprintf("Seqence : %08X, Ack Num %08X\n", BSWAP32(tcp->tx_sequence), BSWAP32(tcp->ack_number));
    iprintf("Flags : %04X, Window %04X\n", BSWAP16(tcp->flags), BSWAP16(tcp->window));
    if (BSWAP16(tcp->dest_port) == 23)
        do_telnet(fd, eth, ipv4, tcp, buf + (4 * TCP_FLG_GET_DATA(BSWAP16(tcp->flags))));
}

/* 
 * You'd play with UDP packets in here
 */

void do_udp (int fd, eth_header_t * eth, ipv4_packet_t * ipv4, uint8_t *buf)
{
    iprintf("***** Processing UDP *****\n");
}

/*
 * Rip apart and display the IPV4 packet. 
 * If my fake IP is the destination, route it to either of the
 * ICMP, TCP or UDP functions based on the protocol.
 * Other protocols are ignored.
 */

void parse_ip (int fd, eth_header_t * eth, uint8_t *buf)
{
    ipv4_packet_t *ipv4 = (ipv4_packet_t *)buf;

    iprintf("***** Processing IPV4 *****\n");
    iprintf("Version %01X, Header Len %01X\n", (ipv4->ver_ihl) >> 4, (ipv4->ver_ihl) & 0x0F);
    iprintf("Packet Length %d\n", BSWAP16(ipv4->len));
    iprintf("Protocol %02X\n", ipv4->protocol);
    iprintf("Source : ");
    printIPAddr(ipv4->spa);
    iprintf ("\n");
    iprintf("Target : ");
    printIPAddr(ipv4->tpa);
    iprintf ("\n");
    if (ipv4_addr_match(ipv4->tpa, my_ip)) {
        switch (ipv4->protocol) {
        case ETH_PROTO_ICMP:
            do_icmp(fd, eth, ipv4, buf + (4 * ((ipv4->ver_ihl) & 0x0F)));
            break;
        case ETH_PROTO_TCP:
            do_tcp(fd, eth, ipv4, buf + (4 * ((ipv4->ver_ihl) & 0x0F)));
            break;
        case ETH_PROTO_UDP:
            do_udp(fd, eth, ipv4, buf + (4 * ((ipv4->ver_ihl) & 0x0F)));
            break;
        default:
            iprintf("!!!!! Unknown IP Protocol !!!!!\n");
            break;
        }
    }
}

/*
 * Parse the basic ethernet frame header.
 * Print the hardware addresses, the header type.
 * If it is either an ARP or IPV4 frame, call the next level
 * parsing functions. Otherwise ignore.
 */

void parse_enet_frame(int fd, uint8_t *buf)
{
    eth_header_t *eth_header;

    eth_header = (eth_header_t *) buf;
    iprintf ("Ethernet Header\n");
    iprintf ("Dest : ");
    printMacAddr(eth_header->dest);
    iprintf ("\nSource : ");
    printMacAddr(eth_header->src);
    iprintf ("\nType : %04X\n",BSWAP16(eth_header->type));
    
    switch BSWAP16(eth_header->type) {
    case ETHTYPE_ARP:
        parse_arp(fd, buf+ETH_HDR_LEN);
        break;
    case ETHTYPE_IPV4:
        parse_ip(fd, eth_header, buf+ETH_HDR_LEN);
        break;
    default:
        iprintf ("!!!!! Unhandled EthType !!!!!\n");
        break;
    }
}

/*
 * Loop, reading packets from the driver. The driver is non-blocking
 * and will return -1 until it has a packet in the internal buffers.
 * The entire packet is copied into the passed buffer up to the passed
 * length. If you pass a shorter length the driver still reads processes
 * the entire packet, it just gives you what you asked for and dumps the 
 * rest.
 *
 * Leave this by pressing SW1 and it powers down the ethernet channel and
 * closes the file descriptor.
 */

void do_packets(int fd)
{
    int len;

    while (1) {
        len = read(fd, rx_buffer, ETH_MAX_FRM);
        if (len != -1) {
            iprintf("======================================================\n");
            iprintf("Got Packet of %d bytes\n",len);
            parse_enet_frame(fd, rx_buffer);
        }
        delay();
        if (gpioRead(N_SWITCH_1_PORT, N_SWITCH_1_PIN) == 0)
            break;
    }
}

/*
 * Dump the up to 32 PHY registers in human form.
 * Uses IOCTL to do direct reads from the phy.
 * Will fail miserably if the interface is not in the ENET_ON state
 */

void dumpPhyState(int fd)
{
    int i;
    int phy_data;
    
    iprintf("\n    PHY Register Block\n");
    iprintf("--------------------------------");
    for (i = 0; i < 32; i++)
    {
        phy_data = i;
        if (!ioctl(fd, IO_IOCTL_ENET_GET_PHY_REG, (int)&phy_data))
            iprintf("\nReg %02X Failed",i);
        else if (!(i % 4))
            iprintf("\n0x%02X-0x%02X : %04X ", i, i + 3, phy_data);
        else
            iprintf("%04X ", phy_data);
    }
    iprintf("\n");
}

/*
 * Dump the configuration and link status from the driver
 */

void dumpEthernetState(int fd)
{
    enet_status_t status;
    enet_cfg_t config;

    ioctl(fd, IO_IOCTL_ENET_GET_PHY_CONFIG, (int)&config);
    ioctl(fd, IO_IOCTL_ENET_GET_PHY_STATUS, (int)&status);

    iprintf ("MAC Address : ");
    printMacAddr(config.mac_addr);
    iprintf ("\n");
    iprintf ("IPV4 Address : ");
    printIPAddr(my_ip);
    iprintf ("\n");
    if (status.link == ENET_LINK_UP)
        iprintf("\nLink : PRESENT");
    else
        iprintf("\nLink : MISSING");

    if (config.autoneg == ENET_AUTONEG_ON)
        iprintf("\nAuto Negotiate : ON");
    else
        iprintf("\nAuto Negotiate : OFF");

    if (status.speed == ENET_10BASET)
        iprintf("\nSpeed : 10-BASET");
    else
        iprintf("\nSpeed : 100-BASET");
    
    if (status.duplex == ENET_DUPLEX_HALF)
        iprintf("\nDuplex : HALF");
    else
        iprintf("\nDuplex : FULL");
    
    iprintf("\n");
}

/*
 * After the open command gets you a file descriptor,
 * you must at a minimum program a MAC address using the IOCTL
 * interface, and then tell the driver to set the state to ENET_ON.
 * If there is no cable plugged into your board, it will try, but
 * fail to go into the ENET_ON state, so nothing will work.
 */

int initialize_enet(int fd)
{  
    int eth_state;

    ioctl(fd, IO_IOCTL_ENET_SET_MAC_ADDRESS, (int) my_addr);
    ioctl(fd, IO_IOCTL_ENET_SET_ENET_STATE, ENET_ON);
    dumpPhyState(fd);
    dumpEthernetState(fd);
    ioctl(fd, IO_IOCTL_ENET_GET_ENET_STATE, (int)&eth_state);
    return (eth_state == ENET_ON);
}

/*
 * Open the eth0 device, call the initialization funtion.
 * If that works, run off and play with packets until SW1 
 * is pressed.
 */

void enet_posix_test(void)
{
    int eth_channel;

    eth_channel = open("eth0",0,0);

    if (eth_channel == -1) {
        iprintf("Failed to open eth0\n");
        return;
    }
    else {
        if (initialize_enet(eth_channel))
            do_packets(eth_channel);
    }
    close(eth_channel);
    iprintf("Closed Ethernet eth0\n");
}

/*
 * Rob's good old LED masher. Board comes up in the led
 * twiddling state. If you push SW1, it runs off and trys to
 * open the ethernet driver. 
 */

int main(void)
{
    int fd;
#if defined(K60N512)
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_2, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);
#elif defined(K60F120)
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_3, DIVIDE_BY_5);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_120MHZ);
#endif
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

    gpioConfig(N_SWITCH_1_PORT,   N_SWITCH_1_PIN,   GPIO_INPUT);

    enet_install();
    uart_install();
    fd = fdevopen(stdin,  STDIO_UART, 0, 0);
    fd = fdevopen(stdout, STDIO_UART, 0, 0);
    fd = fdevopen(stderr, STDIO_UART, 0, 0);
    ioctl(fd, IO_IOCTL_UART_BAUD_SET, 115200);
    iprintf("Starting ENET Demo\n");
    for (;;) {
        delay();
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        delay();
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        delay();
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

        delay();
        gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        delay();
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        delay();
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        delay();
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

        if (gpioRead(N_SWITCH_1_PORT, N_SWITCH_1_PIN) == 0) {

            enet_posix_test();
        }
    }

    return 0;
}

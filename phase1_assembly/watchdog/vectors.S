
    .syntax unified
    .thumb

    .extern _bootstack_top

    .section .isr_vector
    .globl  __isr_vector

__isr_vector:
    .long   _bootstack_top
    .long   __isr_reset
    .long   __isr_nmi
    .long   __isr_hardFault
    .long   __isr_mpuFault
    .long   __isr_busFault
    .long   __isr_usageFault
    .long   __isr_reserved_7
    .long   __isr_reserved_8
    .long   __isr_reserved_9
    .long   __isr_reserved_10
    .long   __isr_svcall
    .long   __isr_debug
    .long   __isr_reserved_13
    .long   __isr_pendsv
    .long   __isr_systick
    .long   __isr_dma0
    .long   __isr_dma1
    .long   __isr_dma2
    .long   __isr_dma3
    .long   __isr_dma4
    .long   __isr_dma5
    .long   __isr_dma6
    .long   __isr_dma7
    .long   __isr_dma8
    .long   __isr_dma9
    .long   __isr_dma10
    .long   __isr_dma11
    .long   __isr_dma12
    .long   __isr_dma13
    .long   __isr_dma14
    .long   __isr_dma15
    .long   __isr_dma_error
    .long   __isr_mcm
    .long   __isr_flash_command_complete
    .long   __isr_flash_read_collision
    .long   __isr_mode_controller
    .long   __isr_llwu
    .long   __isr_wdog
    .long   __isr_rngb
    .long   __isr_i2c0
    .long   __isr_i2c1
    .long   __isr_spi0
    .long   __isr_spi1
    .long   __isr_spi2
    .long   __isr_can0_ored_message_buffer
    .long   __isr_can0_buf_off
    .long   __isr_can0_error
    .long   __isr_can0_transmit_warning
    .long   __isr_can0_receive_warning
    .long   __isr_can0_wake_up
    .long   __isr_can0_imeu
    .long   __isr_can0_lost_receive
    .long   __isr_can1_ored_message_buffer
    .long   __isr_can1_buf_off
    .long   __isr_can1_error
    .long   __isr_can1_transmit_warning
    .long   __isr_can1_receive_warning
    .long   __isr_can1_wake_up
    .long   __isr_can1_imeu
    .long   __isr_can1_lost_receive
    .long   __isr_uart0_status_sources
    .long   __isr_uart0_error_sources
    .long   __isr_uart1_status_sources
    .long   __isr_uart1_error_sources
    .long   __isr_uart2_status_sources
    .long   __isr_uart2_error_sources
    .long   __isr_uart3_status_sources
    .long   __isr_uart3_error_sources
    .long   __isr_uart4_status_sources
    .long   __isr_uart4_error_sources
    .long   __isr_uart5_status_sources
    .long   __isr_uart5_error_sources
    .long   __isr_adc0
    .long   __isr_adc1
    .long   __isr_cmp0
    .long   __isr_cmp1
    .long   __isr_cmp2
    .long   __isr_ftm0
    .long   __isr_ftm1
    .long   __isr_ftm2
    .long   __isr_cmt
    .long   __isr_rtc
    .long   __isr_reserved_83
    .long   __isr_pit0
    .long   __isr_pit1
    .long   __isr_pit2
    .long   __isr_pit3
    .long   __isr_pdb
    .long   __isr_usb_otg
    .long   __isr_usb_charger_detect
    .long   __isr_ethernet_mac_timer
    .long   __isr_ethernet_mac_transmit
    .long   __isr_ethernet_mac_receive
    .long   __isr_ethernet_mac_error
    .long   __isr_i2s
    .long   __isr_sdhc
    .long   __isr_dac0
    .long   __isr_dac1
    .long   __isr_tsi
    .long   __isr_mcg
    .long   __isr_low_power_timer
    .long   __isr_reserved_102
    .long   __isr_port_control_module_a
    .long   __isr_port_control_module_b
    .long   __isr_port_control_module_c
    .long   __isr_port_control_module_d
    .long   __isr_port_control_module_e

    .section .isr_handlers,"ax",%progbits
    .thumb_func
    .weak   __isr_reset
    .globl  __isr_reset
__isr_reset:                B   __start_s
    .weak   __isr_nmi
    .global __isr_nmi
    .thumb_func
__isr_nmi:                  B   __isr_nmi
    .weak   __isr_hardFault
    .global __isr_hardFault
    .thumb_func
__isr_hardFault:            B   __isr_hardFault
    .weak   __isr_mpuFault
    .global __isr_mpuFault
    .thumb_func
__isr_mpuFault:             B   __isr_mpuFault
    .weak   __isr_busFault
    .global __isr_busFault
    .thumb_func
__isr_busFault:             B   __isr_busFault
    .weak   __isr_usageFault
    .global __isr_usageFault
    .thumb_func
__isr_usageFault:           B   __isr_usageFault
    .weak   __isr_reserved_7
    .global __isr_reserved_7
    .thumb_func
__isr_reserved_7:           B   __isr_reserved_7
    .weak   __isr_reserved_8
    .global __isr_reserved_8
    .thumb_func
__isr_reserved_8:           B   __isr_reserved_8
    .weak   __isr_reserved_9
    .global __isr_reserved_9
    .thumb_func
__isr_reserved_9:           B   __isr_reserved_9
    .weak   __isr_reserved_10
    .global __isr_reserved_10
    .thumb_func
__isr_reserved_10:          B   __isr_reserved_10
    .weak   __isr_svcall
    .global __isr_svcall
    .thumb_func
__isr_svcall:               B   __isr_svcall
    .weak   __isr_debug
    .global __isr_debug
    .thumb_func
__isr_debug:                B   __isr_debug
    .weak   __isr_reserved_13
    .global __isr_reserved_13
    .thumb_func
__isr_reserved_13:          B   __isr_reserved_13
    .weak   __isr_pendsv
    .global __isr_pendsv
    .thumb_func
__isr_pendsv:               B   __isr_pendsv
    .weak   __isr_systick
    .global __isr_systick
    .thumb_func
__isr_systick:              B   __isr_systick
    .weak   __isr_dma0
    .globl  __isr_dma0
    .thumb_func
__isr_dma0:                 B   __isr_dma0
    .weak   __isr_dma1
    .globl  __isr_dma1
    .thumb_func
__isr_dma1:                 B   __isr_dma1
    .weak   __isr_dma2
    .globl  __isr_dma2
    .thumb_func
__isr_dma2:                 B   __isr_dma2
    .weak   __isr_dma3
    .globl  __isr_dma3
    .thumb_func
__isr_dma3:                 B   __isr_dma3
    .weak   __isr_dma4
    .globl  __isr_dma4
    .thumb_func
__isr_dma4:                 B   __isr_dma4
    .weak   __isr_dma5
    .globl  __isr_dma5
    .thumb_func
__isr_dma5:                 B   __isr_dma5
    .weak   __isr_dma6
    .globl  __isr_dma6
    .thumb_func
__isr_dma6:                 B   __isr_dma6
    .weak   __isr_dma7
    .globl  __isr_dma7
    .thumb_func
__isr_dma7:                 B   __isr_dma7
    .weak   __isr_dma8
    .globl  __isr_dma8
    .thumb_func
__isr_dma8:                 B   __isr_dma8
    .weak   __isr_dma9
    .globl  __isr_dma9
    .thumb_func
__isr_dma9:                 B   __isr_dma9
    .weak   __isr_dma10
    .globl  __isr_dma10
    .thumb_func
__isr_dma10:                B   __isr_dma10
    .weak   __isr_dma11
    .globl  __isr_dma11
    .thumb_func
__isr_dma11:                B   __isr_dma11
    .weak   __isr_dma12
    .globl  __isr_dma12
    .thumb_func
__isr_dma12:                B   __isr_dma12
    .weak   __isr_dma13
    .globl  __isr_dma13
    .thumb_func
__isr_dma13:                B   __isr_dma13
    .weak   __isr_dma14
    .globl  __isr_dma14
    .thumb_func
__isr_dma14:                B   __isr_dma14
    .weak   __isr_dma15
    .globl  __isr_dma15
    .thumb_func
__isr_dma15:                B   __isr_dma15
    .weak   __isr_dma_error
    .globl  __isr_dma_error
    .thumb_func
__isr_dma_error:            B   __isr_dma_error
    .weak   __isr_mcm
    .globl  __isr_mcm
    .thumb_func
__isr_mcm:                  B   __isr_mcm
    .weak   __isr_flash_command_complete
    .globl  __isr_flash_command_complete
    .thumb_func
__isr_flash_command_complete:               B   __isr_flash_command_complete
    .weak   __isr_flash_read_collision
    .globl  __isr_flash_read_collision
    .thumb_func
__isr_flash_read_collision:                 B   __isr_flash_read_collision
    .weak   __isr_mode_controller
    .globl  __isr_mode_controller
    .thumb_func
__isr_mode_controller:                      B   __isr_mode_controller
    .weak   __isr_llwu
    .globl  __isr_llwu
    .thumb_func
__isr_llwu:                 B   __isr_llwu
    .weak   __isr_wdog
    .globl  __isr_wdog
    .thumb_func
__isr_wdog:                 B   __isr_wdog
    .weak   __isr_rngb
    .globl  __isr_rngb
    .thumb_func
__isr_rngb:                 B   __isr_rngb
    .weak   __isr_i2c0
    .globl  __isr_i2c0
    .thumb_func
__isr_i2c0:                 B   __isr_i2c0
    .weak   __isr_i2c1
    .globl  __isr_i2c1
    .thumb_func
__isr_i2c1:                 B   __isr_i2c1
    .weak   __isr_spi0
    .globl  __isr_spi0
    .thumb_func
__isr_spi0:                 B   __isr_spi0
    .weak   __isr_spi1
    .globl  __isr_spi1
    .thumb_func
__isr_spi1:                 B   __isr_spi1
    .weak   __isr_spi2
    .globl  __isr_spi2
    .thumb_func
__isr_spi2:                 B   __isr_spi2
    .weak   __isr_can0_ored_message_buffer
    .globl  __isr_can0_ored_message_buffer
    .thumb_func
__isr_can0_ored_message_buffer:             B   __isr_can0_ored_message_buffer
    .weak   __isr_can0_buf_off
    .globl  __isr_can0_buf_off
    .thumb_func
__isr_can0_buf_off:                         B   __isr_can0_buf_off
    .weak   __isr_can0_error
    .globl  __isr_can0_error
    .thumb_func
__isr_can0_error:                           B   __isr_can0_error
    .weak   __isr_can0_transmit_warning
    .globl  __isr_can0_transmit_warning
    .thumb_func
__isr_can0_transmit_warning:                B   __isr_can0_transmit_warning
    .weak   __isr_can0_receive_warning
    .globl  __isr_can0_receive_warning
    .thumb_func
__isr_can0_receive_warning:                 B   __isr_can0_receive_warning
    .weak   __isr_can0_wake_up
    .globl  __isr_can0_wake_up
    .thumb_func
__isr_can0_wake_up:                         B   __isr_can0_wake_up
    .weak   __isr_can0_imeu
    .globl  __isr_can0_imeu
    .thumb_func
__isr_can0_imeu:                            B   __isr_can0_imeu
    .weak   __isr_can0_lost_receive
    .globl  __isr_can0_lost_receive
    .thumb_func
__isr_can0_lost_receive:                    B   __isr_can0_lost_receive
    .weak   __isr_can1_ored_message_buffer
    .globl  __isr_can1_ored_message_buffer
    .thumb_func
__isr_can1_ored_message_buffer:             B   __isr_can1_ored_message_buffer
    .weak   __isr_can1_buf_off
    .globl  __isr_can1_buf_off
    .thumb_func
__isr_can1_buf_off:                         B   __isr_can1_buf_off
    .weak   __isr_can1_error
    .globl  __isr_can1_error
    .thumb_func
__isr_can1_error:                           B   __isr_can1_error
    .weak   __isr_can1_transmit_warning
    .globl  __isr_can1_transmit_warning
    .thumb_func
__isr_can1_transmit_warning:                B   __isr_can1_transmit_warning
    .weak   __isr_can1_receive_warning
    .globl  __isr_can1_receive_warning
    .thumb_func
__isr_can1_receive_warning:                 B   __isr_can1_receive_warning
    .weak   __isr_can1_wake_up
    .globl  __isr_can1_wake_up
    .thumb_func
__isr_can1_wake_up:                         B   __isr_can1_wake_up
    .weak   __isr_can1_imeu
    .globl  __isr_can1_imeu
    .thumb_func
__isr_can1_imeu:                            B   __isr_can1_imeu
    .weak   __isr_can1_lost_receive
    .globl  __isr_can1_lost_receive
    .thumb_func
__isr_can1_lost_receive:                    B   __isr_can1_lost_receive
    .weak   __isr_uart0_status_sources
    .globl  __isr_uart0_status_sources
    .thumb_func
__isr_uart0_status_sources:                 B   __isr_uart0_status_sources
    .weak   __isr_uart0_error_sources
    .globl  __isr_uart0_error_sources
    .thumb_func
__isr_uart0_error_sources:                  B   __isr_uart0_error_sources
    .weak   __isr_uart1_status_sources
    .globl  __isr_uart1_status_sources
    .thumb_func
__isr_uart1_status_sources:                 B   __isr_uart1_status_sources
    .weak   __isr_uart1_error_sources
    .globl  __isr_uart1_error_sources
    .thumb_func
__isr_uart1_error_sources:                  B   __isr_uart1_error_sources
    .weak   __isr_uart2_status_sources
    .globl  __isr_uart2_status_sources
    .thumb_func
__isr_uart2_status_sources:                 B   __isr_uart2_status_sources
    .weak   __isr_uart2_error_sources
    .globl  __isr_uart2_error_sources
    .thumb_func
__isr_uart2_error_sources:                  B   __isr_uart2_error_sources
    .weak   __isr_uart3_status_sources
    .globl  __isr_uart3_status_sources
    .thumb_func
__isr_uart3_status_sources:                 B   __isr_uart3_status_sources
    .weak   __isr_uart3_error_sources
    .globl  __isr_uart3_error_sources
    .thumb_func
__isr_uart3_error_sources:                  B   __isr_uart3_error_sources
    .weak   __isr_uart4_status_sources
    .globl  __isr_uart4_status_sources
    .thumb_func
__isr_uart4_status_sources:                 B   __isr_uart4_status_sources
    .weak   __isr_uart4_error_sources
    .globl  __isr_uart4_error_sources
    .thumb_func
__isr_uart4_error_sources:                  B   __isr_uart4_error_sources
    .weak   __isr_uart5_status_sources
    .globl  __isr_uart5_status_sources
    .thumb_func
__isr_uart5_status_sources:                 B   __isr_uart5_status_sources
    .weak   __isr_uart5_error_sources
    .globl  __isr_uart5_error_sources
    .thumb_func
__isr_uart5_error_sources:                  B   __isr_uart5_error_sources
    .weak   __isr_adc0
    .globl  __isr_adc0
    .thumb_func
__isr_adc0:                 B   __isr_adc0
    .weak   __isr_adc1
    .globl  __isr_adc1
    .thumb_func
__isr_adc1:                 B   __isr_adc1
    .weak   __isr_cmp0
    .globl  __isr_cmp0
    .thumb_func
__isr_cmp0:                 B   __isr_cmp0
    .weak   __isr_cmp1
    .globl  __isr_cmp1
    .thumb_func
__isr_cmp1:                 B   __isr_cmp1
    .weak   __isr_cmp2
    .globl  __isr_cmp2
    .thumb_func
__isr_cmp2:                 B   __isr_cmp2
    .weak   __isr_ftm0
    .globl  __isr_ftm0
    .thumb_func
__isr_ftm0:                 B   __isr_ftm0
    .weak   __isr_ftm1
    .globl  __isr_ftm1
    .thumb_func
__isr_ftm1:                 B   __isr_ftm1
    .weak   __isr_ftm2
    .globl  __isr_ftm2
    .thumb_func
__isr_ftm2:                 B   __isr_ftm2
    .weak   __isr_cmt
    .globl  __isr_cmt
    .thumb_func
__isr_cmt:                  B   __isr_cmt
    .weak   __isr_rtc
    .globl  __isr_rtc
    .thumb_func
__isr_rtc:                  B   __isr_rtc
    .weak   __isr_reserved_83
    .globl  __isr_reserved_83
    .thumb_func
__isr_reserved_83:          B   __isr_reserved_83
    .weak   __isr_pit0
    .globl  __isr_pit0
    .thumb_func
__isr_pit0:                 B   __isr_pit0
    .weak   __isr_pit1
    .globl  __isr_pit1
    .thumb_func
__isr_pit1:                 B   __isr_pit1
    .weak   __isr_pit2
    .globl  __isr_pit2
    .thumb_func
__isr_pit2:                 B   __isr_pit2
    .weak   __isr_pit3
    .globl  __isr_pit3
    .thumb_func
__isr_pit3:                 B   __isr_pit3
    .weak   __isr_pdb
    .globl  __isr_pdb
    .thumb_func
__isr_pdb:                  B   __isr_pdb
    .weak   __isr_usb_otg
    .globl  __isr_usb_otg
    .thumb_func
__isr_usb_otg:              B   __isr_usb_otg
    .weak   __isr_usb_charger_detect
    .globl  __isr_usb_charger_detect
    .thumb_func
__isr_usb_charger_detect:                   B   __isr_usb_charger_detect
    .weak   __isr_ethernet_mac_timer
    .globl  __isr_ethernet_mac_timer
    .thumb_func
__isr_ethernet_mac_timer:                   B   __isr_ethernet_mac_timer
    .weak   __isr_ethernet_mac_transmit
    .globl  __isr_ethernet_mac_transmit
    .thumb_func
__isr_ethernet_mac_transmit:                B   __isr_ethernet_mac_transmit
    .weak   __isr_ethernet_mac_receive
    .globl  __isr_ethernet_mac_receive
    .thumb_func
__isr_ethernet_mac_receive:                 B   __isr_ethernet_mac_receive
    .weak   __isr_ethernet_mac_error
    .globl  __isr_ethernet_mac_error
    .thumb_func
__isr_ethernet_mac_error:                   B   __isr_ethernet_mac_error
    .weak   __isr_i2s
    .globl  __isr_i2s
    .thumb_func
__isr_i2s:                  B   __isr_i2s
    .weak   __isr_sdhc
    .globl  __isr_sdhc
    .thumb_func
__isr_sdhc:                 B   __isr_sdhc
    .weak   __isr_dac0
    .globl  __isr_dac0
    .thumb_func
__isr_dac0:                 B   __isr_dac0
    .weak   __isr_dac1
    .globl  __isr_dac1
    .thumb_func
__isr_dac1:                 B   __isr_dac1
    .weak   __isr_tsi
    .globl  __isr_tsi
    .thumb_func
__isr_tsi:                  B   __isr_tsi
    .weak   __isr_mcg
    .globl  __isr_mcg
    .thumb_func
__isr_mcg:                  B   __isr_mcg
    .weak   __isr_low_power_timer
    .globl  __isr_low_power_timer
    .thumb_func
__isr_low_power_timer:                      B   __isr_low_power_timer
    .weak   __isr_reserved_102
    .globl  __isr_reserved_102
    .thumb_func
__isr_reserved_102:                         B   __isr_reserved_102
    .weak   __isr_port_control_module_a
    .globl  __isr_port_control_module_a
    .thumb_func
__isr_port_control_module_a:                B   __isr_port_control_module_a
    .weak   __isr_port_control_module_b
    .globl  __isr_port_control_module_b
    .thumb_func
__isr_port_control_module_b:                B   __isr_port_control_module_b
    .weak   __isr_port_control_module_c
    .globl  __isr_port_control_module_c
    .thumb_func
__isr_port_control_module_c:                B   __isr_port_control_module_c
    .weak   __isr_port_control_module_d
    .globl  __isr_port_control_module_d
    .thumb_func
__isr_port_control_module_d:                B   __isr_port_control_module_d
    .weak   __isr_port_control_module_e
    .globl  __isr_port_control_module_e
    .thumb_func
__isr_port_control_module_e:                B   __isr_port_control_module_e

    .end

/* ----------------------------------------------------------------------------
   -- UART
   ---------------------------------------------------------------------------- */

#define BDH_OFFSET 0x00   /* UART Baud Rate Registers: High */
#define BDL_OFFSET 0x01   /* UART Baud Rate Registers: Low  */
#define C1_OFFSET  0x02   /* UART Control Register 1        */
#define C2_OFFSET  0x03   /* UART Control Register 2        */
#define S1_OFFSET  0x04   /* UART Status Register 1         */
#define S2_OFFSET  0x05   /* UART Status Register 2         */
#define C3_OFFSET  0x06   /* UART Control Register 3        */
#define D_OFFSET   0x07   /* UART Data Register             */
#define MA1_OFFSET 0x08   /* UART Match Address Register 1  */
#define MA2_OFFSET 0x09   /* UART Match Address Register 2  */
#define C4_OFFSET  0x0a   /* UART Control Register 4        */
#define C5_OFFSET  0x0b   /* UART Control Register 5        */

/** UART - Peripheral register structure */
#if 0
typedef struct UART_MemMap {
  uint8_t BDH;                                     /**< UART Baud Rate Registers:High, offset: 0x0 */
  uint8_t BDL;                                     /**< UART Baud Rate Registers: Low, offset: 0x1 */
  uint8_t C1;                                      /**< UART Control Register 1, offset: 0x2 */
  uint8_t C2;                                      /**< UART Control Register 2, offset: 0x3 */
  uint8_t S1;                                      /**< UART Status Register 1, offset: 0x4 */
  uint8_t S2;                                      /**< UART Status Register 2, offset: 0x5 */
  uint8_t C3;                                      /**< UART Control Register 3, offset: 0x6 */
  uint8_t D;                                       /**< UART Data Register, offset: 0x7 */
  uint8_t MA1;                                     /**< UART Match Address Registers 1, offset: 0x8 */
  uint8_t MA2;                                     /**< UART Match Address Registers 2, offset: 0x9 */
  uint8_t C4;                                      /**< UART Control Register 4, offset: 0xA */
  uint8_t C5;                                      /**< UART Control Register 5, offset: 0xB */
  uint8_t ED;                                      /**< UART Extended Data Register, offset: 0xC */
  uint8_t MODEM;                                   /**< UART Modem Register, offset: 0xD */
  uint8_t IR;                                      /**< UART Infrared Register, offset: 0xE */
  uint8_t RESERVED_0[1];
  uint8_t PFIFO;                                   /**< UART FIFO Parameters, offset: 0x10 */
  uint8_t CFIFO;                                   /**< UART FIFO Control Register, offset: 0x11 */
  uint8_t SFIFO;                                   /**< UART FIFO Status Register, offset: 0x12 */
  uint8_t TWFIFO;                                  /**< UART FIFO Transmit Watermark, offset: 0x13 */
  uint8_t TCFIFO;                                  /**< UART FIFO Transmit Count, offset: 0x14 */
  uint8_t RWFIFO;                                  /**< UART FIFO Receive Watermark, offset: 0x15 */
  uint8_t RCFIFO;                                  /**< UART FIFO Receive Count, offset: 0x16 */
  uint8_t RESERVED_1[1];
  uint8_t C7816;                                   /**< UART 7816 Control Register, offset: 0x18 */
  uint8_t IE7816;                                  /**< UART 7816 Interrupt Enable Register, offset: 0x19 */
  uint8_t IS7816;                                  /**< UART 7816 Interrupt Status Register, offset: 0x1A */
  union {                                          /* offset: 0x1B */
    uint8_t WP7816_T_TYPE0;                          /**< UART 7816 Wait Parameter Register, offset: 0x1B */
    uint8_t WP7816_T_TYPE1;                          /**< UART 7816 Wait Parameter Register, offset: 0x1B */
  };
  uint8_t WN7816;                                  /**< UART 7816 Wait N Register, offset: 0x1C */
  uint8_t WF7816;                                  /**< UART 7816 Wait FD Register, offset: 0x1D */
  uint8_t ET7816;                                  /**< UART 7816 Error Threshold Register, offset: 0x1E */
  uint8_t TL7816;                                  /**< UART 7816 Transmit Length Register, offset: 0x1F */
} volatile *UART_MemMapPtr;
#endif

/* ----------------------------------------------------------------------------
   -- UART - Register accessor macros
   ---------------------------------------------------------------------------- */

/* UART - Register accessors */
#define UART_BDH_REG(base)                       (base + BDH_OFFSET)
#define UART_BDL_REG(base)                       (base + BDL_OFFSET)
#define UART_C1_REG(base)                        (base + C1_OFFSET)
#define UART_C2_REG(base)                        (base + C2_OFFSET)
#define UART_S1_REG(base)                        (base + S1_OFFSET)
#define UART_S2_REG(base)                        (base + S2_OFFSET)
#define UART_C3_REG(base)                        (base + C3_OFFSET)
#define UART_D_REG(base)                         (base + D_OFFSET)
#define UART_MA1_REG(base)                       (base + MA1_OFFSET)
#define UART_MA2_REG(base)                       (base + MA2_OFFSET)
#define UART_C4_REG(base)                        (base + C4_OFFSET)
#define UART_C5_REG(base)                        (base + C5_OFFSET)

#if 0
#define UART_ED_REG(base)                        (base + ED_OFFSET)
#define UART_MODEM_REG(base)                     (base + MODEM_OFFSET)
#define UART_IR_REG(base)                        (base + IR_OFFSET)
#define UART_PFIFO_REG(base)                     (base + PFIFO_OFFSET)
#define UART_CFIFO_REG(base)                     (base + CFIFO_OFFSET)
#define UART_SFIFO_REG(base)                     (base + SFIFO_OFFSET)
#define UART_TWFIFO_REG(base)                    (base + TWFIFO_OFFSET)
#define UART_TCFIFO_REG(base)                    (base + TCFIFO_OFFSET)
#define UART_RWFIFO_REG(base)                    (base + RWFIFO_OFFSET)
#define UART_RCFIFO_REG(base)                    (base + RCFIFO_OFFSET)
#define UART_C7816_REG(base)                     (base + C7816_OFFSET)
#define UART_IE7816_REG(base)                    (base + IE7816_OFFSET)
#define UART_IS7816_REG(base)                    (base + IS7816_OFFSET)
#define UART_WP7816_T_TYPE0_REG(base)            (base + WP7816_T_TYPE0_OFFSET)
#define UART_WP7816_T_TYPE1_REG(base)            (base + WP7816_T_TYPE1_OFFSET)
#define UART_WN7816_REG(base)                    (base + WN7816_OFFSET)
#define UART_WF7816_REG(base)                    (base + WF7816_OFFSET)
#define UART_ET7816_REG(base)                    (base + ET7816_OFFSET)
#define UART_TL7816_REG(base)                    (base + TL7816_OFFSET)
#endif

/* UART - Peripheral instance base addresses */
/** Peripheral UART0 base pointer */
#define UART0_BASE_PTR                           (0x4006A000)
/** Peripheral UART1 base pointer */
#define UART1_BASE_PTR                           (0x4006B000)
/** Peripheral UART2 base pointer */
#define UART2_BASE_PTR                           (0x4006C000)
/** Peripheral UART3 base pointer */
#define UART3_BASE_PTR                           (0x4006D000)
/** Peripheral UART4 base pointer */
#define UART4_BASE_PTR                           (0x400EA000)
/** Peripheral UART5 base pointer */
#define UART5_BASE_PTR                           (0x400EB000)

/* UART3 */
#define UART3_BDH                                UART_BDH_REG(UART3_BASE_PTR)
#define UART3_BDL                                UART_BDL_REG(UART3_BASE_PTR)
#define UART3_C1                                 UART_C1_REG(UART3_BASE_PTR)
#define UART3_C2                                 UART_C2_REG(UART3_BASE_PTR)
#define UART3_S1                                 UART_S1_REG(UART3_BASE_PTR)
#define UART3_S2                                 UART_S2_REG(UART3_BASE_PTR)
#define UART3_C3                                 UART_C3_REG(UART3_BASE_PTR)
#define UART3_D                                  UART_D_REG(UART3_BASE_PTR)
#define UART3_MA1                                UART_MA1_REG(UART3_BASE_PTR)
#define UART3_MA2                                UART_MA2_REG(UART3_BASE_PTR)
#define UART3_C4                                 UART_C4_REG(UART3_BASE_PTR)
#define UART3_C5                                 UART_C5_REG(UART3_BASE_PTR)
#define UART3_ED                                 UART_ED_REG(UART3_BASE_PTR)
#define UART3_MODEM                              UART_MODEM_REG(UART3_BASE_PTR)
#define UART3_IR                                 UART_IR_REG(UART3_BASE_PTR)
#define UART3_PFIFO                              UART_PFIFO_REG(UART3_BASE_PTR)
#define UART3_CFIFO                              UART_CFIFO_REG(UART3_BASE_PTR)
#define UART3_SFIFO                              UART_SFIFO_REG(UART3_BASE_PTR)
#define UART3_TWFIFO                             UART_TWFIFO_REG(UART3_BASE_PTR)
#define UART3_TCFIFO                             UART_TCFIFO_REG(UART3_BASE_PTR)
#define UART3_RWFIFO                             UART_RWFIFO_REG(UART3_BASE_PTR)
#define UART3_RCFIFO                             UART_RCFIFO_REG(UART3_BASE_PTR)

/* ----------------------------------------------------------------------------
   -- UART Register Masks
   ----------------------------------------------------------------------------
 */

/* BDH Bit Fields */
#define UART_BDH_SBR_MASK                        0x1F
#define UART_BDH_SBR_SHIFT                       0
#define UART_BDH_SBR(x)                          ((uint8_t) (((uint8_t)(x))<<UART_BDH_SBR_SHIFT))&UART_BDH_SBR_MASK)
#define UART_BDH_RXEDGIE_MASK                    0x40
#define UART_BDH_RXEDGIE_SHIFT                   6
#define UART_BDH_LBKDIE_MASK                     0x80
#define UART_BDH_LBKDIE_SHIFT                    7
/* BDL Bit Fields */
#define UART_BDL_SBR_MASK                        0xFF
#define UART_BDL_SBR_SHIFT                       0
#define UART_BDL_SBR(x)                          (((uint8_t)(((uint8_t)(x))<<UART_BDL_SBR_SHIFT))&UART_BDL_SBR_MASK)
/* C1 Bit Fields */
#define UART_C1_PT_MASK                          0x1
#define UART_C1_PT_SHIFT                         0
#define UART_C1_PE_MASK                          0x2
#define UART_C1_PE_SHIFT                         1
#define UART_C1_ILT_MASK                         0x4
#define UART_C1_ILT_SHIFT                        2
#define UART_C1_WAKE_MASK                        0x8
#define UART_C1_WAKE_SHIFT                       3
#define UART_C1_M_MASK                           0x10
#define UART_C1_M_SHIFT                          4
#define UART_C1_RSRC_MASK                        0x20
#define UART_C1_RSRC_SHIFT                       5
#define UART_C1_UARTSWAI_MASK                    0x40
#define UART_C1_UARTSWAI_SHIFT                   6
#define UART_C1_LOOPS_MASK                       0x80
#define UART_C1_LOOPS_SHIFT                      7
/* C2 Bit Fields */
#define UART_C2_SBK_MASK                         0x1
#define UART_C2_SBK_SHIFT                        0
#define UART_C2_RWU_MASK                         0x2
#define UART_C2_RWU_SHIFT                        1
#define UART_C2_RE_MASK                          0x4
#define UART_C2_RE_SHIFT                         2
#define UART_C2_TE_MASK                          0x8
#define UART_C2_TE_SHIFT                         3
#define UART_C2_ILIE_MASK                        0x10
#define UART_C2_ILIE_SHIFT                       4
#define UART_C2_RIE_MASK                         0x20
#define UART_C2_RIE_SHIFT                        5
#define UART_C2_TCIE_MASK                        0x40
#define UART_C2_TCIE_SHIFT                       6
#define UART_C2_TIE_MASK                         0x80
#define UART_C2_TIE_SHIFT                        7
/* S1 Bit Fields */
#define UART_S1_PF_MASK                          0x1
#define UART_S1_PF_SHIFT                         0
#define UART_S1_FE_MASK                          0x2
#define UART_S1_FE_SHIFT                         1
#define UART_S1_NF_MASK                          0x4
#define UART_S1_NF_SHIFT                         2
#define UART_S1_OR_MASK                          0x8
#define UART_S1_OR_SHIFT                         3
#define UART_S1_IDLE_MASK                        0x10
#define UART_S1_IDLE_SHIFT                       4
#define UART_S1_RDRF_MASK                        0x20
#define UART_S1_RDRF_SHIFT                       5
#define UART_S1_TC_MASK                          0x40
#define UART_S1_TC_SHIFT                         6
#define UART_S1_TDRE_MASK                        0x80
#define UART_S1_TDRE_SHIFT                       7
/* S2 Bit Fields */
#define UART_S2_RAF_MASK                         0x1
#define UART_S2_RAF_SHIFT                        0
#define UART_S2_LBKDE_MASK                       0x2
#define UART_S2_LBKDE_SHIFT                      1
#define UART_S2_BRK13_MASK                       0x4
#define UART_S2_BRK13_SHIFT                      2
#define UART_S2_RWUID_MASK                       0x8
#define UART_S2_RWUID_SHIFT                      3
#define UART_S2_RXINV_MASK                       0x10
#define UART_S2_RXINV_SHIFT                      4
#define UART_S2_MSBF_MASK                        0x20
#define UART_S2_MSBF_SHIFT                       5
#define UART_S2_RXEDGIF_MASK                     0x40
#define UART_S2_RXEDGIF_SHIFT                    6
#define UART_S2_LBKDIF_MASK                      0x80
#define UART_S2_LBKDIF_SHIFT                     7
/* C3 Bit Fields */
#define UART_C3_PEIE_MASK                        0x1
#define UART_C3_PEIE_SHIFT                       0
#define UART_C3_FEIE_MASK                        0x2
#define UART_C3_FEIE_SHIFT                       1
#define UART_C3_NEIE_MASK                        0x4
#define UART_C3_NEIE_SHIFT                       2
#define UART_C3_ORIE_MASK                        0x8
#define UART_C3_ORIE_SHIFT                       3
#define UART_C3_TXINV_MASK                       0x10
#define UART_C3_TXINV_SHIFT                      4
#define UART_C3_TXDIR_MASK                       0x20
#define UART_C3_TXDIR_SHIFT                      5
#define UART_C3_T8_MASK                          0x40
#define UART_C3_T8_SHIFT                         6
#define UART_C3_R8_MASK                          0x80
#define UART_C3_R8_SHIFT                         7
/* D Bit Fields */
#define UART_D_RT_MASK                           0xFF
#define UART_D_RT_SHIFT                          0
#define UART_D_RT(x)                             (((uint8_t)(((uint8_t)(x))<<UART_D_RT_SHIFT))&UART_D_RT_MASK)
/* MA1 Bit Fields */
#define UART_MA1_MA_MASK                         0xFF
#define UART_MA1_MA_SHIFT                        0
#define UART_MA1_MA(x)                           (((uint8_t)(((uint8_t)(x))<<UART_MA1_MA_SHIFT))&UART_MA1_MA_MASK)
/* MA2 Bit Fields */
#define UART_MA2_MA_MASK                         0xFF
#define UART_MA2_MA_SHIFT                        0
#define UART_MA2_MA(x)                           (((uint8_t)(((uint8_t)(x))<<UART_MA2_MA_SHIFT))&UART_MA2_MA_MASK)
/* C4 Bit Fields */
#define UART_C4_BRFA_MASK                        0x1F
#define UART_C4_BRFA_SHIFT                       0
#define UART_C4_BRFA(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C4_BRFA_SHIFT))&UART_C4_BRFA_MASK)
#define UART_C4_M10_MASK                         0x20
#define UART_C4_M10_SHIFT                        5
#define UART_C4_MAEN2_MASK                       0x40
#define UART_C4_MAEN2_SHIFT                      6
#define UART_C4_MAEN1_MASK                       0x80
#define UART_C4_MAEN1_SHIFT                      7
/* C5 Bit Fields */
#define UART_C5_RDMAS_MASK                       0x20
#define UART_C5_RDMAS_SHIFT                      5
#define UART_C5_TDMAS_MASK                       0x80
#define UART_C5_TDMAS_SHIFT                      7
/* ED Bit Fields */
#define UART_ED_PARITYE_MASK                     0x40
#define UART_ED_PARITYE_SHIFT                    6
#define UART_ED_NOISY_MASK                       0x80
#define UART_ED_NOISY_SHIFT                      7
/* MODEM Bit Fields */
#define UART_MODEM_TXCTSE_MASK                   0x1
#define UART_MODEM_TXCTSE_SHIFT                  0
#define UART_MODEM_TXRTSE_MASK                   0x2
#define UART_MODEM_TXRTSE_SHIFT                  1
#define UART_MODEM_TXRTSPOL_MASK                 0x4
#define UART_MODEM_TXRTSPOL_SHIFT                2
#define UART_MODEM_RXRTSE_MASK                   0x8
#define UART_MODEM_RXRTSE_SHIFT                  3
/* IR Bit Fields */
#define UART_IR_TNP_MASK                         0x3
#define UART_IR_TNP_SHIFT                        0
#define UART_IR_TNP(x)                           (((uint8_t)(((uint8_t)(x))<<UART_IR_TNP_SHIFT))&UART_IR_TNP_MASK)
#define UART_IR_IREN_MASK                        0x4
#define UART_IR_IREN_SHIFT                       2
/* PFIFO Bit Fields */
#define UART_PFIFO_RXFIFOSIZE_MASK               0x7
#define UART_PFIFO_RXFIFOSIZE_SHIFT              0
#define UART_PFIFO_RXFIFOSIZE(x)                 (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_RXFIFOSIZE_SHIFT))&UART_PFIFO_RXFIFOSIZE_MASK)
#define UART_PFIFO_RXFE_MASK                     0x8
#define UART_PFIFO_RXFE_SHIFT                    3
#define UART_PFIFO_TXFIFOSIZE_MASK               0x70
#define UART_PFIFO_TXFIFOSIZE_SHIFT              4
#define UART_PFIFO_TXFIFOSIZE(x)                 (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_TXFIFOSIZE_SHIFT))&UART_PFIFO_TXFIFOSIZE_MASK)
#define UART_PFIFO_TXFE_MASK                     0x80
#define UART_PFIFO_TXFE_SHIFT                    7
/* CFIFO Bit Fields */
#define UART_CFIFO_RXUFE_MASK                    0x1
#define UART_CFIFO_RXUFE_SHIFT                   0
#define UART_CFIFO_TXOFE_MASK                    0x2
#define UART_CFIFO_TXOFE_SHIFT                   1
#define UART_CFIFO_RXFLUSH_MASK                  0x40
#define UART_CFIFO_RXFLUSH_SHIFT                 6
#define UART_CFIFO_TXFLUSH_MASK                  0x80
#define UART_CFIFO_TXFLUSH_SHIFT                 7
/* SFIFO Bit Fields */
#define UART_SFIFO_RXUF_MASK                     0x1
#define UART_SFIFO_RXUF_SHIFT                    0
#define UART_SFIFO_TXOF_MASK                     0x2
#define UART_SFIFO_TXOF_SHIFT                    1
#define UART_SFIFO_RXEMPT_MASK                   0x40
#define UART_SFIFO_RXEMPT_SHIFT                  6
#define UART_SFIFO_TXEMPT_MASK                   0x80
#define UART_SFIFO_TXEMPT_SHIFT                  7
/* TWFIFO Bit Fields */
#define UART_TWFIFO_TXWATER_MASK                 0xFF
#define UART_TWFIFO_TXWATER_SHIFT                0
#define UART_TWFIFO_TXWATER(x)                   (((uint8_t)(((uint8_t)(x))<<UART_TWFIFO_TXWATER_SHIFT))&UART_TWFIFO_TXWATER_MASK)
/* TCFIFO Bit Fields */
#define UART_TCFIFO_TXCOUNT_MASK                 0xFF
#define UART_TCFIFO_TXCOUNT_SHIFT                0
#define UART_TCFIFO_TXCOUNT(x)                   (((uint8_t)(((uint8_t)(x))<<UART_TCFIFO_TXCOUNT_SHIFT))&UART_TCFIFO_TXCOUNT_MASK)
/* RWFIFO Bit Fields */
#define UART_RWFIFO_RXWATER_MASK                 0xFF
#define UART_RWFIFO_RXWATER_SHIFT                0
#define UART_RWFIFO_RXWATER(x)                   (((uint8_t)(((uint8_t)(x))<<UART_RWFIFO_RXWATER_SHIFT))&UART_RWFIFO_RXWATER_MASK)
/* RCFIFO Bit Fields */
#define UART_RCFIFO_RXCOUNT_MASK                 0xFF
#define UART_RCFIFO_RXCOUNT_SHIFT                0
#define UART_RCFIFO_RXCOUNT(x)                   (((uint8_t)(((uint8_t)(x))<<UART_RCFIFO_RXCOUNT_SHIFT))&UART_RCFIFO_RXCOUNT_MASK)

/*
 * Copyright (c) 2018 Karsten Koenig
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _MCP2517_H_
#define _MCP2517_H_

#include <drivers/gpio.h>
#include <drivers/can.h>



#define MCP251XFD_TX_FIFO_INDEX 0
#define MCP251XFD_RX_FIFO_INDEX 1
#define MCP2517FD_MAX_MSG_SIZE 76
#define MCP2517FD_TXEVENT_FIFO_SIZE 0xC
#define MCP2517FD_TX_FIFO_SIZE 0xC
#define MCP2517FD_RX_FIFO_SIZE 0x1F
#define MCP2517FD_NUM_SPI_RETRIES 10

struct mcp251xfd_tx_cb {
	struct k_sem sem;
	uint32_t tx_fifo_addr;
	uint8_t tx_seq;
	can_tx_callback_t cb;
	void *cb_arg;
};

struct mcp251xfd_data {
	/* interrupt data */
	struct gpio_callback int_gpio_cb;
	struct k_thread int_thread;
	k_thread_stack_t *int_thread_stack;
	struct k_sem int_sem;

	/* tx data */
	struct k_sem tx_sem;
	struct mcp251xfd_tx_cb tx_cb[MCP2517FD_TX_FIFO_SIZE];
	uint8_t tx_seq;

	/* filter data */
	uint32_t filter_usage;
	can_rx_callback_t rx_cb[CONFIG_CAN_MAX_FILTER];
	void *cb_arg[CONFIG_CAN_MAX_FILTER];
	struct zcan_filter filter[CONFIG_CAN_MAX_FILTER];
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;

	/* general data */
	struct k_mutex mutex;
	enum can_state old_state;
	uint8_t sjw;
	uint32_t osc_freq;
	uint8_t mcp2517_mode;
};

struct mcp2517frame {
	uint32_t id;

	struct {
		uint32_t DLC : 4;
		uint32_t IDE : 1;
		uint32_t RTR : 1;
		uint32_t BRS : 1;
		uint32_t FDF : 1;
		uint32_t ESI : 1;
		uint32_t FILHIT0 : 7;
		uint32_t unimplemented1 : 16;
	} ctrl;
	uint8_t data[8];
} __packed;

typedef enum {
    CAN_PLSIZE_8,
    CAN_PLSIZE_12,
    CAN_PLSIZE_16,
    CAN_PLSIZE_20,
    CAN_PLSIZE_24,
    CAN_PLSIZE_32,
    CAN_PLSIZE_48,
    CAN_PLSIZE_64
} can_plsize;

struct mcp251xfd_config {
	/* spi configuration */
	struct spi_dt_spec bus;

	/* interrupt configuration */
	struct gpio_dt_spec int_gpio;
	size_t int_thread_stack_size;
	int int_thread_priority;

	/* CAN timing */
	uint8_t tq_sjw;
	uint8_t tq_prop;
	uint8_t tq_bs1;
	uint8_t tq_bs2;
	uint8_t tq_sjw_data;
	uint8_t tq_prop_data;
	uint8_t tq_bs1_data;
	uint8_t tq_bs2_data;
	
	uint32_t bus_speed;
	uint32_t bus_speed_data;
	uint32_t osc_freq;
	uint16_t sample_point;
	bool pll_enable;
	const struct device *clock;

	/* CAN transceiver */
	const struct device *phy;
	uint32_t max_bitrate;
	can_plsize plsize;
};

#define MCP251XFD_ADDR_CON                  0x00
#define MCP251XFD_ADDR_NBTCFG               0x04
#define MCP251XFD_ADDR_DBTCFG               0x08
#define MCP251XFD_ADDR_TDC                  0x0c
#define MCP251XFD_ADDR_TBC                  0x10
#define MCP251XFD_ADDR_TSCON                0x14
#define MCP251XFD_ADDR_VEC                  0x18
#define MCP251XFD_ADDR_INT                  0x1c
#define MCP251XFD_ADDR_RXIF                 0x20
#define MCP251XFD_ADDR_TXIF                 0x24
#define MCP251XFD_ADDR_RXOVIF               0x28
#define MCP251XFD_ADDR_TXATIF               0x2c
#define MCP251XFD_ADDR_TXREQ                0x30
#define MCP251XFD_ADDR_TREC                 0x34
#define MCP251XFD_ADDR_BDIAG0               0x38
#define MCP251XFD_ADDR_BDIAG1               0x3c
#define MCP251XFD_ADDR_TEFCON               0x40
#define MCP251XFD_ADDR_TEFSTA               0x44
#define MCP251XFD_ADDR_TEFUA                0x48
#define MCP251XFD_ADDR_TXQCON               0x50
#define MCP251XFD_ADDR_TXQSTA               0x54
#define MCP251XFD_ADDR_TXQUA                0x58
#define MCP251XFD_ADDR_FIFOCON(x)           (0x5C + 0xc * (x))
#define MCP251XFD_ADDR_FIFOSTA(x)           (0x60 + 0xc * (x))
#define MCP251XFD_ADDR_FIFOUA(x)            (0x64 + 0xc * (x))
#define MCP251XFD_ADDR_FLTCON(x)            (0x1d0 + 0x4 * (x))
#define MCP251XFD_ADDR_FLTOBJ(x)            (0x1f0 + 0x8 * (x))
#define MCP251XFD_ADDR_FLTMASK(x)           (0x1f4 + 0x8 * (x))
#define MCP251XFD_ADDR_OSC                  0xe00
#define MCP251XFD_ADDR_IOCON                0xe04
#define MCP251XFD_ADDR_CRC                  0xe08
#define MCP251XFD_ADDR_ECCCON               0xe0c
#define MCP251XFD_ADDR_ECCSTAT              0xe10

#define MCP251XFD_RAM_START 0x400
#define MCP251XFD_RAM_SIZE 0x800

#define MCP251XFD_CON_TXBWS_MASK            GENMASK(31, 28)
#define MCP251XFD_CON_ABAT                  BIT(27)
#define MCP251XFD_CON_REQOP_MASK            GENMASK(26, 24)
#define MCP251XFD_CON_MODE_MIXED            0
#define MCP251XFD_CON_MODE_SLEEP            1
#define MCP251XFD_CON_MODE_INT_LOOPBACK     2
#define MCP251XFD_CON_MODE_LISTENONLY       3
#define MCP251XFD_CON_MODE_CONFIG           4
#define MCP251XFD_CON_MODE_EXT_LOOPBACK     5
#define MCP251XFD_CON_MODE_CAN2_0           6
#define MCP251XFD_CON_MODE_RESTRICTED       7
#define MCP251XFD_CON_OPMOD_MASK            GENMASK(23, 21)
#define MCP251XFD_CON_TXQEN                 BIT(20)
#define MCP251XFD_CON_STEF                  BIT(19)
#define MCP251XFD_CON_SERR2LOM              BIT(18)
#define MCP251XFD_CON_ESIGM                 BIT(17)
#define MCP251XFD_CON_RTXAT                 BIT(16)
#define MCP251XFD_CON_BRSDIS                BIT(12)
#define MCP251XFD_CON_BUSY                  BIT(11)
#define MCP251XFD_CON_WFT_MASK              GENMASK(10, 9)
#define MCP251XFD_CON_WFT_T00FILTER         0x0
#define MCP251XFD_CON_WFT_T01FILTER         0x1
#define MCP251XFD_CON_WFT_T10FILTER         0x2
#define MCP251XFD_CON_WFT_T11FILTER         0x3
#define MCP251XFD_CON_WAKFIL                BIT(8)
#define MCP251XFD_CON_PXEDIS                BIT(6)
#define MCP251XFD_CON_ISOCRCEN              BIT(5)
#define MCP251XFD_CON_DNCNT_MASK            GENMASK(4, 0)

#define MCP251XFD_NBTCFG_BRP_MASK           GENMASK(31, 24)
#define MCP251XFD_NBTCFG_TSEG1_MASK         GENMASK(23, 16)
#define MCP251XFD_NBTCFG_TSEG2_MASK         GENMASK(14, 8)
#define MCP251XFD_NBTCFG_SJW_MASK           GENMASK(6, 0)

#define MCP251XFD_DBTCFG_BRP_MASK           GENMASK(31, 24)
#define MCP251XFD_DBTCFG_TSEG1_MASK         GENMASK(20, 16)
#define MCP251XFD_DBTCFG_TSEG2_MASK         GENMASK(11, 8)
#define MCP251XFD_DBTCFG_SJW_MASK           GENMASK(3, 0)

#define MCP251XFD_TDC_EDGFLTEN              BIT(25)
#define MCP251XFD_TDC_SID11EN               BIT(24)
#define MCP251XFD_TDC_TDCMOD_MASK           GENMASK(17, 16)
#define MCP251XFD_TDC_TDCMOD_AUTO           2
#define MCP251XFD_TDC_TDCMOD_MANUAL         1
#define MCP251XFD_TDC_TDCMOD_DISABLED       0
#define MCP251XFD_TDC_TDCO_MASK             GENMASK(14, 8)
#define MCP251XFD_TDC_TDCV_MASK             GENMASK(5, 0)

#define MCP251XFD_TSCON_TSRES               BIT(18)
#define MCP251XFD_TSCON_TSEOF               BIT(17)
#define MCP251XFD_TSCON_TBCEN               BIT(16)
#define MCP251XFD_TSCON_TBCPRE_MASK         GENMASK(9, 0)


#define MCP251XFD_VEC_RXCODE_MASK           GENMASK(30, 24)
#define MCP251XFD_VEC_TXCODE_MASK           GENMASK(22, 16)
#define MCP251XFD_VEC_FILHIT_MASK           GENMASK(12, 8)
#define MCP251XFD_VEC_ICODE_MASK            GENMASK(6, 0)

#define MCP251XFD_INT_IF_MASK               GENMASK(15, 0)
#define MCP251XFD_INT_IE_MASK               GENMASK(31, 16)
#define MCP251XFD_INT_IVMIE                 BIT(31)
#define MCP251XFD_INT_WAKIE                 BIT(30)
#define MCP251XFD_INT_CERRIE                BIT(29)
#define MCP251XFD_INT_SERRIE                BIT(28)
#define MCP251XFD_INT_RXOVIE                BIT(27)
#define MCP251XFD_INT_TXATIE                BIT(26)
#define MCP251XFD_INT_SPICRCIE              BIT(25)
#define MCP251XFD_INT_ECCIE                 BIT(24)
#define MCP251XFD_INT_TEFIE                 BIT(20)
#define MCP251XFD_INT_MODIE                 BIT(19)
#define MCP251XFD_INT_TBCIE                 BIT(18)
#define MCP251XFD_INT_RXIE                  BIT(17)
#define MCP251XFD_INT_TXIE                  BIT(16)
#define MCP251XFD_INT_IVMIF                 BIT(15)
#define MCP251XFD_INT_WAKIF                 BIT(14)
#define MCP251XFD_INT_CERRIF                BIT(13)
#define MCP251XFD_INT_SERRIF                BIT(12)
#define MCP251XFD_INT_RXOVIF                BIT(11)
#define MCP251XFD_INT_TXATIF                BIT(10)
#define MCP251XFD_INT_SPICRCIF              BIT(9)
#define MCP251XFD_INT_ECCIF                 BIT(8)
#define MCP251XFD_INT_TEFIF                 BIT(4)
#define MCP251XFD_INT_MODIF                 BIT(3)
#define MCP251XFD_INT_TBCIF                 BIT(2)
#define MCP251XFD_INT_RXIF                  BIT(1)
#define MCP251XFD_INT_TXIF                  BIT(0)

#define MCP251XFD_INT_IF_CLEARABLE_MASK \
	(MCP251XFD_INT_IVMIF | MCP251XFD_INT_WAKIF | \
	 MCP251XFD_INT_CERRIF |  MCP251XFD_INT_SERRIF | \
	 MCP251XFD_INT_MODIF)

#define MCP251XFD_TREC_TXBO                 BIT(21)
#define MCP251XFD_TREC_TXBP                 BIT(20)
#define MCP251XFD_TREC_RXBP                 BIT(19)
#define MCP251XFD_TREC_TXWARN               BIT(18)
#define MCP251XFD_TREC_RXWARN               BIT(17)
#define MCP251XFD_TREC_EWARN                BIT(16)
#define MCP251XFD_TREC_TEC_MASK             GENMASK(15, 8)
#define MCP251XFD_TREC_REC_MASK             GENMASK(7, 0)

#define MCP251XFD_BDIAG0_DTERRCNT_MASK      GENMASK(31, 24)
#define MCP251XFD_BDIAG0_DRERRCNT_MASK      GENMASK(23, 16)
#define MCP251XFD_BDIAG0_NTERRCNT_MASK      GENMASK(15, 8)
#define MCP251XFD_BDIAG0_NRERRCNT_MASK      GENMASK(7, 0)


#define MCP251XFD_BDIAG1_DLCMM              BIT(31)
#define MCP251XFD_BDIAG1_ESI                BIT(30)
#define MCP251XFD_BDIAG1_DCRCERR            BIT(29)
#define MCP251XFD_BDIAG1_DSTUFERR           BIT(28)
#define MCP251XFD_BDIAG1_DFORMERR           BIT(27)
#define MCP251XFD_BDIAG1_DBIT1ERR           BIT(25)
#define MCP251XFD_BDIAG1_DBIT0ERR           BIT(24)
#define MCP251XFD_BDIAG1_TXBOERR            BIT(23)
#define MCP251XFD_BDIAG1_NCRCERR            BIT(21)
#define MCP251XFD_BDIAG1_NSTUFERR           BIT(20)
#define MCP251XFD_BDIAG1_NFORMERR           BIT(19)
#define MCP251XFD_BDIAG1_NACKERR            BIT(18)
#define MCP251XFD_BDIAG1_NBIT1ERR           BIT(17)
#define MCP251XFD_BDIAG1_NBIT0ERR           BIT(16)
#define MCP251XFD_BDIAG1_BERR_MASK \
	(MCP251XFD_BDIAG1_DLCMM | MCP251XFD_BDIAG1_ESI | \
	 MCP251XFD_BDIAG1_DCRCERR | MCP251XFD_BDIAG1_DSTUFERR | \
	 MCP251XFD_BDIAG1_DFORMERR | MCP251XFD_BDIAG1_DBIT1ERR | \
	 MCP251XFD_BDIAG1_DBIT0ERR | MCP251XFD_BDIAG1_TXBOERR | \
	 MCP251XFD_BDIAG1_NCRCERR | MCP251XFD_BDIAG1_NSTUFERR | \
	 MCP251XFD_BDIAG1_NFORMERR | MCP251XFD_BDIAG1_NACKERR | \
	 MCP251XFD_BDIAG1_NBIT1ERR | MCP251XFD_BDIAG1_NBIT0ERR)
#define MCP251XFD_BDIAG1_EFMSGCNT_MASK      GENMASK(15, 0)


#define MCP251XFD_TEFCON_FSIZE_MASK         GENMASK(28, 24)
#define MCP251XFD_TEFCON_FRESET             BIT(10)
#define MCP251XFD_TEFCON_UINC               BIT(8)
#define MCP251XFD_TEFCON_TEFTSEN            BIT(5)
#define MCP251XFD_TEFCON_TEFOVIE            BIT(3)
#define MCP251XFD_TEFCON_TEFFIE             BIT(2)
#define MCP251XFD_TEFCON_TEFHIE             BIT(1)
#define MCP251XFD_TEFCON_TEFNEIE            BIT(0)

#define MCP251XFD_TEFSTA_TEFOVIF            BIT(3)
#define MCP251XFD_TEFSTA_TEFFIF             BIT(2)
#define MCP251XFD_TEFSTA_TEFHIF             BIT(1)
#define MCP251XFD_TEFSTA_TEFNEIF            BIT(0)

#define MCP251XFD_TXQCON_PLSIZE_MASK        GENMASK(31, 29)
#define MCP251XFD_TXQCON_PLSIZE_8           0
#define MCP251XFD_TXQCON_PLSIZE_12          1
#define MCP251XFD_TXQCON_PLSIZE_16          2
#define MCP251XFD_TXQCON_PLSIZE_20          3
#define MCP251XFD_TXQCON_PLSIZE_24          4
#define MCP251XFD_TXQCON_PLSIZE_32          5
#define MCP251XFD_TXQCON_PLSIZE_48          6
#define MCP251XFD_TXQCON_PLSIZE_64          7
#define MCP251XFD_TXQCON_FSIZE_MASK         GENMASK(28, 24)
#define MCP251XFD_TXQCON_TXAT_UNLIMITED     3
#define MCP251XFD_TXQCON_TXAT_THREE_SHOT    1
#define MCP251XFD_TXQCON_TXAT_ONE_SHOT      0
#define MCP251XFD_TXQCON_TXAT_MASK          GENMASK(22, 21)
#define MCP251XFD_TXQCON_TXPRI_MASK         GENMASK(20, 16)
#define MCP251XFD_TXQCON_FRESET             BIT(10)
#define MCP251XFD_TXQCON_TXREQ              BIT(9)
#define MCP251XFD_TXQCON_UINC               BIT(8)
#define MCP251XFD_TXQCON_TXEN               BIT(7)
#define MCP251XFD_TXQCON_TXATIE             BIT(4)
#define MCP251XFD_TXQCON_TXQEIE             BIT(2)
#define MCP251XFD_TXQCON_TXQNIE             BIT(0)

#define MCP251XFD_TXQSTA_TXQCI_MASK         GENMASK(12, 8)
#define MCP251XFD_TXQSTA_TXABT              BIT(7)
#define MCP251XFD_TXQSTA_TXLARB             BIT(6)
#define MCP251XFD_TXQSTA_TXERR              BIT(5)
#define MCP251XFD_TXQSTA_TXATIF             BIT(4)
#define MCP251XFD_TXQSTA_TXQEIF             BIT(2)
#define MCP251XFD_TXQSTA_TXQNIF             BIT(0)

#define MCP251XFD_FIFOCON_PLSIZE_MASK       GENMASK(31, 29)
#define MCP251XFD_FIFOCON_PLSIZE_8          0
#define MCP251XFD_FIFOCON_PLSIZE_12         1
#define MCP251XFD_FIFOCON_PLSIZE_16         2
#define MCP251XFD_FIFOCON_PLSIZE_20         3
#define MCP251XFD_FIFOCON_PLSIZE_24         4
#define MCP251XFD_FIFOCON_PLSIZE_32         5
#define MCP251XFD_FIFOCON_PLSIZE_48         6
#define MCP251XFD_FIFOCON_PLSIZE_64         7
#define MCP251XFD_FIFOCON_FSIZE_MASK        GENMASK(28, 24)
#define MCP251XFD_FIFOCON_TXAT_MASK         GENMASK(22, 21)
#define MCP251XFD_FIFOCON_TXAT_ONE_SHOT     0
#define MCP251XFD_FIFOCON_TXAT_THREE_SHOT   1
#define MCP251XFD_FIFOCON_TXAT_UNLIMITED    3
#define MCP251XFD_FIFOCON_TXPRI_MASK        GENMASK(20, 16)
#define MCP251XFD_FIFOCON_FRESET            BIT(10)
#define MCP251XFD_FIFOCON_TXREQ             BIT(9)
#define MCP251XFD_FIFOCON_UINC              BIT(8)
#define MCP251XFD_FIFOCON_TXEN              BIT(7)
#define MCP251XFD_FIFOCON_RTREN             BIT(6)
#define MCP251XFD_FIFOCON_RXTSEN            BIT(5)
#define MCP251XFD_FIFOCON_TXATIE            BIT(4)
#define MCP251XFD_FIFOCON_RXOVIE            BIT(3)
#define MCP251XFD_FIFOCON_TFERFFIE          BIT(2)
#define MCP251XFD_FIFOCON_TFHRFHIE          BIT(1)
#define MCP251XFD_FIFOCON_TFNRFNIE          BIT(0)

#define MCP251XFD_FIFOSTA_FIFOCI_MASK       GENMASK(12, 8)
#define MCP251XFD_FIFOSTA_TXABT             BIT(7)
#define MCP251XFD_FIFOSTA_TXLARB            BIT(6)
#define MCP251XFD_FIFOSTA_TXERR             BIT(5)
#define MCP251XFD_FIFOSTA_TXATIF            BIT(4)
#define MCP251XFD_FIFOSTA_RXOVIF            BIT(3)
#define MCP251XFD_FIFOSTA_TFERFFIF          BIT(2)
#define MCP251XFD_FIFOSTA_TFHRFHIF          BIT(1)
#define MCP251XFD_FIFOSTA_TFNRFNIF          BIT(0)


#define MCP251XFD_FLTCON_FLTEN3             BIT(31)
#define MCP251XFD_FLTCON_F3BP_MASK          GENMASK(28, 24)
#define MCP251XFD_FLTCON_FLTEN2             BIT(23)
#define MCP251XFD_FLTCON_F2BP_MASK          GENMASK(20, 16)
#define MCP251XFD_FLTCON_FLTEN1             BIT(15)
#define MCP251XFD_FLTCON_F1BP_MASK          GENMASK(12, 8)
#define MCP251XFD_FLTCON_FLTEN0             BIT(7)
#define MCP251XFD_FLTCON_F0BP_MASK          GENMASK(4, 0)
#define MCP251XFD_FLTCON_FLTEN(x)           (BIT(7) << 8 * ((x) & 0x3))
#define MCP251XFD_FLTCON_FLT_MASK(x)        (GENMASK(7, 0) << (8 * ((x) & 0x3)))
#define MCP251XFD_FLTCON_FBP(x, fifo)       ((fifo) << 8 * ((x) & 0x3))


#define MCP251XFD_FLTOBJ_EXIDE              BIT(30)
#define MCP251XFD_FLTOBJ_SID11              BIT(29)
#define MCP251XFD_FLTOBJ_EID_MASK           GENMASK(28, 11)
#define MCP251XFD_FLTOBJ_SID_MASK           GENMASK(10, 0)

#define MCP251XFD_MASK_MIDE                 BIT(30)
#define MCP251XFD_MASK_MSID11               BIT(29)
#define MCP251XFD_MASK_MEID_MASK            GENMASK(28, 11)
#define MCP251XFD_MASK_MSID_MASK            GENMASK(10, 0)

#define MCP251XFD_OBJ_ID_SID11              BIT(29)
#define MCP251XFD_OBJ_ID_EID_MASK           GENMASK(28, 11)
#define MCP251XFD_OBJ_ID_SID_MASK           GENMASK(10, 0)
#define MCP251XFD_OBJ_FLAGS_SEQ_MCP2518FD_MASK GENMASK(31, 9)
#define MCP251XFD_OBJ_FLAGS_SEQ_MCP2517FD_MASK GENMASK(15, 9)
#define MCP251XFD_OBJ_FLAGS_SEQ_MASK        MCP251XFD_OBJ_FLAGS_SEQ_MCP2518FD_MASK
#define MCP251XFD_OBJ_FLAGS_ESI             BIT(8)
#define MCP251XFD_OBJ_FLAGS_FDF             BIT(7)
#define MCP251XFD_OBJ_FLAGS_BRS             BIT(6)
#define MCP251XFD_OBJ_FLAGS_RTR             BIT(5)
#define MCP251XFD_OBJ_FLAGS_IDE             BIT(4)
#define MCP251XFD_OBJ_FLAGS_DLC             GENMASK(3, 0)

#define MCP251XFD_FRAME_EFF_SID_MASK        GENMASK(28, 18)
#define MCP251XFD_FRAME_EFF_EID_MASK        GENMASK(17, 0)

#define MCP251XFD_OSC_SCLKRDY               BIT(12)
#define MCP251XFD_OSC_OSCRDY                BIT(10)
#define MCP251XFD_OSC_PLLRDY                BIT(8)
#define MCP251XFD_OSC_CLKODIV_10            3
#define MCP251XFD_OSC_CLKODIV_4             2
#define MCP251XFD_OSC_CLKODIV_2             1
#define MCP251XFD_OSC_CLKODIV_1             0
#define MCP251XFD_OSC_CLKODIV_MASK          GENMASK(6, 5)
#define MCP251XFD_OSC_SCLKDIV               BIT(4)
#define MCP251XFD_OSC_OSCDIS                BIT(2)
#define MCP251XFD_OSC_PLLEN                 BIT(0)

#define MCP251XFD_IOCON_INTOD               BIT(30)
#define MCP251XFD_IOCON_SOF                 BIT(29)
#define MCP251XFD_IOCON_TXCANOD             BIT(28)
#define MCP251XFD_IOCON_PM1                 BIT(25)
#define MCP251XFD_IOCON_PM0                 BIT(24)
#define MCP251XFD_IOCON_GPIO1               BIT(17)
#define MCP251XFD_IOCON_GPIO0               BIT(16)
#define MCP251XFD_IOCON_LAT1                BIT(9)
#define MCP251XFD_IOCON_LAT0                BIT(8)
#define MCP251XFD_IOCON_XSTBYEN             BIT(6)
#define MCP251XFD_IOCON_TRIS1               BIT(1)
#define MCP251XFD_IOCON_TRIS0               BIT(0)

#define MCP251XFD_CRC_FERRIE                BIT(25)
#define MCP251XFD_CRC_CRCERRIE              BIT(24)
#define MCP251XFD_CRC_FERRIF                BIT(17)
#define MCP251XFD_CRC_CRCERRIF              BIT(16)
#define MCP251XFD_CRC_IF_MASK               GENMASK(17, 16)
#define MCP251XFD_CRC_MASK                  GENMASK(15, 0)

#define MCP251XFD_ECCCON_PARITY_MASK        GENMASK(14, 8)
#define MCP251XFD_ECCCON_DEDIE              BIT(2)
#define MCP251XFD_ECCCON_SECIE              BIT(1)
#define MCP251XFD_ECCCON_ECCEN              BIT(0)

#define MCP251XFD_ECCSTAT_ERRADDR_MASK      GENMASK(27, 16)
#define MCP251XFD_ECCSTAT_IF_MASK           GENMASK(2, 1)
#define MCP251XFD_ECCSTAT_DEDIF             BIT(2)
#define MCP251XFD_ECCSTAT_SECIF             BIT(1)

#define MCP251XFD_DEVID_ID_MASK             GENMASK(7, 4)
#define MCP251XFD_DEVID_REV_MASK            GENMASK(3, 0)

/* SPI commands */
#define MCP251XFD_SPI_INSTRUCTION_RESET 	0x0
#define MCP251XFD_SPI_INSTRUCTION_WRITE 	0x2
#define MCP251XFD_SPI_INSTRUCTION_READ 		0x3
#define MCP251XFD_SPI_INSTRUCTION_WRITE_CRC 0xa
#define MCP251XFD_SPI_INSTRUCTION_READ_CRC 	0xb
#define MCP251XFD_SPI_INSTRUCTION_WRITE_CRC_SAFE 0xc
#define MCP251XFD_SPI_ADDRESS_MASK 			GENMASK(11, 0)

#endif /* __MCP251XFD_H */
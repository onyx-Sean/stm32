/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) STMicroelectronics 2015 - All Rights Reserved
 * Author: Ludovic Barre <ludovic.barre@st.com> for STMicroelectronics.
 */
#define SDMMC_POWER		0x000
#define POWERCTRL_SHIFT			0
#define POWERCTRL_MASK			GENMASK(1, 0)
#define POWERCTRL_OFF			0x00
#define POWERCTRL_CYC			0x02
#define POWERCTRL_ON			0x03
#define POWER_VSWITCH			BIT(2)
#define POWER_VSWITCHEN			BIT(3)
#define POWER_DIRPOL			BIT(4)

#define SDMMC_CLKCR		0x004
#define CLKCR_CLKDIV_SHIFT		0
#define CLKCR_CLKDIV_MASK		GENMASK(9, 0)
#define CLKCR_CLKDIV_MAX		CLKCR_CLKDIV_MASK
#define CLKCR_PWRSAV			BIT(12)
#define CLKCR_WIDBUS_4			BIT(14)
#define CLKCR_WIDBUS_8			BIT(15)
#define CLKCR_NEGEDGE			BIT(16)
#define CLKCR_HWFC_EN			BIT(17)
#define CLKCR_DDR			BIT(18)
#define CLKCR_BUSSPEED			BIT(19)
#define CLKCR_SELCLKRX_SHIFT		20
#define CLKCR_SELCLKRX_MASK		GENMASK(21, 20)
#define CLKCR_SELCLKRX_CK		(0 << 20)
#define CLKCR_SELCLKRX_CKIN		(1 << 20)
#define CLKCR_SELCLKRX_FBCK		(2 << 20)

#define SDMMC_ARGR		0x008

#define SDMMC_CMDR		0x00C
#define CMDR_CMDTRANS			BIT(6)
#define CMDR_CMDSTOP			BIT(7)
#define CMDR_WAITRESP_SHIFT		8
#define CMDR_WAITRESP_MASK		GENMASK(9, 8)
#define CMDR_WAITRESP_NORSP		(0 << 8)
#define CMDR_WAITRESP_SRSP_CRC		(1 << 8)
#define CMDR_WAITRESP_SRSP		(2 << 8)
#define CMDR_WAITRESP_LRSP_CRC		(3 << 8)
#define CMDR_WAITINT			BIT(10)
#define CMDR_WAITPEND			BIT(11)
#define CMDR_CPSMEM			BIT(12)
#define CMDR_DTHOLD			BIT(13)
#define CMDR_BOOTMODE			BIT(14)
#define CMDR_BOOTEN			BIT(15)
#define CMDR_CMDSUSPEND			BIT(16)

#define SDMMC_RESPCMDR		0x010
#define SDMMC_RESP1R		0x014
#define SDMMC_RESP2R		0x018
#define SDMMC_RESP3R		0x01c
#define SDMMC_RESP4R		0x020

#define SDMMC_DTIMER		0x024

#define SDMMC_DLENR		0x028
#define DLENR_DATALENGHT_SHIFT		0
#define DLENR_DATALENGHT_MASK		GENMASK(24, 0)
#define DLENR_DATALENGHT_MAX		DLENR_DATALENGHT_MASK

#define SDMMC_DCTRLR		0x02C
#define DCTRLR_DTEN			BIT(0)
#define DCTRLR_DTDIR			BIT(1)
#define DCTRLR_DTMODE_SHIFT		2
#define DCTRLR_DTMODE_MASK		GENMASK(3, 2)
#define DCTRLR_DTMODE_BLOCK		(0 << 2)
#define DCTRLR_DTMODE_SDIO		(1 << 2)
#define DCTRLR_DTMODE_MMC		(2 << 2)
#define DCTRLR_DTMODE_BLOCK_STOP	(3 << 2)
#define DCTRLR_DBLOCKSIZE_SHIFT		4
#define DCTRLR_DBLOCKSIZE_MASK		GENMASK(7, 4)
#define DCTRLR_DBLOCKSIZE_MAX		14
#define DCTRLR_RWSTART			BIT(8)
#define DCTRLR_RWSTOP			BIT(9)
#define DCTRLR_RWMOD			BIT(10)
#define DCTRLR_SDIOEN			BIT(11)
#define DCTRLR_BOOTACKEN		BIT(12)
#define DCTRLR_FIFORST			BIT(13)

#define SDMMC_DCNTR		0x030

#define SDMMC_STAR		0x034
#define STAR_CCRCFAIL			BIT(0)
#define STAR_DCRCFAIL			BIT(1)
#define STAR_CTIMEOUT			BIT(2)
#define STAR_DTIMEOUT			BIT(3)
#define STAR_TXUNDERR			BIT(4)
#define STAR_RXOVERR			BIT(5)
#define STAR_CMDREND			BIT(6)
#define STAR_CMDSENT			BIT(7)
#define STAR_DATAEND			BIT(8)
#define STAR_DHOLD			BIT(9)
#define STAR_DBCKEND			BIT(10)
#define STAR_DABORT			BIT(11)
#define STAR_DPSMACT			BIT(12)
#define STAR_CPSMACT			BIT(13)
#define STAR_TXFIFOHE			BIT(14)
#define STAR_TXFIFOHF			BIT(15)
#define STAR_TXFIFOF			BIT(16)
#define STAR_RXFIFOF			BIT(17)
#define STAR_TXFIFOE			BIT(18)
#define STAR_RXFIFOE			BIT(19)
#define STAR_BUSYD0			BIT(20)
#define STAR_BUSYD0END			BIT(21)
#define STAR_SDIOIT			BIT(22)
#define STAR_ACKFAIL			BIT(23)
#define STAR_ACKTIMEOUT			BIT(24)
#define STAR_VSWEND			BIT(25)
#define STAR_CKSTOP			BIT(26)
#define STAR_IDMATE			BIT(27)
#define STAR_IDMABTC			BIT(28)

#define SDMMC_ICR		0x038
#define ICR_CCRCFAILC			BIT(0)
#define ICR_DCRCFAILC			BIT(1)
#define ICR_CTIMEOUTC			BIT(2)
#define ICR_DTIMEOUTC			BIT(3)
#define ICR_TXUNDERRC			BIT(4)
#define ICR_RXOVERRC			BIT(5)
#define ICR_CMDRENDC			BIT(6)
#define ICR_CMDSENTC			BIT(7)
#define ICR_DATAENDC			BIT(8)
#define ICR_DHOLDC			BIT(9)
#define ICR_DBCKENDC			BIT(10)
#define ICR_DABORTC			BIT(11)
#define ICR_BUSYD0ENDC			BIT(21)
#define ICR_SDIOITC			BIT(22)
#define ICR_ACKFAILC			BIT(23)
#define ICR_ACKTIMEOUTC			BIT(24)
#define ICR_VSWENDC			BIT(25)
#define ICR_CKSTOPC			BIT(26)
#define ICR_IDMATEC			BIT(27)
#define ICR_IDMABTCC			BIT(28)
#define ICR_STATIC_FLAG			((GENMASK(28, 21)) | (GENMASK(11, 0)))

#define SDMMC_MASKR		0x03C
#define MASKR_CCRCFAILIE		BIT(0)
#define MASKR_DCRCFAILIE		BIT(1)
#define MASKR_CTIMEOUTIE		BIT(2)
#define MASKR_DTIMEOUTIE		BIT(3)
#define MASKR_TXUNDERRIE		BIT(4)
#define MASKR_RXOVERRIE			BIT(5)
#define MASKR_CMDRENDIE			BIT(6)
#define MASKR_CMDSENTIE			BIT(7)
#define MASKR_DATAENDIE			BIT(8)
#define MASKR_DHOLDIE			BIT(9)
#define MASKR_DBCKENDIE			BIT(10)
#define MASKR_DABORTIE			BIT(11)
#define MASKR_TXFIFOHEIE		BIT(14)
#define MASKR_RXFIFOHFIE		BIT(15)
#define MASKR_RXFIFOFIE			BIT(17)
#define MASKR_TXFIFOEIE			BIT(18)
#define MASKR_BUSYD0ENDIE		BIT(21)
#define MASKR_SDIOITIE			BIT(22)
#define MASKR_ACKFAILIE			BIT(23)
#define MASKR_ACKTIMEOUTIE		BIT(24)
#define MASKR_VSWENDIE			BIT(25)
#define MASKR_CKSTOPIE			BIT(26)
#define MASKR_IDMABTCIE			BIT(28)

#define SDMMC_ACKTIMER		0x040
#define ACKTIMER_ACKTIME_SHIFT		0
#define ACKTIMER_ACKTIME_MASK		GENMASK(24, 0)

#define SDMMC_FIFOR		0x080

#define SDMMC_IDMACTRLR		0x050
#define IDMACTRLR_IDMAEN		BIT(0)
#define IDMACTRLR_IDMALLIEN		BIT(1)

#define SDMMC_IDMABSIZER	0x054
#define IDMABSIZER_IDMABNDT_SHIFT	5
#define IDMABSIZER_IDMABNDT_MASK	GENMASK(12, 5)

#define SDMMC_IDMABASE0R	0x058
#define SDMMC_IDMABASE1R	0x05C

#define SDMMC_IDMALAR		0x64
#define IDMALAR_IDMALA_MASK		GENMASK(13, 0)
#define IDMALAR_ABR			BIT(29)
#define IDMALAR_ULS			BIT(30)
#define IDMALAR_ULA			BIT(31)

#define SDMMC_IDMABAR		0x68

#define SDMMC_VER		0x3FC
#define VER_MINREV_MASK		GENMASK(3, 0)
#define VER_MAJREV_MASK		GENMASK(7, 4)

#define DLYB_CR			0x000
#define DLYB_CR_DEN			BIT(0)
#define DLYB_CR_SEN			BIT(1)

#define DLYB_CFGR		0x004
#define DLYB_CFGR_SEL_MASK		GENMASK(3, 0)
#define DLYB_CFGR_UNIT_MASK		GENMASK(14, 8)
#define DLYB_CFGR_LNG_MASK		GENMASK(27, 16)
#define DLYB_CFGR_LNGF			BIT(31)

#define DLYB_NB_DELAY		11
#define DLYB_CFGR_SEL_MAX	(DLYB_NB_DELAY + 1)
#define DLYB_CFGR_UNIT_MAX	127

#define SDMMC_LLI_BUF_LEN	PAGE_SIZE
#define SDMMC_IDMA_BURST	BIT(IDMABSIZER_IDMABNDT_SHIFT)

#define SDMMC_AUTOSUSPEND_DELAY_MS 50

enum stm32_sdmmc_cookie {
	COOKIE_UNMAPPED,
	COOKIE_PRE_MAPPED,	/* mapped by pre_req() of stm32 */
	COOKIE_MAPPED,		/* mapped by prepare_data() of stm32 */
};

struct sdmmc_stat {
	unsigned long		n_req;
	unsigned long		n_datareq;
	unsigned long		n_ctimeout;
	unsigned long		n_ccrcfail;
	unsigned long		n_dtimeout;
	unsigned long		n_dcrcfail;
	unsigned long		n_txunderrun;
	unsigned long		n_rxoverrun;
	unsigned long		nb_dma_err;
};

struct sdmmc_lli_desc {
	u32 idmalar;
	u32 idmabase;
	u32 idmasize;
};

struct sdmmc_drv_data {
	u32 sdmmc_ver_ofst;
	u32 idmabsize_mask;
	bool has_idma_lli;
};

struct sdmmc_host {
	void __iomem		*base;
	struct mmc_host		*mmc;
	struct clk		*clk;
	struct reset_control	*rst;

	const struct sdmmc_drv_data *drv_data;

	u32			clk_reg_add;
	u32			pwr_reg_add;

	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_command	stop_abort;
	bool			dpsm_abort;

	/* protect host registers access */
	spinlock_t		lock;

	unsigned int		sdmmcclk;
	unsigned int		sdmmc_ck;

	bool			vqmmc_enabled;
	bool			keep_clk;

	dma_addr_t		sg_dma;
	void			*sg_cpu;

	u32			size;

	void __iomem		*dlyb;
	unsigned int		dlyb_unit;
	unsigned int		dlyb_max;

	u32			ip_ver;
	struct sdmmc_stat	stat;
};

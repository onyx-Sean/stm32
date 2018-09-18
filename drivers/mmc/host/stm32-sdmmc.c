// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2015 - All Rights Reserved
 * Author: Ludovic Barre <ludovic.barre@st.com> for STMicroelectronics.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include "stm32-sdmmc.h"

#define DRIVER_NAME "stm32-sdmmc"

#ifdef CONFIG_DEBUG_FS
static int stm32_sdmmc_stat_show(struct seq_file *s, void *v)
{
	struct sdmmc_host *host = s->private;
	struct sdmmc_stat *stat = &host->stat;

	seq_puts(s, "\033[1;34mstm32 sdmmc statistic\033[0m\n");
	seq_printf(s, "%-20s:%d\n", "sdmmc ck", host->sdmmc_ck);
	seq_printf(s, "%-20s:%ld\n", "nb request", stat->n_req);
	seq_printf(s, "%-20s:%ld\n", "nb data req", stat->n_datareq);
	seq_printf(s, "%-20s:%ld\n", "nb cmd timeout", stat->n_ctimeout);
	seq_printf(s, "%-20s:%ld\n", "nb cmd crcfail", stat->n_ccrcfail);
	seq_printf(s, "%-20s:%ld\n", "nb dat timeout", stat->n_dtimeout);
	seq_printf(s, "%-20s:%ld\n", "nb dat crcfail", stat->n_dcrcfail);
	seq_printf(s, "%-20s:%ld\n", "nb rx overrun", stat->n_rxoverrun);
	seq_printf(s, "%-20s:%ld\n", "nb tx underrun", stat->n_txunderrun);

	return 0;
}

static ssize_t stm32_sdmmc_stat_reset(struct file *filp,
				      const char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct seq_file *seqf = filp->private_data;
	struct sdmmc_host *host = seqf->private;

	mutex_lock(&seqf->lock);
	memset(&host->stat, 0, sizeof(host->stat));
	mutex_unlock(&seqf->lock);

	return count;
}

static int stm32_sdmmc_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, stm32_sdmmc_stat_show, inode->i_private);
}

static const struct file_operations stm32_sdmmc_stat_fops = {
	.owner		= THIS_MODULE,
	.open		= stm32_sdmmc_stat_open,
	.read		= seq_read,
	.write		= stm32_sdmmc_stat_reset,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void stm32_sdmmc_stat_init(struct sdmmc_host *host)
{
	struct mmc_host	*mmc = host->mmc;
	struct dentry *root;

	root = mmc->debugfs_root;
	if (!root)
		return;

	if (!debugfs_create_file("stat", S_IRUSR | S_IWUSR, root, host,
				 &stm32_sdmmc_stat_fops))
		dev_err(mmc_dev(host->mmc), "failed to initialize debugfs\n");
}

#define STAT_INC(stat) ((stat)++)
#else
static void stm32_sdmmc_stat_init(struct sdmmc_host *host)
{
}

#define STAT_INC(stat)
#endif

static inline u32 enable_imask(struct sdmmc_host *host, u32 imask)
{
	u32 newmask;

	newmask = readl_relaxed(host->base + SDMMC_MASKR);
	newmask |= imask;

	dev_vdbg(mmc_dev(host->mmc), "mask:%#x\n", newmask);

	writel_relaxed(newmask, host->base + SDMMC_MASKR);

	return newmask;
}

static inline u32 disable_imask(struct sdmmc_host *host, u32 imask)
{
	u32 newmask;

	newmask = readl_relaxed(host->base + SDMMC_MASKR);
	newmask &= ~imask;

	dev_vdbg(mmc_dev(host->mmc), "mask:%#x\n", newmask);

	writel_relaxed(newmask, host->base + SDMMC_MASKR);

	return newmask;
}

static inline void clear_imask(struct sdmmc_host *host)
{
	u32 mask = readl_relaxed(host->base + SDMMC_MASKR);

	/* preserve the SDIO IRQ mask state */
	mask &= MASKR_SDIOITIE;

	dev_vdbg(mmc_dev(host->mmc), "mask:%#x\n", mask);

	writel_relaxed(mask, host->base + SDMMC_MASKR);
}

static int stm32_sdmmc_card_busy(struct mmc_host *mmc)
{
	struct sdmmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&host->lock, flags);
	status = readl_relaxed(host->base + SDMMC_STAR);
	spin_unlock_irqrestore(&host->lock, flags);

	return !!(status & STAR_BUSYD0);
}

static void stm32_sdmmc_request_end(struct sdmmc_host *host,
				    struct mmc_request *mrq)
{
	writel_relaxed(0, host->base + SDMMC_CMDR);
	writel_relaxed(ICR_STATIC_FLAG, host->base + SDMMC_ICR);

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	clear_imask(host);

	mmc_request_done(host->mmc, mrq);
}

static void stm32_dlyb_input_ck(struct sdmmc_host *host)
{
	writel_relaxed(0, host->dlyb + DLYB_CR);
}

static void stm32_dlyb_set_cfgr(struct sdmmc_host *host,
				int unit, int phase, bool sampler)
{
	u32 cr, cfgr;

	writel_relaxed(DLYB_CR_SEN, host->dlyb + DLYB_CR);

	cfgr = FIELD_PREP(DLYB_CFGR_UNIT_MASK, unit) |
	       FIELD_PREP(DLYB_CFGR_SEL_MASK, phase);
	writel_relaxed(cfgr, host->dlyb + DLYB_CFGR);

	cr = DLYB_CR_DEN;
	if (sampler)
		cr |= DLYB_CR_SEN;

	writel_relaxed(cr, host->dlyb + DLYB_CR);
}

static int stm32_dlyb_lng_tuning(struct sdmmc_host *host)
{
	u32 cfgr;
	int i, lng, ret;

	for (i = 0; i <= DLYB_CFGR_UNIT_MAX; i++) {
		stm32_dlyb_set_cfgr(host, i, DLYB_CFGR_SEL_MAX, true);

		ret = readl_relaxed_poll_timeout(host->dlyb + DLYB_CFGR, cfgr,
						 (cfgr & DLYB_CFGR_LNGF),
						 1, 1000);
		if (ret) {
			dev_warn(mmc_dev(host->mmc),
				 "delay line cfg timeout\n");
			continue;
		}

		lng = FIELD_GET(DLYB_CFGR_LNG_MASK, cfgr);
		if (lng < BIT(DLYB_NB_DELAY) && lng > 0)
			break;
	}

	if (i > DLYB_CFGR_UNIT_MAX)
		return -EINVAL;

	host->dlyb_unit = i;
	host->dlyb_max = __fls(lng);

	return 0;
}

static int stm32_dlyb_phase_tuning(struct sdmmc_host *host, u32 opcode)
{
	int cur_len = 0, max_len = 0, end_of_len = 0;
	int phase;

	for (phase = 0; phase <= host->dlyb_max; phase++) {
		stm32_dlyb_set_cfgr(host, host->dlyb_unit, phase, false);

		if (mmc_send_tuning(host->mmc, opcode, NULL)) {
			cur_len = 0;
		} else {
			cur_len++;
			if (cur_len > max_len) {
				max_len = cur_len;
				end_of_len = phase;
			}
		}
	}

	if (!max_len) {
		dev_err(mmc_dev(host->mmc), "no tuning point found\n");
		return -EINVAL;
	}

	phase = end_of_len - max_len / 2;
	stm32_dlyb_set_cfgr(host, host->dlyb_unit, phase, false);

	dev_dbg(mmc_dev(host->mmc), "unit:%d max_dly:%d phase:%d\n",
		host->dlyb_unit, host->dlyb_max, phase);

	return 0;
}

static int stm32_sdmmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdmmc_host *host = mmc_priv(mmc);

	if (stm32_dlyb_lng_tuning(host))
		return -EINVAL;

	return stm32_dlyb_phase_tuning(host, opcode);
}

static int stm32_sdmmc_volt_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdmmc_host *host = mmc_priv(mmc);
	u32 status, power;
	int ret;

	if (IS_ERR_OR_NULL(mmc->supply.vqmmc))
		return 0;

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		ret = regulator_set_voltage(mmc->supply.vqmmc,
					    2700000, 3600000);
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		ret = regulator_set_voltage(mmc->supply.vqmmc,
					    1700000, 1950000);

		if (ret)
			break;

		/*
		 * start the timing critical section of
		 * the voltage switch sequence
		 */
		power = readl_relaxed(host->base + SDMMC_POWER);
		writel_relaxed(power | POWER_VSWITCH,
			       host->base + SDMMC_POWER);

		/* wait voltage switch completion while 10ms */
		ret = readl_relaxed_poll_timeout(host->base + SDMMC_STAR,
						 status,
						 (status & STAR_VSWEND),
						 10, 10000);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void stm32_sdmmc_pwroff(struct sdmmc_host *host)
{
	/* Only a reset could disable sdmmc */
	reset_control_assert(host->rst);
	udelay(2);
	reset_control_deassert(host->rst);

	/*
	 * Set the SDMMC in Power-cycle state. This will make that the
	 * SDMMC_D[7:0], SDMMC_CMD and SDMMC_CK are driven low,
	 * to prevent the Card from being supplied through the signal lines.
	 */
	writel_relaxed(POWERCTRL_CYC | host->pwr_reg_add,
		       host->base + SDMMC_POWER);

	if (!IS_ERR(host->mmc->supply.vqmmc) && host->vqmmc_enabled) {
		regulator_disable(host->mmc->supply.vqmmc);
		host->vqmmc_enabled = false;
	}
}

static void stm32_sdmmc_pwron(struct sdmmc_host *host)
{
	if (!IS_ERR(host->mmc->supply.vqmmc) && !host->vqmmc_enabled) {
		if (regulator_enable(host->mmc->supply.vqmmc) < 0)
			dev_err(mmc_dev(host->mmc),
				"failed to enable vqmmc regulator\n");
		else
			host->vqmmc_enabled = true;
	}

	/*
	 * After a power-cycle state, we must set the SDMMC in Power-off.
	 * The SDMMC_D[7:0], SDMMC_CMD and SDMMC_CK are driven high.
	 * Then we can set the SDMMC to Power-on state
	 */
	writel_relaxed(POWERCTRL_OFF | host->pwr_reg_add,
		       host->base + SDMMC_POWER);
	mdelay(1);
	writel_relaxed(POWERCTRL_ON | host->pwr_reg_add,
		       host->base + SDMMC_POWER);
}

static void stm32_sdmmc_set_clkreg(struct sdmmc_host *host, struct mmc_ios *ios)
{
	u32 desired = ios->clock;
	u32 clk = 0, ddr = 0;

	if (host->mmc->ios.timing == MMC_TIMING_MMC_DDR52 ||
	    host->mmc->ios.timing == MMC_TIMING_UHS_DDR50)
		ddr = CLKCR_DDR;

	/*
	 * sdmmc_ck = sdmmcclk/(2*clkdiv)
	 * clkdiv 0 => bypass
	 * in ddr mode bypass is not possible
	 */
	if (desired) {
		if (desired >= host->sdmmcclk && !ddr) {
			host->sdmmc_ck = host->sdmmcclk;
		} else {
			clk = DIV_ROUND_UP(host->sdmmcclk, 2 * desired);
			if (clk > CLKCR_CLKDIV_MAX)
				clk = CLKCR_CLKDIV_MAX;

			host->sdmmc_ck = host->sdmmcclk / (2 * clk);
		}
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		clk |= CLKCR_WIDBUS_4;
	if (ios->bus_width == MMC_BUS_WIDTH_8)
		clk |= CLKCR_WIDBUS_8;

	clk |= CLKCR_HWFC_EN;
	clk |= host->clk_reg_add;
	clk |= ddr;

	/*
	 * SDMMC_FBCK is selected when an external Delay Block is needed
	 * with SDR104.
	 */
	if (host->mmc->ios.timing >= MMC_TIMING_UHS_SDR50) {
		clk |= CLKCR_BUSSPEED;
		if (host->mmc->ios.timing == MMC_TIMING_UHS_SDR104) {
			clk &= ~CLKCR_SELCLKRX_MASK;
			clk |= CLKCR_SELCLKRX_FBCK;
		}
	}

	writel_relaxed(clk, host->base + SDMMC_CLKCR);
}

static void stm32_sdmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdmmc_host *host = mmc_priv(mmc);

	stm32_sdmmc_set_clkreg(host, ios);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

		stm32_sdmmc_pwroff(host);
		break;
	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		break;
	case MMC_POWER_ON:
		stm32_dlyb_input_ck(host);
		stm32_sdmmc_pwron(host);
		break;
	}
}

static int stm32_sdmmc_validate_data(struct sdmmc_host *host,
				     struct mmc_data *data, int cookie)
{
	struct scatterlist *sg;
	int i, n_elem;

	if (!data || data->host_cookie == COOKIE_PRE_MAPPED)
		return 0;
	if ((host->mmc->card && !mmc_card_sdio(host->mmc->card)) &&
	    !is_power_of_2(data->blksz)) {
		dev_err(mmc_dev(host->mmc),
			"unsupported block size (%d bytes)\n", data->blksz);
		return -EINVAL;
	}

	/*
	 * idma has constraints on idmabase & idmasize of each elements
	 * expected the last element which has no constraint on idmasize
	 */
	for_each_sg(data->sg, sg, data->sg_len - 1, i) {
		if (!IS_ALIGNED(sg_dma_address(data->sg), sizeof(u32)) ||
		    !IS_ALIGNED(sg_dma_len(data->sg), SDMMC_IDMA_BURST)) {
			dev_err(mmc_dev(host->mmc),
				"unaligned scatterlist: ofst:%x length:%d\n",
				data->sg->offset, data->sg->length);
			return -EINVAL;
		}
	}

	if (!IS_ALIGNED(sg_dma_address(data->sg), sizeof(u32))) {
		dev_err(mmc_dev(host->mmc),
			"unaligned last scatterlist: ofst:%x length:%d\n",
			data->sg->offset, data->sg->length);
		return -EINVAL;
	}

	n_elem = dma_map_sg(mmc_dev(host->mmc),
			    data->sg,
			    data->sg_len,
			    mmc_get_dma_dir(data));

	if (!n_elem) {
		dev_err(mmc_dev(host->mmc), "dma_map_sg failed\n");
		return -EINVAL;
	}

	data->host_cookie = cookie;

	return 0;
}

static void stm32_sdmmc_idma_submit(struct sdmmc_host *host,
				    struct mmc_data *data)
{
	struct sdmmc_lli_desc *desc = (struct sdmmc_lli_desc *)host->sg_cpu;
	struct scatterlist *sg;
	int i;

	if (!host->drv_data->has_idma_lli || data->sg_len == 1) {
		writel_relaxed(sg_dma_address(data->sg),
			       host->base + SDMMC_IDMABASE0R);
		writel_relaxed(IDMACTRLR_IDMAEN, host->base + SDMMC_IDMACTRLR);
		return;
	}

	for_each_sg(data->sg, sg, data->sg_len, i) {
		desc[i].idmalar = (i + 1) * sizeof(struct sdmmc_lli_desc);
		desc[i].idmalar |= IDMALAR_ULA | IDMALAR_ULS | IDMALAR_ABR;
		desc[i].idmabase = sg_dma_address(sg);
		desc[i].idmasize = sg_dma_len(sg);
	}

	/* notice the end of link list */
	desc[data->sg_len - 1].idmalar &= ~IDMALAR_ULA;

	dma_wmb();
	writel_relaxed(host->sg_dma, host->base + SDMMC_IDMABAR);
	writel_relaxed(desc[0].idmalar, host->base + SDMMC_IDMALAR);
	writel_relaxed(desc[0].idmabase, host->base + SDMMC_IDMABASE0R);
	writel_relaxed(desc[0].idmasize, host->base + SDMMC_IDMABSIZER);
	writel_relaxed(IDMACTRLR_IDMAEN | IDMACTRLR_IDMALLIEN,
		       host->base + SDMMC_IDMACTRLR);
}

static void stm32_sdmmc_start_data(struct sdmmc_host *host,
				   struct mmc_data *data)
{
	u32 datactrl, imask;

	dev_dbg(mmc_dev(host->mmc), "blksz %d blks %d flags %08x\n",
		data->blksz, data->blocks, data->flags);

	STAT_INC(host->stat.n_datareq);
	host->data = data;
	host->size = data->blksz * data->blocks;
	data->bytes_xfered = 0;

	writel_relaxed(host->size, host->base + SDMMC_DLENR);

	datactrl = FIELD_PREP(DCTRLR_DBLOCKSIZE_MASK, ilog2(data->blksz));

	if (data->flags & MMC_DATA_READ) {
		datactrl |= DCTRLR_DTDIR;
		imask = MASKR_RXOVERRIE;
	} else {
		imask = MASKR_TXUNDERRIE;
	}

	if (host->mmc->card && mmc_card_sdio(host->mmc->card)) {
		if (data->blocks > 1)
			datactrl |= DCTRLR_DTMODE_BLOCK;
		else
			datactrl |= DCTRLR_DTMODE_SDIO;

		datactrl |= DCTRLR_SDIOEN;
	}

	stm32_sdmmc_idma_submit(host, data);

	imask |= MASKR_DATAENDIE | MASKR_DTIMEOUTIE | MASKR_DCRCFAILIE;
	enable_imask(host, imask);

	writel_relaxed(datactrl, host->base + SDMMC_DCTRLR);
}

static void stm32_sdmmc_start_cmd(struct sdmmc_host *host,
				  struct mmc_command *cmd, u32 c)
{
	u32 imsk, timeout = 0;
	unsigned long long clks;

	dev_dbg(mmc_dev(host->mmc), "op %u arg %08x flags %08x\n",
		cmd->opcode, cmd->arg, cmd->flags);

	/* Temporary hack to fix timing issue */
	udelay(1500);

	STAT_INC(host->stat.n_req);

	if (host->dpsm_abort)
		c |= CMDR_CMDSTOP;

	host->dpsm_abort = false;

	if (readl_relaxed(host->base + SDMMC_CMDR) & CMDR_CPSMEM)
		writel_relaxed(0, host->base + SDMMC_CMDR);

	c |= cmd->opcode | CMDR_CPSMEM;
	if (cmd->flags & MMC_RSP_PRESENT) {
		imsk = MASKR_CMDRENDIE | MASKR_CTIMEOUTIE;
		if (cmd->flags & MMC_RSP_CRC)
			imsk |= MASKR_CCRCFAILIE;

		if (cmd->flags & MMC_RSP_136)
			c |= CMDR_WAITRESP_LRSP_CRC;
		else if (cmd->flags & MMC_RSP_CRC)
			c |= CMDR_WAITRESP_SRSP_CRC;
		else
			c |= CMDR_WAITRESP_SRSP;
	} else {
		c &= ~CMDR_WAITRESP_MASK;
		imsk = MASKR_CMDSENTIE;
	}

	if (cmd->opcode == SD_SWITCH_VOLTAGE)
		writel_relaxed(readl_relaxed(host->base + SDMMC_POWER)
			       | POWER_VSWITCHEN, host->base + SDMMC_POWER);

	host->cmd = cmd;

	/*
	 * SDMMC_DTIME must be set in two case:
	 * - on data transfert.
	 * - on busy request or cmd12 without busy response
	 * If not done or too short, the dtimeout flag occur and DPSM stays
	 * enabled/busy and wait for Abort (stop transmission cmd).
	 * Next data command is not possible whereas DPSM is activated.
	 */
	if (host->data) {
		clks = (unsigned long long)host->data->timeout_ns *
			host->sdmmc_ck;
		do_div(clks, NSEC_PER_SEC);
		timeout = host->data->timeout_clks + (unsigned int)clks;
	} else {
		writel_relaxed(0x0, host->base + SDMMC_DCTRLR);

		if (cmd->flags & MMC_RSP_BUSY ||
		    cmd->opcode == MMC_STOP_TRANSMISSION) {
			if (!cmd->busy_timeout)
				cmd->busy_timeout = 1000;

			clks = (unsigned long long)cmd->busy_timeout *
				host->sdmmc_ck;
			do_div(clks, MSEC_PER_SEC);
			timeout = (unsigned int)clks;

			imsk |= MASKR_BUSYD0ENDIE | MASKR_DTIMEOUTIE;
		}
	}

	enable_imask(host, imsk);

	writel_relaxed(timeout, host->base + SDMMC_DTIMER);
	writel_relaxed(cmd->arg, host->base + SDMMC_ARGR);
	writel_relaxed(c, host->base + SDMMC_CMDR);
}

static void stm32_sdmmc_cmd_irq(struct sdmmc_host *host, u32 status)
{
	struct mmc_command *cmd = host->cmd;
	struct mmc_command *stop_abort = NULL;

	if (!cmd)
		return;

	if (status & STAR_CTIMEOUT) {
		STAT_INC(host->stat.n_ctimeout);
		cmd->error = -ETIMEDOUT;
		host->dpsm_abort = true;
	} else if (status & STAR_CCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		STAT_INC(host->stat.n_ccrcfail);
		cmd->error = -EILSEQ;
		host->dpsm_abort = true;
	} else if (status & STAR_DTIMEOUT && !host->data) {
		STAT_INC(host->stat.n_dtimeout);
		cmd->error = -ETIMEDOUT;
		stop_abort = &host->stop_abort;
		host->dpsm_abort = true;
		dev_err(mmc_dev(host->mmc),
			"dtimeout on cmd:%d, busy timeout:%d\n",
			cmd->opcode, cmd->busy_timeout);
	} else if (status & STAR_CMDREND && cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl_relaxed(host->base + SDMMC_RESP1R);
		cmd->resp[1] = readl_relaxed(host->base + SDMMC_RESP2R);
		cmd->resp[2] = readl_relaxed(host->base + SDMMC_RESP3R);
		cmd->resp[3] = readl_relaxed(host->base + SDMMC_RESP4R);

		/* If BUSYD0 signals busy, wait for BUSYD0END flag */
		if (cmd->flags & MMC_RSP_BUSY && (status & STAR_BUSYD0)) {
			/* Warning if busyd0 and STAR_BUSYD0END */
			WARN_ON(status & STAR_BUSYD0END);
			return;
		}
	}

	host->cmd = NULL;

	if (!host->data) {
		if (stop_abort)
			stm32_sdmmc_start_cmd(host, stop_abort, 0);
		else
			stm32_sdmmc_request_end(host, host->mrq);
	}
}

static void stm32_sdmmc_data_irq(struct sdmmc_host *host, u32 status)
{
	struct mmc_data	*data = host->data;
	struct mmc_command *stop_abort = &host->stop_abort;

	if (!data)
		return;

	if (status & STAR_DCRCFAIL) {
		STAT_INC(host->stat.n_dcrcfail);
		data->error = -EILSEQ;
		if (readl_relaxed(host->base + SDMMC_DCNTR))
			host->dpsm_abort = true;
	} else if (status & STAR_DTIMEOUT) {
		STAT_INC(host->stat.n_dtimeout);
		data->error = -ETIMEDOUT;
		host->dpsm_abort = true;
	} else if (status & STAR_TXUNDERR) {
		STAT_INC(host->stat.n_txunderrun);
		data->error = -EIO;
		host->dpsm_abort = true;
	} else if (status & STAR_RXOVERR) {
		STAT_INC(host->stat.n_rxoverrun);
		data->error = -EIO;
		host->dpsm_abort = true;
	}

	if (status & STAR_DATAEND || data->error || host->dpsm_abort) {
		host->data = NULL;

		writel_relaxed(0, host->base + SDMMC_IDMACTRLR);

		if (!data->error)
			data->bytes_xfered = data->blocks * data->blksz;

		/*
		 * To stop Data Path State Machine, a stop_transmission command
		 * shall be send on cmd or data errors of single, multi,
		 * pre-defined block and stream request.
		 */
		if (host->dpsm_abort && !data->stop)
			data->stop = stop_abort;

		disable_imask(host, MASKR_RXOVERRIE | MASKR_TXUNDERRIE
			      | MASKR_DCRCFAILIE | MASKR_DATAENDIE
			      | MASKR_DTIMEOUTIE);

		if (!data->stop)
			stm32_sdmmc_request_end(host, data->mrq);
		else
			stm32_sdmmc_start_cmd(host, data->stop, 0);
	}
}

static irqreturn_t stm32_sdmmc_irq(int irq, void *dev_id)
{
	struct sdmmc_host *host = dev_id;
	u32 status;

	spin_lock(&host->lock);

	status = readl_relaxed(host->base + SDMMC_STAR);
	dev_dbg(mmc_dev(host->mmc), "irq sta:%#x\n", status);
	writel_relaxed(status & ICR_STATIC_FLAG, host->base + SDMMC_ICR);

	stm32_sdmmc_cmd_irq(host, status);
	stm32_sdmmc_data_irq(host, status);

	spin_unlock(&host->lock);

	return IRQ_HANDLED;
}

static void stm32_sdmmc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sdmmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	/* This data might be unmapped at this time */
	data->host_cookie = COOKIE_UNMAPPED;

	if (!stm32_sdmmc_validate_data(host, mrq->data, COOKIE_PRE_MAPPED))
		data->host_cookie = COOKIE_UNMAPPED;
}

static void stm32_sdmmc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
				 int err)
{
	struct sdmmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	if (data->host_cookie != COOKIE_UNMAPPED)
		dma_unmap_sg(mmc_dev(host->mmc),
			     data->sg,
			     data->sg_len,
			     mmc_get_dma_dir(data));

	data->host_cookie = COOKIE_UNMAPPED;
}

static void stm32_sdmmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	unsigned int cmdat = 0;
	struct sdmmc_host *host = mmc_priv(mmc);
	unsigned long flags;

	mrq->cmd->error = stm32_sdmmc_validate_data(host, mrq->data,
						    COOKIE_MAPPED);
	if (mrq->cmd->error) {
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;

	if (mrq->data) {
		u32 status;

		status = readl_relaxed(host->base + SDMMC_STAR);
		WARN(status & STAR_DPSMACT,
		     "DPSM not in idle (status:%#x)\n", status);

		stm32_sdmmc_start_data(host, mrq->data);
		cmdat |= CMDR_CMDTRANS;
	}

	stm32_sdmmc_start_cmd(host, mrq->cmd, cmdat);

	spin_unlock_irqrestore(&host->lock, flags);
}

static struct mmc_host_ops stm32_sdmmc_ops = {
	.request	= stm32_sdmmc_request,
	.pre_req	= stm32_sdmmc_pre_req,
	.post_req	= stm32_sdmmc_post_req,
	.set_ios	= stm32_sdmmc_set_ios,
	.get_cd		= mmc_gpio_get_cd,
	.card_busy	= stm32_sdmmc_card_busy,
	.start_signal_voltage_switch = stm32_sdmmc_volt_switch,
	.execute_tuning	= stm32_sdmmc_execute_tuning,
};

static const struct sdmmc_drv_data stm32_sdmmc_cfg = {
	.sdmmc_ver_ofst = 0x3fc,
	.idmabsize_mask = GENMASK(12, 5),
	.has_idma_lli = false,
};

static const struct sdmmc_drv_data stm32_sdmmc_mp1_cfg = {
	.sdmmc_ver_ofst = 0x3f4,
	.idmabsize_mask = GENMASK(16, 5),
	.has_idma_lli = true,
};

static const struct of_device_id stm32_sdmmc_match[] = {
	{ .compatible = "st,stm32-sdmmc2", .data = &stm32_sdmmc_cfg },
	{ .compatible = "st,stm32mp1-sdmmc2", .data = &stm32_sdmmc_mp1_cfg},
	{},
};
MODULE_DEVICE_TABLE(of, stm32_sdmmc_match);

static int stm32_sdmmc_of_parse(struct device_node *np, struct mmc_host *mmc)
{
	struct sdmmc_host *host = mmc_priv(mmc);
	int ret = mmc_of_parse(mmc);

	if (ret)
		return ret;

	host->drv_data = of_device_get_match_data(mmc_dev(host->mmc));
	if (!host->drv_data)
		return -EINVAL;

	if (of_get_property(np, "st,negedge", NULL))
		host->clk_reg_add |= CLKCR_NEGEDGE;
	if (of_get_property(np, "st,dirpol", NULL))
		host->pwr_reg_add |= POWER_DIRPOL;
	if (of_get_property(np, "st,pin-ckin", NULL))
		host->clk_reg_add |= CLKCR_SELCLKRX_CKIN;
	if (of_get_property(np, "st,keep-clock", NULL))
		host->keep_clk = true;

	return 0;
}

static int stm32_sdmmc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sdmmc_host *host;
	struct mmc_host *mmc;
	struct resource *res;
	int irq, ret;

	if (!np) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	mmc = mmc_alloc_host(sizeof(struct sdmmc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	platform_set_drvdata(pdev, mmc);

	/* prepare the stop command, used to abort and reinitialized the DPSM */
	host->stop_abort.opcode = MMC_STOP_TRANSMISSION;
	host->stop_abort.arg = 0;
	host->stop_abort.flags = MMC_RSP_R1B | MMC_CMD_AC;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sdmmc");
	host->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto host_free;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "delay");
	host->dlyb = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->dlyb)) {
		ret = PTR_ERR(host->dlyb);
		goto host_free;
	}

	writel_relaxed(0, host->base + SDMMC_MASKR);
	writel_relaxed(~0UL, host->base + SDMMC_ICR);

	ret = devm_request_irq(&pdev->dev, irq, stm32_sdmmc_irq, IRQF_SHARED,
			       DRIVER_NAME " (cmd)", host);
	if (ret)
		goto host_free;

	host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto host_free;
	}

	ret = clk_prepare_enable(host->clk);
	if (ret)
		goto host_free;

	host->sdmmcclk = clk_get_rate(host->clk);
	mmc->f_min = DIV_ROUND_UP(host->sdmmcclk, 2 * CLKCR_CLKDIV_MAX);
	mmc->f_max = host->sdmmcclk;

	mmc->max_busy_timeout = ~0UL / (mmc->f_max / MSEC_PER_SEC);
	mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY;

	ret = stm32_sdmmc_of_parse(np, mmc);
	if (ret)
		goto clk_disable;

	host->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(host->rst)) {
		ret = PTR_ERR(host->rst);
		goto clk_disable;
	}

	stm32_sdmmc_pwroff(host);

	/* Get regulators and the supported OCR mask */
	ret = mmc_regulator_get_supply(mmc);
	if (ret == -EPROBE_DEFER)
		goto clk_disable;

	if (!mmc->ocr_avail)
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	mmc->ops = &stm32_sdmmc_ops;

	mmc->max_req_size = DLENR_DATALENGHT_MAX;

	if (host->drv_data->has_idma_lli) {
		host->sg_cpu = dmam_alloc_coherent(&pdev->dev,
						   SDMMC_LLI_BUF_LEN,
						   &host->sg_dma, GFP_KERNEL);
		if (!host->sg_cpu) {
			dev_err(&pdev->dev, "Failed to alloc DMA descriptor\n");
			ret = -ENOMEM;
			goto clk_disable;
		}
		mmc->max_segs = SDMMC_LLI_BUF_LEN /
			sizeof(struct sdmmc_lli_desc);
		mmc->max_seg_size = host->drv_data->idmabsize_mask;
	} else {
		mmc->max_segs = 1;
		mmc->max_seg_size = mmc->max_req_size;
	}

	mmc->max_blk_size = 1 << DCTRLR_DBLOCKSIZE_MAX;

	/*
	 * Limit the number of blocks transferred so that we don't overflow
	 * the maximum request size.
	 */
	mmc->max_blk_count = mmc->max_req_size >> DCTRLR_DBLOCKSIZE_MAX;

	spin_lock_init(&host->lock);

	/* sync pm_runtime state */
	 pm_runtime_get_noresume(&pdev->dev);
	/* common pm_runtime init */
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 SDMMC_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);

	mmc_add_host(mmc);

	stm32_sdmmc_stat_init(host);

	host->ip_ver = readl_relaxed(host->base +
				     host->drv_data->sdmmc_ver_ofst);
	dev_info(&pdev->dev, "%s: rev:%ld.%ld irq:%d\n",
		 mmc_hostname(mmc),
		 FIELD_GET(VER_MAJREV_MASK, host->ip_ver),
		 FIELD_GET(VER_MINREV_MASK, host->ip_ver), irq);

	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;

clk_disable:
	clk_disable_unprepare(host->clk);
host_free:
	mmc_free_host(mmc);
	return ret;
}

static int stm32_sdmmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct sdmmc_host *host = mmc_priv(mmc);

	pm_runtime_get_sync(&pdev->dev);

	/* Debugfs stuff is cleaned up by mmc core */
	mmc_remove_host(mmc);
	clk_disable_unprepare(host->clk);
	mmc_free_host(mmc);

	/* unconfigure pm_runtime */
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int stm32_sdmmc_runtime_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct sdmmc_host *host = mmc_priv(mmc);

	if (!host->keep_clk)
		clk_disable_unprepare(host->clk);
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int stm32_sdmmc_runtime_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct sdmmc_host *host = mmc_priv(mmc);
	int ret = 0;

	pinctrl_pm_select_default_state(dev);
	if (!host->keep_clk)
		ret = clk_prepare_enable(host->clk);

	return ret;
}

#endif

static const struct dev_pm_ops sdmmc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(stm32_sdmmc_runtime_suspend,
			   stm32_sdmmc_runtime_resume, NULL)
};

static struct platform_driver stm32_sdmmc_driver = {
	.probe		= stm32_sdmmc_probe,
	.remove		= stm32_sdmmc_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.of_match_table = stm32_sdmmc_match,
		.pm = &sdmmc_dev_pm_ops,
	},
};

module_platform_driver(stm32_sdmmc_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 MMC/SD Card Interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ludovic Barre <ludovic.barre@st.com>");

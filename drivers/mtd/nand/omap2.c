/*
 * Copyright © 2004 Texas Instruments, Jian Zhang <jzhang@ti.com>
 * Copyright © 2004 Micron Technology Inc.
 * Copyright © 2004 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/hardirq.h>

#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#define GPMC_ECC_CONFIG		0x1F4
#define GPMC_ECC_CONTROL	0x1F8
#define GPMC_ECC_SIZE_CONFIG	0x1FC
#define GPMC_ECC1_RESULT	0x200
#define GPMC_ECC_BCH_RESULT_0   0x240

#define	DRIVER_NAME	"omap2-nand"

#define	NAND_WP_OFF	0
#define NAND_WP_BIT	0x00000010

#define	GPMC_BUF_FULL	0x00000001
#define	GPMC_BUF_EMPTY	0x00000000

#define NAND_Ecc_P1e		(1 << 0)
#define NAND_Ecc_P2e		(1 << 1)
#define NAND_Ecc_P4e		(1 << 2)
#define NAND_Ecc_P8e		(1 << 3)
#define NAND_Ecc_P16e		(1 << 4)
#define NAND_Ecc_P32e		(1 << 5)
#define NAND_Ecc_P64e		(1 << 6)
#define NAND_Ecc_P128e		(1 << 7)
#define NAND_Ecc_P256e		(1 << 8)
#define NAND_Ecc_P512e		(1 << 9)
#define NAND_Ecc_P1024e		(1 << 10)
#define NAND_Ecc_P2048e		(1 << 11)

#define NAND_Ecc_P1o		(1 << 16)
#define NAND_Ecc_P2o		(1 << 17)
#define NAND_Ecc_P4o		(1 << 18)
#define NAND_Ecc_P8o		(1 << 19)
#define NAND_Ecc_P16o		(1 << 20)
#define NAND_Ecc_P32o		(1 << 21)
#define NAND_Ecc_P64o		(1 << 22)
#define NAND_Ecc_P128o		(1 << 23)
#define NAND_Ecc_P256o		(1 << 24)
#define NAND_Ecc_P512o		(1 << 25)
#define NAND_Ecc_P1024o		(1 << 26)
#define NAND_Ecc_P2048o		(1 << 27)

#define TF(value)	(value ? 1 : 0)

#define P2048e(a)	(TF(a & NAND_Ecc_P2048e)	<< 0)
#define P2048o(a)	(TF(a & NAND_Ecc_P2048o)	<< 1)
#define P1e(a)		(TF(a & NAND_Ecc_P1e)		<< 2)
#define P1o(a)		(TF(a & NAND_Ecc_P1o)		<< 3)
#define P2e(a)		(TF(a & NAND_Ecc_P2e)		<< 4)
#define P2o(a)		(TF(a & NAND_Ecc_P2o)		<< 5)
#define P4e(a)		(TF(a & NAND_Ecc_P4e)		<< 6)
#define P4o(a)		(TF(a & NAND_Ecc_P4o)		<< 7)

#define P8e(a)		(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o(a)		(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e(a)		(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o(a)		(TF(a & NAND_Ecc_P16o)		<< 3)
#define P32e(a)		(TF(a & NAND_Ecc_P32e)		<< 4)
#define P32o(a)		(TF(a & NAND_Ecc_P32o)		<< 5)
#define P64e(a)		(TF(a & NAND_Ecc_P64e)		<< 6)
#define P64o(a)		(TF(a & NAND_Ecc_P64o)		<< 7)

#define P128e(a)	(TF(a & NAND_Ecc_P128e)		<< 0)
#define P128o(a)	(TF(a & NAND_Ecc_P128o)		<< 1)
#define P256e(a)	(TF(a & NAND_Ecc_P256e)		<< 2)
#define P256o(a)	(TF(a & NAND_Ecc_P256o)		<< 3)
#define P512e(a)	(TF(a & NAND_Ecc_P512e)		<< 4)
#define P512o(a)	(TF(a & NAND_Ecc_P512o)		<< 5)
#define P1024e(a)	(TF(a & NAND_Ecc_P1024e)	<< 6)
#define P1024o(a)	(TF(a & NAND_Ecc_P1024o)	<< 7)

#define P8e_s(a)	(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o_s(a)	(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e_s(a)	(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o_s(a)	(TF(a & NAND_Ecc_P16o)		<< 3)
#define P1e_s(a)	(TF(a & NAND_Ecc_P1e)		<< 4)
#define P1o_s(a)	(TF(a & NAND_Ecc_P1o)		<< 5)
#define P2e_s(a)	(TF(a & NAND_Ecc_P2e)		<< 6)
#define P2o_s(a)	(TF(a & NAND_Ecc_P2o)		<< 7)

#define P4e_s(a)	(TF(a & NAND_Ecc_P4e)		<< 0)
#define P4o_s(a)	(TF(a & NAND_Ecc_P4o)		<< 1)

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH
static int use_prefetch = 1;

/* "modprobe ... use_prefetch=0" etc */
module_param(use_prefetch, bool, 0);
MODULE_PARM_DESC(use_prefetch, "enable/disable use of PREFETCH");

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH_DMA
static int use_dma = 1;

/* "modprobe ... use_dma=0" etc */
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "enable/disable use of DMA");
#else
static const int use_dma;
#endif
#else
const int use_prefetch;
static const int use_dma;
#endif

#ifdef CONFIG_MTD_NAND_OMAP_BCH
#include <linux/bch.h>
struct bch_control;
int omap3_bch_init(struct mtd_info *mtd, int max_errors, int error_threshold);
void omap3_bch_free(struct mtd_info *mtd);
#endif
static int omap_onfi_set(struct mtd_info* mtd, int mode);

struct omap_nand_info {
	struct nand_hw_control		controller;
	struct omap_nand_platform_data	*pdata;
	struct mtd_info			mtd;
	struct mtd_partition		*parts;
	struct nand_chip		nand;
	struct platform_device		*pdev;

	int				gpmc_cs;
	unsigned long			phys_base;
	void __iomem			*gpmc_cs_baseaddr;
	void __iomem			*gpmc_baseaddr;
	void __iomem			*nand_pref_fifo_add;
	struct completion		comp;
	int				dma_ch;

#ifdef CONFIG_MTD_NAND_OMAP_BCH
	struct bch_control             *bch;
	struct nand_ecclayout           ecclayout;
	int                             error_threshold;
#endif
};

static struct nand_ecclayout nand_x8_hw_romcode_oob_64 = {
	.eccbytes = 12,
	.eccpos = {
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
	},
	.oobfree = {
		{.offset = 13,
		.length = 51}
	}
};

/*
 * Define some generic bad / good block scan pattern which are used
 * while scanning a device for factory marked good / bad blocks
 */
static uint8_t scan_ff_pattern[] = { 0xff };

static struct
nand_bbt_descr bb_descrip_flashbased = {
	.options = NAND_BBT_SCANEMPTY | NAND_BBT_SCANALLPAGES,
	.offs = 0,
	.len = 1,
	.pattern = scan_ff_pattern,
};

static struct nand_ecclayout nand_x16_hw_romcode_oob_64 = {
	.eccbytes = 12,
	.eccpos = {
		2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
	},
	.oobfree = {
		{.offset = 14,
		.length = 50}
	}
};

static int optim_rd, optim_wr;

/**
 * omap_nand_wp - This function enable or disable the Write Protect feature
 * @mtd: MTD device structure
 * @mode: WP ON/OFF
 */
static void omap_nand_wp(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);

	unsigned long config = __raw_readl(info->gpmc_baseaddr + GPMC_CONFIG);

	if (mode)
		config &= ~(NAND_WP_BIT);	/* WP is ON */
	else
		config |= (NAND_WP_BIT);	/* WP is OFF */

	__raw_writel(config, (info->gpmc_baseaddr + GPMC_CONFIG));
}

/**
 * omap_hwcontrol - hardware specific access to control-lines
 * @mtd: MTD device structure
 * @cmd: command to device
 * @ctrl:
 * NAND_NCE: bit 0 -> don't care
 * NAND_CLE: bit 1 -> Command Latch
 * NAND_ALE: bit 2 -> Address Latch
 *
 * NOTE: boards may use different bits for these!!
 */
static void omap_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	switch (ctrl) {
	case NAND_CTRL_CHANGE | NAND_CTRL_CLE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_COMMAND;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;

	case NAND_CTRL_CHANGE | NAND_CTRL_ALE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_ADDRESS;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;

	case NAND_CTRL_CHANGE | NAND_NCE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;
	}

	if (cmd != NAND_CMD_NONE)
		__raw_writeb(cmd, info->nand.IO_ADDR_W);
}

/**
 * omap_read_buf8 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf8(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;

	ioread8_rep(nand->IO_ADDR_R, buf, len);
}

/**
 * omap_write_buf8 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf8(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u_char *p = (u_char *)buf;

	while (len--) {
		iowrite8(*p++, info->nand.IO_ADDR_W);
		while (GPMC_BUF_EMPTY == (readl(info->gpmc_baseaddr +
						GPMC_STATUS) & GPMC_BUF_FULL));
	}
}

/**
 * omap_read_buf16 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf16(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;

	ioread16_rep(nand->IO_ADDR_R, buf, len / 2);
}

/**
 * omap_write_buf16 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf16(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u16 *p = (u16 *) buf;

	/* FIXME try bursts of writesw() or DMA ... */
	len >>= 1;

	while (len--) {
		iowrite16(*p++, info->nand.IO_ADDR_W);

		while (GPMC_BUF_EMPTY == (readl(info->gpmc_baseaddr +
						GPMC_STATUS) & GPMC_BUF_FULL))
			;
	}
}

/**
 * omap_read_buf_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t pfpw_status = 0, r_count = 0;
	int ret = 0;
	u32 *p = (u32 *)buf;

	/* take care of subpage reads */
	if (len % 4) {
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_read_buf16(mtd, buf, len % 4);
		else
			omap_read_buf8(mtd, buf, len % 4);
		p = (u32 *) (buf + len % 4);
		len -= len % 4;
	}

	/* configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs, 0x0, len, 0x0, optim_rd);
	if (ret) {
		/* PFPW engine is busy, use cpu copy method */
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_read_buf16(mtd, buf, len);
		else
			omap_read_buf8(mtd, buf, len);
	} else {
		do {
			pfpw_status = gpmc_prefetch_status();
			r_count = ((pfpw_status >> 24) & 0x7F) >> 2;
			ioread32_rep(info->nand_pref_fifo_add, p, r_count);
			p += r_count;
			len -= r_count << 2;
		} while (len);

		/* disable and stop the PFPW engine */
		gpmc_prefetch_reset();
	}
}

/**
 * omap_write_buf_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t pfpw_status = 0, w_count = 0;
	int i = 0, ret = 0;
	u16 *p = (u16 *) buf;

	/* take care of subpage writes */
	if (len % 2 != 0) {
		writeb(*buf, info->nand.IO_ADDR_R);
		p = (u16 *)(buf + 1);
		len--;
	}

	/*  configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs, 0x0, len, 0x1, optim_wr);
	if (ret) {
		/* PFPW engine is busy, use cpu copy method */
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_write_buf16(mtd, buf, len);
		else
			omap_write_buf8(mtd, buf, len);
	} else {
		pfpw_status = gpmc_prefetch_status();
		while (pfpw_status & 0x3FFF) {
			w_count = ((pfpw_status >> 24) & 0x7F) >> 1;
			for (i = 0; (i < w_count) && len; i++, len -= 2)
				iowrite16(*p++, info->nand_pref_fifo_add);
			pfpw_status = gpmc_prefetch_status();
		}

		/* disable and stop the PFPW engine */
		gpmc_prefetch_reset();
	}
}

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH_DMA
/*
 * omap_nand_dma_cb: callback on the completion of dma transfer
 * @lch: logical channel
 * @ch_satuts: channel status
 * @data: pointer to completion data structure
 */
static void omap_nand_dma_cb(int lch, u16 ch_status, void *data)
{
	complete((struct completion *) data);
}

/*
 * omap_nand_dma_transfer: configer and start dma transfer
 * @mtd: MTD device structure
 * @addr: virtual address in RAM of source/destination
 * @len: number of data bytes to be transferred
 * @is_write: flag for read/write operation
 */
static inline int omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
					unsigned int len, int is_write)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	uint32_t prefetch_status = 0;
	enum dma_data_direction dir = is_write ? DMA_TO_DEVICE :
							DMA_FROM_DEVICE;
	dma_addr_t dma_addr;
	int ret;

	/* The fifo depth is 64 bytes. We have a sync at each frame and frame
	 * length is 64 bytes.
	 */
	int buf_len = len >> 6;

	if (addr >= high_memory) {
		struct page *p1;

		if (((size_t)addr & PAGE_MASK) !=
			((size_t)(addr + len - 1) & PAGE_MASK))
			goto out_copy;
		p1 = vmalloc_to_page(addr);
		if (!p1)
			goto out_copy;
		addr = page_address(p1) + ((size_t)addr & ~PAGE_MASK);
	}

	dma_addr = dma_map_single(&info->pdev->dev, addr, len, dir);
	if (dma_mapping_error(&info->pdev->dev, dma_addr)) {
		dev_err(&info->pdev->dev,
			"Couldn't DMA map a %d byte buffer\n", len);
		goto out_copy;
	}

	if (is_write) {
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_DST_SYNC);
	} else {
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_SRC_SYNC);
	}
	/*  configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs, 0x1, len, is_write,
			is_write?optim_wr:optim_rd);
	if (ret)
		/* PFPW engine is busy, use cpu copy methode */
		goto out_copy;

	init_completion(&info->comp);

	omap_start_dma(info->dma_ch);

	/* setup and start DMA using dma_addr */
	wait_for_completion(&info->comp);

	while (0x3fff & (prefetch_status = gpmc_prefetch_status()))
		;
	/* disable and stop the PFPW engine */
	gpmc_prefetch_reset();

	dma_unmap_single(&info->pdev->dev, dma_addr, len, dir);
	return 0;

out_copy:
	if (info->nand.options & NAND_BUSWIDTH_16)
		is_write == 0 ? omap_read_buf16(mtd, (u_char *) addr, len)
			: omap_write_buf16(mtd, (u_char *) addr, len);
	else
		is_write == 0 ? omap_read_buf8(mtd, (u_char *) addr, len)
			: omap_write_buf8(mtd, (u_char *) addr, len);
	return 0;
}
#else
static void omap_nand_dma_cb(int lch, u16 ch_status, void *data) {}
static inline int omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
					unsigned int len, int is_write)
{
	return 0;
}
#endif

/**
 * omap_read_buf_dma_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_dma_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	if (len <= mtd->oobsize
                || (!IS_ALIGNED((unsigned long)buf, 4)) )
		omap_read_buf_pref(mtd, buf, len);
	else
		/* start transfer in DMA mode */
		omap_nand_dma_transfer(mtd, buf, len, 0x0);
}

/**
 * omap_write_buf_dma_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_dma_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	if (len <= mtd->oobsize || in_interrupt() || oops_in_progress)
		omap_write_buf_pref(mtd, buf, len);
	else
		/* start transfer in DMA mode */
		omap_nand_dma_transfer(mtd, (u_char *) buf, len, 0x1);
}

/**
 * omap_verify_buf - Verify chip data against buffer
 * @mtd: MTD device structure
 * @buf: buffer containing the data to compare
 * @len: number of bytes to compare
 */
static int omap_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	u16 *p = (u16 *) buf;

	len >>= 1;
	while (len--) {
		if (*p++ != cpu_to_le16(readw(info->nand.IO_ADDR_R)))
			return -EFAULT;
	}

	return 0;
}

/**
 * omap_hwecc_init - Initialize the HW ECC for NAND flash in GPMC controller
 * @mtd: MTD device structure
 */
static void omap_hwecc_init(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	struct nand_chip *chip = mtd->priv;
	unsigned long val = 0x0;

	/* Read from ECC Control Register */
	val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_CONTROL);
	/* Clear all ECC | Enable Reg1 */
	val = ((0x00000001<<8) | 0x00000001);
	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_CONTROL);

	/* Read from ECC Size Config Register */
	val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_SIZE_CONFIG);
	/* ECCSIZE1=512 | Select eccResultsize[0-3] */
	val = ((((chip->ecc.size >> 1) - 1) << 22) | (0x0000000F));
	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_SIZE_CONFIG);
}

/**
 * gen_true_ecc - This function will generate true ECC value
 * @ecc_buf: buffer to store ecc code
 *
 * This generated true ECC value can be used when correcting
 * data read from NAND flash memory core
 */
static void gen_true_ecc(u8 *ecc_buf)
{
	u32 tmp = ecc_buf[0] | (ecc_buf[1] << 16) |
		((ecc_buf[2] & 0xF0) << 20) | ((ecc_buf[2] & 0x0F) << 8);

	ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) |
			P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp));
	ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) |
			P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
	ecc_buf[2] = ~(P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) |
			P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}

/**
 * omap_compare_ecc - Detect (2 bits) and correct (1 bit) error in data
 * @ecc_data1:  ecc code from nand spare area
 * @ecc_data2:  ecc code from hardware register obtained from hardware ecc
 * @page_data:  page data
 *
 * This function compares two ECC's and indicates if there is an error.
 * If the error can be corrected it will be corrected to the buffer.
 */
static int omap_compare_ecc(u8 *ecc_data1,	/* read from NAND memory */
			    u8 *ecc_data2,	/* read from register */
			    u8 *page_data)
{
	uint	i;
	u8	tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
	u8	comp0_bit[8], comp1_bit[8], comp2_bit[8];
	u8	ecc_bit[24];
	u8	ecc_sum = 0;
	u8	find_bit = 0;
	uint	find_byte = 0;
	int	isEccFF;

	isEccFF = ((*(u32 *)ecc_data1 & 0xFFFFFF) == 0xFFFFFF);

	gen_true_ecc(ecc_data1);
	gen_true_ecc(ecc_data2);

	for (i = 0; i <= 2; i++) {
		*(ecc_data1 + i) = ~(*(ecc_data1 + i));
		*(ecc_data2 + i) = ~(*(ecc_data2 + i));
	}

	for (i = 0; i < 8; i++) {
		tmp0_bit[i]     = *ecc_data1 % 2;
		*ecc_data1	= *ecc_data1 / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp1_bit[i]	 = *(ecc_data1 + 1) % 2;
		*(ecc_data1 + 1) = *(ecc_data1 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp2_bit[i]	 = *(ecc_data1 + 2) % 2;
		*(ecc_data1 + 2) = *(ecc_data1 + 2) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp0_bit[i]     = *ecc_data2 % 2;
		*ecc_data2       = *ecc_data2 / 2;
	}

	for (i = 0; i < 8; i++) {
		comp1_bit[i]     = *(ecc_data2 + 1) % 2;
		*(ecc_data2 + 1) = *(ecc_data2 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp2_bit[i]     = *(ecc_data2 + 2) % 2;
		*(ecc_data2 + 2) = *(ecc_data2 + 2) / 2;
	}

	for (i = 0; i < 6; i++)
		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

	ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
	ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

	for (i = 0; i < 24; i++)
		ecc_sum += ecc_bit[i];

	switch (ecc_sum) {
	case 0:
		/* Not reached because this function is not called if
		 *  ECC values are equal
		 */
		return 0;

	case 1:
		/* Uncorrectable error */
		DEBUG(MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR 1\n");
		return -1;

	case 11:
		/* UN-Correctable error */
		DEBUG(MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR B\n");
		return -1;

	case 12:
		/* Correctable error */
		find_byte = (ecc_bit[23] << 8) +
			    (ecc_bit[21] << 7) +
			    (ecc_bit[19] << 6) +
			    (ecc_bit[17] << 5) +
			    (ecc_bit[15] << 4) +
			    (ecc_bit[13] << 3) +
			    (ecc_bit[11] << 2) +
			    (ecc_bit[9]  << 1) +
			    ecc_bit[7];

		find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];

		DEBUG(MTD_DEBUG_LEVEL0, "Correcting single bit ECC error at "
				"offset: %d, bit: %d\n", find_byte, find_bit);

		page_data[find_byte] ^= (1 << find_bit);

		return 0;
	default:
		if (isEccFF) {
			if (ecc_data2[0] == 0 &&
			    ecc_data2[1] == 0 &&
			    ecc_data2[2] == 0)
				return 0;
		}
		DEBUG(MTD_DEBUG_LEVEL0, "UNCORRECTED_ERROR default\n");
		return -1;
	}
}

/**
 * omap_correct_data - Compares the ECC read with HW generated ECC
 * @mtd: MTD device structure
 * @dat: page data
 * @read_ecc: ecc read from nand flash
 * @calc_ecc: ecc read from HW ECC registers
 *
 * Compares the ecc read from nand spare area with ECC registers values
 * and if ECC's mismached, it will call 'omap_compare_ecc' for error detection
 * and correction.
 */
static int omap_correct_data(struct mtd_info *mtd, u_char *dat,
				u_char *read_ecc, u_char *calc_ecc)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	int blockCnt = 0, i = 0, ret = 0;

	/* Ex NAND_ECC_HW12_2048 */
	if ((info->nand.ecc.mode == NAND_ECC_HW) &&
			(info->nand.ecc.size  == 2048))
		blockCnt = 4;
	else
		blockCnt = 1;

	for (i = 0; i < blockCnt; i++) {
		if (memcmp(read_ecc, calc_ecc, 3) != 0) {
			ret = omap_compare_ecc(read_ecc, calc_ecc, dat);
			if (ret < 0)
				return ret;
		}
		read_ecc += 3;
		calc_ecc += 3;
		dat      += 512;
	}
	return 0;
}

/**
 * omap_calcuate_ecc - Generate non-inverted ECC bytes.
 * @mtd: MTD device structure
 * @dat: The pointer to data on which ecc is computed
 * @ecc_code: The ecc_code buffer
 *
 * Using noninverted ECC can be considered ugly since writing a blank
 * page ie. padding will clear the ECC bytes. This is no problem as long
 * nobody is trying to write data on the seemingly unused page. Reading
 * an erased page will produce an ECC mismatch between generated and read
 * ECC bytes that has to be dealt with separately.
 */
static int omap_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				u_char *ecc_code)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned long val = 0x0;
	unsigned long reg;

	/* Start Reading from HW ECC1_Result = 0x200 */
	reg = (unsigned long)(info->gpmc_baseaddr + GPMC_ECC1_RESULT);
	val = __raw_readl(reg);
	*ecc_code++ = val;          /* P128e, ..., P1e */
	*ecc_code++ = val >> 16;    /* P128o, ..., P1o */
	/* P2048o, P1024o, P512o, P256o, P2048e, P1024e, P512e, P256e */
	*ecc_code++ = ((val >> 8) & 0x0f) | ((val >> 20) & 0xf0);
	reg += 4;

	return 0;
}

/**
 * omap_enable_hwecc - This function enables the hardware ecc functionality
 * @mtd: MTD device structure
 * @mode: Read/Write mode
 */
static void omap_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	struct nand_chip *chip = mtd->priv;
	unsigned int dev_width = (chip->options & NAND_BUSWIDTH_16) ? 1 : 0;
	unsigned long val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_CONFIG);

	switch (mode) {
	case NAND_ECC_READ:
		__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	case NAND_ECC_READSYN:
		 __raw_writel(0x100, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	case NAND_ECC_WRITE:
		__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	default:
		DEBUG(MTD_DEBUG_LEVEL0, "Error: Unrecognized Mode[%d]!\n",
					mode);
		break;
	}

	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_CONFIG);
}

/**
 * omap_wait - wait until the command is done
 * @mtd: MTD device structure
 * @chip: NAND Chip structure
 *
 * Wait function is called during Program and erase operations and
 * the way it is called from MTD layer, we should wait till the NAND
 * chip is ready after the programming/erase operation has completed.
 *
 * Erase can take up to 400ms and program up to 20ms according to
 * general NAND and SmartMedia specs
 */
static int omap_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct nand_chip *this = mtd->priv;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned long timeo = jiffies;
	int status = NAND_STATUS_FAIL, state = this->state;

	if (state == FL_ERASING)
		timeo += (HZ * 400) / 1000;
	else
		timeo += (HZ * 20) / 1000;

	this->IO_ADDR_W = (void *) info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_COMMAND;
	this->IO_ADDR_R = (void *) info->gpmc_cs_baseaddr + GPMC_CS_NAND_DATA;

	__raw_writeb(NAND_CMD_STATUS & 0xFF, this->IO_ADDR_W);

	while (time_before(jiffies, timeo)) {
		status = __raw_readb(this->IO_ADDR_R);
		if (status & NAND_STATUS_READY)
			break;
		cond_resched();
	}
	status = __raw_readb(this->IO_ADDR_R);
	return status;
}

/**
 * omap_dev_ready - calls the platform specific dev_ready function
 * @mtd: MTD device structure
 */
static int omap_dev_ready(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	unsigned int val = __raw_readl(info->gpmc_baseaddr + GPMC_STATUS);
	return val & 0x100;
}

static int __devinit omap_nand_probe(struct platform_device *pdev)
{
	struct omap_nand_info		*info;
	struct omap_nand_platform_data	*pdata;
	int				err;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct omap_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	info->pdev = pdev;

	info->gpmc_cs		= pdata->cs;
	info->gpmc_baseaddr	= pdata->gpmc_baseaddr;
	info->gpmc_cs_baseaddr	= pdata->gpmc_cs_baseaddr;
	info->phys_base		= pdata->phys_base;

	info->mtd.priv		= &info->nand;
	info->mtd.name		= dev_name(&pdev->dev);
	info->mtd.owner		= THIS_MODULE;

	info->nand.options		|= NAND_BUSWIDTH_AUTO;
	//info->nand.options	|= NAND_SKIP_BBTSCAN;

	/* NAND write protect off */
	omap_nand_wp(&info->mtd, NAND_WP_OFF);

	if (!request_mem_region(info->phys_base, NAND_IO_SIZE,
				pdev->dev.driver->name)) {
		err = -EBUSY;
		goto out_free_info;
	}

	info->nand.IO_ADDR_R = ioremap(info->phys_base, NAND_IO_SIZE);
	if (!info->nand.IO_ADDR_R) {
		err = -ENOMEM;
		goto out_release_mem_region;
	}

	info->nand.controller = &info->controller;

	info->nand.IO_ADDR_W = info->nand.IO_ADDR_R;
	info->nand.cmd_ctrl  = omap_hwcontrol;

	/*
	 * If RDY/BSY line is connected to OMAP then use the omap ready
	 * funcrtion and the generic nand_wait function which reads the status
	 * register after monitoring the RDY/BSY line.Otherwise use a standard
	 * chip delay which is slightly more than tR (AC Timing) of the NAND
	 * device and read status register until you get a failure or success
	 */
	if (pdata->dev_ready) {
		info->nand.dev_ready = omap_dev_ready;
		info->nand.chip_delay = 0;
	} else {
		info->nand.waitfunc = omap_wait;
		info->nand.chip_delay = 50;
	}

	if (use_prefetch) {
		dev_info( &pdev->dev, "Use prefetch\n");
		/* copy the virtual address of nand base for fifo access */
		info->nand_pref_fifo_add = info->nand.IO_ADDR_R;

		info->nand.read_buf   = omap_read_buf_pref;
		info->nand.write_buf  = omap_write_buf_pref;
		if (use_dma) {
			err = omap_request_dma(OMAP24XX_DMA_GPMC, "NAND",
				omap_nand_dma_cb, &info->comp, &info->dma_ch);
			if (err < 0) {
				info->dma_ch = -1;
				printk(KERN_WARNING "DMA request failed."
					" Non-dma data transfer mode\n");
			} else {
				omap_set_dma_dest_burst_mode(info->dma_ch,
						OMAP_DMA_DATA_BURST_16);
				omap_set_dma_src_burst_mode(info->dma_ch,
						OMAP_DMA_DATA_BURST_16);

				info->nand.read_buf   = omap_read_buf_dma_pref;
				info->nand.write_buf  = omap_write_buf_dma_pref;
			}
		}
	} else {
		info->nand.read_buf   = omap_read_buf8;
		info->nand.write_buf  = omap_write_buf8;
	}
	info->nand.verify_buf = omap_verify_buf;

	/* default to mode 0 for init */
	omap_onfi_set(&info->mtd, 0);

	if (nand_scan_ident(&info->mtd, 1, NULL)) {
		err = -ENXIO;
		goto out_release_mem_region;
	}
	info->nand.options		|= NAND_NO_SUBPAGE_WRITE;

	if (pdata->ecc_opt & 0x3) {
		info->nand.ecc.bytes            = 3;
		info->nand.ecc.size             = 512;
		if (info->nand.options & NAND_BUSWIDTH_16) {
			info->nand.ecc.layout   = &nand_x16_hw_romcode_oob_64;
		} else {
			info->nand.ecc.layout   = &nand_x8_hw_romcode_oob_64;
			info->nand.badblock_pattern = &bb_descrip_flashbased;
		}
		info->nand.ecc.calculate        = omap_calculate_ecc;
		info->nand.ecc.hwctl            = omap_enable_hwecc;
		info->nand.ecc.correct          = omap_correct_data;
		info->nand.ecc.mode             = NAND_ECC_HW;
		/* init HW ECC */
		omap_hwecc_init(&info->mtd);
		/* FIXME: use ecc_opt to configure bch correction */
#ifdef CONFIG_MTD_NAND_OMAP_BCH
		{
#ifdef CONFIG_MTD_NAND_OMAP_BCH_8
			const int max_errors = 8, error_threshold = 3;
#else
			const int max_errors = 4, error_threshold = 2;
#endif
			if (omap3_bch_init(&info->mtd, max_errors,
					   error_threshold)) {
				err = -ENOMEM;
				goto out_release_mem_region;
			}
		}
#endif
	} else {
		info->nand.ecc.mode = NAND_ECC_SOFT;
	}

	/* update for 16 bits device */
	if (info->nand.options & NAND_BUSWIDTH_16) {
		/* update bus width */
		u32 cfg1 = gpmc_cs_read_reg(info->gpmc_cs, GPMC_CS_CONFIG1);
		cfg1 |= GPMC_CONFIG1_DEVICESIZE(1);
		gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG1, cfg1);

		if (!use_prefetch) {
			info->nand.read_buf   = omap_read_buf16;
			info->nand.write_buf  = omap_write_buf16;
		}
	}

	if (info->nand.onfi_speed >= 0)
		omap_onfi_set(&info->mtd, info->nand.onfi_speed);

	/* second phase scan */
	if (nand_scan_tail(&info->mtd)) {
		err = -ENXIO;
		goto out_release_mem_region;
	}

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		add_mtd_device(&info->mtd);

	platform_set_drvdata(pdev, &info->mtd);

	return 0;

out_release_mem_region:
	release_mem_region(info->phys_base, NAND_IO_SIZE);
out_free_info:
	kfree(info);

	return err;
}

static int omap_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	platform_set_drvdata(pdev, NULL);
	if (use_dma)
		omap_free_dma(info->dma_ch);

#ifdef CONFIG_MTD_NAND_OMAP_BCH
	omap3_bch_free(&info->mtd);
#endif

	/* Release NAND device, its internal structures and partitions */
	nand_release(&info->mtd);
	iounmap(info->nand_pref_fifo_add);
	kfree(&info->mtd);
	return 0;
}

static struct platform_driver omap_nand_driver = {
	.probe		= omap_nand_probe,
	.remove		= omap_nand_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init omap_nand_init(void)
{
	printk(KERN_INFO "%s driver initializing\n", DRIVER_NAME);

	/* This check is required if driver is being
	 * loaded run time as a module
	 */
	if ((1 == use_dma) && (0 == use_prefetch)) {
		printk(KERN_INFO"Wrong parameters: 'use_dma' can not be 1 "
				"without use_prefetch'. Prefetch will not be"
				" used in either mode (mpu or dma)\n");
	}
	return platform_driver_register(&omap_nand_driver);
}

static void __exit omap_nand_exit(void)
{
	platform_driver_unregister(&omap_nand_driver);
}

module_init(omap_nand_init);
module_exit(omap_nand_exit);

#ifdef CONFIG_MTD_NAND_OMAP_BCH

/*
 * OMAP3 hardware BCH ecc correction (4 or 8 bits)
 *
 * Copyright � 2011 Ivan Djelic <ivan.djelic@parrot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define ECC_MAX_CORRECTION      8

/**
 * omap_hwecc_init - Initialize the HW ECC for NAND flash in GPMC controller
 * @mtd: MTD device structure
 */
static void omap3_bch_hwecc_init(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
						   mtd);
	/* clear all ECC | enable Reg1 */
	__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
}

/**
 * omap3_bch_calculate_ecc8 - Generate 13 ecc bytes per block of 512 data bytes
 * @mtd: MTD device structure
 * @dat: The pointer to data on which ecc is computed
 * @ecc: The ecc output buffer
 */
static int omap3_bch_calculate_ecc8(struct mtd_info *mtd, const u_char *dat,
				    u_char *ecc)
{
	int i;
	unsigned long reg, val1, val2, val3, val4;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
						   mtd);

	for (i = 0; i < info->nand.ecc.size/512; i++) {

		reg = (unsigned long)(info->gpmc_baseaddr +
				      GPMC_ECC_BCH_RESULT_0 + (0x10 * i));

		/* read hw-computed remainder */
		val1 = __raw_readl(reg);
		val2 = __raw_readl(reg + 4);
		val3 = __raw_readl(reg + 8);
		val4 = __raw_readl(reg + 12);

		/* get ecc and apply empty-page inversion mask */
		*ecc++ = 0xef ^ (val4 & 0xFF);
		*ecc++ = 0x51 ^ ((val3 >> 24) & 0xFF);
		*ecc++ = 0x2e ^ ((val3 >> 16) & 0xFF);
		*ecc++ = 0x09 ^ ((val3 >> 8) & 0xFF);
		*ecc++ = 0xed ^ (val3 & 0xFF);
		*ecc++ = 0x93 ^ ((val2 >> 24) & 0xFF);
		*ecc++ = 0x9a ^ ((val2 >> 16) & 0xFF);
		*ecc++ = 0xc2 ^ ((val2 >> 8) & 0xFF);
		*ecc++ = 0x97 ^ (val2 & 0xFF);
		*ecc++ = 0x79 ^ ((val1 >> 24) & 0xFF);
		*ecc++ = 0xe5 ^ ((val1 >> 16) & 0xFF);
		*ecc++ = 0x24 ^ ((val1 >> 8) & 0xFF);
		*ecc++ = 0xb5 ^ (val1 & 0xFF);
	}
	return 0;
}

/**
 * omap3_bch_calculate_ecc4 - Generate 7 ecc bytes per block of 512 data bytes
 * @mtd: MTD device structure
 * @dat: The pointer to data on which ecc is computed
 * @ecc: The ecc output buffer
 */
static int omap3_bch_calculate_ecc4(struct mtd_info *mtd, const u_char *dat,
				    u_char *ecc)
{
	int i;
	unsigned long reg, val1, val2;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
						   mtd);

	for (i = 0; i < info->nand.ecc.size/512; i++) {

		reg = (unsigned long)(info->gpmc_baseaddr +
				      GPMC_ECC_BCH_RESULT_0 + (0x10 * i));

		/* read hw-computed remainder */
		val1 = __raw_readl(reg);
		val2 = __raw_readl(reg + 4);

		/* get left-justified ecc and apply empty-page inversion mask */
		*ecc++ = 0x28 ^ ((val2 >> 12) & 0xFF);
		*ecc++ = 0x13 ^ ((val2 >>  4) & 0xFF);
		*ecc++ = 0xcc ^ (((val2 & 0xF) << 4)|((val1 >> 28) & 0xF));
		*ecc++ = 0x39 ^ ((val1 >> 20) & 0xFF);
		*ecc++ = 0x96 ^ ((val1 >> 12) & 0xFF);
		*ecc++ = 0xac ^ ((val1 >> 4) & 0xFF);
		*ecc++ = 0x7f ^ ((val1 & 0xF) << 4);
	}
	return 0;
}

static void omap3_bch_correct_block(struct omap_nand_info *info,
				    u_char *data, u_char *calc_ecc, int *ret)
{
	int i, count;
	unsigned int errloc[ECC_MAX_CORRECTION];

	count = decode_bch(info->bch, NULL, 512, NULL, calc_ecc, NULL, errloc);
	if (count > 0) {
		/* correct errors */
		for (i = 0; i < count; i++) {
			/* correct data only, not ecc bytes */
			if (errloc[i] < 8*512)
				data[errloc[i]/8] ^= 1 << (errloc[i] & 7);

			DEBUG(MTD_DEBUG_LEVEL0, "%s: corrected bitflip %u\n",
			      __func__, errloc[i]);
		}
		/*
		 * FIXME: in order to prevent upper layers (such as UBI) from
		 * torturing and marking a block as bad as soon as 1 bitflip
		 * is persistent, we implement a threshold below which errors
		 * are corrected but not reported. Instead, mtd should provide
		 * a generic way to handle this situation.
		 */
		if (count < info->error_threshold) {
			DEBUG(MTD_DEBUG_LEVEL0, "%s: concealing %d errors "
			      "below threshold of %u\n", __func__, count,
			      info->error_threshold);
			count = 0;
		}

		/* accumulate errors unless a failure occured */
		if ((*ret) >= 0)
			(*ret) += count;

	} else if (count < 0) {
		(*ret) = -1;
		printk(KERN_ERR "ecc unrecoverable error\n");
	}
}

/**
 * omap3_bch_correct_data - Decode received data and correct errors
 * @mtd: MTD device structure
 * @data: page data
 * @read_ecc: ecc read from nand flash
 * @calc_ecc: ecc read from HW ECC registers
 */
static int omap3_bch_correct_data(struct mtd_info *mtd, u_char *data,
				  u_char *read_ecc, u_char *calc_ecc)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
						   mtd);
	int i, j, ret = 0, eccbytes, eccflag, steps;

	steps = info->nand.ecc.size/512;
	eccbytes = info->nand.ecc.bytes/steps;

	for (i = 0; i < steps; i++) {

		/* compare read and calculated ecc */
		for (j = 0, eccflag = 0; j < eccbytes; j++) {
			calc_ecc[j] ^= read_ecc[j];
			eccflag |= calc_ecc[j];
		}
		if (eccflag)
			/* an error was detected, perform correction */
			omap3_bch_correct_block(info, data, calc_ecc, &ret);

		calc_ecc += eccbytes;
		read_ecc += eccbytes;
		data     += 512;
	}
	return ret;
}

static void omap3_bch_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
						   mtd);
	register struct nand_chip *chip = mtd->priv;
	unsigned int dev_width;
	unsigned int bch_mod, ecc_conf, ecc_size_conf;

	dev_width = (chip->options & NAND_BUSWIDTH_16)? 1 : 0;
	bch_mod = (info->nand.ecc.bytes == 52)? 1 : 0;

	/* ECCSIZE1=32 | ECCSIZE0=00 */
	ecc_size_conf = (0x20 << 22) | (0x00 << 12);

	ecc_conf = ((0x01          << 16) | /* BCH */
		    (bch_mod       << 12) | /* 8 or 4 bits */
		    (0x06          <<  8) | /* wrap mode = 6 */
		    (dev_width     <<  7) | /* bus width */
		    (0x03          <<  4) | /* 4 sectors */
		    (info->gpmc_cs <<  1) | /* ECC CS */
		    (0x1));                 /* enable ECC */

	__raw_writel(0x1, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
	__raw_writel(ecc_size_conf, info->gpmc_baseaddr + GPMC_ECC_SIZE_CONFIG);
	__raw_writel(ecc_conf, info->gpmc_baseaddr + GPMC_ECC_CONFIG);
	__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
}


int omap3_bch_init(struct mtd_info *mtd, int max_errors, int error_threshold)
{
	int i, oobsize;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
						   mtd);
	struct nand_ecclayout *layout = &info->ecclayout;

#ifdef CONFIG_MTD_NAND_OMAP_BCH_8
	const int hw_errors = 8;
#else
	const int hw_errors = 4;
#endif
	info->bch = NULL;

	/* sanity check: OMAP3 only supports 4 and 8 bits correction */
	if (max_errors != hw_errors) {
		printk(KERN_ERR "unsupported bch correction capability %d\n",
		       max_errors);
		goto fail;
	}
	info->error_threshold = error_threshold;

	/* configure software bch library, only used for error decoding */
	info->bch = init_bch(13, max_errors, 0x201b);
	if (!info->bch)
		goto fail;

	/* process 2048 bytes at a time */
	info->nand.ecc.size    = 2048;
	info->nand.ecc.hwctl   = omap3_bch_enable_hwecc;
	info->nand.ecc.correct = omap3_bch_correct_data;
	info->nand.ecc.mode    = NAND_ECC_HW;
	if (max_errors == 8) {
		info->nand.ecc.bytes     = 52; /* 4 x 13 bytes */
		info->nand.ecc.calculate = omap3_bch_calculate_ecc8;
	} else {
		info->nand.ecc.bytes     = 28; /* 4 x 7 bytes */
		info->nand.ecc.calculate = omap3_bch_calculate_ecc4;
	}
	/* build oob layout */
	layout->eccbytes = info->nand.ecc.bytes;

	/* FIXME: ecc configuration is done before nand_scan... */
	oobsize = mtd->oobsize ? mtd->oobsize : 64;

	/* reserve 2 bytes for bad block marker */
	if (layout->eccbytes+2 > oobsize) {
		printk(KERN_WARNING "no suitable oob scheme available "
		       "for oobsize %d eccbytes %u\n", oobsize,
		       layout->eccbytes);
		goto fail;
	}
	/* put ecc bytes at oob tail */
	for (i = 0; i < layout->eccbytes; i++)
		layout->eccpos[i] = oobsize-layout->eccbytes+i;

	layout->oobfree[0].offset = 2;
	layout->oobfree[0].length = oobsize-2-layout->eccbytes;
	info->nand.ecc.layout = layout;

	if (!(info->nand.options & NAND_BUSWIDTH_16))
		info->nand.badblock_pattern = &bb_descrip_flashbased;

	printk(KERN_INFO "enabling hardware nand bch ecc, %d errors max, "
	       "threshold %d\n", max_errors, error_threshold);

	omap3_bch_hwecc_init(mtd);
	return 0;
fail:
	omap3_bch_free(mtd);
	return -1;
}

void omap3_bch_free(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
						   mtd);
	if (info->bch) {
		free_bch(info->bch);
		info->bch = NULL;
	}
}

#endif /* CONFIG_MTD_NAND_OMAP_BCH */


//! Update hardware configuration after device geometry has been queried
static int omap_onfi_set(struct mtd_info* mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);

    const struct reduced_onfi *tn = &(nand_timing[mode]);
	int tmp;
	struct gpmc_timings t;

	printk(KERN_DEBUG "setting onfi mode %d\n", mode);


	if (!cpu_is_omap34xx()) {
		/* wr_access is shared with access */
		return -EINVAL;
	}

	memset(&t, 0, sizeof(t));

	/* all signal start at the same time :
	   we could delay nRE, nWE, but this won't 
	   work with prefetcher optimisation...
	 */

	t.cs_on = 0;
	t.adv_on = 0;
	t.oe_on = 0;
	t.we_on = 0;

	tmp = gpmc_round_ns_to_ticks(tn->twp);
	/* nCS low to nWE high */
	t.we_off = gpmc_round_ns_to_ticks(tn->twsetup) + tmp;

	/* nCS low to end */
	t.wr_cycle = t.we_off + max(tn->twh, tn->twc - tmp);

	/* BCH need 4 cycles */
	if (gpmc_ns_to_ticks(t.wr_cycle) < 4)
		t.wr_cycle = gpmc_ticks_to_ns(4);
	else if (gpmc_ns_to_ticks(t.wr_cycle) > 0x1f)
		t.wr_cycle = gpmc_ticks_to_ns(0x1f);

	t.cs_wr_off = t.wr_cycle;
	t.adv_wr_off = t.wr_cycle;
	t.wr_access = t.we_off;

	tmp = gpmc_round_ns_to_ticks(tn->trp);
	/* nCS low to nRE high */
	t.oe_off = gpmc_round_ns_to_ticks(tn->trsetup) + tmp;

	/* nCS low to end */
	t.rd_cycle = t.oe_off + max(tn->treh, tn->trc - tmp);;

	/* BCH need 4 cycles */
	if (gpmc_ns_to_ticks(t.rd_cycle) < 4)
		t.rd_cycle = gpmc_ticks_to_ns(4);
	else if (gpmc_ns_to_ticks(t.rd_cycle) > 0x1f)
		t.rd_cycle = gpmc_ticks_to_ns(0x1f);

	t.cs_rd_off = t.rd_cycle;
	t.adv_rd_off = t.rd_cycle; /* not used */
	if (tn->edo)
		t.access = t.rd_cycle;
	else
		t.access = t.oe_off;

	t.page_burst_access = 0;  /* not used */
	t.wr_data_mux_bus = 0; /* not used not in MUXADDDATA mode */

	/* we often overflow here ... */
	tmp = gpmc_ns_to_ticks(tn->bta);
	if (tmp > 0xf)
		tmp = 0xf;
	t.busturnaround = gpmc_ticks_to_ns(tmp);

	tmp = gpmc_ns_to_ticks(tn->twhr);
	if (tmp > 0xf)
		tmp = 0xf;
	t.cycle2cycledelay = gpmc_ticks_to_ns(tmp);

	/*
	   XXX tbusy is not configurable
	   trm is not clear how much the gpmc wait between WAIT high and read.
	   But the linux driver doesn't use SYNCHROMODE in GPMC_PREFETCH_CONFIG1,
	   so we should be safe
	 */
	printk("read %d %d\n", t.oe_off, t.rd_cycle);
	printk("read %d %d\n", gpmc_ns_to_ticks(t.oe_off), gpmc_ns_to_ticks(t.rd_cycle));
	printk("write %d %d\n", t.we_off, t.wr_cycle);
	printk("write %d %d\n", gpmc_ns_to_ticks(t.we_off), gpmc_ns_to_ticks(t.wr_cycle));
	/* make sure timming register got sane default */
	gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG2, 0);
	gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG3, 0);
	gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG4, 0);
	gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG5, 0);
	gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG6, (1<<31) | (1<<7));
	gpmc_cs_set_timings(info->gpmc_cs, &t);

	/* for burst we can remove twsetup/trsetup, but we should
	   make sure cycle is not less than 4 (for bch)

	   This number is the cycles to be
	   subtracted from RdCycleTime, WrCycleTime,
	   AccessTime, CSRdOffTime, CSWrOffTime,
	   ADVRdOffTime, ADVWrOffTime, OEOffTime, WEOffTime
	 */
	optim_wr = min(gpmc_ns_to_ticks(t.wr_cycle) - 4,
			gpmc_ns_to_ticks(tn->twsetup));
	optim_rd = min(gpmc_ns_to_ticks(t.rd_cycle) - 4,
			gpmc_ns_to_ticks(tn->trsetup));
	
	return 0;
}

MODULE_ALIAS(DRIVER_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Glue layer for NAND flash on TI OMAP boards");

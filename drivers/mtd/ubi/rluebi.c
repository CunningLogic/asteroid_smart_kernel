/*
 * Copyright (c) International Business Machines Corp., 2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: Artem Bityutskiy (Битюцкий Артём), Joern Engel
 */

/*
 * This is a small driver which implements fake MTD devices on top of UBI
 * volumes. This sounds strange, but it is in fact quite useful to make
 * MTD-oriented software (including all the legacy software) work on top of
 * UBI.
 *
 * Gluebi emulates MTD devices of "MTD_UBIVOLUME" type. Their minimal I/O unit
 * size (@mtd->writesize) is equivalent to the UBI minimal I/O unit. The
 * eraseblock size is equivalent to the logical eraseblock size of the volume.
 */

#include <linux/err.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/ubi.h>
#include <linux/mtd/mtd.h>
#include "ubi-media.h"

#define err_msg(fmt, ...)                                   \
	printk(KERN_DEBUG "gluebi (pid %d): %s: " fmt "\n", \
	       current->pid, __func__, ##__VA_ARGS__)

/**
 * struct gluebi_device - a gluebi device description data structure.
 * @mtd: emulated MTD device description object
 * @refcnt: gluebi device reference count
 * @desc: UBI volume descriptor
 * @ubi_num: UBI device number this gluebi device works on
 * @vol_id: ID of UBI volume this gluebi device works on
 * @list: link in a list of gluebi devices
 */
struct gluebi_device {
	struct mtd_info mtd;
	int refcnt;
	struct ubi_volume_desc *desc;
	int ubi_num;
	int vol_id;
	struct list_head list;
	int locked;
};

/* List of all gluebi devices */
static LIST_HEAD(gluebi_devices);
static DEFINE_MUTEX(devices_mutex);

static struct gluebi_device *g_gluebi;
static struct mtd_info *g_mtd;
/**
 * find_gluebi_nolock - find a gluebi device.
 * @ubi_num: UBI device number
 * @vol_id: volume ID
 *
 * This function seraches for gluebi device corresponding to UBI device
 * @ubi_num and UBI volume @vol_id. Returns the gluebi device description
 * object in case of success and %NULL in case of failure. The caller has to
 * have the &devices_mutex locked.
 */
static struct gluebi_device *find_gluebi_nolock(int ubi_num, int vol_id)
{
	struct gluebi_device *gluebi;

	list_for_each_entry(gluebi, &gluebi_devices, list)
		if (gluebi->ubi_num == ubi_num && gluebi->vol_id == vol_id)
			return gluebi;
	return NULL;
}

/**
 * gluebi_get_device - get MTD device reference.
 * @mtd: the MTD device description object
 *
 * This function is called every time the MTD device is being opened and
 * implements the MTD get_device() operation. Returns zero in case of success
 * and a negative error code in case of failure.
 */
static int gluebi_get_device(struct mtd_info *mtd)
{
	struct gluebi_device *gluebi;
	int ubi_mode = UBI_READONLY;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	if (mtd->flags & MTD_WRITEABLE) {
		printk("want a rw mtd !!!!\n");
		return -EINVAL;
	}

	gluebi = container_of(mtd, struct gluebi_device, mtd);
	mutex_lock(&devices_mutex);
	if (gluebi->refcnt > 0) {
		/*
		 * The MTD device is already referenced and this is just one
		 * more reference. MTD allows many users to open the same
		 * volume simultaneously and do not distinguish between
		 * readers/writers/exclusive openers as UBI does. So we do not
		 * open the UBI volume again - just increase the reference
		 * counter and return.
		 */
		gluebi->refcnt += 1;
		mutex_unlock(&devices_mutex);
		return 0;
	}

	/*
	 * This is the first reference to this UBI volume via the MTD device
	 * interface. Open the corresponding volume in read-write mode.
	 */
	gluebi->desc = ubi_open_volume(gluebi->ubi_num, gluebi->vol_id,
				       ubi_mode);
	if (IS_ERR(gluebi->desc)) {
		mutex_unlock(&devices_mutex);
		module_put(THIS_MODULE);
		return PTR_ERR(gluebi->desc);
	}
	gluebi->refcnt += 1;
	mutex_unlock(&devices_mutex);
	return 0;
}

/**
 * gluebi_put_device - put MTD device reference.
 * @mtd: the MTD device description object
 *
 * This function is called every time the MTD device is being put. Returns
 * zero in case of success and a negative error code in case of failure.
 */
static void gluebi_put_device(struct mtd_info *mtd)
{
	struct gluebi_device *gluebi;

	gluebi = container_of(mtd, struct gluebi_device, mtd);
	mutex_lock(&devices_mutex);
	gluebi->refcnt -= 1;
	if (gluebi->refcnt == 0)
		ubi_close_volume(gluebi->desc);
	module_put(THIS_MODULE);
	mutex_unlock(&devices_mutex);
}

/**
 * gluebi_read - read operation of emulated MTD devices.
 * @mtd: MTD device description object
 * @from: absolute offset from where to read
 * @len: how many bytes to read
 * @retlen: count of read bytes is returned here
 * @buf: buffer to store the read data
 *
 * This function returns zero in case of success and a negative error code in
 * case of failure.
 */
static int gluebi_read(struct mtd_info *mtd, loff_t from, size_t len,
		       size_t *retlen, unsigned char *buf)
{
	int err = 0, lnum, offs, total_read;
	struct gluebi_device *gluebi;

	if (len < 0 || from < 0 || from + len > mtd->size)
		return -EINVAL;

	gluebi = container_of(mtd, struct gluebi_device, mtd);

	if (gluebi->locked)
		return -EINVAL;

	lnum = div_u64_rem(from, mtd->erasesize, &offs);
	total_read = len;
	while (total_read) {
		size_t to_read = mtd->erasesize - offs;

		if (to_read > total_read)
			to_read = total_read;

		err = ubi_read(gluebi->desc, lnum, buf, offs, to_read);
		if (err)
			break;

		lnum += 1;
		offs = 0;
		total_read -= to_read;
		buf += to_read;
	}

	*retlen = len - total_read;
	return err;
}

/**
 * gluebi_write - write operation of emulated MTD devices.
 * @mtd: MTD device description object
 * @to: absolute offset where to write
 * @len: how many bytes to write
 * @retlen: count of written bytes is returned here
 * @buf: buffer with data to write
 *
 * This function returns zero in case of success and a negative error code in
 * case of failure.
 */
static int gluebi_write(struct mtd_info *mtd, loff_t to, size_t len,
			size_t *retlen, const u_char *buf)
{
	return -EROFS;
}

/**
 * gluebi_erase - erase operation of emulated MTD devices.
 * @mtd: the MTD device description object
 * @instr: the erase operation description
 *
 * This function calls the erase callback when finishes. Returns zero in case
 * of success and a negative error code in case of failure.
 */
static int gluebi_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	return -EROFS;
}

/**
 * gluebi_create - create a gluebi device for an UBI volume.
 * @di: UBI device description object
 * @vi: UBI volume description object
 *
 * This function is called when a new UBI volume is created in order to create
 * corresponding fake MTD device. Returns zero in case of success and a
 * negative error code in case of failure.
 */
static int gluebi_create(struct ubi_device_info *di,
			 struct ubi_volume_info *vi)
{
	struct gluebi_device *gluebi, *g;
	struct mtd_info *mtd;

	gluebi = kzalloc(sizeof(struct gluebi_device), GFP_KERNEL);
	if (!gluebi)
		return -ENOMEM;

	g_gluebi = gluebi;

	mtd = &gluebi->mtd;
	mtd->name = kmemdup(vi->name, vi->name_len + 1, GFP_KERNEL);
	if (!mtd->name) {
		kfree(gluebi);
		return -ENOMEM;
	}

	gluebi->vol_id = vi->vol_id;
	gluebi->ubi_num = vi->ubi_num;
	mtd->type = MTD_UBIVOLUME;
	//if (!di->ro_mode)
	//	mtd->flags = MTD_WRITEABLE;
	mtd->owner      = THIS_MODULE;
	mtd->writesize  = di->min_io_size;
	mtd->erasesize  = vi->usable_leb_size;
	mtd->read       = gluebi_read;
	mtd->write      = gluebi_write;
	mtd->erase      = gluebi_erase;
	mtd->get_device = gluebi_get_device;
	mtd->put_device = gluebi_put_device;

	//
	//mtd->suspend;
	//mtd->resume;

	/*
	 * In case of dynamic a volume, MTD device size is just volume size. In
	 * case of a static volume the size is equivalent to the amount of data
	 * bytes.
	 */
	if (vi->vol_type == UBI_DYNAMIC_VOLUME)
		mtd->size = (unsigned long long)vi->usable_leb_size * vi->size;
	else
		mtd->size = vi->used_bytes;

	/* Just a sanity check - make sure this gluebi device does not exist */
	mutex_lock(&devices_mutex);
	g = find_gluebi_nolock(vi->ubi_num, vi->vol_id);
	if (g)
		err_msg("gluebi MTD device %d form UBI device %d volume %d "
			"already exists", g->mtd.index, vi->ubi_num,
			vi->vol_id);
	mutex_unlock(&devices_mutex);

	if (add_mtd_device(mtd)) {
		err_msg("cannot add MTD device");
		kfree(mtd->name);
		kfree(gluebi);
		return -ENFILE;
	}

	mutex_lock(&devices_mutex);
	list_add_tail(&gluebi->list, &gluebi_devices);
	mutex_unlock(&devices_mutex);
	return 0;
}

/**
 * gluebi_remove - remove a gluebi device.
 * @vi: UBI volume description object
 *
 * This function is called when an UBI volume is removed and it removes
 * corresponding fake MTD device. Returns zero in case of success and a
 * negative error code in case of failure.
 */
static int gluebi_remove(struct ubi_volume_info *vi)
{
	int err = 0;
	struct mtd_info *mtd;
	struct gluebi_device *gluebi;

	mutex_lock(&devices_mutex);
	gluebi = find_gluebi_nolock(vi->ubi_num, vi->vol_id);
	if (!gluebi) {
		err_msg("got remove notification for unknown UBI device %d "
			"volume %d", vi->ubi_num, vi->vol_id);
		err = -ENOENT;
	} else if (gluebi->refcnt)
		err = -EBUSY;
	else
		list_del(&gluebi->list);
	mutex_unlock(&devices_mutex);
	if (err)
		return err;

	mtd = &gluebi->mtd;
	err = del_mtd_device(mtd);
	if (err) {
		err_msg("cannot remove fake MTD device %d, UBI device %d, "
			"volume %d, error %d", mtd->index, gluebi->ubi_num,
			gluebi->vol_id, err);
		mutex_lock(&devices_mutex);
		list_add_tail(&gluebi->list, &gluebi_devices);
		mutex_unlock(&devices_mutex);
		return err;
	}

	kfree(mtd->name);
	kfree(gluebi);
	return 0;
}

/**
 * gluebi_notify - UBI notification handler.
 * @nb: registered notifier block
 * @l: notification type
 * @ptr: pointer to the &struct ubi_notification object
 */
static int gluebi_notify(struct notifier_block *nb, unsigned long l,
			 void *ns_ptr)
{
	struct ubi_notification *nt = ns_ptr;

	printk("ubi notify %d for %s\n", (int)l, nt->vi.name);

	if (strcmp(nt->vi.name, "system") != 0)
		return NOTIFY_OK;

	/* in suspend skip */
	if (g_mtd)
		return NOTIFY_OK;

	switch (l) {
	case UBI_VOLUME_ADDED:
		gluebi_create(&nt->di, &nt->vi);
		break;
	case UBI_VOLUME_REMOVED:
		gluebi_remove(&nt->vi);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block gluebi_notifier = {
	.notifier_call	= gluebi_notify,
};

static int __init ubi_gluebi_init(void)
{
	printk("rlubi init\n");
	return ubi_register_volume_notifier(&gluebi_notifier, 0);
}

static void __exit ubi_gluebi_exit(void)
{
	struct gluebi_device *gluebi, *g;

	list_for_each_entry_safe(gluebi, g, &gluebi_devices, list) {
		int err;
		struct mtd_info *mtd = &gluebi->mtd;

		err = del_mtd_device(mtd);
		if (err)
			err_msg("error %d while removing gluebi MTD device %d, "
				"UBI device %d, volume %d - ignoring", err,
				mtd->index, gluebi->ubi_num, gluebi->vol_id);
		kfree(mtd->name);
		kfree(gluebi);
	}
	ubi_unregister_volume_notifier(&gluebi_notifier);
}

module_init(ubi_gluebi_init);
module_exit(ubi_gluebi_exit);
MODULE_DESCRIPTION("MTD emulation layer over UBI volumes");
MODULE_AUTHOR("Artem Bityutskiy, Joern Engel");
MODULE_LICENSE("GPL");

#include "ubi.h"

int rluebi_suspend(void)
{
	struct gluebi_device *gluebi = g_gluebi;
	if (g_gluebi == NULL)
		return 0;

	mutex_lock(&devices_mutex);
	gluebi->locked = 1;
	/* save mtd info pointer */
	g_mtd = gluebi->desc->vol->ubi->mtd;
	/* close our volume : no error, ref counting ... */
	ubi_close_volume(gluebi->desc);
	/* detach ubi : warning this wait thread end, and suppose
	   that kernel thread are not frozen
	 */
	return ubi_detach_mtd_dev(gluebi->ubi_num, 0);
}

int rluebi_resume(void)
{
	struct gluebi_device *gluebi = g_gluebi;
	int ret;

	if (g_gluebi == NULL || g_mtd == NULL)
		return 0;

	ret = ubi_attach_mtd_dev(g_mtd, gluebi->ubi_num, 2048);
	if (ret < 0)
		return ret;
	g_mtd = NULL;
	gluebi->desc = ubi_open_volume(gluebi->ubi_num, gluebi->vol_id,
				       UBI_READONLY);
	if (IS_ERR(gluebi->desc)) {
		return PTR_ERR(gluebi->desc);
	}

	gluebi->locked = 0;
	mutex_unlock(&devices_mutex);

	return 0;
}

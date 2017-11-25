/*

 * drivers/staging/android/ion/ion_zte_fragment_opt.c
 *
 * Copyright (C) 2017 ZTE, Inc.
 * Copyright (c) 2017-2017, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <linux/memblock.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/idr.h>
#include <linux/msm_ion.h>
#include <linux/msm_dma_iommu_mapping.h>
#include <trace/events/kmem.h>
#include <linux/module.h>

#include "ion.h"
#include "ion_priv.h"
#include "compat_ion.h"
#include "ion_zte_fragment_opt_priv.h"

#define K_TO_BYTES 1024
/* DEN is for denominator */
#define K_TO_M_DEN 1024
#define BYTES_TO_K_DEN 1024

#define ZTE_ION_MAX_ORDERS MAX_ORDER

/* only 4 kinds of order used, expected to be in ascending order
  * should be equal to the num_orders in ion_system_heap */
#define ORDER_USED_NUM 4

/* seq should be consistent with the index in orders[] above to
* represent how many page blocks in each order were dedicated.
* high low are not distinguished, so shrink is tightly avoided.
* The names are for ion system dedicated vars */
struct ion_sys_count_type { /* order 9 corresponding to index 0 */
	unsigned int cached_count[ORDER_USED_NUM]; /* for each order*/
	unsigned int uncached_count[ORDER_USED_NUM];
};

static int inner_ion_zte_initialized;
int ion_zte_fragment_opt_initialized(void)
{
	return inner_ion_zte_initialized;
}

enum {
	ION_SYS_DEBUG_USER_MAX = 0x1,
	ION_SYS_DEBUG_SHRINK = 0x2,
};

/* global data, take effect immediately */
static int ion_sys_debug_mask;
module_param_named(
	debug_mask, ion_sys_debug_mask, int,
	S_IRUGO | S_IWUSR | S_IWGRP
);
/**
 * debug print control for shrink operation.
 */
int zte_ion_sys_debug_shrink(void)
{
	return ion_sys_debug_mask & ION_SYS_DEBUG_SHRINK;
}
/**
 * debug print control for max info.
 * no need to be controlled under the specific micros.
 */
int zte_ion_sys_debug_user_max(void)
{
	return ion_sys_debug_mask & ION_SYS_DEBUG_USER_MAX;
}

/* how many scatterlist(=sg) item used in the specific sized block, 4K, 8K, ... , 4M,
   and the [11] is for unexpected orders */
static unsigned int ion_buffer_order_count[ZTE_ION_MAX_ORDERS+1];
static unsigned int ion_buffer_order_size[ZTE_ION_MAX_ORDERS] = { PAGE_SIZE << 0,
	PAGE_SIZE << 1, PAGE_SIZE << 2, PAGE_SIZE << 3, PAGE_SIZE << 4, PAGE_SIZE << 5,
	PAGE_SIZE << 6, PAGE_SIZE << 7, PAGE_SIZE << 8, PAGE_SIZE << 9, PAGE_SIZE << 10
};
static unsigned int ion_buffer_sg_num;

/* vars for largest ion buffer stats */
static unsigned int ion_buffer_largest_size;
static unsigned int ion_buffer_largest_num; /* how many ion buffers */
/* corresponding to the same desc as above */
static unsigned int ion_buffer_largest_order_count[ZTE_ION_MAX_ORDERS+1];
static unsigned int ion_buffer_largest_sg_count; /* the total sg count including all orders */

static const char *ion_buffer_order_desc[ZTE_ION_MAX_ORDERS+1] = { "used 4K", "used 8K",
	"used 16K", "used 32K", "used 64K", "used 128K", "used 256K", "used 512K",
	"used 1M", "used 2M", "used 4M", "unexpected" };
/* for example, if order is 1, represent 2 pages, that is 8K, so 8 is the value from order to K */
static const unsigned int ion_buffer_order_to_k[ZTE_ION_MAX_ORDERS + 1] = { 4, 8, 16, 32, 64, 128,
	256, 512, 1024, 2048, 4096, 1 };

/* local used flags */
#define ORDER_BEST_STAT_FLAG_UNEXPECTED -1
#define ORDER_BEST_STAT_FLAG_INIT 0

static const unsigned int order_range_base[ORDER_USED_NUM] = { 0x1000, /* 4K delimiter */
	0x10000, /* 64K */
	0x100000, /* 1M */
	0x200000 /* 2M */
};
/* for allocating if the least fragments happening */
static unsigned int order_best_count[ORDER_USED_NUM];
/* 0 for init, -1 for some abnormal case, others not used yet */
static int order_best_flag = ORDER_BEST_STAT_FLAG_INIT;
static const char *order_desc[ORDER_USED_NUM] = { "best 4K",
	"best 64K", "best 1M", "best 2M" };
static const unsigned int order_to_k[ORDER_USED_NUM] = {4,
	64, 1024, 2048};
static unsigned int order_index_to_order[ORDER_USED_NUM] = {9, 8, 4, 0};
static unsigned int order_index_to_bytes[ORDER_USED_NUM] = {
	PAGE_SIZE << 9, PAGE_SIZE << 8,
	PAGE_SIZE << 4, PAGE_SIZE << 0};

#ifdef CONFIG_ION_ZTE_DEDICATED_PAGE_BLOCK
#define ION_SYS_CUT_STEP_BYTES  0x200000 /* 2M */
struct mutex ion_sys_mutex;

struct ion_sys_dm_type {
	/* How many bytes user demand(DM) ion system buffer */
	unsigned int total_max;
	unsigned int max_times;
	/* and what's the distribution of the orders */
	struct ion_sys_count_type max_count;

	unsigned int cur_bytes;
	struct ion_sys_count_type cur_count;
};

struct ion_sys_dedi_type {
	int enable;
	/* What's the size related to dedicated order blocks */
	unsigned int yellow_value;
	/* How many times reach the yellow line */
	unsigned int yellow_count;
	unsigned int size;
	struct ion_sys_count_type count;

	/* if yellow value decreased, which count cut first */
	unsigned int cut_reverse_index;
};

struct ion_sys_dedi_type ion_sys_dedi_data;
struct ion_sys_dm_type ion_sys_dm_data;

static int zte_ion_dedi_mem_config_init(void);
static int zte_ion_sys_dedi_info_print(struct seq_file *s);
static int zte_ion_sys_dedi_size_recalculate(void);
static int zte_ion_sys_dedi_yellow_value_set(int new_yellow_value);
static int yellow_value_set(const char *val, struct kernel_param *kp);

static int ion_sys_yellow_value;
module_param_call(yellow_value, yellow_value_set, param_get_int,
			&ion_sys_yellow_value, 0644);

/**
 * user only allowed to decrease this value.
 */
static int yellow_value_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = ion_sys_yellow_value;

	ret = param_set_int(val, kp);

	if (ret) /*unexpected param */
		return ret;

	/* decreased value and valid check */
	if (old_val > ion_sys_yellow_value &&
		ion_sys_yellow_value >= 0 &&
		zte_ion_sys_dedi_mem_config()) {
		/* take effect */
		mutex_lock(&ion_sys_mutex);
		/* not changed or errors */
		if (zte_ion_sys_dedi_yellow_value_set(
			ion_sys_yellow_value) <= 0)
			ion_sys_yellow_value = old_val;
		mutex_unlock(&ion_sys_mutex);
	} else
		ion_sys_yellow_value = /* restore the old value*/
			old_val;

	return 0;
}

/**
 * resize the dedi orders count[] of cached or uncached.
 * under the protection of ion_sys_mutex.
 * @param new_yellow_value: new set value.
 * @return: 1 for set, 0 for unchanged, or negative for errors.
 */
static int zte_ion_sys_dedi_yellow_value_set(int new_yellow_value)
{
	int ret = 0; /* init to unchanged */
	int cut_size; /* record the size need to dec */
	int order_index, order_size; /* local var used many times */
	int count_cached, count_uncached, saved_count; /* count left */
	int step_count, step_size, cut_count;
	/* 70 loops (about 32M, may none in some loops) */
	int guard_loop = 70; /* just to avoid possible while(1) */

	/* should decrease the count[] nums */
	cut_size = ion_sys_dedi_data.yellow_value -
		new_yellow_value;

	pr_info("zte_ion_dbg yellow_value_set, 0x%x - 0x%x, 0x%x, %d\n",
		ion_sys_dedi_data.yellow_value,
		new_yellow_value,
		cut_size,
		ion_sys_dedi_data.cut_reverse_index);

	/* no need to check the count[], which are not enough large */
	if (ion_sys_dedi_data.size <= new_yellow_value) {
		ion_sys_dedi_data.yellow_value =
			ion_sys_yellow_value;
		pr_info("yellow_value_set skipped %d, %d\n",
			new_yellow_value,
			ion_sys_dedi_data.size);
		return 1; /* use the new value */
	}

	order_index = ion_sys_dedi_data.cut_reverse_index;
	/* cut at least ION_SYS_CUT_STEP_BYTES in each order loop */
	while (cut_size > 0 && guard_loop-- > 0) {
		/* init 0 for how many did cut in the cur order */
		order_size = order_index_to_bytes[order_index];
		count_cached =
			ion_sys_dedi_data.count.cached_count[order_index];
		count_uncached =
			ion_sys_dedi_data.count.uncached_count[order_index];
		saved_count = count_cached + count_uncached;
		step_size = saved_count * order_size;

		/* get the size to decrease */
		if (step_size >= ION_SYS_CUT_STEP_BYTES)
			step_size = ION_SYS_CUT_STEP_BYTES;

		/* get the count to decrease in the cur order */
		step_count = (step_size + order_size - 1) / order_size;
		if (step_count > 0) { /* not  empty */
			/* may larger than actually existing */
			step_size = step_count * order_size;

			/* cached first, dec half */
			if (count_cached >= step_count / 2) {
				cut_count = step_count / 2;
				count_cached -= step_count / 2;
			} else { /* dec all */
				cut_count = count_cached;
				count_cached = 0;
			}
			/* now target num left */
			step_count -= cut_count;

			/* step_count still need to be dec */
			if (count_uncached >= step_count) {
				cut_count = step_count;
				count_uncached -= step_count;
			} else { /* dec all */
				cut_count = count_uncached;
				count_uncached = 0;
			}
			/* total target num left */
			step_count -= cut_count;

			/* again dec in cached */
			if (step_count > 0) {
				if (count_cached >= step_count) {
					cut_count = step_count;
					count_cached -= step_count;
				} else { /* unexpected case */
					cut_count = count_cached;
					count_cached = 0;
				}
				step_count -= cut_count;
			}

			if (step_count > 0 &&
				(count_cached > 0 || count_uncached > 0))
				pr_warn("yellow_value_set unexpected %d, %d, %d\n",
					step_count,
					count_cached,
					count_uncached);
			else {
				saved_count = saved_count -
					count_cached - count_uncached;
				pr_info("yellow_value_set dec 0x%x (leak %d), unit=(0x%x, %d), and %d\n",
					step_size,
					step_count,
					order_size,
					order_index,
					saved_count);
			}

			/* reset to the new value */
			ion_sys_dedi_data.count.cached_count[order_index] =
				count_cached;
			ion_sys_dedi_data.count.uncached_count[order_index] =
				count_uncached;

			/* may left a count */
			cut_size = cut_size - step_size +
				step_count * order_size;
			/* set the count[] changed flag */
			ret = 1;
		}

		if (order_index == 0)
			order_index = ORDER_USED_NUM - 1;
		else
			order_index--;
	}

	ion_sys_dedi_data.cut_reverse_index = order_index;
	ion_sys_dedi_data.yellow_value =
		ion_sys_yellow_value;
	zte_ion_sys_dedi_size_recalculate();
	pr_info("zte_ion_dbg yellow_value_set ret\n");
	zte_ion_sys_dedi_info_print(NULL);

	return ret;
}

enum {
	ION_SYS_OPT_DEDI_MEM_CONFIG = 0x1, /* dedication mem */
};
static int ion_sys_opt_enable;
static int opt_enable_set(const char *val, struct kernel_param *kp);
module_param_call(opt_enable, opt_enable_set, param_get_int,
			&ion_sys_opt_enable, 0644);
/**
 * user may changed the feature's open status.
 */
static int opt_enable_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = ion_sys_opt_enable;

	ret = param_set_int(val, kp);

	if (ret) /*unexpected param */
		return ret;

	if (ion_zte_fragment_opt_initialized() &&
		old_val != ion_sys_opt_enable) { /* feature changed */
		if (zte_ion_sys_dedi_mem_config()) {
			/* reinit some data */
			mutex_lock(&ion_sys_mutex);
			zte_ion_dedi_mem_config_init();
			ion_sys_dedi_data.enable = ion_sys_opt_enable;
			mutex_unlock(&ion_sys_mutex);
		}
	} else
		ion_sys_opt_enable = /* restore the old value*/
			old_val;

	return 0;
}
/**
 * if feature enabled.
 * take effect immediately.
 */
int zte_ion_sys_dedi_mem_config(void)
{
	return ion_sys_dedi_data.enable & ION_SYS_OPT_DEDI_MEM_CONFIG;
}

/**
 * Init before starting up the shrink(dedicate memory) method feature.
 * zero all dm data, but
 * leave dedi yellow_value and enable unchanged.
 */
static int zte_ion_dedi_mem_config_init(void)
{
	int i;

	for (i = 0; i < ORDER_USED_NUM; i++) {
		ion_sys_dedi_data.count.cached_count[i] = 0;
		ion_sys_dedi_data.count.uncached_count[i] = 0;
		ion_sys_dm_data.max_count.cached_count[i] = 0;
		ion_sys_dm_data.max_count.uncached_count[i] = 0;
		ion_sys_dm_data.cur_count.cached_count[i] = 0;
		ion_sys_dm_data.cur_count.uncached_count[i] = 0;
	}

	ion_sys_dedi_data.yellow_count = 0;
	ion_sys_dedi_data.cut_reverse_index =
		ORDER_USED_NUM - 1;
	ion_sys_dedi_data.size = 0;
	ion_sys_dm_data.total_max = 0;
	ion_sys_dm_data.max_times = 0;
	ion_sys_dm_data.cur_bytes = 0;

	return 0;
}

/**
 * Print the dedicated ion system heap pool status.
 * used when data set/update or stats print
 */
static int zte_ion_sys_dedi_info_print(struct seq_file *s)
{
	unsigned int i;
	unsigned int total_size, total_count, tmp_size, count;

	if (!zte_ion_sys_dedi_mem_config())
		return 0; /* skip if not enabled */

	if (NULL != s) {
		seq_printf(s, "mask (enable = %d (%d), debug = %d)\n",
			ion_sys_dedi_data.enable,
			ion_sys_opt_enable,
			ion_sys_debug_mask);
		seq_printf(s,
			"yellow line 0x%x (%d), dedi 0x%x, times %u\n",
			ion_sys_dedi_data.yellow_value,
			ion_sys_yellow_value,
			ion_sys_dedi_data.size,
			ion_sys_dedi_data.yellow_count);

		tmp_size = 0;
		total_size = 0; /* for the total size */
		total_count = 0;
		for (i = 0; i < ORDER_USED_NUM; i++) {
			/* i=0 corresponding to 2M block, reverse order_to_k */
			count = ion_sys_dedi_data.count.cached_count[i] +
				ion_sys_dedi_data.count.uncached_count[i];
			tmp_size = count * order_to_k[ORDER_USED_NUM - 1 - i];
			seq_printf(s,
				"%d (%d K) dedi cached %d, uncached %d, 0x%x K\n",
				i,
				order_to_k[ORDER_USED_NUM - 1 - i],
				ion_sys_dedi_data.count.cached_count[i],
				ion_sys_dedi_data.count.uncached_count[i],
				tmp_size);
			total_size +=  tmp_size;
			total_count += count;
		}
		seq_printf(s, "and total size = 0x%x Bytes, total count = %d\n",
			total_size * K_TO_BYTES,
			total_count);
	} else {
		pr_info("zte_ion_dbg mask (enable = %d (%d), %d)\n",
			ion_sys_dedi_data.enable,
			ion_sys_opt_enable,
			ion_sys_debug_mask);
		pr_info("yellow line 0x%x (%d), dedi 0x%x, times %u\n",
				ion_sys_dedi_data.yellow_value,
				ion_sys_yellow_value,
				ion_sys_dedi_data.size,
				ion_sys_dedi_data.yellow_count);
		for (i = 0; i < ORDER_USED_NUM; i++) {
			pr_info("%d (%d K) dedi cached %d, uncached %d\n",
				i, order_to_k[ORDER_USED_NUM - 1 - i],
				ion_sys_dedi_data.count.cached_count[i],
				ion_sys_dedi_data.count.uncached_count[i]);
		}
	}

	return 0;
}

/**
 * increment the dm cur_count of cached or uncached.
 * @param index: which order to be filtered for input count.
 * @param cached: which order pool.
 * @return: if update or not, or negative for errors.
 */
int zte_ion_sys_dm_cur_count_inc(int index, int cached)
{
	int ret = 0; /* the init value for no update */
	/* ignore check if index valid */
	if (index < ORDER_USED_NUM && index >= 0) {
		if (cached)
			ion_sys_dm_data.cur_count.cached_count[index]++;
		else
			ion_sys_dm_data.cur_count.uncached_count[index]++;

		ret++; /* updated */
	} else /* invalid params */
		ret = -EINVAL;

	return ret;
}

/**
 * decrease the dm cur_count of cached or uncached.
 * @param index: which order to be filtered for input count.
 * @param cached: which order pool.
 * @return: if update or not, or negative for errors.
 */
int zte_ion_sys_dm_cur_count_dec(int index, int cached)
{
	int ret = 0; /* the init value for no update */
	/* ignore check if index valid */
	if (index < ORDER_USED_NUM && index >= 0) {
		if (cached)
			ion_sys_dm_data.cur_count.cached_count[index]--;
		else
			ion_sys_dm_data.cur_count.uncached_count[index]--;

		ret++; /* updated */
	} else /* invalid params */
		ret = -EINVAL;

	return ret;
}

/**
 * increment the dm cur_count of cached or uncached.
 * @param size: how many bytes increased.
 * @return: 0.
 */
int zte_ion_sys_add_dm_cur_bytes(unsigned int size)
{
	ion_sys_dm_data.cur_bytes += size;
	return 0;
}

/**
 * decrease the dm cur_count of cached or uncached.
 * @param size: how many bytes increased.
 * @return: 0.
 */
int zte_ion_sys_sub_dm_cur_bytes(unsigned int size)
{
	ion_sys_dm_data.cur_bytes -= size;
	return 0;
}

/**
 * control stats update under the protecting of ion_sys_mutex.
 * Count the dedicated blocks, not bigger than 1 block than
 * the yellow value
 * @param index: which order to be filtered for input count.
 * @param cached: which order pool.
 * @return: if update or not, or negative for errors.
 */
int zte_ion_sys_dedi_alloc_buffer_update(int index, int cached)
{
	int ret = 0; /* the init value for no update */
	/* ignore check if index valid */
	if (index < ORDER_USED_NUM && index >= 0) {
		if (ion_sys_dedi_data.size <
			ion_sys_dedi_data.yellow_value) {
			if (cached)
				ion_sys_dedi_data.count.cached_count[index]++;
			else
				ion_sys_dedi_data.count.uncached_count[index]++;

			ion_sys_dedi_data.size += order_index_to_bytes[index];
			if (ion_sys_dedi_data.size >=
				ion_sys_dedi_data.yellow_value) { /* stat */
				ion_sys_dedi_data.yellow_count++;
				zte_ion_sys_dedi_info_print(NULL);
			}
			ret++; /* updated */
		}
	} else /* invalid params */
		ret = -EINVAL;

	return ret;
}

/**
 * dedi data changed so
 * need to recalculate some data to keep the data consistency
 * between the data members
 * yellow_count is not reset, but may less the new yellow line
 * should used under the mutex protection of dedi data
 */
static int zte_ion_sys_dedi_size_recalculate(void)
{
	int index;
	unsigned int count;
	unsigned int total_size;

	/* case 1: count changed, size need to recalculate */
	total_size = 0;
	for (index = 0; index < ORDER_USED_NUM; index++) {
		count = ion_sys_dedi_data.count.cached_count[index] +
			ion_sys_dedi_data.count.uncached_count[index];
		total_size += order_index_to_bytes[index] * count;
	}

	ion_sys_dedi_data.size = total_size;

	return 0;
}

/**
 * How many pages should be released.
 * The dedication feature will remove the dedicated pages.
 * @param index: which order to be filtered for input count.
 * @param cached: which order pool.
 * @param page_count: the original value to be filtered.
 * @return: the new page_count filtered.
 */
int zte_ion_sys_dedi_shrink_filter(int index, int cached, int page_count)
{
	int compared_count, order_count;

	/* ignore check if index valid */
	if (index < ORDER_USED_NUM && index >= 0) {
		if (cached)
			order_count =
				ion_sys_dedi_data.count.cached_count[index];
		else
			order_count =
				ion_sys_dedi_data.count.uncached_count[index];

		compared_count = order_count <<
			order_index_to_order[index];
		/* new page count after removing the dedicated part */
		if (page_count > compared_count) {
			if (zte_ion_sys_debug_shrink())
				pr_info("uncached %d (%u), %d, dedi=%d (0x%x)\n",
					index,
					order_index_to_order[index],
					page_count,
					compared_count,
					ion_sys_dedi_data.size);
			page_count = page_count - compared_count;
		} else /* gfp may demand high */
			/* pool free blocks are less or equal */
			page_count = 0;
	} else {
		/* nothing filtered */
		pr_info("ATTENTION warning: unexpected dedi shrink parms\n");
	}

	return page_count;
}

/**
 * Print the cur monitor infor.
 */
int zte_ion_sys_cur_info_print_add(struct seq_file *s)
{
	unsigned int i;
	unsigned int total_size, total_count, tmp_size, count;

	if (!zte_ion_sys_dedi_mem_config())
		return 0; /* skip if not enabled */

	if (NULL != s) {
		/* prepare for the below current info print */
		seq_puts(s, "---------------------------------------------------- cur info\n");
		seq_printf(s, "cur demand bytes = 0x%x\n",
				ion_sys_dm_data.cur_bytes);

		tmp_size = 0;
		total_size = 0; /* for the total size */
		total_count = 0;
		for (i = 0; i < ORDER_USED_NUM; i++) {
			/* i=0 corresponding to 2M block, reverse order_to_k */
			count = ion_sys_dm_data.cur_count.cached_count[i] +
				ion_sys_dm_data.cur_count.uncached_count[i];
			tmp_size = count * order_to_k[ORDER_USED_NUM - 1 - i];
			seq_printf(s, "%d (%d K) cur cached %d, uncached %d, 0x%x K\n",
				i, order_to_k[ORDER_USED_NUM - 1 - i],
				ion_sys_dm_data.cur_count.cached_count[i],
				ion_sys_dm_data.cur_count.uncached_count[i],
				tmp_size);
			total_size +=  tmp_size;
			total_count += count;
		}
		seq_printf(s, "and cur total = 0x%x Bytes, total count = %d\n",
			total_size * K_TO_BYTES,
			total_count);
	} else if (zte_ion_sys_debug_user_max()) {
		pr_info("and cur bytes 0x%x\n",
			ion_sys_dm_data.cur_bytes);
		for (i = 0; i < ORDER_USED_NUM; i++) {
			pr_info("%d (%d K) cur cached %d, uncached %d\n",
				i, order_to_k[ORDER_USED_NUM - 1 - i],
				ion_sys_dm_data.cur_count.cached_count[i],
				ion_sys_dm_data.cur_count.uncached_count[i]);
		}
	}

	return 0;
}

/**
 * Print the max monitor infor.
 */
int zte_ion_sys_max_info_print(struct seq_file *s)
{
	unsigned int i;
	unsigned int total_size, total_count, tmp_size, count;

	if (!zte_ion_sys_dedi_mem_config())
		return 0; /* skip if not enabled */

	if (NULL != s) {
		seq_printf(s, "user max demand bytes = 0x%x, times = %u\n",
				ion_sys_dm_data.total_max,
				ion_sys_dm_data.max_times);

		tmp_size = 0;
		total_size = 0; /* for the total size */
		total_count = 0;
		for (i = 0; i < ORDER_USED_NUM; i++) {
			/* i=0 corresponding to 2M block, reverse order_to_k */
			count = ion_sys_dm_data.max_count.cached_count[i] +
				ion_sys_dm_data.max_count.uncached_count[i];
			tmp_size = count * order_to_k[ORDER_USED_NUM - 1 - i];
			seq_printf(s, "%d (%d K) max cached %d, uncached %d, 0x%x K\n",
				i, order_to_k[ORDER_USED_NUM - 1 - i],
				ion_sys_dm_data.max_count.cached_count[i],
				ion_sys_dm_data.max_count.uncached_count[i],
				tmp_size);
			total_size +=  tmp_size;
			total_count += count;
		}
		seq_printf(s, "and max total = 0x%x Bytes, total count = %d\n",
			total_size * K_TO_BYTES,
			total_count);
	} else if (zte_ion_sys_debug_user_max()) {
		pr_info("zte_ion_dbg max bytes 0x%x, times = %d\n",
			ion_sys_dm_data.total_max, ion_sys_dm_data.max_times);
		for (i = 0; i < ORDER_USED_NUM; i++) {
			pr_info("%d (%d K) max cached count = %d, uncached = %d\n",
				i, order_to_k[ORDER_USED_NUM - 1 - i],
				ion_sys_dm_data.max_count.cached_count[i],
				ion_sys_dm_data.max_count.uncached_count[i]);
		}
	}
	/* add current info print */
	zte_ion_sys_cur_info_print_add(s);

	return 0;
}

/**
 * Update other current info of  max after the current values update.
 * Should under the protection of ion_sys_mutex
 * Return 1 if any update happened
 */
int zte_ion_sys_max_other_update(void)
{
	int i;
	int ret = 1; /* default set to update yes */

	if (ion_sys_dm_data.cur_bytes > ion_sys_dm_data.total_max) {
		ion_sys_dm_data.max_times = 1;
		ion_sys_dm_data.total_max = ion_sys_dm_data.cur_bytes;

		for (i = 0; i < ORDER_USED_NUM; i++) {
			ion_sys_dm_data.max_count.cached_count[i] =
				ion_sys_dm_data.cur_count.cached_count[i];
			ion_sys_dm_data.max_count.uncached_count[i] =
				ion_sys_dm_data.cur_count.uncached_count[i];
		}

		if (zte_ion_sys_debug_user_max())
			zte_ion_sys_max_info_print(NULL);
	} else if (ion_sys_dm_data.cur_bytes == ion_sys_dm_data.total_max) {
		ion_sys_dm_data.max_times++;
		if (zte_ion_sys_debug_user_max())
			zte_ion_sys_max_info_print(NULL);
	} else /* not any update */
		ret = 0;

	return ret;
}
#endif

/**
 * Init largest ion buffer stats, only for inner using.
 * Not including the largest size value
 */
int inner_zte_ion_buffer_largest_order_count_init(void)
{
	int i;

	/* count[ZTE_ION_MAX_ORDERS] is for unexpected orders */
	for (i = 0; i < ZTE_ION_MAX_ORDERS + 1; i++) {
		ion_buffer_largest_order_count[i] = 0;
	}
	ion_buffer_largest_num = 0;
	ion_buffer_largest_sg_count = 0;

	return 0;
}

/**
 * seq puts the stats.
 * @param seq_puts handle.
 * must be protected under &dev->buffer_lock
 */
int zte_ion_buffer_stats_print(struct seq_file *s)
{
	unsigned int count, tmp_total_count;
	unsigned int tmp_size, total_size;
	unsigned int bpe_used, bpe_best;
	int i;

	if (ion_buffer_sg_num > 0) { /* there are some target buffers */
		seq_puts(s, "---------------------------------------------------- the largest stats\n");
		tmp_size = ion_buffer_largest_size * ion_buffer_largest_num;
		seq_printf(s, "%s = 0x%x, and the num = %u, total size = 0x%x ( %u M)\n",
			"the largest ion buffer size",
			ion_buffer_largest_size, ion_buffer_largest_num,
			tmp_size, tmp_size / BYTES_TO_K_DEN / K_TO_M_DEN);
		for (i = 0; i < ZTE_ION_MAX_ORDERS + 1; i++) {
			count =  ion_buffer_largest_order_count[i];
			if (count > 0) { /* there are some sgs of this order block */
				seq_printf(s, "%16.s        %u\n", ion_buffer_order_desc[i],
					   count);
			}
		}
		seq_printf(s, "%s = %u\n", "and the total nents ", ion_buffer_largest_sg_count);

		seq_puts(s, "---------------------------------------------------- ion system heap stats\n");
		total_size = 0; /* for the total size */
		tmp_total_count = 0; /* how many entries did the whole ion system heap use */
		bpe_used = 0;
		for (i = 0; i < ZTE_ION_MAX_ORDERS + 1; i++) {
			count =  ion_buffer_order_count[i];
			if (count > 0) { /* there are some sgs of this order block */
				tmp_size = count * ion_buffer_order_to_k[i];
				seq_printf(s, "%16.s        %u        = %u K\n", ion_buffer_order_desc[i],
					   count, count * ion_buffer_order_to_k[i]);
				total_size = total_size + tmp_size;
				tmp_total_count = tmp_total_count + count;
			}
		}
		/* total size from bytes to M, but we use bytes when calculating the BPE */
		if (tmp_total_count > 0) {
			bpe_used = (total_size * K_TO_BYTES) / tmp_total_count;
			seq_printf(s, "%16.s        %u        = %u M,        %d Bytes Per Entry\n",
				"used total size = ",
				total_size, total_size / K_TO_M_DEN, bpe_used);
		} else {
			seq_puts(s, " ATTENTION Warning: unexpected 0 used entries\n");
		}
		seq_printf(s, "%s = %u\n", "and total nents ", ion_buffer_sg_num);

		if (order_best_flag != ORDER_BEST_STAT_FLAG_UNEXPECTED) {
			seq_puts(s, "---------------------------------------------------- if least fragments\n");
			total_size = 0; /* for the total size */
			tmp_total_count = 0; /* how many entries does the least fragment need */
			bpe_best = 0;
			for (i = 0; i < ORDER_USED_NUM; i++) {
				count =  order_best_count[i];
				if (count > 0) { /* there are some sgs of this order block */
					tmp_size = count * order_to_k[i];
					seq_printf(s, "%16.s        %u        = %u K\n", order_desc[i],
						   count, tmp_size);
					total_size = total_size + tmp_size;
					tmp_total_count = tmp_total_count + count;
				}
			}
			/* from bytes to M, but use bytes when calculating the BPE */
			if (tmp_total_count > 0) {
				bpe_best = (total_size * K_TO_BYTES)
					/ tmp_total_count;
				seq_printf(s, "%16.s        %u        = %u M,        %d Bytes Per Entry\n",
					"best total size = ",
					total_size, total_size / K_TO_M_DEN,
					bpe_best);
				seq_printf(s, "%s = %u\n", "and best total nents ", tmp_total_count);
			}
			seq_puts(s, "----------------------------------------------------\n");
			/* The indicator for the optimizing, the less, the good
				bpe_best is expected less than bpe_used */
			if (bpe_used > 0) {
				seq_printf(s, "                                BPE best/used = %u\n",
					bpe_best / bpe_used);
			} else {
				seq_puts(s, " ATTENTION Warning: unexpected 0 bytes per entry best\n");
			}
		} else {
			seq_printf(s, " ATTENTION Warning: unexpected order_best_flag %d\n", order_best_flag);
		}
	}

	#ifdef CONFIG_ION_ZTE_DEDICATED_PAGE_BLOCK
		seq_puts(s, "---------------------------------------------------- dedicated status\n");
		zte_ion_sys_dedi_info_print(s);
		seq_puts(s, "---------------------------------------------------- max info\n");
		zte_ion_sys_max_info_print(s);
	#endif

	return 0;
}

/**
 * stat one ion buffer.
 * @param ion buffer pointer.
 * must be protected under &dev->buffer_lock
 */
int zte_ion_buffer_add_one_stat(struct ion_buffer *buffer)
{
	struct scatterlist *sg;
	struct sg_table *table;
	unsigned int len, size_remaining;
	int largest_count_flag = 0;
	int i, j, k;

	if (buffer) {
		len = buffer->size;

		/* best count for the least fragments */
		size_remaining = len;
		for (i = ORDER_USED_NUM - 1; i >= 0; i--) {
			if (size_remaining >= order_range_base[i]) {
				/* use k order i for least fragments */
				k = size_remaining / order_range_base[i];
				size_remaining = size_remaining - (k * order_range_base[i]);
				order_best_count[i] = order_best_count[i] + k;
			}
		}
		/* the size should has been aligned to PAGE SIZE = 4K */
		if (size_remaining != 0) {
			order_best_flag = ORDER_BEST_STAT_FLAG_UNEXPECTED; /* this is the  */
		}

		if (len > ion_buffer_largest_size) { /* a new and bigger one found */
			ion_buffer_largest_size = len;
			inner_zte_ion_buffer_largest_order_count_init(); /* just init to zero */
		}
		/* and counting will begin right after */
		if (len == ion_buffer_largest_size) {
			ion_buffer_largest_num++;
			largest_count_flag = 1;
		}

		table = buffer->sg_table;
		/* actually the sgl must be consistent with nents */
		if (table && table->sgl && table->nents) {
			for_each_sg(table->sgl, sg, table->nents, i) {
				len = sg->length; /* the len should be one of the orders size */
				/* i is used as the sg count, so use j instead */
				for (j = 0; j < ZTE_ION_MAX_ORDERS; j++) {
					if (len == ion_buffer_order_size[j]) {
						break;
					}
				}
				/* i may be [0 - ZTE_ION_MAX_ORDERS), or ZTE_ION_MAX_ORDERS when no one matched */
				ion_buffer_order_count[j]++;
				ion_buffer_sg_num++;

				/* for the largest part */
				if (largest_count_flag) {
					ion_buffer_largest_order_count[j]++;
					ion_buffer_largest_sg_count++;
				}
			}
		}
	}

	return 0;
}

/**
 * Init each order count to be 0 when start a heap's ion buffer debug data scan.
 */
int zte_ion_buffer_order_count_init(void)
{
	int i;

	/* count[ZTE_ION_MAX_ORDERS] is for unexpected orders */
	for (i = 0; i < ZTE_ION_MAX_ORDERS + 1; i++) {
		ion_buffer_order_count[i] = 0;
	}
	ion_buffer_sg_num = 0;

	/* largest ion buffer stats part */
	inner_zte_ion_buffer_largest_order_count_init();
	ion_buffer_largest_size = 0;

	/* best count for the least fragments if the order's buffers unlimited */
	for (i = 0; i < ORDER_USED_NUM; i++) {
		order_best_count[i] = 0;
	}
	order_best_flag = ORDER_BEST_STAT_FLAG_INIT;

	return 0;
}

/**
 * once time init when kernel startup, before the module is used.
 */
int zte_ion_startup_init(void)
{
	mutex_init(&ion_sys_mutex);
	/* ION_SYS_DEBUG_USER_MAX | ION_SYS_DEBUG_SHRINK */
	ion_sys_debug_mask = 0;  /* set default debug off */

#ifdef CONFIG_ION_ZTE_DEDICATED_PAGE_BLOCK
	/* opt enable initial value is 0, so need to set */
	ion_sys_opt_enable = ION_SYS_OPT_DEDI_MEM_CONFIG;
	zte_ion_dedi_mem_config_init(); /* dedi and dm data*/

	/* override to the initial value */
	ion_sys_yellow_value = SZ_64M;
	ion_sys_dedi_data.yellow_value = ion_sys_yellow_value;
	ion_sys_dedi_data.enable = ion_sys_opt_enable;
#endif

	/* leave a place to init when start up */

	inner_ion_zte_initialized = 1; /* should  put in the last */
	return 0;
}

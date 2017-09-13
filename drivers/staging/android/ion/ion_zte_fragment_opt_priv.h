/*
 * drivers/staging/android/ion/ion_zte_fragment_opt_priv.h
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

#ifndef _LINUX_ION_ZTE_FRAGMENT_OPT_H
#define _LINUX_ION_ZTE_FRAGMENT_OPT_H

#include <linux/err.h>
#include "../uapi/ion.h"

struct ion_handle;
struct ion_device;
struct ion_heap;
struct ion_mapper;
struct ion_client;
struct ion_buffer;

#ifdef CONFIG_ION
#ifdef CONFIG_ION_ZTE_FRAGMENT_OPT

#ifdef CONFIG_ION_ZTE_DEDICATED_PAGE_BLOCK

extern struct mutex ion_sys_mutex;

extern int zte_ion_sys_dedi_mem_config(void);
extern int zte_ion_sys_dedi_alloc_buffer_update(int index, int cached);
extern int zte_ion_sys_dedi_shrink_filter(int index, int cached,
	int page_count);

extern int zte_ion_sys_dm_cur_count_inc(int index, int cached);
extern int zte_ion_sys_dm_cur_count_dec(int index, int cached);
extern int zte_ion_sys_add_dm_cur_bytes(unsigned int size);
extern int zte_ion_sys_sub_dm_cur_bytes(unsigned int size);

extern int zte_ion_sys_debug_user_max(void);
extern int zte_ion_sys_debug_shrink(void);
extern int zte_ion_sys_max_other_update(void);
#endif

int zte_ion_buffer_stats_print(struct seq_file *s);
int zte_ion_buffer_add_one_stat(struct ion_buffer *buffer);
int zte_ion_buffer_order_count_init(void);
int zte_ion_startup_init(void);

#endif /* CONFIG_ION_ZTE_FRAGMENT_OPT */
#endif /* CONFIG_ION */
#endif /* _LINUX_ION_ZTE_FRAGMENT_OPT_H */

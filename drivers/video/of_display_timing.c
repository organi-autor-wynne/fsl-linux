/*
 * OF helpers for parsing display timings
 *
 * Copyright (c) 2012 Steffen Trumtrar <s.trumtrar@pengutronix.de>, Pengutronix
 *
 * based on of_videomode.c by Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This file is released under the GPLv2
 */
#include <linux/export.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <video/display_timing.h>
#include <linux/fb.h>
#include <uapi/linux/fb.h>
#include <uapi/linux/mxcfb.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#define DISP_DEV_NAME_MAX_LENGTH 32

/**
 * parse_timing_property - parse timing_entry from device_node
 * @np: device_node with the property
 * @name: name of the property
 * @result: will be set to the return value
 *
 * DESCRIPTION:
 * Every display_timing can be specified with either just the typical value or
 * a range consisting of min/typ/max. This function helps handling this
 **/
static int parse_timing_property(struct device_node *np, const char *name,
			  struct timing_entry *result)
{
	struct property *prop;
	int length, cells, ret;

	prop = of_find_property(np, name, &length);
	if (!prop) {
		pr_err("%s: could not find property %s\n",
			of_node_full_name(np), name);
		return -EINVAL;
	}

	cells = length / sizeof(u32);
	if (cells == 1) {
		ret = of_property_read_u32(np, name, &result->typ);
		result->min = result->typ;
		result->max = result->typ;
	} else if (cells == 3) {
		ret = of_property_read_u32_array(np, name, &result->min, cells);
	} else {
		pr_err("%s: illegal timing specification in %s\n",
			of_node_full_name(np), name);
		return -EINVAL;
	}

	return ret;
}

char* fb_disp_store_devname(struct device_node *np)
{
	char* store_disp_name = NULL;
	struct device_node *timings_np;
	int num_timings = 0;
	struct device_node *entry;
	int i = 0;

	timings_np = of_find_node_by_name(np, "display-timings");
	num_timings = of_get_child_count(timings_np);
	store_disp_name = kmalloc(num_timings*DISP_DEV_NAME_MAX_LENGTH, GFP_KERNEL);
	for_each_child_of_node(timings_np, entry) {
		strncpy((store_disp_name + (i*DISP_DEV_NAME_MAX_LENGTH)), entry->name, DISP_DEV_NAME_MAX_LENGTH);
		*(store_disp_name + ((i+1)*DISP_DEV_NAME_MAX_LENGTH)-1) = 0;
		i ++;
	}
	return store_disp_name; 
}

struct fb_videomode* of_get_display_timings_autorock(struct device_node *np, int* size)
{
	struct display_timings* disp = NULL;
	int i = 0;
	struct fb_videomode *fb_vm = NULL;
	struct videomode vm;
	char* store_disp_name = NULL;
	
	disp = of_get_display_timings(np);
	if (!disp) {
		pr_err("of_get_display_timings_autorock disp is NULL\r\n");
		return NULL;
	}
	*size = disp->num_timings;
	fb_vm = kmalloc(sizeof(struct fb_videomode)*(*size), GFP_KERNEL);
	if (!fb_vm) {
		display_timings_release(disp);
		pr_err("of_get_display_timings_autorock fb_vm is NULL\r\n");
		return NULL;
	}
	store_disp_name = fb_disp_store_devname(np);
	for(i = 0; i < disp->num_timings; i ++) {
		videomode_from_timing(disp->timings[i], &vm);
		fb_videomode_from_videomode(&vm, (fb_vm + i));
		if (vm.flags & DISPLAY_FLAGS_DE_LOW)
			(fb_vm + i)->sync |= FB_SYNC_OE_LOW_ACT;
		if (vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
			(fb_vm + i)->sync |= FB_SYNC_CLK_LAT_FALL;
		(fb_vm + i)->name = (store_disp_name + (i*DISP_DEV_NAME_MAX_LENGTH));
	}
	display_timings_release(disp);
	return fb_vm;
}

/**
 * of_get_display_timing - parse display_timing entry from device_node
 * @np: device_node with the properties
 **/
static struct display_timing *of_get_display_timing(struct device_node *np)
{
	struct display_timing *dt;
	u32 val = 0;
	int ret = 0;

	dt = kzalloc(sizeof(*dt), GFP_KERNEL);
	if (!dt) {
		pr_err("%s: could not allocate display_timing struct\n",
			of_node_full_name(np));
		return NULL;
	}

	ret |= parse_timing_property(np, "hback-porch", &dt->hback_porch);
	ret |= parse_timing_property(np, "hfront-porch", &dt->hfront_porch);
	ret |= parse_timing_property(np, "hactive", &dt->hactive);
	ret |= parse_timing_property(np, "hsync-len", &dt->hsync_len);
	ret |= parse_timing_property(np, "vback-porch", &dt->vback_porch);
	ret |= parse_timing_property(np, "vfront-porch", &dt->vfront_porch);
	ret |= parse_timing_property(np, "vactive", &dt->vactive);
	ret |= parse_timing_property(np, "vsync-len", &dt->vsync_len);
	ret |= parse_timing_property(np, "clock-frequency", &dt->pixelclock);

	dt->flags = 0;
	if (!of_property_read_u32(np, "vsync-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_VSYNC_HIGH :
				DISPLAY_FLAGS_VSYNC_LOW;
	if (!of_property_read_u32(np, "hsync-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_HSYNC_HIGH :
				DISPLAY_FLAGS_HSYNC_LOW;
	if (!of_property_read_u32(np, "de-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_DE_HIGH :
				DISPLAY_FLAGS_DE_LOW;
	if (!of_property_read_u32(np, "pixelclk-active", &val))
		dt->flags |= val ? DISPLAY_FLAGS_PIXDATA_POSEDGE :
				DISPLAY_FLAGS_PIXDATA_NEGEDGE;

	if (of_property_read_bool(np, "interlaced"))
		dt->flags |= DISPLAY_FLAGS_INTERLACED;
	if (of_property_read_bool(np, "doublescan"))
		dt->flags |= DISPLAY_FLAGS_DOUBLESCAN;

	if (ret) {
		pr_err("%s: error reading timing properties\n",
			of_node_full_name(np));
		kfree(dt);
		return NULL;
	}

	return dt;
}

/**
 * of_get_display_timings - parse all display_timing entries from a device_node
 * @np: device_node with the subnodes
 **/
struct display_timings *of_get_display_timings(struct device_node *np)
{
	struct device_node *timings_np;
	struct device_node *entry;
	struct device_node *native_mode;
	struct display_timings *disp;

	if (!np) {
		pr_err("%s: no devicenode given\n", of_node_full_name(np));
		return NULL;
	}

	timings_np = of_find_node_by_name(np, "display-timings");
	if (!timings_np) {
		pr_err("%s: could not find display-timings node\n",
			of_node_full_name(np));
		return NULL;
	}

	disp = kzalloc(sizeof(*disp), GFP_KERNEL);
	if (!disp) {
		pr_err("%s: could not allocate struct disp'\n",
			of_node_full_name(np));
		goto dispfail;
	}

	entry = of_parse_phandle(timings_np, "native-mode", 0);
	/* assume first child as native mode if none provided */
	if (!entry)
		entry = of_get_next_child(np, NULL);
	/* if there is no child, it is useless to go on */
	if (!entry) {
		pr_err("%s: no timing specifications given\n",
			of_node_full_name(np));
		goto entryfail;
	}

	pr_debug("%s: using %s as default timing\n",
		of_node_full_name(np), entry->name);

	native_mode = entry;

	disp->num_timings = of_get_child_count(timings_np);
	if (disp->num_timings == 0) {
		/* should never happen, as entry was already found above */
		pr_err("%s: no timings specified\n", of_node_full_name(np));
		goto entryfail;
	}

	disp->timings = kzalloc(sizeof(struct display_timing *) *
				disp->num_timings, GFP_KERNEL);
	if (!disp->timings) {
		pr_err("%s: could not allocate timings array\n",
			of_node_full_name(np));
		goto entryfail;
	}

	disp->num_timings = 0;
	disp->native_mode = 0;

	for_each_child_of_node(timings_np, entry) {
		struct display_timing *dt;

		dt = of_get_display_timing(entry);
		if (!dt) {
			/*
			 * to not encourage wrong devicetrees, fail in case of
			 * an error
			 */
			pr_err("%s: error in timing %d\n",
				of_node_full_name(np), disp->num_timings + 1);
			goto timingfail;
		}

		if (native_mode == entry)
			disp->native_mode = disp->num_timings;

		disp->timings[disp->num_timings] = dt;
		disp->num_timings++;
	}
	of_node_put(timings_np);
	/*
	 * native_mode points to the device_node returned by of_parse_phandle
	 * therefore call of_node_put on it
	 */
	of_node_put(native_mode);

	pr_debug("%s: got %d timings. Using timing #%d as default\n",
		of_node_full_name(np), disp->num_timings,
		disp->native_mode + 1);

	return disp;

timingfail:
	if (native_mode)
		of_node_put(native_mode);
	display_timings_release(disp);
entryfail:
	kfree(disp);
dispfail:
	of_node_put(timings_np);
	return NULL;
}
EXPORT_SYMBOL_GPL(of_get_display_timings);

/**
 * of_display_timings_exist - check if a display-timings node is provided
 * @np: device_node with the timing
 **/
int of_display_timings_exist(struct device_node *np)
{
	struct device_node *timings_np;

	if (!np)
		return -EINVAL;

	timings_np = of_parse_phandle(np, "display-timings", 0);
	if (!timings_np)
		return -EINVAL;

	of_node_put(timings_np);
	return 1;
}
EXPORT_SYMBOL_GPL(of_display_timings_exist);

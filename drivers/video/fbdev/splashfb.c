
#include <linux/bootmem.h>
#include <linux/decompress/inflate.h>
#include <linux/splashfb.h>
#include <asm/io.h>
#include <linux/fb.h>
#include <linux/memblock.h>

static unsigned long splash_addr,splash_size;
#define SPLASH_MAGIC		0xb87fe731

typedef struct {
	unsigned int magic;
	unsigned short x, y;
	unsigned short cx, cy;
}splash_header;

static int __init splash_setup(char *str)
{
	char *p;

	if(str == NULL)
		return 0;

	splash_addr = simple_strtol(str, &p, 0);
	p = strchr(p, ',');
	if(p == NULL)
		return 0;
	p++;
	splash_size = simple_strtol(p, NULL, 0);
	memblock_reserve(splash_addr, splash_size);
	return 0;
}

static void *splash_dst, *splash_dst_end;

static int __init decompress_flush(void* buf, unsigned int size)
{
	unsigned int space;

	space = (unsigned int)(splash_dst_end - splash_dst);
	size = space>size ? size : space;

	memcpy(splash_dst, buf, size);
	splash_dst+=size;
	return size;
}

static void decompress_output(char *buf)
{
	printk("%s\n", buf);
}

int __init show_splash_autorock(const struct fb_info *info)
{
	int ret=0;
	splash_header *hd;
	if(splash_addr == 0 || splash_size == 0)
		return -1;

	hd = (splash_header*)__phys_to_virt(splash_addr);
	
	if (!hd) {
		printk("function %s, __phys_to_virt fail\r\n",__FUNCTION__);
		return -1;
	}

	if (hd->magic != SPLASH_MAGIC) {
		printk("wrong splash format\n");
		return -1;
	}
	splash_dst = info->screen_base;
	splash_dst_end = info->screen_base + info->fix.smem_len;

	if(gunzip((unsigned char*)(hd+1), splash_size, NULL, decompress_flush, NULL, NULL, decompress_output)){
		printk("unzip splash failed\n");
		ret=-1;
	}

	memblock_free(splash_addr, splash_size);
	return ret;
}

__setup("splash=", splash_setup);

#ifndef __SPLASHFB_H__
#define __SPLASHFB_H__

#ifdef CONFIG_FB_CMD_SPLASH
#include <linux/fb.h>
int __init show_splash_autorock(const struct fb_info *info);

#else
#define show_splash_autorock(info)     (-1)

#endif

#endif


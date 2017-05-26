/*
 * Many of the syscalls used in this file expect some of the arguments
 * to be __user pointers not __kernel pointers.  To limit the sparse
 * noise, turn off sparse checking for this file.
 */
#ifdef __CHECKER__
#undef __CHECKER__
#warning "Sparse checking disabled for this file"
#endif

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fd.h>
#include <linux/tty.h>
#include <linux/suspend.h>
#include <linux/root_dev.h>
#include <linux/security.h>
#include <linux/delay.h>
#include <linux/genhd.h>
#include <linux/mount.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/initrd.h>
#include <linux/async.h>
#include <linux/fs_struct.h>
#include <linux/slab.h>
#include <linux/ramfs.h>
#include <linux/shmem_fs.h>

#include <linux/nfs_fs.h>
#include <linux/nfs_fs_sb.h>
#include <linux/nfs_mount.h>

#include <linux/clk.h>

#include "do_mounts.h"

int __initdata rd_doload;	/* 1 = load RAM disk, 0 = don't load */

int root_mountflags = MS_RDONLY | MS_SILENT;
static char * __initdata root_device_name;
static char __initdata saved_root_name[64];
static int root_wait;

dev_t ROOT_DEV;

#ifdef CONFIG_SUPPORT_INITROOT
#include <linux/namei.h>

static char initroot_line[128], *pinitroot;
static unsigned long root_timeout;
static int realroot;

static int __init do_mount_root(char *name, char *fs, int flags, void *data);

struct initroot_info{
	char *dev_name;
	char *fs;
	char *initscript;
};

static struct initroot_info last_initroot_info;

/*
	format: initroot=[wait time]:[device],[filesystem],[initscript];[device],[filesystem],[initscript]
	example: 
		initroot=20:/dev/mtdblock4,squashfs,/linurc
*/
static int get_initroot_info(struct initroot_info *info)
{
	char *s=pinitroot;

	if(!s || *s==0){
		return -1;
	}

	if((((u32)s)-((u32)initroot_line))>sizeof(initroot_line)){	//first use
		strncpy(initroot_line, s, sizeof(initroot_line));
		s = pinitroot = initroot_line;
	}

	memset(info, 0, sizeof(struct initroot_info));
	
	info->dev_name = s;
	for(;;s++){
		if (*s == ','){	//next
			*s++ = 0;
			break;
		}
		if (*s == ';' || *s == 0){	//end
			*s++ = 0;
			pinitroot = s;
			printk(KERN_WARNING"error initroot format at device name\n");
			return -1;
		}
	}

	info->fs = s;
	for(;;s++){
		if (*s == ','){	//next
			*s++ = 0;
			break;
		}
		if (*s == ';' || *s == 0){	//end
			*s++ = 0;
			pinitroot = s;
			printk(KERN_WARNING"error initroot format at fs name\n");
			return -1;
		}
	}

	info->initscript = s;
	for(;;s++){
		if (*s == ','){	//next
			*s++ = 0;
			break;
		}
		if (*s == ';' || *s == 0){	//end
			*s++ = 0;
			break;
		}
	}

	memcpy(&last_initroot_info, info, sizeof(last_initroot_info));
	pinitroot = s;

	return 0;
}

static struct k_sigaction old_sa;

static void dummy_fn(int sig)
{
}

static int set_sigchld_dummy(void)
{
	struct k_sigaction new_sa;
 
	new_sa.sa.sa_handler = dummy_fn;
	new_sa.sa.sa_flags = SA_ONESHOT | SA_NOMASK;
	sigemptyset(&new_sa.sa.sa_mask);

	return do_sigaction(SIGCHLD, &new_sa, &old_sa);
}

static int recover_sigchld(void)
{
	return do_sigaction(SIGCHLD, &old_sa, NULL);
}

static int __init do_initroot(char *init_filename)
{
	static char *argv[2];
	extern char * envp_init[];

	argv[0] = init_filename;
	argv[1] = NULL;

	sys_close(realroot);
	sys_close(0);sys_close(1);sys_close(2);

	(void) sys_open("/dev/console",O_RDWR,0);
	(void) sys_dup(0);
	(void) sys_dup(0);

	sys_chdir("/initroot");
	sys_chroot(".");
	devtmpfs_mount("dev");
	
	sys_mkdir("proc", 0755);
 	sys_mount("nondev", "proc", "proc", MS_SILENT, NULL);
 
	return sys_execve(init_filename,
		(const char __user *const __user *)argv, 
		(const char __user *const __user *)envp_init);
}

static int __init wait_thread_cpu_time_below(pid_t pid, int percent, int timeout_seconds)
{
	cputime_t utime, stime;
	unsigned long cur, total;
	struct task_struct *task = find_task_by_vpid(pid);
	if (!task){
		printk("Can't find task by pid %d\n", pid);
		return 0;
	}

	for(;timeout_seconds>0;timeout_seconds--){
		thread_group_cputime_adjusted(task, &utime, &stime);
		cur = cputime_to_jiffies(utime+stime);
		total = jiffies;

		msleep(1000);
		thread_group_cputime_adjusted(task, &utime, &stime);
		cur = cputime_to_jiffies(utime+stime) - cur;
		total = jiffies - total;

//		printk("%s(%d): %ld\n", __FUNCTION__, pid, (cur*100)/total);
		if((cur*100)/total<percent)
			return 0;
	}
	return -1;
}

extern void do_deferred_initcalls(void);

#ifdef CONFIG_UBOOT_SMP_BOOT
static unsigned long uboot_spl_start;// = 0x17780000;
static unsigned long uboot_spl_end;//   = 0x17800000;

void __init early_init_dt_setup_uboot_spl_range(unsigned long start, unsigned long end)
{
	uboot_spl_start = start;
	uboot_spl_end = end;
}
#endif

static int __init try_initroot(void)
{
	struct initroot_info info;
	dev_t initroot_dev;
	int retv=0, mount_initroot_sucess=0, no_initrootfs=0;
	pid_t pid;
printk("try_initroot\n");
	retv=get_initroot_info(&info);
	if(retv)
		goto end;
printk("try_initroot info:	dev:%s, fs:%s, initscript:%s \n ", info.dev_name, info.fs, info.initscript);
#ifdef CONFIG_BLK_DEV_INITRD
	if(strcmp(info.fs, "initramfs")==0){
		extern int wait_populate_initrootfs_done(void);
		no_initrootfs = wait_populate_initrootfs_done();
		if(!no_initrootfs)
			mount_initroot_sucess=1;
	}
	else
#endif
	{
		initroot_dev = name_to_dev_t(info.dev_name);
		/* wait for any asynchronous scanning to complete */
		if(initroot_dev==0){
			unsigned int time=0;
			printk(KERN_INFO "Waiting for initroot device %s...\n",	info.dev_name);
			while (driver_probe_done() != 0 ||
				(initroot_dev = name_to_dev_t(info.dev_name))==0){
				time++;
				if(time>root_timeout*10){
					printk(KERN_INFO "Waiting for initroot device %s timeout\n", info.dev_name);
					goto end;
				}
				msleep(100);
			}
			async_synchronize_full();
		}

		create_dev("/dev/initroot", initroot_dev);

		sys_mkdir("/initroot", 0700);
		/* mount initroot on rootfs /initroot */
		mount_initroot_sucess = 
			(sys_mount("/dev/initroot", "/initroot", info.fs, MS_RDONLY|MS_SILENT, NULL)==0);
	}

	if(mount_initroot_sucess){
		long ret=0;
		realroot = sys_open("/", 0, 0);

		set_sigchld_dummy();

		/*
		 * In case that a resume from disk is carried out by linuxrc or one of
		 * its children, we need to tell the freezer not to wait for us.
		 */
		current->flags |= PF_FREEZER_SKIP;

		pid = kernel_thread((int (*)(void *))do_initroot, info.initscript, SIGCHLD|CLONE_FS);
		if (pid > 0 && root_timeout > 0){
			sigset_t waitset;

			struct timespec timeout={.tv_sec = root_timeout};
			sigisemptyset(&waitset);
			sigaddset(&waitset, SIGUSR1);
			sigaddset(&waitset, SIGCHLD);
			for(;;){
				ret = sys_rt_sigtimedwait(&waitset, NULL, &timeout, sizeof(sigset_t));
				if (ret != -EINTR){// Interrupted by a signal other than SIGUSR1.
					break;
				}
				printk("Wait int by others\n");
			}
			if (ret == -EAGAIN){
				printk(KERN_WARNING"Wait SIGUSR1 Timeout, go on\n");
				sys_kill(pid,SIGKILL);
			}
			else if(ret < 0){
				printk(KERN_WARNING"sigtimedwait error\n");
			}
			else{ //init run ok
				flush_signals(current); //we must flush it, otherwise init won't be run
			}
		}
		current->flags &= ~PF_FREEZER_SKIP;
		recover_sigchld();
		wait_thread_cpu_time_below(pid+1, 50, root_timeout);

		sys_fchdir(realroot);
		sys_chroot(".");
		if(ret == SIGCHLD || ret == -EAGAIN){
			printk("initroot exit\n");
			sys_umount("/initroot/dev", MNT_DETACH);
			if(strcmp(info.fs, "initramfs"))
				sys_umount("/initroot", MNT_DETACH);
		}

		sys_close(realroot);
	}

#ifdef CONFIG_UBOOT_SMP_BOOT
	if(!no_initrootfs){
		struct clk *usdhc2, *usdhc3;
		// free clocks enabled for UBOOT load initrd
		usdhc2 = clk_get_sys(NULL, "usdhc2");
		usdhc3 = clk_get_sys(NULL, "usdhc3");
		if (IS_ERR(usdhc2) || IS_ERR(usdhc3))
			printk(KERN_ERR "UBOOT SMP BOOT: get mmc clock fail\n");
		else {
			clk_disable_unprepare(usdhc2);
			clk_disable_unprepare(usdhc3);
		}

		if(uboot_spl_start){
			// free memory reserved for UBOOT SPL
			free_reserved_area(phys_to_virt(uboot_spl_start), 
							   phys_to_virt(uboot_spl_end), 
							   0, "uboot-spl");
		}

		// bring up secodary CPU
		if (cpu_possible(1) && !cpu_online(1))
			cpu_up(1);
	}
#endif

end:
	do_deferred_initcalls();

	return retv;
}

static int __init initroot_dev_setup(char *line)
{
	root_timeout = simple_strtoul(line, &line, 0);
printk("initroot_dev_setup line:%s \n", line);
	if(*line!=':'){
		root_timeout = 0;
		pinitroot = NULL;
		return 1;
	}
	line++;

	pinitroot = line;
	return 1;
}

__setup("initroot=", initroot_dev_setup);
#endif

static int __init load_ramdisk(char *str)
{
	rd_doload = simple_strtol(str,NULL,0) & 3;
	return 1;
}
__setup("load_ramdisk=", load_ramdisk);

static int __init readonly(char *str)
{
	if (*str)
		return 0;
	root_mountflags |= MS_RDONLY;
	return 1;
}

static int __init readwrite(char *str)
{
	if (*str)
		return 0;
	root_mountflags &= ~MS_RDONLY;
	return 1;
}

__setup("ro", readonly);
__setup("rw", readwrite);

#ifdef CONFIG_BLOCK
struct uuidcmp {
	const char *uuid;
	int len;
};

/**
 * match_dev_by_uuid - callback for finding a partition using its uuid
 * @dev:	device passed in by the caller
 * @data:	opaque pointer to the desired struct uuidcmp to match
 *
 * Returns 1 if the device matches, and 0 otherwise.
 */
static int match_dev_by_uuid(struct device *dev, const void *data)
{
	const struct uuidcmp *cmp = data;
	struct hd_struct *part = dev_to_part(dev);

	if (!part->info)
		goto no_match;

	if (strncasecmp(cmp->uuid, part->info->uuid, cmp->len))
		goto no_match;

	return 1;
no_match:
	return 0;
}


/**
 * devt_from_partuuid - looks up the dev_t of a partition by its UUID
 * @uuid_str:	char array containing ascii UUID
 *
 * The function will return the first partition which contains a matching
 * UUID value in its partition_meta_info struct.  This does not search
 * by filesystem UUIDs.
 *
 * If @uuid_str is followed by a "/PARTNROFF=%d", then the number will be
 * extracted and used as an offset from the partition identified by the UUID.
 *
 * Returns the matching dev_t on success or 0 on failure.
 */
static dev_t devt_from_partuuid(const char *uuid_str)
{
	dev_t res = 0;
	struct uuidcmp cmp;
	struct device *dev = NULL;
	struct gendisk *disk;
	struct hd_struct *part;
	int offset = 0;
	bool clear_root_wait = false;
	char *slash;

	cmp.uuid = uuid_str;

	slash = strchr(uuid_str, '/');
	/* Check for optional partition number offset attributes. */
	if (slash) {
		char c = 0;
		/* Explicitly fail on poor PARTUUID syntax. */
		if (sscanf(slash + 1,
			   "PARTNROFF=%d%c", &offset, &c) != 1) {
			clear_root_wait = true;
			goto done;
		}
		cmp.len = slash - uuid_str;
	} else {
		cmp.len = strlen(uuid_str);
	}

	if (!cmp.len) {
		clear_root_wait = true;
		goto done;
	}

	dev = class_find_device(&block_class, NULL, &cmp,
				&match_dev_by_uuid);
	if (!dev)
		goto done;

	res = dev->devt;

	/* Attempt to find the partition by offset. */
	if (!offset)
		goto no_offset;

	res = 0;
	disk = part_to_disk(dev_to_part(dev));
	part = disk_get_part(disk, dev_to_part(dev)->partno + offset);
	if (part) {
		res = part_devt(part);
		put_device(part_to_dev(part));
	}

no_offset:
	put_device(dev);
done:
	if (clear_root_wait) {
		pr_err("VFS: PARTUUID= is invalid.\n"
		       "Expected PARTUUID=<valid-uuid-id>[/PARTNROFF=%%d]\n");
		if (root_wait)
			pr_err("Disabling rootwait; root= is invalid.\n");
		root_wait = 0;
	}
	return res;
}
#endif

/*
 *	Convert a name into device number.  We accept the following variants:
 *
 *	1) <hex_major><hex_minor> device number in hexadecimal represents itself
 *         no leading 0x, for example b302.
 *	2) /dev/nfs represents Root_NFS (0xff)
 *	3) /dev/<disk_name> represents the device number of disk
 *	4) /dev/<disk_name><decimal> represents the device number
 *         of partition - device number of disk plus the partition number
 *	5) /dev/<disk_name>p<decimal> - same as the above, that form is
 *	   used when disk name of partitioned disk ends on a digit.
 *	6) PARTUUID=00112233-4455-6677-8899-AABBCCDDEEFF representing the
 *	   unique id of a partition if the partition table provides it.
 *	   The UUID may be either an EFI/GPT UUID, or refer to an MSDOS
 *	   partition using the format SSSSSSSS-PP, where SSSSSSSS is a zero-
 *	   filled hex representation of the 32-bit "NT disk signature", and PP
 *	   is a zero-filled hex representation of the 1-based partition number.
 *	7) PARTUUID=<UUID>/PARTNROFF=<int> to select a partition in relation to
 *	   a partition with a known unique id.
 *	8) <major>:<minor> major and minor number of the device separated by
 *	   a colon.
 *
 *	If name doesn't have fall into the categories above, we return (0,0).
 *	block_class is used to check if something is a disk name. If the disk
 *	name contains slashes, the device name has them replaced with
 *	bangs.
 */

dev_t name_to_dev_t(const char *name)
{
	char s[32];
	char *p;
	dev_t res = 0;
	int part;

#ifdef CONFIG_BLOCK
	if (strncmp(name, "PARTUUID=", 9) == 0) {
		name += 9;
		res = devt_from_partuuid(name);
		if (!res)
			goto fail;
		goto done;
	}
#endif

	if (strncmp(name, "/dev/", 5) != 0) {
		unsigned maj, min, offset;
		char dummy;

		if ((sscanf(name, "%u:%u%c", &maj, &min, &dummy) == 2) ||
		    (sscanf(name, "%u:%u:%u:%c", &maj, &min, &offset, &dummy) == 3)) {
			res = MKDEV(maj, min);
			if (maj != MAJOR(res) || min != MINOR(res))
				goto fail;
		} else {
			res = new_decode_dev(simple_strtoul(name, &p, 16));
			if (*p)
				goto fail;
		}
		goto done;
	}

	name += 5;
	res = Root_NFS;
	if (strcmp(name, "nfs") == 0)
		goto done;
	res = Root_RAM0;
	if (strcmp(name, "ram") == 0)
		goto done;

	if (strlen(name) > 31)
		goto fail;
	strcpy(s, name);
	for (p = s; *p; p++)
		if (*p == '/')
			*p = '!';
	res = blk_lookup_devt(s, 0);
	if (res)
		goto done;

	/*
	 * try non-existent, but valid partition, which may only exist
	 * after revalidating the disk, like partitioned md devices
	 */
	while (p > s && isdigit(p[-1]))
		p--;
	if (p == s || !*p || *p == '0')
		goto fail;

	/* try disk name without <part number> */
	part = simple_strtoul(p, NULL, 10);
	*p = '\0';
	res = blk_lookup_devt(s, part);
	if (res)
		goto done;

	/* try disk name without p<part number> */
	if (p < s + 2 || !isdigit(p[-2]) || p[-1] != 'p')
		goto fail;
	p[-1] = '\0';
	res = blk_lookup_devt(s, part);
	if (res)
		goto done;

fail:
	return 0;
done:
	return res;
}
EXPORT_SYMBOL_GPL(name_to_dev_t);

static int __init root_dev_setup(char *line)
{
	strlcpy(saved_root_name, line, sizeof(saved_root_name));
	return 1;
}

__setup("root=", root_dev_setup);

static int __init rootwait_setup(char *str)
{
	if (*str)
		return 0;
	root_wait = 1;
	return 1;
}

__setup("rootwait", rootwait_setup);

static char * __initdata root_mount_data;
static int __init root_data_setup(char *str)
{
	root_mount_data = str;
	return 1;
}

static char * __initdata root_fs_names;
static int __init fs_names_setup(char *str)
{
	root_fs_names = str;
	return 1;
}

static unsigned int __initdata root_delay;
static int __init root_delay_setup(char *str)
{
	root_delay = simple_strtoul(str, NULL, 0);
	return 1;
}

__setup("rootflags=", root_data_setup);
__setup("rootfstype=", fs_names_setup);
__setup("rootdelay=", root_delay_setup);

static void __init get_fs_names(char *page)
{
	char *s = page;

	if (root_fs_names) {
		strcpy(page, root_fs_names);
		while (*s++) {
			if (s[-1] == ',')
				s[-1] = '\0';
		}
	} else {
		int len = get_filesystem_list(page);
		char *p, *next;

		page[len] = '\0';
		for (p = page-1; p; p = next) {
			next = strchr(++p, '\n');
			if (*p++ != '\t')
				continue;
			while ((*s++ = *p++) != '\n')
				;
			s[-1] = '\0';
		}
	}
	*s = '\0';
}

static int __init do_mount_root(char *name, char *fs, int flags, void *data)
{
	struct super_block *s;
	int err = sys_mount(name, "/root", fs, flags, data);
	if (err)
		return err;

	sys_chdir("/root");
	s = current->fs->pwd.dentry->d_sb;
	ROOT_DEV = s->s_dev;
	printk(KERN_INFO
	       "VFS: Mounted root (%s filesystem)%s on device %u:%u.\n",
	       s->s_type->name,
	       s->s_flags & MS_RDONLY ?  " readonly" : "",
	       MAJOR(ROOT_DEV), MINOR(ROOT_DEV));
	return 0;
}

void __init mount_block_root(char *name, int flags)
{
	struct page *page = alloc_page(GFP_KERNEL |
					__GFP_NOTRACK_FALSE_POSITIVE);
	char *fs_names = page_address(page);
	char *p;
#ifdef CONFIG_BLOCK
	char b[BDEVNAME_SIZE];
#else
	const char *b = name;
#endif

	get_fs_names(fs_names);
retry:
	for (p = fs_names; *p; p += strlen(p)+1) {
		int err = do_mount_root(name, p, flags, root_mount_data);
		switch (err) {
			case 0:
				goto out;
			case -EACCES:
			case -EINVAL:
				continue;
		}
#ifdef CONFIG_SUPPORT_INITROOT
		goto out;
#endif
	        /*
		 * Allow the user to distinguish between failed sys_open
		 * and bad superblock on root device.
		 * and give them a list of the available devices
		 */
#ifdef CONFIG_BLOCK
		__bdevname(ROOT_DEV, b);
#endif
		printk("VFS: Cannot open root device \"%s\" or %s: error %d\n",
				root_device_name, b, err);
		printk("Please append a correct \"root=\" boot option; here are the available partitions:\n");

		printk_all_partitions();
#ifdef CONFIG_DEBUG_BLOCK_EXT_DEVT
		printk("DEBUG_BLOCK_EXT_DEVT is enabled, you need to specify "
		       "explicit textual name for \"root=\" boot option.\n");
#endif
		panic("VFS: Unable to mount root fs on %s", b);
	}
#ifdef CONFIG_SUPPORT_INITROOT
	goto out;
#endif
	if (!(flags & MS_RDONLY)) {
		flags |= MS_RDONLY;
		goto retry;
	}

	printk("List of all partitions:\n");
	printk_all_partitions();
	printk("No filesystem could mount root, tried: ");
	for (p = fs_names; *p; p += strlen(p)+1)
		printk(" %s", p);
	printk("\n");
#ifdef CONFIG_BLOCK
	__bdevname(ROOT_DEV, b);
#endif
	panic("VFS: Unable to mount root fs on %s", b);
out:
	put_page(page);
}
 
#ifdef CONFIG_ROOT_NFS

#define NFSROOT_TIMEOUT_MIN	5
#define NFSROOT_TIMEOUT_MAX	30
#define NFSROOT_RETRY_MAX	5

static int __init mount_nfs_root(void)
{
	char *root_dev, *root_data;
	unsigned int timeout;
	int try, err;

	err = nfs_root_data(&root_dev, &root_data);
	if (err != 0)
		return 0;

	/*
	 * The server or network may not be ready, so try several
	 * times.  Stop after a few tries in case the client wants
	 * to fall back to other boot methods.
	 */
	timeout = NFSROOT_TIMEOUT_MIN;
	for (try = 1; ; try++) {
		err = do_mount_root(root_dev, "nfs",
					root_mountflags, root_data);
		if (err == 0)
			return 1;
		if (try > NFSROOT_RETRY_MAX)
			break;

		/* Wait, in case the server refused us immediately */
		ssleep(timeout);
		timeout <<= 1;
		if (timeout > NFSROOT_TIMEOUT_MAX)
			timeout = NFSROOT_TIMEOUT_MAX;
	}
	return 0;
}
#endif

#if defined(CONFIG_BLK_DEV_RAM) || defined(CONFIG_BLK_DEV_FD)
void __init change_floppy(char *fmt, ...)
{
	struct termios termios;
	char buf[80];
	char c;
	int fd;
	va_list args;
	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	fd = sys_open("/dev/root", O_RDWR | O_NDELAY, 0);
	if (fd >= 0) {
		sys_ioctl(fd, FDEJECT, 0);
		sys_close(fd);
	}
	printk(KERN_NOTICE "VFS: Insert %s and press ENTER\n", buf);
	fd = sys_open("/dev/console", O_RDWR, 0);
	if (fd >= 0) {
		sys_ioctl(fd, TCGETS, (long)&termios);
		termios.c_lflag &= ~ICANON;
		sys_ioctl(fd, TCSETSF, (long)&termios);
		sys_read(fd, &c, 1);
		termios.c_lflag |= ICANON;
		sys_ioctl(fd, TCSETSF, (long)&termios);
		sys_close(fd);
	}
}
#endif

void __init mount_root(void)
{
#ifdef CONFIG_ROOT_NFS
	if (ROOT_DEV == Root_NFS) {
		if (mount_nfs_root())
			return;

		printk(KERN_ERR "VFS: Unable to mount root fs via NFS, trying floppy.\n");
		ROOT_DEV = Root_FD0;
	}
#endif
#ifdef CONFIG_BLK_DEV_FD
	if (MAJOR(ROOT_DEV) == FLOPPY_MAJOR) {
		/* rd_doload is 2 for a dual initrd/ramload setup */
		if (rd_doload==2) {
			if (rd_load_disk(1)) {
				ROOT_DEV = Root_RAM1;
				root_device_name = NULL;
			}
		} else
			change_floppy("root floppy");
	}
#endif
#ifdef CONFIG_BLOCK
	create_dev("/dev/root", ROOT_DEV);
	mount_block_root("/dev/root", root_mountflags);
#endif
}

/*
 * Prepare the namespace - decide what/where to mount, load ramdisks, etc.
 */
void __init prepare_namespace(void)
{
	int is_floppy;

	if (root_delay) {
		printk(KERN_INFO "Waiting %d sec before mounting root device...\n",
		       root_delay);
		ssleep(root_delay);
	}

	/*
	 * wait for the known devices to complete their probing
	 *
	 * Note: this is a potential source of long boot delays.
	 * For example, it is not atypical to wait 5 seconds here
	 * for the touchpad of a laptop to initialize.
	 */
	wait_for_device_probe();

	md_run_setup();
	
#ifdef CONFIG_SUPPORT_INITROOT
	try_initroot();
#endif

	if (saved_root_name[0]) {
		root_device_name = saved_root_name;
		if (!strncmp(root_device_name, "mtd", 3) ||
		    !strncmp(root_device_name, "ubi", 3)) {
			mount_block_root(root_device_name, root_mountflags);
			goto out;
		}
		ROOT_DEV = name_to_dev_t(root_device_name);
		if (strncmp(root_device_name, "/dev/", 5) == 0)
			root_device_name += 5;
	}

	if (initrd_load())
		goto out;

	/* wait for any asynchronous scanning to complete */
	if ((ROOT_DEV == 0) && root_wait) {
		printk(KERN_INFO "Waiting for root device %s...\n",
			saved_root_name);
		while (driver_probe_done() != 0 ||
			(ROOT_DEV = name_to_dev_t(saved_root_name)) == 0)
			msleep(100);
		async_synchronize_full();
	}

	is_floppy = MAJOR(ROOT_DEV) == FLOPPY_MAJOR;

	if (is_floppy && rd_doload && rd_load_disk(0))
		ROOT_DEV = Root_RAM0;

	mount_root();
out:
	devtmpfs_mount("dev");
	sys_mount(".", "/", NULL, MS_MOVE, NULL);
	sys_chroot(".");
}

static bool is_tmpfs;
static struct dentry *rootfs_mount(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
	static unsigned long once;
	void *fill = ramfs_fill_super;

	if (test_and_set_bit(0, &once))
		return ERR_PTR(-ENODEV);

	if (IS_ENABLED(CONFIG_TMPFS) && is_tmpfs)
		fill = shmem_fill_super;

	return mount_nodev(fs_type, flags, data, fill);
}

static struct file_system_type rootfs_fs_type = {
	.name		= "rootfs",
	.mount		= rootfs_mount,
	.kill_sb	= kill_litter_super,
};

int __init init_rootfs(void)
{
	int err = register_filesystem(&rootfs_fs_type);

	if (err)
		return err;

	if (IS_ENABLED(CONFIG_TMPFS) && !saved_root_name[0] &&
		(!root_fs_names || strstr(root_fs_names, "tmpfs"))) {
		err = shmem_init();
		is_tmpfs = true;
	} else {
		err = init_ramfs_fs();
	}

	if (err)
		unregister_filesystem(&rootfs_fs_type);

	return err;
}

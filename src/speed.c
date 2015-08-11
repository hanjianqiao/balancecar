#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/io.h>


/* 在开发板执行 cat /proc/devices 查看空项设备号 */
static int major = 100;


int hello_open(struct inode *inode, struct file *file)
{
 printk("hello_open ***\n");
 return 0;
}

ssize_t hello_read(struct file *file, char __user *buf, size_t size, loff_t *offset)
{
 printk("hello_read ***\n");
 return 0; 
}

ssize_t hello_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
 printk("hello_write ***\n");
 return 0; 
}

static const struct file_operations hello_fops = {
 .owner = THIS_MODULE,
 .write = hello_write,
 .read = hello_read,
 .open = hello_open
};


int hello_init(void)
{

printk(KERN_INFO "hello_init!\n");（KERN_INFO为printk的优先级）
register_chrdev(major, "hello", &hello_fops);（如果major为0，则是让系统自动分配）
 return 0;
}

void hello_exit(void)
{

printk(KERN_INFO "hello_exit!\n");
 unregister_chrdev(major, "hello");
}

/*宏定义, 声明一个模块的初始化和清理函数*/
module_init(hello_init);
module_exit(hello_exit);

MODULE_LICENSE("Dual BSD/GPL");



#include "speed.h"

extern int printf(const char *format, ...);

static int *speed_real_LH;
static int *speed_real_RH;

void Right_Wheel (void)
{
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)==1)
	{
		printf("encode R:-");
		*speed_real_LH--;
	}else
	{
		printf("encode R:+");
		*speed_real_LH++;				
	}
}

void Left_Wheel(void)
{
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)
	{
		printf("encode L:+");
		*speed_real_RH++;
	}else
	{
		printf("encode L:-");
		*speed_real_RH--;
	}
}


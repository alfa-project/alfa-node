/*
 * Copyright 2023 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/ioctl.h>
#include <linux/dmaengine.h>
#include <asm/page.h>
#include <linux/slab.h>
#include <linux/string.h>

#define DEVICE_NAME "alfa"
#define CLASS_NAME "alfa_class"

// IOCTL commands
#define ALFA_IOCTL_TRIGGER_READ _IO('k', 1)
#define ALFA_IOCTL_TRIGGER_WRITE _IO('k', 2)
#define ALFA_IOCTL_CHECK_MEM _IO('k', 3)

#define MAX_MEM_PER_ID 0x200000

static unsigned long number_of_points = MAX_MEM_PER_ID/8;
static size_t mem_size_pointcloud_per_id = MAX_MEM_PER_ID;
static dma_addr_t pointcloud_cachable_phys_mem, pointcloud_non_cachable_phys_mem;
void * pointcloud_cachable_virt_mem, *pointcloud_non_cachable_virt_mem;
static struct reserved_mem *res_cachable, *res_non_cachable;
static void __iomem *cdma_virt_mem;
static struct class * alfa_class;
static struct platform_device *g_pdev = NULL;

static unsigned int mem_done = 0;

static bool file_open = false;
static bool class_created = false;

void memcpy_transfer(void * dest, void * src, size_t lenght)
{
	memcpy(dest, src, lenght);
}

static int alfa_open(struct inode *inode, struct file *filp)
{
	if(file_open)
	{
		pr_err("alfa:Failed to  open /dev/alfa because is already open\n");
		return -EBUSY;
	}

	printk("alfa:Opening /dev/alfa\n");
	file_open = true;
	return 0;
}

static int alfa_release(struct inode *inode, struct file *filp)
{
	if(!file_open)
	{
		pr_err("alfa:Failed to realse /dev/alfa because is not open\n");
		return -EBUSY;
	}
		printk("alfa:Closing /dev/alfa\n");
	file_open = false;
	return 0;
}

static long alfa_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	off_t mem_pc_pbase = pointcloud_cachable_phys_mem;
	off_t mem_pc_cdma_pbase = pointcloud_non_cachable_phys_mem;
	unsigned long offset = (unsigned long)arg;
	
	int ret;

	size_t mem_size_pointcloud;

        // Check if the user provided mem_size_pointcloud
        if (copy_from_user(&mem_size_pointcloud, (void __user *)arg, sizeof(size_t)))
        {
			printk("ALFA ERROR \n");
            return -EFAULT; // Copy from user-space failed
        }

	switch (cmd)
	{
	case ALFA_IOCTL_TRIGGER_READ:
		// Perform a static read from the mapped memory
		memcpy_transfer(pointcloud_cachable_virt_mem, pointcloud_non_cachable_virt_mem, mem_size_pointcloud);
		break;

	case ALFA_IOCTL_TRIGGER_WRITE:
		// Perform a static write to the mapped memory
		memcpy_transfer(pointcloud_non_cachable_virt_mem, pointcloud_cachable_virt_mem, mem_size_pointcloud);
		break;

	case ALFA_IOCTL_CHECK_MEM:
		int rsize = sizeof(mem_done);
		if (!access_ok( (void __user *)arg, rsize)) {
			return -EFAULT; // Bad user-space buffer
		}
		mem_done=1;
		printk("ALFA check - Begin\n");
		int i;
		uint32_t *mem1, *mem2;
		mem1 = (uint32_t*) pointcloud_non_cachable_virt_mem;
		mem2 = (uint32_t*) pointcloud_cachable_virt_mem;
		for(i=0; i<(number_of_points); i++)
		{
			
			if(mem1[i]!=mem2[i]) 
			{
				printk("Mem 1: %d  Mem 2: %d\n", mem1[i], mem2[i]);
				mem_done=0;
				printk("ID: %d", i);
			}
		}
		// Copy data to user space
		ret = copy_to_user((void __user *)arg, &mem_done, rsize);
		if (ret != 0) {
			return -EFAULT; // Copy operation failed
		}
		printk("ALFA check - End\n");
		break;

	default:
		pr_err("alfa:Invalid ioctl command\n");
		return -ENOTTY; // Invalid ioctl command
	}

	return 0;
}

static int alfa_mmap(struct file *filp, struct vm_area_struct *vma)
{
	vma->vm_flags |= VM_READ | VM_WRITE;
	struct device *dev = &g_pdev->dev;

	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;

	// Check for valid mapping size
	if (size > mem_size_pointcloud_per_id || size + off > res_non_cachable->size)
	{
		pr_err("Invalid mmap size or offset\n");
		return -EINVAL;
	}

	printk("mmap non cachable region in %lu \n", pointcloud_non_cachable_phys_mem + off);
	printk("mmap non cachable virtual region in %lu \n", pointcloud_non_cachable_virt_mem + off);
	printk("offset %lu \n", off);

	dma_addr_t phys_addr = virt_to_phys(pointcloud_cachable_virt_mem);
	unsigned long pfn = phys_addr >> PAGE_SHIFT;
	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot))
	{
		pr_err("Failed to remap non cachable memory\n");
		return -EAGAIN;
	}

	return 0;
}

static struct file_operations alfa_fops = {
	.owner = THIS_MODULE,
	.open = alfa_open,
	.release = alfa_release,
	.unlocked_ioctl = alfa_ioctl,
	.mmap = alfa_mmap,
};

static int alfa_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node_cache, *node_non_cache;
	g_pdev = pdev; // Store pdev globally

	// Retrieve the resource for the cacheable region from the Device Tree
	node_cache = of_find_node_by_name(NULL, "reserved_cachable_alfa");
	node_non_cache = of_find_node_by_name(NULL, "reserved_non_cachable_alfa");

	if(!node_cache || !node_non_cache)
	{
		pr_err("alfa:Failed to get alfa reserved resources from Device Tree\n");
		return -ENODEV;
	}
	else
	{
		res_non_cachable = of_reserved_mem_lookup(node_non_cache);
		res_cachable = of_reserved_mem_lookup(node_cache);

		if(!res_cachable || !res_non_cachable)
		{
			pr_err("alfa:Failed to get cacheable and non-cachable resources from Device Tree\n");
			return -ENODEV;
		}
	}
	
	pointcloud_cachable_phys_mem = res_cachable->base;
	pointcloud_non_cachable_phys_mem = res_non_cachable->base;

	printk("Phys cache address: %lx \n", pointcloud_cachable_phys_mem);
	printk("Phys non cache address: %lx \n", pointcloud_non_cachable_phys_mem);

	// Allocate physical memory
	pointcloud_cachable_virt_mem = kmalloc(res_non_cachable->size/8, GFP_ATOMIC);
	if (!pointcloud_cachable_virt_mem)
	{
		pr_err("alfa:Failed to allocate pointcloud_cachable_virt_mem cacheable memory\n");
		return -ENOMEM;
	}

	pointcloud_non_cachable_virt_mem = ioremap(pointcloud_non_cachable_phys_mem, res_non_cachable->size);

	if (!pointcloud_non_cachable_virt_mem)
	{
		pr_err("alfa:Failed to allocate pointcloud_non_cachable_virt_mem non-cacheable memory\n");
		dma_free_coherent(&pdev->dev, res_cachable->size, pointcloud_cachable_virt_mem, pointcloud_cachable_phys_mem);
		kfree(pointcloud_cachable_virt_mem);
		return -ENOMEM;
	}

	// Create device class
	if(class_created) 
	{
		printk("alfa:Class already exists\n");
	}
	else
	{
		alfa_class = class_create(THIS_MODULE, CLASS_NAME);
			if(IS_ERR(alfa_class))
			{
				pr_err("alfa:Failed to create device class");
				return PTR_ERR(alfa_class);
			}
			class_created = true;
	}

	// Create a character device to access the mapped memory
	ret = register_chrdev(0, DEVICE_NAME, &alfa_fops);

	if (ret < 0)
	{
		pr_err("alfa:Failed to register character device\n");
		dma_free_coherent(&pdev->dev, res_cachable->size, pointcloud_cachable_virt_mem, pointcloud_cachable_phys_mem);
		kfree(pointcloud_cachable_virt_mem);
		dma_free_coherent(&pdev->dev, res_non_cachable->size, pointcloud_non_cachable_virt_mem, pointcloud_non_cachable_phys_mem);
		iounmap(cdma_virt_mem);
		class_destroy(alfa_class);
		class_created = false;
		return ret;
	}


	device_create(alfa_class,NULL, MKDEV(ret,0), NULL, DEVICE_NAME);	

	of_node_put(node_cache);
	of_node_put(node_non_cache);
	pr_info("alfa:ALFA probed with success\n");
	return 0;
}

static int alfa_remove(struct platform_device *pdev)
{
	// Release the character device and unmap the memory
	unregister_chrdev(0, DEVICE_NAME);
	dma_free_coherent(&pdev->dev, res_cachable->size, pointcloud_cachable_virt_mem, pointcloud_cachable_phys_mem);
	kfree(pointcloud_cachable_virt_mem);
	dma_free_coherent(&pdev->dev, res_non_cachable->size, pointcloud_non_cachable_virt_mem, pointcloud_non_cachable_phys_mem);
	iounmap(cdma_virt_mem);
	pr_info("alfa:Unmapped physical memory\n");
	class_destroy(alfa_class);
	g_pdev = NULL;
	class_created = false;
	return 0;
}

static const struct of_device_id alfa_of_ids [] = {
	{.compatible = "alfadd",},
	{}
};
MODULE_DEVICE_TABLE(of,alfa_of_ids);

static struct platform_driver alfa = {
	.probe = alfa_probe,
	.remove = alfa_remove,
	.driver = {
		.name = "alfadd",
		.owner = THIS_MODULE,
		.of_match_table = alfa_of_ids,
	},
};

static int __init alfa_init(void)
{
	return platform_driver_register(&alfa);
}

static void __exit alfa_exit(void)
{
	platform_driver_unregister(&alfa);
}

module_init(alfa_init);
module_exit(alfa_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ricardo Roriz");
MODULE_DESCRIPTION("ALFA Device driver to map physical memory to user space with ioctl for static read and write");
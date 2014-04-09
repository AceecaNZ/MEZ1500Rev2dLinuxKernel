#include <linux/kernel.h>
#include <linux/module.h>


static int __init MEZ1500_hello_module_init(void)
{
    printk("Hello, MEZ1500 module is installed !\n");
    return 0;
}

static void __exit MEZ1500_hello_module_cleanup(void)
{
    printk("Good-bye, MEZ1500 module was removed!\n");
}

module_init(MEZ1500_hello_module_init);
module_exit(MEZ1500_hello_module_cleanup);
MODULE_LICENSE("GPL");

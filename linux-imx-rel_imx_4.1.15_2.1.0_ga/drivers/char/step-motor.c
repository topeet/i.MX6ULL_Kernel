#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/of_gpio.h>


#define DRIVER_NAME "step_motor_driver"
#define DEVICE_NAME "step_motor_driver"

#define CMD_STEPMOTOR_A    _IOW('L', 0, unsigned long)
#define CMD_STEPMOTOR_B    _IOW('L', 1, unsigned long)
#define CMD_STEPMOTOR_C	   _IOW('L', 2, unsigned long)
#define CMD_STEPMOTOR_D    _IOW('L', 3, unsigned long)

#ifdef  STEPMOTOR_DEBUG
#define DPRINTK(x...) printk("STEPMOTOR_DEBUG DEBUG:" x)
#else
#define DPRINTK(x...)
#endif

uint32_t step_motor_A = 0;
uint32_t step_motor_B = 0;
uint32_t step_motor_C = 0;
uint32_t step_motor_D = 0;

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("wdb");


static long step_motor_ioctl( struct file *files, unsigned int cmd, unsigned long arg)
{
	switch(cmd){
		case CMD_STEPMOTOR_A : gpio_set_value(step_motor_A, arg);  break;
		case CMD_STEPMOTOR_B : gpio_set_value(step_motor_B, arg);  break;
		case CMD_STEPMOTOR_C : gpio_set_value(step_motor_C, arg);  break; 
		case CMD_STEPMOTOR_D : gpio_set_value(step_motor_D, arg);  break; 
		default :
			break;       
	}
   	return 0;
}

static int step_motor_release(struct inode *inode, struct file *file){
	DPRINTK(KERN_EMERG "hello release\n");
	return 0;
}

static int step_motor_open(struct inode *inode, struct file *file){
	DPRINTK(KERN_EMERG "hello open\n");
	return 0;
}

static struct file_operations step_motor_ops = {
	.owner = THIS_MODULE,
	.open = step_motor_open,
	.release = step_motor_release,
	.unlocked_ioctl = step_motor_ioctl,
};

static  struct miscdevice step_motor_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &step_motor_ops,
};

static int step_motor_probe(struct platform_device *pdev){

	int ret;
	struct device_node *np = pdev->dev.of_node;

        step_motor_A = of_get_named_gpio(np, "motor-gpio-a", 0);
        if (step_motor_A == -EPROBE_DEFER)
                return step_motor_A;
        if (step_motor_A < 0) {
                dev_err(&pdev->dev, "error acquiring motor gpio: %d\n", step_motor_A);
                return step_motor_A;
        }

        ret = devm_gpio_request_one(&pdev->dev, step_motor_A, 0, "itop-motor");
        if(ret) {
                dev_err(&pdev->dev, "error requesting motor gpio: %d\n", ret);
                return ret;
        }
		
	step_motor_B = of_get_named_gpio(np, "motor-gpio-b", 0);
        if (step_motor_B == -EPROBE_DEFER)
                return step_motor_B;
        if (step_motor_B < 0) {
                dev_err(&pdev->dev, "error acquiring motor gpio: %d\n", step_motor_B);
                return step_motor_B;
        }

        ret = devm_gpio_request_one(&pdev->dev, step_motor_B, 0, "itop-motor");
        if(ret) {
                dev_err(&pdev->dev, "error requesting motor gpio: %d\n", ret);
                return ret;
        }
		
	step_motor_C = of_get_named_gpio(np, "motor-gpio-c", 0);
        if (step_motor_C == -EPROBE_DEFER)
                return step_motor_C;
        if (step_motor_C < 0) {
                dev_err(&pdev->dev, "error acquiring motor gpio: %d\n", step_motor_C);
                return step_motor_C;
        }

        ret = devm_gpio_request_one(&pdev->dev, step_motor_C, 0, "itop-motor");
        if(ret) {
                dev_err(&pdev->dev, "error requesting motor gpio: %d\n", ret);
                return ret;
        }
		
	step_motor_D = of_get_named_gpio(np, "motor-gpio-d", 0);
        if (step_motor_D == -EPROBE_DEFER)
                return step_motor_D;
        if (step_motor_D < 0) {
                dev_err(&pdev->dev, "error acquiring motor gpio: %d\n", step_motor_D);
                return step_motor_D;
        }

        ret = devm_gpio_request_one(&pdev->dev, step_motor_D, 0, "itop-motor");
        if(ret) {
                dev_err(&pdev->dev, "error requesting motor gpio: %d\n", ret);
                return ret;
        }
		
        gpio_direction_output(step_motor_A, 0);
	gpio_direction_output(step_motor_B, 0);
	gpio_direction_output(step_motor_C, 0);
	gpio_direction_output(step_motor_D, 0);
		
	ret = misc_register(&step_motor_dev);

        printk(DEVICE_NAME "\tinitialized\n");

        return 0;
}

static int step_motor_remove(struct platform_device *pdv){
	int i ;
	DPRINTK(KERN_EMERG "\tremove\n");
	gpio_free(step_motor_A);
        gpio_free(step_motor_B);
        gpio_free(step_motor_C);
        gpio_free(step_motor_D);

	
	misc_deregister(&step_motor_dev);
	return 0;
}

static void step_motor_shutdown(struct platform_device *pdv){
	
	;
}

static int step_motor_suspend(struct platform_device *pdv,pm_message_t pmt){
	
	return 0;
}

static int step_motor_resume(struct platform_device *pdv){
	
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id motor_of_match[] = {
        { .compatible = "itop_motor" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, motor_of_match);
#endif


struct platform_driver step_motor_driver=
{
	.probe = step_motor_probe,
	.remove = step_motor_remove,
	.shutdown = step_motor_shutdown,
	.suspend = step_motor_suspend,
	.resume = step_motor_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(motor_of_match),
	}
};

static int step_motor_init(void)
{
	int DriverState;
	printk(KERN_EMERG "init enter succeed!\n");

	DriverState=platform_driver_register(&step_motor_driver);

	return 0;
}

static void step_motor_exit(void)
{
	printk(KERN_EMERG " step_motor_exit succeed!\n");

	platform_driver_unregister(&step_motor_driver);
}

late_initcall_sync(step_motor_init);
module_exit(step_motor_exit);


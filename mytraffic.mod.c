#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xf8cdd757, "module_layout" },
	{ 0xe222cb8, "param_ops_int" },
	{ 0x3e18c761, "kthread_stop" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0xc00d5473, "wake_up_process" },
	{ 0xdc1e5ca7, "kthread_create_on_node" },
	{ 0x21e01071, "class_destroy" },
	{ 0xfae8f523, "device_create" },
	{ 0xe42dbab4, "__class_create" },
	{ 0x6ad5f0ce, "__register_chrdev" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0x1f4934e0, "gpiod_to_irq" },
	{ 0x9a76f11f, "__mutex_init" },
	{ 0xf9a482f9, "msleep" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0xa8d44dac, "gpiod_set_raw_value" },
	{ 0xad27f361, "__warn_printk" },
	{ 0xb44ad4b3, "_copy_to_user" },
	{ 0x4ca9669f, "scnprintf" },
	{ 0x82829e39, "gpiod_direction_input" },
	{ 0x2bb2e4ed, "gpiod_direction_output_raw" },
	{ 0x47229b5c, "gpio_request" },
	{ 0x27e1a049, "printk" },
	{ 0xbba23ebf, "gpiod_get_raw_value" },
	{ 0xdcc45e0b, "gpio_to_desc" },
	{ 0x15ba50a6, "jiffies" },
	{ 0xfe990052, "gpio_free" },
	{ 0xdb7305a1, "__stack_chk_fail" },
	{ 0xa6093a32, "mutex_unlock" },
	{ 0x41aed6e7, "mutex_lock" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x373db350, "kstrtoint" },
	{ 0x362ef408, "_copy_from_user" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xbdfb6dbb, "__fentry__" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "8BDCD9D32C3C9F24B1F6DEC");
MODULE_INFO(rhelversion, "8.10");

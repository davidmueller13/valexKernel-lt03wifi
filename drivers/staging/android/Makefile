ccflags-y += -I$(src)			# needed for trace events

obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o
obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_LOGGER)		+= logger.o
obj-$(CONFIG_ANDROID_PERSISTENT_RAM)	+= persistent_ram.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE)	+= ram_console.o
obj-$(CONFIG_ANDROID_TIMED_OUTPUT)	+= timed_output.o
obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= timed_gpio.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_ANDROID_SWITCH)		+= switch/
obj-$(CONFIG_ANDROID_INTF_ALARM_DEV)	+= alarm-dev.o
obj-$(CONFIG_PERSISTENT_TRACER)		+= trace_persistent.o

obj-$(CONFIG_BATTERY_SAMSUNG)		+= samsung/battery/
obj-$(CONFIG_ICE4_FPGA)			+= samsung/ice4_fpga/
obj-$(CONFIG_MOTOR_SAMSUNG)		+= samsung/motor/
obj-y					+= samsung/gud/
obj-$(CONFIG_SEC_NET_FILTER)		+= secfilter/

CFLAGS_REMOVE_trace_persistent.o = -pg

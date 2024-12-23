#
# Makefile for pwm devices
#
obj-$(CONFIG_GENERIC_PWM) := pwm.o

obj-$(CONFIG_DAVINCI_EHRPWM)	+= ehrpwm.o

obj-$(CONFIG_ECAP_PWM)		+= ecap_module.o

ecap_module-objs := ecap.o ecap_cap.o

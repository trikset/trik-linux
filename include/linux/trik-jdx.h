#ifndef __TRIK_JDX_H__
#define __TRIK_JDX_H__


struct trik_jd1_platform_data {
	//GPIO_TO_PIN(3,3)
	int gpio_d1a;
	//GPIO_TO_PIN(0,13)
	int gpio_d1e;
	//GPIO_TO_PIN(3,2)
	int gpio_d1b;
};

struct trik_jd2_platform_data {
	//GPIO_TO_PIN(3,1)
	int gpio_d2a;
	//GPIO_TO_PIN(0,11)
	int gpio_d2e;
	//GPIO_TO_PIN(3,4)
	int gpio_d2b;
};
#endif  /* __JDX_H__ */
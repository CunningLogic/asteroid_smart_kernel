
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/platform_device.h>

#include <plat/nand.h>

#include "mux.h"

extern struct platform_device omap_33;
extern struct platform_device user_gpio;
extern struct omap_board_mux board_mux[] __initdata;

void parrot_omap_serial_init(void);
void parrot_omap_i2c_init(int i2c_num);
int parrot_gpio_out_init(int gpio, int val);
void parrot_gpio_user_out_init(int gpio, int val, char *alias);
int parrot_gpio_in_init(int gpio, int val);
void parrot_gpio_user_in_init(int gpio, int val, char *alias);
void tsc2007_init(int irq, struct i2c_board_info *info);
void mxt224e_init(int irq, int hw_reset, struct i2c_board_info *info, int reset_value);
u8 mxt224e_read_irq(void);
void parrot_omap_voltage_init(void);


void parrot_omap_map_io(void);
void parrot_omap_init_irq(int ddr_rate);
void parrot_omap_gpmc_nand_config(struct omap_nand_platform_data *);

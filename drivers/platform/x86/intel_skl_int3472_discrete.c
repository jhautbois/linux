// SPDX-License-Identifier: GPL-2.0
/* Author: Dan Scally <djrscally@gmail.com> */

#include <linux/acpi.h>
#include <linux/clkdev.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>

#include "intel_skl_int3472_common.h"

const guid_t int3472_gpio_guid = GUID_INIT(0x79234640, 0x9e10, 0x4fea,
					     0xa5, 0xc1, 0xb5, 0xaa, 0x8b,
					     0x19, 0x75, 0x6f);

const guid_t cio2_sensor_module_guid = GUID_INIT(0x822ace8f, 0x2814, 0x4174,
						 0xa5, 0x6b, 0x5f, 0x02, 0x9f,
						 0xe0, 0x79, 0xee);

/*
 * Here follows platform specific mapping information that we can pass to
 * regulator_init_data when we register our regulators. We need to give
 * the regulator a supply name to work with for each sensor we plan on
 * supporting.
 */
static struct int3472_sensor_supply_map int3472_sensor_supply_maps[] = {
	{ "GNDF140809R", { "i2c-OVTI2680:00", "avdd" }},
	{ "YHCU", { "i2c-INT33BE:00", "avdd" }},
	{ "MSHW0070", { "i2c-INT33BE:00", "avdd" }},
};


/*
 * The regulators have to have .ops to be valid, but the only ops we actually
 * support are .enable and .disable which are handled via .ena_gpiod. Pass an
 * empty struct to clear the check without lying about capabilities.
 */
static const struct regulator_ops int3472_gpio_regulator_ops = { 0 };

static int int3472_clk_enable(struct clk_hw *hw)
{
	struct int3472_gpio_clock *clk = to_int3472_clk(hw);

	gpiod_set_value(clk->gpio, 1);

	return 0;
}

static void int3472_clk_disable(struct clk_hw *hw)
{
	struct int3472_gpio_clock *clk = to_int3472_clk(hw);

	gpiod_set_value(clk->gpio, 0);
}

static int int3472_clk_prepare(struct clk_hw *hw)
{
	/* We're just turning a GPIO on to enable, so nothing to do here */
	return 0;
}

static void int3472_clk_unprepare(struct clk_hw *hw)
{
	/* Likewise, nothing to do here... */
}

static const struct clk_ops int3472_clock_ops = {
	.prepare = int3472_clk_prepare,
	.unprepare = int3472_clk_unprepare,
	.enable = int3472_clk_enable,
	.disable = int3472_clk_disable,
};

static const struct clk_init_data int3472_clk_init_data = {
	.name = "int3472-clk",
	.ops = &int3472_clock_ops
};

static int int3472_map_gpio_to_sensor(struct int3472_device *int3472,
				      struct acpi_resource *ares, char *func)
{
	char *path = ares->data.gpio.resource_source.string_ptr;
	struct gpiod_lookup table_entry;
	struct acpi_device *adev;
	acpi_handle handle;
	acpi_status status;
	int ret;

	/* Make sure we don't overflow, and leave room for a terminator */
	if (int3472->n_sensor_gpios >= INT3472_MAX_SENSOR_GPIOS) {
		dev_warn(&int3472->sensor->dev, "Too many GPIOs mapped\n");
		return -EINVAL;
	}

	/* Fetch ACPI handle for the GPIO chip  */
	status = acpi_get_handle(NULL, path, &handle);
	if (ACPI_FAILURE(status))
		return -EINVAL;

	ret = acpi_bus_get_device(handle, &adev);
	if (ret)
		return -ENODEV;

	table_entry = (struct gpiod_lookup)GPIO_LOOKUP_IDX(acpi_dev_name(adev),
							   ares->data.gpio.pin_table[0],
							   func, 0, GPIO_ACTIVE_HIGH);

	memcpy(&int3472->gpios.table[int3472->n_sensor_gpios], &table_entry,
	       sizeof(table_entry));
	int3472->n_sensor_gpios++;

	return 0;
}

static struct int3472_sensor_supply_map *
int3472_get_sensor_supply_map(struct int3472_device *int3472)
{
	unsigned int i = ARRAY_SIZE(int3472_sensor_supply_maps);
	struct int3472_sensor_supply_map *ret;
	union acpi_object *obj;

	/*
	 * Sensor modules seem to be identified by a unique string. We use that
	 * to make sure we pass the right device and supply names to the new
	 * regulator's consumer_supplies
	 */
	obj = acpi_evaluate_dsm_typed(int3472->sensor->handle,
				      &cio2_sensor_module_guid, 0x00,
				      0x01, NULL, ACPI_TYPE_STRING);

	if (!obj) {
		dev_err(&int3472->sensor->dev,
			"Failed to get sensor module string from _DSM\n");
		return ERR_PTR(-ENODEV);
	}

	if (obj->string.type != ACPI_TYPE_STRING) {
		dev_err(&int3472->sensor->dev,
			"Sensor _DSM returned a non-string value\n");
		ret = ERR_PTR(-EINVAL);
		goto out_free_obj;
	}

	ret = ERR_PTR(-ENODEV);
	while (i--) {
		if (!strcmp(int3472_sensor_supply_maps[i].sensor_module_name,
			    obj->string.pointer)) {
			ret = &int3472_sensor_supply_maps[i];
			goto out_free_obj;
		}
	}

out_free_obj:
	ACPI_FREE(obj);
	return ret;
}

static int int3472_register_clock(struct int3472_device *int3472,
			          struct acpi_resource *ares)
{
	char *path = ares->data.gpio.resource_source.string_ptr;
	int ret;

	int3472->clock.gpio = acpi_get_gpiod(path, ares->data.gpio.pin_table[0]);
	if (IS_ERR(int3472->clock.gpio))
		return PTR_ERR(int3472->clock.gpio);

	int3472->clock.clk_hw.init = &int3472_clk_init_data;
	int3472->clock.clk = clk_register(&int3472->adev->dev,
					  &int3472->clock.clk_hw);
	if (IS_ERR(int3472->clock.clk)) {
		ret = PTR_ERR(int3472->clock.clk);
		goto err_put_gpio;
	}

	/* Most drivers use xvclk as con_id so we'll replicate that here */
	ret = clk_register_clkdev(int3472->clock.clk, "xvclk", int3472->sensor_name);
	if (ret)
		goto err_unregister_clk;
	
	return 0;

err_unregister_clk:
	clk_unregister(int3472->clock.clk);
err_put_gpio:
	gpiod_put(int3472->clock.gpio);

	return ret;
}

static int int3472_register_regulator(struct int3472_device *int3472,
				      struct acpi_resource *ares)
{
	char *path = ares->data.gpio.resource_source.string_ptr;
	struct int3472_sensor_supply_map *regulator_map;
	struct regulator_init_data init_data = { };
	struct int3472_gpio_regulator *regulator;
	struct regulator_config cfg = { };
	int ret;

	/*
	 * We lookup supply names from machine specific tables, based on a
	 * unique identifier in the sensor's _DSM
	 */
	regulator_map = int3472_get_sensor_supply_map(int3472);
	if (IS_ERR_OR_NULL(regulator_map)) {
		dev_err(&int3472->sensor->dev,
			"Found no supplies defined for this sensor\n");
		return PTR_ERR(regulator_map);
	}

	init_data.supply_regulator = NULL;
	init_data.constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS;
	init_data.num_consumer_supplies = 1;
	init_data.consumer_supplies = &regulator_map->supply_map;

	snprintf(int3472->regulator.regulator_name, GPIO_REGULATOR_NAME_LENGTH,
		 "int3472-discrete-regulator");
	snprintf(int3472->regulator.supply_name, GPIO_REGULATOR_SUPPLY_NAME_LENGTH,
		 "supply-0");

	int3472->regulator.rdesc = INT3472_REGULATOR(int3472->regulator.regulator_name,
					     	     int3472->regulator.supply_name,
						     &int3472_gpio_regulator_ops);

	int3472->regulator.gpio = acpi_get_gpiod(path, ares->data.gpio.pin_table[0]);
	if (IS_ERR(int3472->regulator.gpio)) {
		ret = PTR_ERR(int3472->regulator.gpio);
		goto err_free_regulator;
	}

	cfg.dev = &int3472->adev->dev;
	cfg.init_data = &init_data;
	cfg.ena_gpiod = int3472->regulator.gpio;

	int3472->regulator.rdev = regulator_register(&int3472->regulator.rdesc, &cfg);
	if (IS_ERR(int3472->regulator.rdev)) {
		ret = PTR_ERR(int3472->regulator.rdev);
		goto err_free_gpio;
	}

	return 0;

err_free_gpio:
	gpiod_put(regulator->gpio);
err_free_regulator:
	kfree(regulator);

	return ret;
}

/**
 * int3472_handle_gpio_resources: maps PMIC resources to consuming sensor
 * @ares: A pointer to a struct acpi_resource
 * @data: A pointer to a struct int3472_device
 *
 * This function handles GPIO resources that are against an INT3472
 * ACPI device, by checking the value of the corresponding _DSM entry.
 * This will return a 32bit int, where the lowest byte represents the
 * function of the GPIO pin:
 *
 * 0x00 Reset
 * 0x01 Power down
 * 0x0b Power enable
 * 0x0c Clock enable
 * 0x0d Privacy LED
 *
 * GPIOs will either be mapped directly to the sensor device or else used
 * to create clocks and regulators via the usual frameworks.
 * 
 * Return:
 * * 0		- When all resources found are handled properly.
 * * -EINVAL	- If the resource is not a GPIO IO resource
 * * -ENODEV	- If the resource has no corresponding _DSM entry
 * * -Other	- Errors propagated from one of the sub-functions.
 */
static int int3472_handle_gpio_resources(struct acpi_resource *ares,
					 void *data)
{
	struct int3472_device *int3472 = data;
	union acpi_object *obj;
	int ret = 0;

	if (ares->type != ACPI_RESOURCE_TYPE_GPIO ||
	    ares->data.gpio.connection_type != ACPI_RESOURCE_GPIO_TYPE_IO)
		return EINVAL; /* Deliberately positive */

	/*
	 * n_gpios + 2 because the index of this _DSM function is 1-based and
	 * the first function is just a count.
	 */
	obj = acpi_evaluate_dsm_typed(int3472->adev->handle,
				      &int3472_gpio_guid, 0x00,
				      int3472->n_gpios + 2,
				      NULL, ACPI_TYPE_INTEGER);

	if (!obj) {
		dev_warn(&int3472->pdev->dev,
			 "No _DSM entry for this GPIO pin\n");
		return ENODEV;
	}

	switch (obj->integer.value & 0xff) {
	case INT3472_GPIO_TYPE_RESET:
		ret = int3472_map_gpio_to_sensor(int3472, ares, "reset");
		if (ret)
			dev_err(&int3472->pdev->dev,
				 "Failed to map reset pin to sensor\n");

		break;
	case INT3472_GPIO_TYPE_SHUTDOWN:
		ret = int3472_map_gpio_to_sensor(int3472, ares, "shutdown");
		if (ret)
			dev_err(&int3472->pdev->dev,
				 "Failed to map shutdown pin to sensor\n");

		break;
	case INT3472_GPIO_TYPE_CLK_ENABLE:
		ret = int3472_register_clock(int3472, ares);
		if (ret)
			dev_err(&int3472->pdev->dev,
				 "Failed to map clock to sensor\n");

		break;
	case INT3472_GPIO_TYPE_POWER_ENABLE:
		ret = int3472_register_regulator(int3472, ares);
		if (ret) {
			dev_err(&int3472->pdev->dev,
				 "Failed to map regulator to sensor\n");
		}

		break;
	case INT3472_GPIO_TYPE_PRIVACY_LED:
		ret = int3472_map_gpio_to_sensor(int3472, ares, "indicator-led");
		if (ret)
			dev_err(&int3472->pdev->dev,
				 "Failed to map indicator led to sensor\n");

		break;
	default:
		/* if we've gotten here, we're not sure what they are yet */
		dev_warn(&int3472->pdev->dev,
			 "GPIO type 0x%llx unknown; the sensor may not work\n",
			 (obj->integer.value & 0xff));
		ret = EINVAL; /* Deliberately positive */
	}

	int3472->n_gpios++;
	ACPI_FREE(obj);

	return ret;
}

static int int3472_parse_crs(struct int3472_device *int3472)
{
	struct list_head resource_list;
	int ret = 0;

	INIT_LIST_HEAD(&resource_list);

	ret = acpi_dev_get_resources(int3472->adev, &resource_list,
				     int3472_handle_gpio_resources, int3472);

	if (!ret)
		gpiod_add_lookup_table(&int3472->gpios);
	acpi_dev_free_resource_list(&resource_list);

	return ret;
}

int skl_int3472_discrete_probe(struct platform_device *pdev)
{
	struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);
	struct int3472_device *int3472;
	struct int3472_cldb cldb;
	int ret = 0;

	/*
	 * This driver is only intended to support "dummy" INT3472 devices
	 * which appear in ACPI designed for Windows. These are distinguishable
	 * from INT3472 entries representing an actual tps68470 PMIC through
	 * the presence of a CLDB buffer with a particular value set. At time
	 * of writing, those devices were not instantiated as platform_devices
	 * so this driver shouldn't try to bind to them, but better to check
	 * to be safe.
	 */
	ret = skl_int3472_get_cldb_buffer(adev, &cldb);
	if (ret || cldb.control_logic_type != 1)
		return -EINVAL;

	/* Space for 4 GPIOs - one more than we've seen so far plus a null */
	int3472 = kzalloc(sizeof(*int3472) +
			 ((INT3472_MAX_SENSOR_GPIOS + 1) * sizeof(struct gpiod_lookup)),
			 GFP_KERNEL);
	if (!int3472)
		return -ENOMEM;

	int3472->adev = adev;
	int3472->pdev = pdev;
	platform_set_drvdata(pdev, int3472);

	int3472->sensor = acpi_dev_get_next_dep_dev(adev, NULL);
	if (!int3472->sensor) {
		dev_err(&pdev->dev,
			"This INT3472 entry seems to have no dependents.\n");
		ret = -ENODEV;
		goto err_free_int3472;
	}
	int3472->sensor_name = i2c_acpi_dev_name(int3472->sensor);
	int3472->gpios.dev_id = int3472->sensor_name;

	ret = int3472_parse_crs(int3472);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse CRS for this device: %d\n", ret);
		skl_int3472_discrete_remove(pdev);
		goto err_return_ret;
	}

	return 0;

err_free_int3472:
	kfree(int3472);
err_return_ret:
	return ret;
}

int skl_int3472_discrete_remove(struct platform_device *pdev)
{
	struct int3472_device *int3472;

	int3472 = platform_get_drvdata(pdev);

	if (int3472->n_sensor_gpios)
		gpiod_remove_lookup_table(&int3472->gpios);

	if (!IS_ERR_OR_NULL(int3472->regulator.rdev)) {
		gpiod_put(int3472->regulator.gpio);
		regulator_unregister(int3472->regulator.rdev);
	}

	if (!IS_ERR_OR_NULL(int3472->clock.clk)) {
		gpiod_put(int3472->clock.gpio);
		clk_unregister(int3472->clock.clk);
	}

	acpi_dev_put(int3472->sensor);

	kfree(int3472->sensor_name);
	kfree(int3472);

	return 0;
}
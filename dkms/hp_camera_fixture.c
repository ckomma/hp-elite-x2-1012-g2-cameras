// SPDX-License-Identifier: GPL-2.0
/*
 * HP Elite x2 1012 G2 (82CA) Unified Camera Fixture
 * Controls both OV8858 (Rear RGB) and OV7251 (Front IR) on TPS68470 PMIC INT3472:05.
 * Also exposes WLED2 IR illuminator controls for Howdy face authentication.
 */

#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/dmi.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/pci.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <media/v4l2-fwnode.h>

static bool run = true;
module_param(run, bool, 0400);

static int result = -EINPROGRESS;
module_param(result, int, 0444);

static bool rear_ready;
module_param(rear_ready, bool, 0444);

static bool ir_ready;
module_param(ir_ready, bool, 0444);

static bool rear_powered;
module_param(rear_powered, bool, 0444);

static bool ir_powered;
module_param(ir_powered, bool, 0444);

static bool restored;
module_param(restored, bool, 0444);

/* PMIC register snapshots */
static const u8 pmic_regs[] = {
	6,7,8,9,10,11,12,13,15,16,
	0x1c,0x1e,0x20,0x22,0x27,0x3e,0x3f,0x40,0x41,0x42,0x43,0x46,0x47,0x48
};

struct hp_unified_fixture {
	struct i2c_client *c;            /* INT3472:05 PMIC client */
	struct work_struct work;
	
	/* Saved baseline */
	u8 saved_regs[ARRAY_SIZE(pmic_regs)];
	bool snapshot_taken;
	bool started;
	bool held;
	
	/* Rear OV8858 state */
	struct i2c_client *rear_sensor;
	struct acpi_device *rear_adev;
	struct device *ipu_dev;
	bool rear_graph_registered;
	struct software_node rear_nodes[5];
	const struct software_node *rear_group[6];
	struct property_entry rear_devprops[4], rear_portprops[2], rear_epprops[5], rear_iprops[5];
	struct software_node_ref_args rear_local_ref[1], rear_remote_ref[1];
	
	/* IR OV7251 state */
	struct i2c_client *ir_sensor;
	struct acpi_device *ir_adev;
	struct fwnode_handle *ir_fw;
	
	/* WLED2 state */
	bool wled_active;
	bool wled_prepared;
	u64 wled_target_ns;
	u8 wled_saved[11];
};

static struct hp_unified_fixture *g_fixture;
static DEFINE_MUTEX(g_fixture_lock);

static int pmic_read(struct hp_unified_fixture *f, u8 reg)
{
	u8 val;
	struct i2c_msg msgs[2] = {
		{ .addr = f->c->addr, .len = 1, .buf = &reg },
		{ .addr = f->c->addr, .flags = I2C_M_RD, .len = 1, .buf = &val }
	};
	int ret = i2c_transfer(f->c->adapter, msgs, 2);
	return (ret == 2) ? val : (ret < 0 ? ret : -EIO);
}

static int pmic_write(struct hp_unified_fixture *f, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	int ret = i2c_master_send(f->c, buf, 2);
	return (ret == 2) ? 0 : (ret < 0 ? ret : -EIO);
}

static int pmic_update(struct hp_unified_fixture *f, u8 reg, u8 mask, u8 val)
{
	int cur = pmic_read(f, reg);
	if (cur < 0)
		return cur;
	return pmic_write(f, reg, (cur & ~mask) | (val & mask));
}

/* WLED2 sysfs controls */
#define WLED_TIMEOUT_MS 3000
static void wled_auto_off_timer(struct work_struct *work);
static DECLARE_DELAYED_WORK(g_wled_off_work, wled_auto_off_timer);

static int wled_hardware_off(struct hp_unified_fixture *f)
{
	if (!f) return -ENODEV;
	pmic_write(f, 0x36, 0x18); /* Disable WLED2 */
	f->wled_active = false;
	f->wled_prepared = false;
	return 0;
}

static void wled_auto_off_timer(struct work_struct *work)
{
	mutex_lock(&g_fixture_lock);
	if (g_fixture && g_fixture->wled_active) {
		wled_hardware_off(g_fixture);
		dev_info(&g_fixture->c->dev, "HP_UNIFIED: WLED2 auto-timeout safely turned OFF\n");
	}
	mutex_unlock(&g_fixture_lock);
}

static ssize_t wled_pulse_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ms = 0;
	int step = 2; /* Default 65 mA (step 2 * 32.5 mA) */

	if (kstrtoint(buf, 10, &ms) < 0) {
		if (sscanf(buf, "%d %d", &ms, &step) != 2)
			return -EINVAL;
	}
	if (ms <= 0 || ms > 5000)
		return -EINVAL;
	if (step < 1 || step > 6)
		step = 2;

	mutex_lock(&g_fixture_lock);
	if (!g_fixture || !ir_ready) {
		mutex_unlock(&g_fixture_lock);
		return -ENODEV;
	}

	/* Disable WLED first to configure parameters safely */
	pmic_write(g_fixture, 0x36, 0x18);
	pmic_write(g_fixture, 0x29, 0x01); /* Enable thermal monitor */
	pmic_write(g_fixture, 0x2d, (u8)step); /* WLEDMAXT: Torch mode max current */
	pmic_write(g_fixture, 0x35, (u8)step); /* WLEDC2: LED2 sink current */
	usleep_range(200, 300);

	/* Enable Torch / Video Light Mode for WLED2 (DISLED1=1, DISLED2=0, EN=1, MODE=01) */
	pmic_write(g_fixture, 0x36, 0x0d);
	g_fixture->wled_active = true;

	/* Schedule hardware safety OFF */
	mod_delayed_work(system_wq, &g_wled_off_work, msecs_to_jiffies(ms));
	mutex_unlock(&g_fixture_lock);

	return count;
}
static DEVICE_ATTR_WO(wled_pulse);

static ssize_t wled_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	mutex_lock(&g_fixture_lock);
	cancel_delayed_work_sync(&g_wled_off_work);
	if (g_fixture)
		wled_hardware_off(g_fixture);
	mutex_unlock(&g_fixture_lock);
	return count;
}
static DEVICE_ATTR_WO(wled_off);

/* Power controls for OV8858 (Rear) */
bool hp_rear_fixture_ready(void) { return g_fixture && rear_ready; }
EXPORT_SYMBOL_GPL(hp_rear_fixture_ready);

int hp_rear_power(bool on)
{
	int r = 0, i;
	static const u8 clocks[][2] = {
		{0x0a,0xaa},{8,2},{7,3},{0x0b,0x20},{0x0c,1},{6,1},
		{0x10,5},{9,3},{0x0d,0xd0},{0x0f,0x0a},{0x0d,0xd1}
	};

	mutex_lock(&g_fixture_lock);
	if (!g_fixture || !rear_ready) {
		mutex_unlock(&g_fixture_lock);
		return -ENODEV;
	}

	if (!on) {
		/* Keep rear camera rails powered for instant standby/wake */
		mutex_unlock(&g_fixture_lock);
		return 0;
	}

	if (rear_powered) {
		mutex_unlock(&g_fixture_lock);
		return 0;
	}

	/* Power on rear camera */
	pmic_write(g_fixture, 0x41, 0x6d);
	pmic_write(g_fixture, 0x42, 0x0c);
	pmic_write(g_fixture, 0x3f, 0x34);
	pmic_write(g_fixture, 0x40, 0x34);
	for (i = 0; i < ARRAY_SIZE(clocks); i++)
		pmic_write(g_fixture, clocks[i][0], clocks[i][1]);
	usleep_range(5000, 6000);

	pmic_update(g_fixture, 0x43, 3, 3);
	pmic_update(g_fixture, 0x47, 1, 1);
	pmic_update(g_fixture, 0x48, 1, 1);
	usleep_range(1000, 1500);

	pmic_update(g_fixture, 0x22, 4, 4);
	usleep_range(1000, 1500);
	pmic_update(g_fixture, 0x22, 1, 1);
	msleep(20);

	rear_powered = true;
	mutex_unlock(&g_fixture_lock);
	return r;
}
EXPORT_SYMBOL_GPL(hp_rear_power);

/* Power controls for OV7251 (IR) */
bool hp_ir_fixture_ready(void) { return g_fixture && ir_ready; }
EXPORT_SYMBOL_GPL(hp_ir_fixture_ready);

int hp_ir_power(bool on)
{
	int r = 0, i;
	static const u8 clocks[][2] = {
		{10,0xaa},{8,2},{7,3},{11,0x20},{12,1},{6,1},
		{16,5},{9,3},{13,0xd0},{15,0x0a},{13,0xd1}
	};

	mutex_lock(&g_fixture_lock);
	if (!g_fixture || !ir_ready) {
		mutex_unlock(&g_fixture_lock);
		return -ENODEV;
	}

	if (!on) {
		/* Keep sensor rails powered for instant standby/wake */
		mutex_unlock(&g_fixture_lock);
		return 0;
	}

	if (ir_powered) {
		mutex_unlock(&g_fixture_lock);
		return 0;
	}

	/* Power on IR sensor */
	pmic_update(g_fixture, 0x1c, 0xcb, 0x8a);
	pmic_update(g_fixture, 0x1e, 0xcb, 0xc2);
	pmic_update(g_fixture, 0x20, 0xcb, 0xc2);
	pmic_write(g_fixture, 0x3e, 0x34); /* AUX2 1.8V */
	for (i = 0; i < ARRAY_SIZE(clocks); i++)
		pmic_update(g_fixture, clocks[i][0], 0xff, clocks[i][1]);
	usleep_range(5000, 6000);

	pmic_update(g_fixture, 0x43, 3, 3);
	pmic_update(g_fixture, 0x27, 0x10, 0x10);
	usleep_range(1000, 1500);

	pmic_update(g_fixture, 0x46, 1, 1);
	usleep_range(1000, 1500);
	pmic_update(g_fixture, 0x27, 0x60, 0x60);
	msleep(20);

	ir_powered = true;
	mutex_unlock(&g_fixture_lock);
	return r;
}
EXPORT_SYMBOL_GPL(hp_ir_power);

/* Registration and hardware discovery */
static const u32 rear_lanes[] = {1, 2, 3, 4};
static const u64 rear_link_freq[] = {360000000};

static int attach_rear_sensor(struct hp_unified_fixture *f)
{
	struct fwnode_handle *root, *ep = NULL;
	const struct software_node *parent;
	struct i2c_board_info info = { I2C_BOARD_INFO("hp-ov8858-test", 0x10) };
	struct fwnode_endpoint parsed;
	int ret, i;

	f->ipu_dev = bus_find_device_by_name(&pci_bus_type, NULL, "0000:00:14.3");
	if (!f->ipu_dev) return -ENODEV;

	root = dev_fwnode(f->ipu_dev);
	while (root && !is_software_node(root)) root = root->secondary;
	parent = to_software_node(root);
	if (!parent) return -ENODEV;

	while ((ep = fwnode_graph_get_next_endpoint(root, ep))) {
		ret = fwnode_graph_parse_endpoint(ep, &parsed);
		if (ret || parsed.port == 0) { fwnode_handle_put(ep); return ret ? ret : -EEXIST; }
	}

	f->rear_devprops[0] = PROPERTY_ENTRY_U32("clock-frequency", 19200000);
	f->rear_devprops[1] = PROPERTY_ENTRY_U32("rotation", 0);
	f->rear_devprops[2] = PROPERTY_ENTRY_U32("orientation", V4L2_FWNODE_ORIENTATION_BACK);
	f->rear_portprops[0] = PROPERTY_ENTRY_U32("reg", 0);
	f->rear_nodes[0] = (struct software_node){.name = "HP-OV8858-R2A-test", .properties = f->rear_devprops};
	f->rear_nodes[1] = (struct software_node){.name = "port@0", .parent = &f->rear_nodes[0], .properties = f->rear_portprops};
	f->rear_nodes[2] = (struct software_node){.name = "endpoint@0", .parent = &f->rear_nodes[1], .properties = f->rear_epprops};
	f->rear_nodes[3] = (struct software_node){.name = "port@0", .parent = parent, .properties = f->rear_portprops};
	f->rear_nodes[4] = (struct software_node){.name = "endpoint@0", .parent = &f->rear_nodes[3], .properties = f->rear_iprops};

	f->rear_local_ref[0] = SOFTWARE_NODE_REFERENCE(&f->rear_nodes[4]);
	f->rear_remote_ref[0] = SOFTWARE_NODE_REFERENCE(&f->rear_nodes[2]);
	f->rear_epprops[0] = PROPERTY_ENTRY_U32("bus-type", V4L2_FWNODE_BUS_TYPE_CSI2_DPHY);
	f->rear_epprops[1] = PROPERTY_ENTRY_U32_ARRAY("data-lanes", rear_lanes);
	f->rear_epprops[2] = PROPERTY_ENTRY_REF_ARRAY("remote-endpoint", f->rear_local_ref);
	f->rear_epprops[3] = PROPERTY_ENTRY_U64_ARRAY("link-frequencies", rear_link_freq);
	f->rear_iprops[0] = PROPERTY_ENTRY_U32_ARRAY("data-lanes", rear_lanes);
	f->rear_iprops[1] = PROPERTY_ENTRY_REF_ARRAY("remote-endpoint", f->rear_remote_ref);

	for (i = 0; i < 5; i++) f->rear_group[i] = &f->rear_nodes[i];
	ret = software_node_register_node_group(f->rear_group);
	if (ret) return ret;
	f->rear_graph_registered = true;

	f->rear_adev = acpi_dev_get_first_match_dev("INT3477", NULL, -1);
	if (!f->rear_adev) return -ENODEV;
	if (acpi_fwnode_handle(f->rear_adev)->secondary) return -EEXIST;

	info.swnode = &f->rear_nodes[0];
	info.fwnode = acpi_fwnode_handle(f->rear_adev);
	f->rear_sensor = i2c_new_client_device(f->c->adapter, &info);
	if (IS_ERR(f->rear_sensor)) {
		ret = PTR_ERR(f->rear_sensor);
		f->rear_sensor = NULL;
		return ret;
	}

	rear_ready = true;
	return 0;
}

static int attach_ir_sensor(struct hp_unified_fixture *f)
{
	struct i2c_adapter *adap;
	acpi_handle expected;
	struct i2c_board_info info = { I2C_BOARD_INFO("hp-ov7251-ir-test", 0x60) };
	int ret;

	f->ir_adev = acpi_dev_get_first_match_dev("INT347E", NULL, -1);
	if (!f->ir_adev) return -ENODEV;

	f->ir_fw = fwnode_handle_get(acpi_fwnode_handle(f->ir_adev)->secondary);
	if (!f->ir_fw || !is_software_node(f->ir_fw) || fwnode_graph_get_endpoint_count(f->ir_fw, 0) != 1)
		return -ENOLINK;

	adap = i2c_get_adapter(2);
	if (!adap) return -ENODEV;

	if (ACPI_FAILURE(acpi_get_handle(NULL, "\\_SB_.PCI0.I2C3", &expected)) ||
	    ACPI_HANDLE(adap->dev.parent) != expected) {
		i2c_put_adapter(adap);
		return -ENODEV;
	}

	info.swnode = to_software_node(f->ir_fw);
	f->ir_sensor = i2c_new_client_device(adap, &info);
	i2c_put_adapter(adap);

	if (IS_ERR(f->ir_sensor)) {
		ret = PTR_ERR(f->ir_sensor);
		f->ir_sensor = NULL;
		return ret;
	}

	ir_ready = true;
	return 0;
}

static void fixture_init_work(struct work_struct *w)
{
	struct hp_unified_fixture *f = container_of(w, struct hp_unified_fixture, work);
	int i, v, r;

	if (!f->started) {
		f->started = true;

		/* Take initial PMIC register snapshot */
		for (i = 0; i < ARRAY_SIZE(pmic_regs); i++) {
			v = pmic_read(f, pmic_regs[i]);
			if (v < 0) { result = v; return; }
			f->saved_regs[i] = v;
		}
		f->snapshot_taken = true;

		/* Attach Rear sensor (OV8858) */
		r = attach_rear_sensor(f);
		if (r) {
			dev_err(&f->c->dev, "HP_UNIFIED: attach_rear_sensor failed %d\n", r);
			result = r;
			return;
		}

		/* Attach IR sensor (OV7251) */
		r = attach_ir_sensor(f);
		if (r) {
			dev_err(&f->c->dev, "HP_UNIFIED: attach_ir_sensor failed %d\n", r);
			result = r;
			return;
		}

		/* Create sysfs files for WLED2 */
		device_create_file(&f->c->dev, &dev_attr_wled_pulse);
		device_create_file(&f->c->dev, &dev_attr_wled_off);

		result = 0;
		dev_info(&f->c->dev, "HP_UNIFIED: All sensors attached! Rear=%d IR=%d\n", rear_ready, ir_ready);
	}
}

static int probe(struct i2c_client *c)
{
	struct hp_unified_fixture *f;
	int v;

	if (!run || !dmi_match(DMI_SYS_VENDOR, "HP") ||
	    !dmi_match(DMI_PRODUCT_NAME, "HP Elite x2 1012 G2") ||
	    !dmi_match(DMI_BOARD_NAME, "82CA") ||
	    strcmp(dev_name(&c->dev), "i2c-INT3472:05") ||
	    c->addr != 0x4d || c->adapter->nr != 1)
		return -ENODEV;

	if (!i2c_check_functionality(c->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	f = devm_kzalloc(&c->dev, sizeof(*f), GFP_KERNEL);
	if (!f) return -ENOMEM;
	f->c = c;

	v = pmic_read(f, 0xff);
	if (v != 0x21) return (v < 0) ? v : -ENODEV;

	INIT_WORK(&f->work, fixture_init_work);
	i2c_set_clientdata(c, f);
	g_fixture = f;
	f->held = false;

	schedule_work(&f->work);
	return 0;
}

static void remove_fixture(struct i2c_client *c)
{
	struct hp_unified_fixture *f = i2c_get_clientdata(c);

	cancel_delayed_work_sync(&g_wled_off_work);
	cancel_work_sync(&f->work);

	device_remove_file(&c->dev, &dev_attr_wled_pulse);
	device_remove_file(&c->dev, &dev_attr_wled_off);

	if (f->rear_sensor) { i2c_unregister_device(f->rear_sensor); f->rear_sensor = NULL; }
	if (f->rear_adev) { acpi_dev_put(f->rear_adev); f->rear_adev = NULL; }
	if (f->rear_graph_registered) { software_node_unregister_node_group(f->rear_group); f->rear_graph_registered = false; }
	if (f->ipu_dev) { put_device(f->ipu_dev); f->ipu_dev = NULL; }

	if (f->ir_sensor) { i2c_unregister_device(f->ir_sensor); f->ir_sensor = NULL; }
	if (f->ir_fw) { fwnode_handle_put(f->ir_fw); f->ir_fw = NULL; }
	if (f->ir_adev) { acpi_dev_put(f->ir_adev); f->ir_adev = NULL; }

	g_fixture = NULL;
	if (f->held) { f->held = false; module_put(THIS_MODULE); }
}


int hp_ir_calibration_safe(void);
int hp_ir_calibration_safe(void) { return 0; }
EXPORT_SYMBOL_GPL(hp_ir_calibration_safe);

static const struct acpi_device_id ids[] = { { "INT3472", 0 }, {} };
static struct i2c_driver driver = {
	.driver = {
		.name = "hp-camera-fixture",
		.acpi_match_table = ids,
		.suppress_bind_attrs = false
	},
	.probe = probe,
	.remove = remove_fixture
};

static int __init start(void) { return run ? i2c_add_driver(&driver) : -EPERM; }
static void __exit stop(void) { i2c_del_driver(&driver); }

module_init(start);
module_exit(stop);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HP Elite x2 1012 G2 Universal Camera & IR Fixture");

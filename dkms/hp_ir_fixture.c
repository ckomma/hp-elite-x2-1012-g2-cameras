// SPDX-License-Identifier: GPL-2.0
/* HP 82CA opt-in IR-only PMIC fixture. Experimental rail mapping; WLED2 test is default OFF. */
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/dmi.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/property.h>
#include <linux/property.h>
#include <linux/property.h>

static bool run;
module_param(run, bool, 0400);
static bool restored;
static bool detached;
module_param(detached, bool, 0444);
static int result = -EINPROGRESS;
module_param(restored, bool, 0444);
module_param(result, int, 0444);
#define HP_WINDOW (15 * HZ)
#define HP_POWER_LIMIT (10 * HZ)
/* W1C/RO bits in PLLCTL are excluded. No reset, LEDs, main CORE/ANA/VCM/AUX1. */
static const u8 regs[] = {6,7,8,9,10,11,12,13,15,16,0x1c,0x1e,0x20,0x27,0x3e,0x43,0x46};
static u8 owned_mask(u8 r)
{
 switch (r) {
 case 0x0d:return 0xf1;
 case 0x1c:case 0x1e:case 0x20:return 0xcb;
 case 0x27:return 0x70;
 case 0x3e:return 0x7f;
 case 0x43:return 3;
 case 0x46:return 1;
 default:return 0xff;
 }
}
struct hp_sync_edge {u64 lo,hi;u8 level;};
struct hp_ir {
 struct work_struct sync_work;
 struct hp_sync_edge sync_trace[32];
 u64 sync_anchor,sync_period,sync_cycle;
 u64 sync_until,sync_previous;
 unsigned int sync_n;u8 sync_level;
 bool sync_spent,sync_stop;
 struct delayed_work wled_work;
 unsigned long wled_until,power_until;
 bool wled_dirty,wled_held;
 u8 wled_saved[11];
 struct delayed_work led_work;
 unsigned long led_until;
 bool led_started;
 u8 led_before[15];
 struct i2c_client *c,*sensor;
 struct acpi_device *adev; struct fwnode_handle *fw;
 struct work_struct work;
 struct delayed_work deadline;
 u8 saved[ARRAY_SIZE(regs)];
 bool snapshot,dirty,ready,powered,held,started;
 int fault;
 unsigned long expires;
};
static struct hp_ir *owner;
static DEFINE_MUTEX(power_lock);
static int rd(struct hp_ir *s,u8 reg)
{
 u8 val;struct i2c_msg m[2]={{.addr=0x4d,.len=1,.buf=&reg},{.addr=0x4d,.flags=I2C_M_RD,.len=1,.buf=&val}};
 int ret=i2c_transfer(s->c->adapter,m,2);
 return ret==2?val:ret<0?ret:-EIO;
}
static int update(struct hp_ir *s,u8 reg,u8 value)
{
 int i,before,ret,after;u8 buf[2],mask;
 for(i=0;i<ARRAY_SIZE(regs);i++)if(regs[i]==reg)break;
 if(i==ARRAY_SIZE(regs))return -EPERM;
 mask=owned_mask(reg);before=rd(s,reg);if(before<0)return before;
 buf[0]=reg;buf[1]=(before&~mask)|(value&mask);
 if(reg==0x0d)buf[1]&=0xf1; /* Never replay W1C status. */
 s->dirty=true;
 ret=i2c_master_send(s->c,buf,2);if(ret!=2)return ret<0?ret:-EIO;
 after=rd(s,reg);if(after<0)return after;
 return (after&mask)==(value&mask)?0:-EIO;
}
static int data_bits(struct hp_ir *s,u8 mask,bool on)
{
 int v=rd(s,0x27);if(v<0)return v;
 return update(s,0x27,on?v|mask:v&~mask);
}
#include "hp_sync_observer.h"
#include "hp_power_diagnostic.h"
#include "hp_wled2_test.h"
static int off(struct hp_ir *s)
{
 int ret=0,r,i,v;
 s->sync_stop=true;
 r=wled_off_locked(s);if(r)return r;
 if(!s->dirty){s->powered=false;restored=true;return 0;}
#define TRY_OFF(expr) do{r=(expr);if(r<0&&!ret)ret=r;}while(0)
 /* Stop sensor gates before rails. Attempt every cleanup after failures. */
 TRY_OFF(data_bits(s,BIT(5),false));TRY_OFF(data_bits(s,BIT(6),false));usleep_range(1000,1500);
 TRY_OFF(data_bits(s,BIT(4),false));TRY_OFF(update(s,0x46,0));
 TRY_OFF(update(s,0x43,0));TRY_OFF(update(s,0x0d,0x80));TRY_OFF(update(s,0x0f,0));
 for(i=0;i<ARRAY_SIZE(regs);i++)if(regs[i]!=0x0d)TRY_OFF(update(s,regs[i],s->saved[i]));
 for(i=0;i<ARRAY_SIZE(regs);i++)if(regs[i]==0x0d)TRY_OFF(update(s,regs[i],s->saved[i]));
 for(i=0;i<ARRAY_SIZE(regs);i++){
  v=rd(s,regs[i]);if(v<0){if(!ret)ret=v;}else if((v&owned_mask(regs[i]))!=(s->saved[i]&owned_mask(regs[i]))&&!ret)ret=-EUCLEAN;
 }
 s->powered=false;restored=!ret;if(!ret)s->dirty=false;else s->fault=ret;
 return ret;
#undef TRY_OFF
}
static void deadline_work(struct work_struct *w)
{
 struct hp_ir *s=container_of(to_delayed_work(w),struct hp_ir,deadline);
 mutex_lock(&power_lock);
 if(s->powered||s->dirty){s->fault=-ETIMEDOUT;result=off(s);dev_warn(&s->c->dev,"IR power deadline, restore=%d\n",result);}
 mutex_unlock(&power_lock);
}
bool hp_ir_fixture_ready(void);
bool hp_ir_fixture_ready(void){return owner&&owner->ready;}
EXPORT_SYMBOL_GPL(hp_ir_fixture_ready);
int hp_ir_power(bool on);
int hp_ir_power(bool on)
{
 struct hp_ir *s;int ret=0,i;
 static const u8 clocks[][2]={{10,0xaa},{8,2},{7,3},{11,0x20},{12,1},{6,1},{16,5},{9,3},{13,0xd0},{15,0x0a},{13,0xd1}};
 mutex_lock(&power_lock);s=owner;
 if(!s||!s->ready){ret=-ENODEV;goto done;}
 if(!on){cancel_delayed_work(&s->deadline);ret=off(s);goto done;}
 if(s->fault||time_after_eq(jiffies,s->expires)){ret=s->fault?s->fault:-ETIMEDOUT;goto done;}
 if(s->powered)goto done;
 if(s->dirty){ret=-EIO;goto done;}
 restored=false;
 s->power_until=jiffies+min_t(unsigned long,HP_POWER_LIMIT,s->expires-jiffies);
 mod_delayed_work(system_wq,&s->deadline,s->power_until-jiffies);
#define STEP(expr) do{ret=(expr);if(ret<0)goto fail;}while(0)
 STEP(update(s,0x1c,0x8a));STEP(update(s,0x1e,0xc2));STEP(update(s,0x20,0xc2));
 STEP(update(s,0x3e,0x34)); /* Nominal AUX2 1.8006 V; physical DOVDD mapping unverified. */
 for(i=0;i<ARRAY_SIZE(clocks);i++)STEP(update(s,clocks[i][0],clocks[i][1]));
 usleep_range(5000,6000);
 STEP(update(s,0x43,3));
 STEP(data_bits(s,BIT(4),true)); /* Hypothesized external analog enable. */
 usleep_range(1000,1500);
 STEP(update(s,0x46,1));
 usleep_range(1000,1500);
 STEP(data_bits(s,BIT(5),true));STEP(data_bits(s,BIT(6),true));msleep(20);
 s->powered=true;goto done;
fail:
 s->fault=ret;result=off(s);
done:
 mutex_unlock(&power_lock);return ret;
#undef STEP
}
EXPORT_SYMBOL_GPL(hp_ir_power);
static int idle(struct hp_ir *s)
{
 int i,v;
 const u8 disabled[]={0x44,0x45,0x46,0x47,0x48};
 for(i=0;i<ARRAY_SIZE(disabled);i++){v=rd(s,disabled[i]);if(v<0)return v;if(v&1)return -EBUSY;}
 for(i=0;i<3;i++){v=rd(s,0x1d+2*i);if(v<0)return v;if(v&4)return -EBUSY;}
 v=rd(s,0x3f);if(v!=0x34)return v<0?v:-EBUSY;
 v=rd(s,0x40);if(v!=0x34)return v<0?v:-EBUSY;
 v=rd(s,0x22);if(v<0)return v;if(v&5)return -EBUSY;
 for(i=0;i<ARRAY_SIZE(regs);i++){
  v=rd(s,regs[i]);if(v<0)return v;s->saved[i]=v;
  if(regs[i]==13&&(v&0xf1)!=0x80)return -EBUSY;
  if(regs[i]==15&&v)return -EBUSY;
  if(regs[i]==0x27&&(v&0x70))return -EBUSY;
  if((regs[i]==0x1c||regs[i]==0x1e||regs[i]==0x20)&&((v&4)||((v&3)==3)))return -EBUSY;
  if((regs[i]==0x1c||regs[i]==0x1e||regs[i]==0x20)&&((v&4)||((v&3)==3)))return -EBUSY;
  if((regs[i]==0x1c||regs[i]==0x1e||regs[i]==0x20)&&((v&4)||((v&3)==3)))return -EBUSY;
  if(regs[i]==0x43&&(v&3))return -EBUSY;
 }
 s->snapshot=true;restored=true;return 0;
}
static bool graph_restored=true;
module_param(graph_restored,bool,0444);
static int detach(struct hp_ir *s)
{
 if(s->wled_dirty)return -EBUSY;
 if(s->sensor&&s->sensor->dev.driver)return -EBUSY;
 s->ready=false;
 if(s->sensor){i2c_unregister_device(s->sensor);s->sensor=NULL;}
 if(s->fw){
  if(!s->adev || acpi_fwnode_handle(s->adev)->secondary!=s->fw ||
     software_node_fwnode(to_software_node(s->fw))!=s->fw ||
     fwnode_graph_get_endpoint_count(s->fw,0)!=1)
   return -EUCLEAN;
  graph_restored=true;
  fwnode_handle_put(s->fw);s->fw=NULL;
 }
 if(s->adev){acpi_dev_put(s->adev);s->adev=NULL;}
 return 0;
}
static void fixture_work(struct work_struct *w)
{
 struct hp_ir *s=container_of(w,struct hp_ir,work);struct i2c_adapter *adap;acpi_handle expected;int ret;
 struct i2c_board_info info={I2C_BOARD_INFO("hp-ov7251-ir-test",0x60)};
 if(!s->started){
  s->started=true;ret=idle(s);if(ret){result=ret;goto release;}
  s->adev=acpi_dev_get_first_match_dev("INT347E",NULL,-1);
  if(!s->adev){result=-ENODEV;goto release;}
  s->fw=fwnode_handle_get(acpi_fwnode_handle(s->adev)->secondary);
  if(!s->fw||!is_software_node(s->fw)||fwnode_graph_get_endpoint_count(s->fw,0)!=1){result=-ENOLINK;goto release;}
  adap=i2c_get_adapter(2);if(!adap){result=-ENODEV;goto release;}
  if(ACPI_FAILURE(acpi_get_handle(NULL,"\\_SB_.PCI0.I2C3",&expected)) ||
     ACPI_HANDLE(adap->dev.parent)!=expected){i2c_put_adapter(adap);result=-ENODEV;goto release;}
  s->expires=jiffies+HP_WINDOW;s->ready=true;
  /* Do not borrow the bridge secondary as a primary I2C fwnode.
   * The I2C swnode API owns an attachment reference and clears the
   * device association before device_del's second notify path. */
  graph_restored=false;
  info.swnode=to_software_node(s->fw);
  s->sensor=i2c_new_client_device(adap,&info);i2c_put_adapter(adap);
  if(IS_ERR(s->sensor)){ret=PTR_ERR(s->sensor);s->sensor=NULL;result=ret;goto release;}
  result=0;return;
 }
release:
 mutex_lock(&power_lock);ret=wled_off_locked(s);mutex_unlock(&power_lock);
 if(ret){result=ret;return;}
 ret=detach(s);if(s->snapshot){int power_ret;mutex_lock(&power_lock);power_ret=off(s);mutex_unlock(&power_lock);if(!ret)ret=power_ret;}
 if(ret)result=ret;
 if(!s->snapshot)restored=true;
 if(!ret){detached=true;if(s->held){s->held=false;module_put(THIS_MODULE);}}
}

/* Passive LED-backend observation. Never enable/reset/write LED registers. */
static bool led_observe, led_observing, led_config_changed;
static unsigned int led_samples, led_high_samples, led_changes;
static unsigned int led_begin_28, led_begin_29, led_begin_31, led_begin_36;
static int led_observer_error;
static int led_previous = -1;
module_param(led_observe, bool, 0400);
module_param(led_observing, bool, 0444);
module_param(led_config_changed, bool, 0444);
module_param(led_samples, uint, 0444);
module_param(led_high_samples, uint, 0444);
module_param(led_changes, uint, 0444);
module_param(led_begin_28, uint, 0444);
module_param(led_begin_29, uint, 0444);
module_param(led_begin_31, uint, 0444);
module_param(led_begin_36, uint, 0444);
module_param(led_observer_error, int, 0444);
static int led_read(struct hp_ir *s, u8 reg)
{
 if (reg < 0x28 || reg > 0x36)
  return -EPERM;
 return rd(s, reg);
}
static int led_snapshot(struct hp_ir *s, bool begin)
{
 unsigned int i;
 int value;
 for (i = 0; i < 15; i++) {
  u8 reg = 0x28 + i;
  value = led_read(s, reg);
  if (value < 0)
   return value;
  dev_info(&s->c->dev, "HP_LED_SNAPSHOT %s %02x=%02x\n",
           begin ? "begin" : "end", reg, value);
  if (begin)
   s->led_before[i] = value;
  else {
   u8 mask = reg == 0x28 ? 0x74 : reg == 0x29 ? 0x01 :
             reg == 0x31 ? 0x3d : reg == 0x36 ? 0xff : 0;
   if ((value ^ s->led_before[i]) & mask)
    led_config_changed = true;
  }
 }
 if (begin) {
  led_begin_28 = s->led_before[0];
  led_begin_29 = s->led_before[1];
  led_begin_31 = s->led_before[9];
  led_begin_36 = s->led_before[14];
 }
 return 0;
}
static void led_poll(struct work_struct *work)
{
 struct hp_ir *s = container_of(to_delayed_work(work), struct hp_ir, led_work);
 int value;
 mutex_lock(&power_lock);
 if (!led_observing)
  goto done;
 if (!s->ready || led_samples >= 512 || time_after_eq(jiffies, s->led_until)) {
  led_observing = false;
  goto done;
 }
 value = led_read(s, 0x29);
 if (value < 0) {
  led_observer_error = value;
  led_observing = false;
  goto done;
 }
 led_samples++;
 if (value & BIT(7))
  led_high_samples++;
 if (led_previous < 0 || ((value ^ led_previous) & BIT(7))) {
  led_changes++;
 }
 /* Bounded diagnostic trace: all samples allow a measured maximum gap. */
 dev_info(&s->c->dev, "HP_LED_SAMPLE n=%u status29=%02x\n", led_samples, value);
 led_previous = value;
 if (led_samples < 512 && time_before(jiffies, s->led_until))
  mod_delayed_work(system_wq, &s->led_work, msecs_to_jiffies(2));
 else
  led_observing = false;
done:
 mutex_unlock(&power_lock);
}
static void led_stop(struct hp_ir *s)
{
 int ret;
 mutex_lock(&power_lock);
 led_observing = false;
 mutex_unlock(&power_lock);
 cancel_delayed_work_sync(&s->led_work);
 mutex_lock(&power_lock);
 if (s->led_started) {
  ret = led_snapshot(s, false);
  if (ret)
   led_observer_error = ret;
  dev_info(&s->c->dev, "HP_LED_END samples=%u high=%u changes=%u error=%d changed=%d\n",
           led_samples, led_high_samples, led_changes, led_observer_error, led_config_changed);
 }
 mutex_unlock(&power_lock);
}
static int led_start_set(const char *value, const struct kernel_param *kp)
{
 struct hp_ir *s = owner;
 bool yes;
 int ret = kstrtobool(value, &yes);
 if (ret || !yes)
  return -EINVAL;
 if (!led_observe || !s || detached)
  return -EPERM;
 mutex_lock(&power_lock);
 if (!s->ready || s->led_started) { ret = -EBUSY; goto out; }
 s->led_started = true;
 ret = led_snapshot(s, true);
 if (ret) { led_observer_error = ret; goto out; }
 /* Unexpected enabled/current-producing LED backend: no new STROBE experiment. */
 if ((led_begin_28 & 0x44) || (led_begin_31 & 1) || (led_begin_36 & 0x24)) {
  ret = -EBUSY; led_observer_error = ret; goto out;
 }
 s->led_until = jiffies + 5 * HZ;
 led_samples = led_high_samples = led_changes = 0;
 led_previous = -1;
 led_config_changed = false;
 led_observing = true;
 mod_delayed_work(system_wq, &s->led_work, msecs_to_jiffies(2));
 ret = 0;
out:
 mutex_unlock(&power_lock);
 return ret;
}
static int led_stop_set(const char *value, const struct kernel_param *kp)
{
 bool yes;
 int ret = kstrtobool(value, &yes);
 if (ret || !yes || !owner)
  return -EINVAL;
 sync_observer_stop(owner);
 led_stop(owner);
 return 0;
}
static const struct kernel_param_ops led_start_ops = { .set = led_start_set };
static const struct kernel_param_ops led_stop_ops = { .set = led_stop_set };
module_param_cb(led_sample_start, &led_start_ops, NULL, 0200);
module_param_cb(led_sample_stop, &led_stop_ops, NULL, 0200);

static int recover_set(const char *value,const struct kernel_param *kp)
{
 bool yes;int ret=kstrtobool(value,&yes);if(ret||!yes)return -EINVAL;
 if(!owner)return -ENODEV;
 if(detached)return 0;
 if(work_busy(&owner->work))return -EBUSY;
 if(owner->sensor&&owner->sensor->dev.driver)return -EBUSY;
 sync_observer_stop(owner);
 led_stop(owner);
 cancel_delayed_work_sync(&owner->deadline);schedule_work(&owner->work);return 0;
}
static const struct kernel_param_ops recover_ops={.set=recover_set};
module_param_cb(recover,&recover_ops,NULL,0200);
static int probe(struct i2c_client *c)
{
 struct hp_ir *s;int v;
 if(!run||!dmi_match(DMI_SYS_VENDOR,"HP")||!dmi_match(DMI_PRODUCT_NAME,"HP Elite x2 1012 G2")||!dmi_match(DMI_BOARD_NAME,"82CA")||strcmp(dev_name(&c->dev),"i2c-INT3472:05")||c->addr!=0x4d||c->adapter->nr!=1)return -ENODEV;
 if(!i2c_check_functionality(c->adapter,I2C_FUNC_I2C))return -EOPNOTSUPP;
 s=devm_kzalloc(&c->dev,sizeof(*s),GFP_KERNEL);if(!s)return -ENOMEM;s->c=c;
 v=rd(s,0xff);if(v!=0x21)return v<0?v:-ENODEV;
 INIT_WORK(&s->sync_work,sync_observe_work);INIT_DELAYED_WORK(&s->wled_work,wled_timeout);INIT_WORK(&s->work,fixture_work);INIT_DELAYED_WORK(&s->deadline,deadline_work);INIT_DELAYED_WORK(&s->led_work,led_poll);
 i2c_set_clientdata(c,s);owner=s;__module_get(THIS_MODULE);s->held=true;schedule_work(&s->work);return 0;
}
static void remove_fixture(struct i2c_client *c)
{
 struct hp_ir *s=i2c_get_clientdata(c);
 sync_observer_stop(s);
 cancel_delayed_work_sync(&s->wled_work);
 led_stop(s);
 cancel_work_sync(&s->work);cancel_delayed_work_sync(&s->deadline);
 detach(s);if(s->snapshot&&!restored)off(s);owner=NULL;
}
static const struct acpi_device_id ids[]={{"INT3472",0},{}};
/* No MODULE_DEVICE_TABLE, no autoload alias and no force-unbind interface. */
static struct i2c_driver driver={.driver={.name="hp-ir-fixture",.acpi_match_table=ids,.suppress_bind_attrs=true},.probe=probe,.remove=remove_fixture};
static int __init start(void){return run?i2c_add_driver(&driver):-EPERM;}
static void __exit stop(void){i2c_del_driver(&driver);}
module_init(start);module_exit(stop);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HP82CA exclusive bounded OV7251 PMIC experiment; unverified rail mapping");





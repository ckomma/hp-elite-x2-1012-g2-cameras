// SPDX-License-Identifier: GPL-2.0
/* HP 82CA one-shot OV8858 ID diagnostic. No media devices, no sensor writes. */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/pci.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <media/v4l2-fwnode.h>

static bool run;
module_param(run, bool, 0400);
static int result = -EINPROGRESS;
module_param(result, int, 0444);
static bool restored;
static bool hp_powered;
module_param(restored, bool, 0444);
static const u8 regs[] = {6,7,8,9,10,11,12,13,15,16,0x22,0x3f,0x40,0x41,0x42,0x43,0x47,0x48};
struct diag { struct i2c_client *c; struct work_struct work; u8 saved[ARRAY_SIZE(regs)]; bool snapshot, dirty, started, held; struct i2c_client *sensor; struct acpi_device *adev; struct device *ipu; bool graph, ready; struct software_node nodes[5]; const struct software_node *group[6]; struct property_entry devprops[4], portprops[2], epprops[5], iprops[5]; struct software_node_ref_args local_ref[1], remote_ref[1]; unsigned long deadline; };
static struct diag *owner;
static int rd(struct diag *s, u16 addr, u16 reg, bool sensor)
{
 u8 a[2] = {reg >> 8, reg}, v;
 struct i2c_msg msg[2] = {{.addr=addr,.len=sensor?2:1,.buf=sensor?a:&a[1]},
                        {.addr=addr,.flags=I2C_M_RD,.len=1,.buf=&v}};
 int ret=i2c_transfer(s->c->adapter,msg,2);
 return ret==2?v:ret<0?ret:-EIO;
}
static int wr(struct diag *s, u8 reg, u8 val)
{
 u8 b[2]={reg,val}; int i,ret;
 for(i=0;i<ARRAY_SIZE(regs);i++) if(regs[i]==reg) break;
 if(i==ARRAY_SIZE(regs)) return -EPERM; /* RESET50, LEDs, AUX, GPIO4/5/6 forbidden */
 if(reg==0x0d) b[1]&=0xf1; /* never replay W1C/RO status bits */
 s->dirty=true;
 ret=i2c_master_send(s->c,b,2);
 return ret==2?0:ret<0?ret:-EIO;
}
static int update(struct diag *s,u8 reg,u8 mask,u8 val)
{
 int old=rd(s,s->c->addr,reg,false);
 if(old<0) return old;
 return wr(s,reg,(old&~mask)|(val&mask));
}
static u8 mask(u8 reg) { return reg==0x0d?0xf1:0xff; } /* exclude RO and W1C status */
static int cleanup(struct diag *s)
{
 int i,ret=0,v,r;
 if(!s->dirty) { hp_powered=false; restored=true; return 0; }
 /* Always attempt all restoration steps even after one failed I2C transfer. */
#define RESTORE(expr) do { r=(expr); if(r<0 && !ret) ret=r; } while(0)
 RESTORE(update(s,0x22,4,0)); usleep_range(1000,1500);
 RESTORE(update(s,0x22,1,0));
 RESTORE(update(s,0x47,1,0)); RESTORE(update(s,0x48,1,0));
 RESTORE(update(s,0x43,3,0));
 /* CORE is off before stopping the PMIC PLL. */
 RESTORE(update(s,0x0d,0xf1,0x80)); RESTORE(wr(s,0x0f,0));
 for(i=0;i<ARRAY_SIZE(regs);i++) {
  if(regs[i]==0x0d) continue;
  RESTORE(wr(s,regs[i],s->saved[i]));
 }
 for(i=0;i<ARRAY_SIZE(regs);i++) if(regs[i]==0x0d)
  RESTORE(wr(s,0x0d,s->saved[i]&0xf1));
 for(i=0;i<ARRAY_SIZE(regs);i++) {
  v=rd(s,s->c->addr,regs[i],false);
  if(v<0) { if(!ret) ret=v; }
  else if((v&mask(regs[i]))!=(s->saved[i]&mask(regs[i]))) { if(!ret) ret=-EUCLEAN; }
 }
 hp_powered=false; restored=!ret;
 dev_info(&s->c->dev,"HP_REAR_ID cleanup=%d restored=%d\n",ret,restored);
 return ret;
#undef RESTORE
}
static int check_sensor_client(struct device *dev,void *data)
{
 struct diag *s=data; struct i2c_client *c=i2c_verify_client(dev);
 return c && c->adapter==s->c->adapter && (c->addr==0x10 || c->addr==0x60) ? -EBUSY:0;
}
static int idle(struct diag *s)
{
 int r,i;
 const u8 check[]={0x0e,0x44,0x45,0x46};
 for(i=0;i<ARRAY_SIZE(check);i++) { r=rd(s,0x4d,check[i],false); if(r<0)return r; if(r&1)return -EBUSY; }
 /* Strict reset-like clock/control baseline; never take over an active PMIC. */
 for(i=0;i<ARRAY_SIZE(regs);i++) {
  r=rd(s,0x4d,regs[i],false); if(r<0)return r; s->saved[i]=r;
  if(regs[i]==0x0d && (r&0xf1)!=0x80)return -EBUSY;
  if(regs[i]==0x22 && (r&5))return -EBUSY;
  if(regs[i]==0x43 && (r&3))return -EBUSY;
  if((regs[i]==0x47 || regs[i]==0x48) && (r&1))return -EBUSY;
  if(regs[i]==0x0f && r)return -EBUSY;
 }
 r=bus_for_each_dev(&i2c_bus_type,NULL,s,check_sensor_client);
 if(r)return r;
 s->snapshot=true; return 0;
}
static int experiment(struct diag *s)
{
 int r,a,b,c,i; u32 id;
 static const u8 clocks[][2]={{0x0a,0xaa},{8,2},{7,3},{0x0b,0x20},{0x0c,1},{6,1},{0x10,5},{9,3},{0x0d,0xd0},{0x0f,0x0a},{0x0d,0xd1}};
 static const u16 sensor_regs[]={0x0100,0x0300,0x0301,0x0302,0x0303,0x0304,0x030a,0x030b,0x030c,0x030d,0x030e,0x030f,0x0312,0x3020,0x3032,0x3033,0x3106};
#define TRY(expr) do { if(time_after(jiffies,s->deadline))return -ETIMEDOUT; r=(expr); if(r<0)return r; } while(0)
 TRY(rd(s,0x4d,0xff,false)); if(r!=0x21)return -ENODEV;
 TRY(idle(s));
 /* No global reset, MFD children, ACPI dependency clearing, VCM or AUX enable. */
 TRY(wr(s,0x41,0x6d)); TRY(wr(s,0x42,0x0c));
 TRY(wr(s,0x3f,0x34)); TRY(wr(s,0x40,0x34));
 for(i=0;i<ARRAY_SIZE(clocks);i++) TRY(wr(s,clocks[i][0],clocks[i][1]));
 usleep_range(5000,6000);
 TRY(update(s,0x43,3,3)); /* shared I/O only after confirmed inactive baseline */
 TRY(update(s,0x47,1,1)); TRY(update(s,0x48,1,1));
 usleep_range(1000,1500);
 TRY(update(s,0x22,4,4)); usleep_range(1000,1500);
 TRY(update(s,0x22,1,1)); msleep(20);
 TRY(rd(s,0x10,0x0100,true)); if(r&1)return -EBUSY;
 TRY(rd(s,0x10,0x300a,true)); a=r;
 TRY(rd(s,0x10,0x300b,true)); b=r;
 TRY(rd(s,0x10,0x300c,true)); c=r;
 id=(a<<16)|(b<<8)|c;
 dev_info(&s->c->dev,"HP_REAR_ID chip=%06x\n",id);
 if(id!=0x008858)return -ENODEV;
 TRY(rd(s,0x10,0x302a,true));
 dev_info(&s->c->dev,"HP_REAR_ID revision=%02x\n",r);
 if(r!=0xb2)return -EOPNOTSUPP;
 for(i=0;i<ARRAY_SIZE(sensor_regs);i++) {
  TRY(rd(s,0x10,sensor_regs[i],true));
  dev_info(&s->c->dev,"HP_REAR_ID default %04x=%02x\n",sensor_regs[i],r);
 }
 return 0;
#undef TRY
}

static const u32 lanes[]={1,2,3,4};
static const u64 link_freq[]={360000000};
static int attach_sensor(struct diag *s)
{
 struct fwnode_handle *root,*ep=NULL; const struct software_node *parent;
 struct i2c_board_info info={I2C_BOARD_INFO("hp-ov8858-test",0x10)};
 struct fwnode_endpoint parsed; int ret,i;
 s->ipu=bus_find_device_by_name(&pci_bus_type,NULL,"0000:00:14.3");
 if(!s->ipu)return -ENODEV;
 root=dev_fwnode(s->ipu);
 while(root && !is_software_node(root))root=root->secondary;
 parent=to_software_node(root);
 if(!parent)return -ENODEV;
 /* Refuse a collision: never overwrite existing front/IR graph nodes. */
 while((ep=fwnode_graph_get_next_endpoint(root,ep))) {
  ret=fwnode_graph_parse_endpoint(ep,&parsed);
  if(ret || parsed.port==0) { fwnode_handle_put(ep); return ret?ret:-EEXIST; }
 }
 s->devprops[0]=PROPERTY_ENTRY_U32("clock-frequency",19200000);
 s->devprops[1]=PROPERTY_ENTRY_U32("rotation",0);
 s->devprops[2]=PROPERTY_ENTRY_U32("orientation",V4L2_FWNODE_ORIENTATION_BACK);
 s->portprops[0]=PROPERTY_ENTRY_U32("reg",0);
 s->nodes[0]=(struct software_node){.name="HP-OV8858-R2A-test",.properties=s->devprops};
 s->nodes[1]=(struct software_node){.name="port@0",.parent=&s->nodes[0],.properties=s->portprops};
 s->nodes[2]=(struct software_node){.name="endpoint@0",.parent=&s->nodes[1],.properties=s->epprops};
 s->nodes[3]=(struct software_node){.name="port@0",.parent=parent,.properties=s->portprops};
 s->nodes[4]=(struct software_node){.name="endpoint@0",.parent=&s->nodes[3],.properties=s->iprops};
 s->local_ref[0]=SOFTWARE_NODE_REFERENCE(&s->nodes[4]);
 s->remote_ref[0]=SOFTWARE_NODE_REFERENCE(&s->nodes[2]);
 s->epprops[0]=PROPERTY_ENTRY_U32("bus-type",V4L2_FWNODE_BUS_TYPE_CSI2_DPHY);
 s->epprops[1]=PROPERTY_ENTRY_U32_ARRAY("data-lanes",lanes);
 s->epprops[2]=PROPERTY_ENTRY_REF_ARRAY("remote-endpoint",s->local_ref);
 s->epprops[3]=PROPERTY_ENTRY_U64_ARRAY("link-frequencies",link_freq);
 s->iprops[0]=PROPERTY_ENTRY_U32_ARRAY("data-lanes",lanes);
 s->iprops[1]=PROPERTY_ENTRY_REF_ARRAY("remote-endpoint",s->remote_ref);
 for(i=0;i<5;i++)s->group[i]=&s->nodes[i];
 ret=software_node_register_node_group(s->group);
 if(ret)return ret;
 s->graph=true; s->ready=true;
 s->adev=acpi_dev_get_first_match_dev("INT3477",NULL,-1);
 if(!s->adev)return -ENODEV;
 if(acpi_fwnode_handle(s->adev)->secondary)return -EEXIST;
 /* I2C acquires the attachment reference before device_register(). */
 info.swnode=&s->nodes[0];
 info.fwnode=acpi_fwnode_handle(s->adev);
 s->sensor=i2c_new_client_device(s->c->adapter,&info);
 if(IS_ERR(s->sensor)) { ret=PTR_ERR(s->sensor); s->sensor=NULL; return ret; }
 return 0;
}
static int detach_sensor(struct diag *s)
{
 if(s->sensor && s->sensor->dev.driver)return -EBUSY;
 s->ready=false;
 if(s->sensor) { i2c_unregister_device(s->sensor); s->sensor=NULL; }
 if(s->adev) {
  /* i2c_unregister_device removed its software-node attachment above. */
  acpi_dev_put(s->adev); s->adev=NULL;
 }
 if(s->graph) { software_node_unregister_node_group(s->group); s->graph=false; }
 if(s->ipu) { put_device(s->ipu); s->ipu=NULL; }
 return 0;
}
bool hp_rear_fixture_ready(void);
bool hp_rear_fixture_ready(void)
{
 return owner && owner->ready;
}
EXPORT_SYMBOL_GPL(hp_rear_fixture_ready);
static DEFINE_MUTEX(hp_power_mutex);
int hp_rear_power(bool on);
int hp_rear_power(bool on)
{
 struct diag *s; int r=0,i;
 mutex_lock(&hp_power_mutex); s=owner;
 if(!s || !s->ready) { r=-ENODEV; goto out; }
 if(!on) { r=cleanup(s); goto out; }
 if(!restored) { r=hp_powered?0:-EIO; goto out; }
 restored=false; s->deadline=jiffies+30*HZ;
 static const u8 clocks[][2]={{0x0a,0xaa},{8,2},{7,3},{0x0b,0x20},{0x0c,1},{6,1},{0x10,5},{9,3},{0x0d,0xd0},{0x0f,0x0a},{0x0d,0xd1}};
#define RESUME(expr) do { if(time_after(jiffies,s->deadline)){r=-ETIMEDOUT;goto fail;} r=(expr);if(r<0)goto fail;} while(0)
 RESUME(wr(s,0x41,0x6d)); RESUME(wr(s,0x42,0x0c));
 RESUME(wr(s,0x3f,0x34)); RESUME(wr(s,0x40,0x34));
 for(i=0;i<ARRAY_SIZE(clocks);i++) RESUME(wr(s,clocks[i][0],clocks[i][1]));
 usleep_range(5000,6000);
 RESUME(update(s,0x43,3,3)); /* shared I/O only after confirmed inactive baseline */
 RESUME(update(s,0x47,1,1)); RESUME(update(s,0x48,1,1));
 usleep_range(1000,1500);
 RESUME(update(s,0x22,4,4)); usleep_range(1000,1500);
 RESUME(update(s,0x22,1,1)); msleep(20);

#undef RESUME
 hp_powered=true; goto out;
fail:
 cleanup(s);
out:
 mutex_unlock(&hp_power_mutex); return r;
}
EXPORT_SYMBOL_GPL(hp_rear_power);
static void work(struct work_struct *w)
{
 struct diag *s=container_of(w,struct diag,work); int r,c;
 s->deadline=jiffies+30*HZ;
 if(!s->started) {
  r=experiment(s); s->started=true;
  if(!r) { hp_powered=true; r=attach_sensor(s); }
  if(!r) { result=0; dev_info(&s->c->dev,"HP_REAR_FIXTURE ready powered graph attached\n"); return; }
 } else r=result;
 c=detach_sensor(s);
 if(!c)c=cleanup(s);
 result=c?c:r;
 dev_info(&s->c->dev,"HP_REAR_FIXTURE result=%d restored=%d\n",result,restored);
 if(restored && s->held) { s->held=false; module_put(THIS_MODULE); }
}

static int recover_set(const char *val,const struct kernel_param *kp)
{
 bool retry; int ret=kstrtobool(val,&retry);
 if(ret || !retry)return -EINVAL;
 if(!owner)return -ENODEV;
 if(restored && !owner->sensor && !owner->graph)return 0;
 if(!owner->started || work_busy(&owner->work))return -EBUSY;
 if(owner->sensor && owner->sensor->dev.driver)return -EBUSY;
 result=-ESHUTDOWN; schedule_work(&owner->work); return 0;
}
static const struct kernel_param_ops recover_ops={.set=recover_set};
module_param_cb(recover,&recover_ops,NULL,0200);
static int probe(struct i2c_client *c)
{
 struct diag *s;
 if(!run || !dmi_match(DMI_BOARD_NAME,"82CA") || !dmi_match(DMI_PRODUCT_NAME,"HP Elite x2 1012 G2") ||
    strcmp(dev_name(&c->dev),"i2c-INT3472:05") || c->addr!=0x4d || c->adapter->nr!=1)return -ENODEV;
 if(!i2c_check_functionality(c->adapter,I2C_FUNC_I2C))return -EOPNOTSUPP;
 s=devm_kzalloc(&c->dev,sizeof(*s),GFP_KERNEL); if(!s)return -ENOMEM;
 s->c=c; i2c_set_clientdata(c,s); INIT_WORK(&s->work,work);
 owner=s; __module_get(THIS_MODULE); s->held=true;
 schedule_work(&s->work); return 0;
}
static void remove_diag(struct i2c_client *c)
{
 struct diag *s=i2c_get_clientdata(c);
 cancel_work_sync(&s->work);
 if(!detach_sensor(s) && s->snapshot && !restored)cleanup(s);
 owner=NULL;
}
static const struct acpi_device_id match[]={{"INT3472",0},{}};
/* No MODULE_DEVICE_TABLE: never autoload this diagnostic. */
static struct i2c_driver drv={.driver={.name="hp-rear-fixture",.acpi_match_table=match,.suppress_bind_attrs=true},.probe=probe,.remove=remove_diag};
static int __init start(void) { if(!run)return -EPERM; return i2c_add_driver(&drv); }
static void __exit stop(void) { i2c_del_driver(&drv); }
module_init(start); module_exit(stop);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Opt-in bounded HP82CA rear capture fixture; PMIC snapshot restore");

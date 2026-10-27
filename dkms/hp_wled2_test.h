static bool wled_prepared,wled_start_attempted;
static unsigned long long wled_prepared_until_ns,wled_target_ns;
static unsigned long long wled_start_bus_begin_ns,wled_start_bus_end_ns,wled_start_readback_ns;
static unsigned long long wled_off_bus_begin_ns,wled_off_readback_ns,wled_verified_ns;
static unsigned int wled_start_readback_value,wled_status29,wled_status31,wled_status36;
module_param(wled_prepared,bool,0444);module_param(wled_start_attempted,bool,0444);
module_param(wled_target_ns,ullong,0444);
module_param(wled_prepared_until_ns,ullong,0444);
module_param(wled_start_bus_begin_ns,ullong,0444);module_param(wled_start_bus_end_ns,ullong,0444);
module_param(wled_start_readback_ns,ullong,0444);module_param(wled_start_readback_value,uint,0444);
module_param(wled_off_bus_begin_ns,ullong,0444);module_param(wled_off_readback_ns,ullong,0444);module_param(wled_verified_ns,ullong,0444);
module_param(wled_status29,uint,0444);module_param(wled_status31,uint,0444);module_param(wled_status36,uint,0444);
/* SPDX-License-Identifier: GPL-2.0 */
/* Included only by the opt-in HP fixture. No init-time WLED writes. */
static bool wled_test;
static bool wled_active, wled_spent, wled_off_verified = true;
static int wled_error;
module_param(wled_test, bool, 0400);
module_param(wled_active, bool, 0444);
module_param(wled_spent, bool, 0444);
module_param(wled_off_verified, bool, 0444);
module_param(wled_error, int, 0444);
#include "hp_wled_telemetry.h"
static const u8 wled_regs[] = {0x29,0x2c,0x2d,0x2e,0x2f,0x30,0x32,0x33,0x34,0x35,0x36};
static u8 wled_mask(u8 reg)
{
 switch (reg) {
 case 0x29:return 1;
 case 0x2c:case 0x2e:case 0x2f:case 0x34:case 0x35:return 0x1f;
 case 0x2d:return 7;
 case 0x30:return 0xf7;
 case 0x32:return 3;
 case 0x33:case 0x36:return 0xff;
 default:return 0;
 }
}
static int wled_put(struct hp_ir *s, u8 reg, u8 value)
{
 u8 bytes[2], mask=wled_mask(reg);
 int before,ret,after;
 if (!mask || (value & ~mask)) return -EPERM;
 before=rd(s,reg);if(before<0)return before;
 bytes[0]=reg;
 /* 0x29 contains RO status: never replay it. All other owned bits are RW. */
 bytes[1]=reg==0x29?value:(before&~mask)|value;
 s->wled_dirty=true;wled_off_verified=false;
 if(reg==0x36&&value==0x6c){
  u64 now=ktime_get_ns();
  if(now+1000000ULL<wled_target_ns||now>wled_target_ns+1000000ULL)return -ETIME;
  wled_start_attempted=true;wled_start_bus_begin_ns=now;
 }
 if(reg==0x36&&value==0x18&&wled_active&&!wled_off_bus_begin_ns)wled_off_bus_begin_ns=ktime_get_ns();
 if(reg==0x36&&value==0x4c)wled_en_begin_ns=ktime_get_ns();
 ret=i2c_master_send(s->c,bytes,2);
 if(reg==0x36&&value==0x6c)wled_start_transfer_ret=ret;
 if(reg==0x36&&value==0x6c)wled_start_bus_end_ns=ktime_get_ns();
 if(ret!=2)return ret<0?ret:-EIO;
 after=rd(s,reg);if(after<0)return after;
 if(reg==0x36&&value==0x4c)wled_en_verified_ns=ktime_get_ns();
 if(reg==0x36&&value==0x6c){wled_start_readback_ns=ktime_get_ns();wled_start_readback_value=after;}
 if(reg==0x36&&value==0x18&&wled_active)wled_off_readback_ns=ktime_get_ns();
 if(reg==0x36)mask&=~0x20; /* START is the actual pulse state, not a latch. */
 return (after&mask)==(value&mask)?0:-EIO;
}
/* power_lock held. Always clear EN first; never restore an enabled snapshot. */
static int wled_off_locked(struct hp_ir *s)
{
 int ret=0,r,v,i;
 if(!s->wled_dirty){if(s->wled_held){s->wled_held=false;module_put(THIS_MODULE);}return 0;}
#define WLED_TRY(expr) do {r=(expr);if(r<0&&!ret)ret=r;} while(0)
 WLED_TRY(wled_put(s,0x36,0x18));
 WLED_TRY(wled_put(s,0x34,0));WLED_TRY(wled_put(s,0x35,0));
 v=rd(s,0x36);
 if(v<0 || (v&0x24)) {
  if(!ret)ret=v<0?v:-EBUSY;
  goto failed;
 }
 /* Only saved OFF configurations can reach here. Restore ENTMON last. */
 for(i=1;i<ARRAY_SIZE(wled_regs);i++)
  WLED_TRY(wled_put(s,wled_regs[i],s->wled_saved[i]));
 WLED_TRY(wled_put(s,0x29,s->wled_saved[0]));
 for(i=0;i<ARRAY_SIZE(wled_regs);i++) {
  u8 mask=wled_mask(wled_regs[i]);if(wled_regs[i]==0x36)mask=0xff;
  v=rd(s,wled_regs[i]);
  if(v<0) {if(!ret)ret=v;}
  else if((v&mask)!=s->wled_saved[i]&&!ret)ret=-EUCLEAN;
 }
 if(ret)goto failed;
 s->wled_dirty=false;wled_active=false;wled_prepared=false;wled_off_verified=true;wled_verified_ns=ktime_get_ns();
 power_diag_after(s);
 if(s->wled_held){s->wled_held=false;module_put(THIS_MODULE);}
 return 0;
failed:
 wled_error=ret;s->fault=ret;wled_off_verified=false;
 if(!wled_failure_valid){wled_phase_set(19);wled_failure_mark(s,ret);}
 if(wled_failure_off_ret==-ENODATA)wled_failure_off_ret=ret;
 /* Keep module/context pinned. Recovery callback can retry OFF only. */
 return ret;
#undef WLED_TRY
}
static int wled_sensor_quiet(struct hp_ir *s)
{
 const u16 addresses[]={0x0100,0x3005,0x3009,0x3027,0x3b96,0x3b8b};
 const u8 masks[]={1,12,8,8,0x80,255};
 int i,ret;u8 value,addr[2];
 if(!s->sensor || !s->sensor->dev.driver)return -ENODEV;
 for(i=0;i<ARRAY_SIZE(addresses);i++) {
  struct i2c_msg msgs[2]={{.addr=0x60,.len=2,.buf=addr},{.addr=0x60,.flags=I2C_M_RD,.len=1,.buf=&value}};
  addr[0]=addresses[i]>>8;addr[1]=addresses[i]&0xff;
  ret=i2c_transfer(s->sensor->adapter,msgs,2);if(ret!=2)return ret<0?ret:-EIO;
  /* Streaming required, every STROBE/PWM/manual output gate must be off. */
  if((value&masks[i])!=(i==0?1:i==5?5:0))return -EBUSY;
 }
 return 0;
}

#include "hp_sync_wled_core.h"
static int wled_recover_set(const char *value,const struct kernel_param *kp)
{
 bool yes;int ret=kstrtobool(value,&yes);
 if(ret||!yes||!owner)return -EINVAL;
 cancel_delayed_work_sync(&owner->wled_work);
 mutex_lock(&power_lock);ret=wled_off_locked(owner);mutex_unlock(&power_lock);return ret;
}
static const struct kernel_param_ops wled_recover_ops={.set=wled_recover_set};
module_param_cb(wled_recover,&wled_recover_ops,NULL,0200);

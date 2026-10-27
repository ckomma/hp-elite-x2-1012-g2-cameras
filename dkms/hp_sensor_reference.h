/* SPDX-License-Identifier: GPL-2.0 */
/* Reference only; WLED must remain disabled throughout this bounded interval. */
extern int hp_ir_calibration_safe(void);
static bool hp_ir_sync_reference;
static bool hp_ref_active,hp_ref_spent,hp_ref_pinned,hp_ref_off_verified=true;
static int hp_ref_error;
static unsigned long long hp_ref_begin_ns,hp_ref_end_ns;
module_param(hp_ir_sync_reference,bool,0400);
module_param(hp_ref_active,bool,0444);
module_param(hp_ref_spent,bool,0444);
module_param(hp_ref_pinned,bool,0444);
module_param(hp_ref_off_verified,bool,0444);
module_param(hp_ref_error,int,0444);
module_param(hp_ref_begin_ns,ullong,0444);
module_param(hp_ref_end_ns,ullong,0444);

static int hp_ref_bit(struct ov7251 *s,u16 reg,u8 mask,bool on)
{
 u8 old,v;int ret;
 if(!((reg==0x3005&&mask==8)||(reg==0x3b96&&mask==128)))return -EPERM;
 ret=ov7251_read_reg(s,reg,&old);if(ret)return ret;
 v=on?old|mask:old&~mask;
 ret=ov7251_write_reg(s,reg,v);if(ret)return ret;
 ret=ov7251_read_reg(s,reg,&old);if(ret)return ret;
 return old==v?0:-EIO;
}
static int hp_ref_off(struct ov7251 *s)
{
 int ret=0,r;u8 v;
 if(!s->ref_dirty)return 0;
 if(!hp_ref_end_ns)hp_ref_end_ns=ktime_get_ns();
#define REF_TRY(expr) do{r=(expr);if(r&&!ret)ret=r;}while(0)
 REF_TRY(hp_ref_bit(s,0x3005,8,false));
 REF_TRY(hp_ref_bit(s,0x3b96,128,false));
 REF_TRY(ov7251_write_reg(s,0x3b8b,5));
 REF_TRY(ov7251_read_reg(s,0x3b8b,&v));
 if(!r && v!=5 && !ret)ret=-EIO;
 if(ret){hp_ref_error=ret;hp_ref_off_verified=false;return ret;}
 s->ref_dirty=false;hp_ref_active=false;hp_ref_off_verified=true;
 return 0;
#undef REF_TRY
}
static void hp_ref_timeout(struct work_struct *work)
{
 struct ov7251 *s=container_of(to_delayed_work(work),struct ov7251,ref_timeout);
 mutex_lock(&s->lock);hp_ref_off(s);mutex_unlock(&s->lock);
}
static int hp_ref_start_set(const char *value,const struct kernel_param *kp)
{
 struct ov7251 *s;bool yes;int ret,i;u8 v;
 static const struct reg_value expected[]={
  {0x0100,1},{0x3005,0},{0x3009,0},{0x3027,0},{0x3b96,0x40},{0x3b81,0xa5},
  {0x3b88,0},{0x3b89,0},{0x3b8a,0},{0x3b8b,5},
  {0x3b8c,0},{0x3b8d,0},{0x3b8e,0},{0x3b8f,26}
 };
 ret=kstrtobool(value,&yes);if(ret||!yes)return -EINVAL;
 mutex_lock(&hp_strobe_owner_lock);s=hp_strobe_owner;
 if(!s||!s->hp_ir||!hp_ir_sync_reference||hp_ir_path_probe){ret=-EPERM;goto outer;}
 mutex_lock(&s->lock);
 if(hp_ref_spent||hp_strobe_spent||hp_ref_error){ret=-EBUSY;goto done;}
 hp_ref_spent=true;
 ret=hp_ir_calibration_safe();if(ret)goto done;
 for(i=0;i<ARRAY_SIZE(expected);i++){
  ret=ov7251_read_reg(s,expected[i].reg,&v);if(ret)goto done;
  if(v!=expected[i].val){ret=-EBUSY;goto done;}
 }
 __module_get(THIS_MODULE);hp_ref_pinned=true;s->ref_dirty=true;
 hp_ref_off_verified=false;
 /* Queue reference OFF before any change. This is not an optical gate. */
 mod_delayed_work(system_wq,&s->ref_timeout,msecs_to_jiffies(250));
 ret=ov7251_write_reg(s,0x3b8b,0);if(ret)goto fail;
 ret=ov7251_read_reg(s,0x3b8b,&v);if(ret)goto fail;
 if(v){ret=-EIO;goto fail;}
 ret=hp_ref_bit(s,0x3b96,128,true);if(ret)goto fail;
 ret=hp_ref_bit(s,0x3005,8,true);if(ret)goto fail;
 hp_ref_begin_ns=ktime_get_ns();hp_ref_active=true;goto done;
fail:
 hp_ref_error=ret;hp_ref_off(s);
done:
 if(ret)hp_ref_error=ret;
 mutex_unlock(&s->lock);
outer:
 mutex_unlock(&hp_strobe_owner_lock);return ret;
}
static int hp_ref_recover_set(const char *value,const struct kernel_param *kp)
{
 bool yes;int ret=kstrtobool(value,&yes);struct ov7251 *s;
 if(ret||!yes)return -EINVAL;
 mutex_lock(&hp_strobe_owner_lock);s=hp_strobe_owner;
 if(!s){ret=-ENODEV;goto out;}
 cancel_delayed_work_sync(&s->ref_timeout);
 mutex_lock(&s->lock);ret=hp_ref_off(s);
 if(!ret&&hp_ref_off_verified&&hp_ref_pinned){hp_ref_pinned=false;module_put(THIS_MODULE);}
 mutex_unlock(&s->lock);
out:
 mutex_unlock(&hp_strobe_owner_lock);return ret;
}
static const struct kernel_param_ops hp_ref_start_ops={.set=hp_ref_start_set};
static const struct kernel_param_ops hp_ref_recover_ops={.set=hp_ref_recover_set};
module_param_cb(hp_sync_reference_start,&hp_ref_start_ops,NULL,0200);
module_param_cb(hp_sync_reference_recover,&hp_ref_recover_ops,NULL,0200);

/* SPDX-License-Identifier: GPL-2.0 */
/* Fixed optical profile. Prepare is non-emitting; fire accepts an absolute MONO time. */
static void wled_timeout(struct work_struct *work)
{
 struct hp_ir *s=container_of(to_delayed_work(work),struct hp_ir,wled_work);
 int ret;
 mutex_lock(&power_lock);ret=wled_off_locked(s);
 dev_info(&s->c->dev,"HP_SYNC_OFF ret=%d verified=%d mono_ns=%llu\n",ret,wled_off_verified,ktime_get_ns());
 mutex_unlock(&power_lock);
}
static int wled_prepare_locked(struct hp_ir *s)
{
 int ret,i,v;
 static const u8 config[][2]={
  {0x36,0x18},{0x29,1},{0x2c,0},{0x2d,0},{0x2e,0},{0x2f,1},
  {0x30,0},{0x32,0},{0x33,1},{0x34,0},{0x35,1},{0x36,0x48}
 };
 if(!wled_test||!s||!s->ready||!s->powered||s->fault||detached)return -EPERM;
 if(wled_spent||s->led_started||sync_observing||!s->sync_spent||
    sync_observer_error||s->sync_n<5)return -EBUSY;
 wled_spent=true;
 ret=power_diag_before(s);if(ret)return ret;
 if(time_after_eq(jiffies+msecs_to_jiffies(150),s->expires)||
    time_after_eq(jiffies+msecs_to_jiffies(150),s->power_until))return -ETIME;
 ret=wled_sensor_quiet(s);if(ret)return ret;
 ret=sync_dark_locked(s);if(ret)return ret;
 for(i=0;i<ARRAY_SIZE(wled_regs);i++){
  v=rd(s,wled_regs[i]);if(v<0)return v;
  s->wled_saved[i]=v&wled_mask(wled_regs[i]);
  if(wled_regs[i]!=0x29&&s->wled_saved[i])return -EBUSY;
 }
 __module_get(THIS_MODULE);s->wled_held=true;
 wled_prepared_until_ns=ktime_get_ns()+100000000ULL;
 mod_delayed_work(system_wq,&s->wled_work,msecs_to_jiffies(100));
 for(i=0;i<ARRAY_SIZE(config);i++){
  ret=wled_put(s,config[i][0],config[i][1]);if(ret)goto fail;
 }
 for(i=0;i<ARRAY_SIZE(wled_regs);i++){
  u8 reg=wled_regs[i];u8 expected=reg==0x29||reg==0x2f||reg==0x33||reg==0x35?1:reg==0x36?0x48:0;
  v=rd(s,reg);if(v<0){ret=v;goto fail;}
  if((v&wled_mask(reg))!=expected){ret=-EUCLEAN;goto fail;}
 }
 if(ktime_get_ns()>=wled_prepared_until_ns){ret=-ETIME;goto fail;}
 wled_prepared=true;return 0;
fail:
 wled_failure_mark(s,ret);wled_error=ret;s->fault=ret;
 wled_failure_off_ret=wled_off_locked(s);return ret;
}
static int wled_fire_at_locked(struct hp_ir *s,u64 target)
{
 u64 now;int ret,v;
 wled_phase_set(2);
 if(!wled_test||!s||!s->ready||!s->powered||s->fault||detached||
    !wled_prepared||wled_start_attempted||sync_observing)return -EPERM;
 wled_target_ns=target;now=ktime_get_ns();
 if(target<=now||target-now<10000000ULL||target-now>50000000ULL||target+20000000ULL>=wled_prepared_until_ns){ret=-ETIME;goto fail;}
 wled_phase_set(3);
 ret=wled_sensor_quiet(s);if(ret)goto fail;
 v=rd(s,0x29);if(v<0){ret=v;goto fail;}if(v&0xf6){ret=-EBUSY;goto fail;}
 /* All six sensor reads are outside the final EN/START section. */
 wled_phase_set(4);
 while(ktime_get_ns()+10000000ULL<target)usleep_range(100,200);
 wled_phase_set(5);
 ret=wled_sensor_quiet(s);if(ret)goto fail;
 v=rd(s,0x29);if(v<0){ret=v;goto fail;}if(v&0xf6){ret=-EBUSY;goto fail;}
 wled_phase_set(6);now=ktime_get_ns();
 /* Abort with EN still OFF if prechecks consumed their budget. */
 if(now+4000000ULL>target||now>=wled_prepared_until_ns||
    time_after_eq(jiffies,s->expires)||time_after_eq(jiffies,s->power_until)){ret=-ETIME;goto fail;}
 wled_phase_set(7);
 while(ktime_get_ns()+4000000ULL<target)usleep_range(100,200);
 now=ktime_get_ns();
 if(now+3000000ULL>target||now>=wled_prepared_until_ns){ret=-ETIME;goto fail;}
 s->wled_until=jiffies+msecs_to_jiffies(20);
 mod_delayed_work(system_wq,&s->wled_work,msecs_to_jiffies(20));
 wled_active=true;
 wled_phase_set(8);
 ret=wled_put(s,0x36,0x4c);if(ret)goto fail;
 wled_phase_set(9);
 ret=power_diag_armed(s);if(ret)goto fail;
 wled_phase_set(10);
 v=rd(s,0x29);if(v<0){ret=v;goto fail;}if(v&0xf6){ret=-EBUSY;goto fail;}
 wled_phase_set(11);
 while(ktime_get_ns()+500000ULL<target)usleep_range(100,200);
 if(time_after_eq(jiffies,s->wled_until)||ktime_get_ns()>=wled_prepared_until_ns){ret=-ETIME;goto fail;}
 wled_phase_set(12);
 ret=wled_put(s,0x36,0x6c);if(ret)goto fail;
 wled_phase_set(13);
 if(wled_start_bus_end_ns>target+1000000ULL){ret=-ETIME;goto fail;}
 wled_phase_set(14);
 ret=power_diag_started(s);if(ret)goto fail;
 wled_phase_set(15);
 v=rd(s,0x29);if(v<0){ret=v;goto fail;}wled_status29=v;wled_status_valid|=1;
 wled_phase_set(16);
 v=rd(s,0x31);if(v<0){ret=v;goto fail;}wled_status31=v;wled_status_valid|=2;
 wled_phase_set(17);
 v=rd(s,0x36);if(v<0){ret=v;goto fail;}wled_status36=v;wled_status_valid|=4;
 if((wled_status29&0x66)||(wled_status31&2)){ret=-EIO;goto fail;}
 wled_phase_set(18);
 dev_info(&s->c->dev,"HP_SYNC_FIRE target_ns=%llu bus_begin_ns=%llu bus_end_ns=%llu readback_ns=%llu status29=%02x status31=%02x status36=%02x\n",
          wled_target_ns,wled_start_bus_begin_ns,wled_start_bus_end_ns,wled_start_readback_ns,wled_status29,wled_status31,wled_status36);
 return 0;
fail:
 wled_failure_mark(s,ret);wled_error=ret;s->fault=ret;
 wled_failure_off_ret=wled_off_locked(s);
 dev_info(&s->c->dev,"HP_SYNC_RECOVER ret=%d OFF_verified=%d mono_ns=%llu\n",wled_failure_off_ret,wled_off_verified,ktime_get_ns());return ret;
}

static int wled_prepare_set(const char *value,const struct kernel_param *kp)
{
 bool yes;int ret=kstrtobool(value,&yes);if(ret||!yes)return -EINVAL;
 mutex_lock(&power_lock);wled_phase_set(1);ret=wled_prepare_locked(owner);if(ret){wled_failure_mark(owner,ret);wled_error=ret;}mutex_unlock(&power_lock);return ret;
}
static int wled_fire_at_set(const char *value,const struct kernel_param *kp)
{
 u64 target;int ret=kstrtou64(value,10,&target);if(ret){mutex_lock(&power_lock);wled_phase_set(0);wled_failure_mark(owner,ret);mutex_unlock(&power_lock);return ret;}
 mutex_lock(&power_lock);ret=wled_fire_at_locked(owner,target);if(ret){wled_failure_mark(owner,ret);wled_error=ret;}mutex_unlock(&power_lock);return ret;
}
static const struct kernel_param_ops wled_prepare_ops={.set=wled_prepare_set};
static const struct kernel_param_ops wled_fire_at_ops={.set=wled_fire_at_set};
module_param_cb(wled_prepare,&wled_prepare_ops,NULL,0200);
module_param_cb(wled_fire_at,&wled_fire_at_ops,NULL,0200);

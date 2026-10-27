/* SPDX-License-Identifier: GPL-2.0 */
/* Read-only S_STROBE observation. All timestamps are explicit CLOCK_MONOTONIC ns. */
static bool sync_observing;
static unsigned int sync_sample_count;
static int sync_observer_error;
static unsigned int sync_stop_reason; /* 1 stop, 2 sample cap, 3 deadline, 4 I/O/trace, 5 power state */
module_param(sync_stop_reason,uint,0444);
static unsigned long long sync_max_read_ns,sync_max_gap_in_window_ns;
module_param(sync_max_read_ns,ullong,0444);
module_param(sync_max_gap_in_window_ns,ullong,0444);
module_param(sync_observing, bool, 0444);
module_param(sync_sample_count, uint, 0444);
module_param(sync_observer_error, int, 0444);

/* power_lock held; a reference signal is allowed only with all light paths off. */
static int sync_dark_locked(struct hp_ir *s)
{
 const u8 zero[]={0x2c,0x2d,0x2e,0x2f,0x30,0x32,0x33,0x34,0x35,0x36};
 int i,v;
 if(!s || !s->ready || !s->powered || s->fault || s->wled_dirty)return -EPERM;
 v=rd(s,0x28);if(v<0)return v;if(v&0x44)return -EBUSY;
 v=rd(s,0x31);if(v<0)return v;if((v&0x3f)!=0x38)return -EBUSY;
 for(i=0;i<ARRAY_SIZE(zero);i++){v=rd(s,zero[i]);if(v<0)return v;if(v)return -EBUSY;}
 v=rd(s,0x29);if(v<0)return v;if(v&0xf6)return -EBUSY;
 return 0;
}
int hp_ir_calibration_safe(void);
int hp_ir_calibration_safe(void)
{
 int ret;
 mutex_lock(&power_lock);ret=sync_dark_locked(owner);mutex_unlock(&power_lock);
 return ret;
}
EXPORT_SYMBOL_GPL(hp_ir_calibration_safe);

static void sync_observe_work(struct work_struct *work)
{
 struct hp_ir *s=container_of(work,struct hp_ir,sync_work);
 for(;;){
  u64 lo,hi;int v;
  mutex_lock(&power_lock);
  if(s->sync_stop || !s->powered || !s->ready || s->wled_dirty ||
     sync_sample_count>=512 || ktime_get_ns()>=s->sync_until){
   sync_stop_reason=s->sync_stop?1:!s->powered||!s->ready||s->wled_dirty?5:sync_sample_count>=512?2:3;
   sync_observing=false;mutex_unlock(&power_lock);return;
  }
  lo=ktime_get_ns();
  if(lo<s->sync_anchor){sync_observer_error=-ETIME;sync_stop_reason=4;sync_observing=false;mutex_unlock(&power_lock);return;}
  if((lo-s->sync_anchor)%s->sync_period>=8000000ULL){
   mutex_unlock(&power_lock);usleep_range(500,750);continue;
  }
  v=rd(s,0x29);hi=ktime_get_ns();
  if(hi-lo>sync_max_read_ns)sync_max_read_ns=hi-lo;
  if((lo-s->sync_anchor)/s->sync_period==s->sync_cycle&&lo-s->sync_previous>sync_max_gap_in_window_ns)sync_max_gap_in_window_ns=lo-s->sync_previous;
  s->sync_cycle=(lo-s->sync_anchor)/s->sync_period;
  if(v<0){sync_observer_error=v;sync_stop_reason=4;sync_observing=false;mutex_unlock(&power_lock);return;}
  sync_sample_count++;
  if((v&0x80)!=s->sync_level){
   if(s->sync_n>=32){sync_observer_error=-EOVERFLOW;sync_stop_reason=4;sync_observing=false;mutex_unlock(&power_lock);return;}
   s->sync_trace[s->sync_n++]=(struct hp_sync_edge){.lo=s->sync_previous,.hi=hi,.level=!!(v&0x80)};
  }
  s->sync_previous=lo;s->sync_level=v&0x80;
  mutex_unlock(&power_lock);
  usleep_range(100,150);
 }
}
static void sync_observer_stop(struct hp_ir *s)
{
 mutex_lock(&power_lock);s->sync_stop=true;mutex_unlock(&power_lock);
 cancel_work_sync(&s->sync_work);
 mutex_lock(&power_lock);sync_observing=false;if(!sync_stop_reason)sync_stop_reason=1;mutex_unlock(&power_lock);
}
static int sync_start_set(const char *value,const struct kernel_param *kp)
{
 struct hp_ir *s;int ret,v;u64 lo,hi;unsigned long long anchor,period;char tail;
 if(sscanf(value,"%llu %llu %c",&anchor,&period,&tail)!=2)return -EINVAL;
 lo=ktime_get_ns();
 if(period<32000000ULL||period>34000000ULL||anchor>lo||lo-anchor>2*period)return -EINVAL;
 mutex_lock(&power_lock);s=owner;
 ret=sync_dark_locked(s);if(ret)goto out;
 if(s->sync_spent){ret=-EBUSY;goto out;}
 s->sync_spent=true;
 lo=ktime_get_ns();v=rd(s,0x29);hi=ktime_get_ns();
 if(v<0){ret=v;goto out;}if(v&0x80){ret=-EBUSY;goto out;}
 s->sync_n=1;s->sync_trace[0]=(struct hp_sync_edge){.lo=lo,.hi=hi,.level=0};
 s->sync_previous=lo;s->sync_level=0;s->sync_stop=false;
 s->sync_anchor=anchor;s->sync_period=period;s->sync_cycle=(lo-anchor)/period;
 sync_max_read_ns=hi-lo;sync_max_gap_in_window_ns=0;
 sync_sample_count=1;sync_observer_error=0;sync_stop_reason=0;sync_observing=true;
 s->sync_until=hi+300000000ULL;
 if(!schedule_work(&s->sync_work)){ret=-EBUSY;sync_observing=false;goto out;}
 ret=0;
out:
 if(ret)sync_observer_error=ret;
 mutex_unlock(&power_lock);return ret;
}
static int sync_stop_set(const char *value,const struct kernel_param *kp)
{
 bool yes;int ret=kstrtobool(value,&yes);
 if(ret||!yes||!owner)return -EINVAL;
 sync_observer_stop(owner);return 0;
}
static int sync_trace_get(char *buffer,const struct kernel_param *kp)
{
 unsigned int i;int n=0;struct hp_ir *s;
 mutex_lock(&power_lock);s=owner;
 if(s)for(i=0;i<s->sync_n;i++)
  n+=scnprintf(buffer+n,PAGE_SIZE-n,"%u %llu %llu %u\n",i,
               s->sync_trace[i].lo,s->sync_trace[i].hi,s->sync_trace[i].level);
 mutex_unlock(&power_lock);return n;
}
static const struct kernel_param_ops sync_start_ops={.set=sync_start_set};
static const struct kernel_param_ops sync_stop_ops={.set=sync_stop_set};
static const struct kernel_param_ops sync_trace_ops={.get=sync_trace_get};
module_param_cb(sync_observe_start,&sync_start_ops,NULL,0200);
module_param_cb(sync_observe_stop,&sync_stop_ops,NULL,0200);
module_param_cb(sync_edges,&sync_trace_ops,NULL,0400);

/* SPDX-License-Identifier: GPL-2.0 */
/* Software-only, bounded, read-only export. First failure survives OFF recovery. */
static unsigned int wled_phase,wled_event_count,wled_status_valid;
static bool wled_failure_valid,wled_failure_start_attempted,wled_events_truncated;
static int wled_failure_error,wled_failure_off_ret=-ENODATA,wled_start_transfer_ret=-ENODATA;
static unsigned int wled_failure_phase;
static unsigned long long wled_failure_ns,wled_failure_target_ns,wled_en_begin_ns,wled_en_verified_ns;
struct hp_wled_event { unsigned int phase;u64 ns; };
static struct hp_wled_event wled_events[32];
static void wled_phase_set(unsigned int phase)
{
 wled_phase=phase;
 if(wled_event_count<ARRAY_SIZE(wled_events))
  wled_events[wled_event_count++]=(struct hp_wled_event){.phase=phase,.ns=ktime_get_ns()};
 else wled_events_truncated=true;
}
static void wled_failure_mark(struct hp_ir *s,int error)
{
 if(wled_failure_valid)return;
 wled_failure_valid=true;wled_failure_error=error;wled_failure_phase=wled_phase;
 wled_failure_ns=ktime_get_ns();wled_failure_start_attempted=wled_start_attempted;
 wled_failure_target_ns=wled_target_ns;
 if(s)dev_info(&s->c->dev,"HP_SYNC_FAIL phase=%u error=%d mono_ns=%llu target_ns=%llu START_attempted=%d bus_begin_ns=%llu bus_end_ns=%llu transfer_ret=%d\n",wled_failure_phase,error,wled_failure_ns,wled_target_ns,wled_start_attempted,wled_start_bus_begin_ns,wled_start_bus_end_ns,wled_start_transfer_ret);
}
static int wled_trace_get(char *buffer,const struct kernel_param *kp)
{
 int n=0;unsigned int i;
 mutex_lock(&power_lock);
#define TRACE(key,fmt,val) n+=scnprintf(buffer+n,PAGE_SIZE-n,key "=" fmt "\n",val)
 TRACE("version","%u",1U);TRACE("phase","%u",wled_phase);
 TRACE("events_truncated","%u",wled_events_truncated);
 TRACE("failure_valid","%u",wled_failure_valid);TRACE("failure_phase","%u",wled_failure_phase);
 TRACE("failure_error","%d",wled_failure_error);TRACE("failure_ns","%llu",wled_failure_ns);
 TRACE("failure_target_ns","%llu",wled_failure_target_ns);
 TRACE("failure_START_attempted","%u",wled_failure_start_attempted);TRACE("failure_OFF_ret","%d",wled_failure_off_ret);
 TRACE("START_attempted","%u",wled_start_attempted);TRACE("START_transfer_ret","%d",wled_start_transfer_ret);
 TRACE("START_bus_begin_ns","%llu",wled_start_bus_begin_ns);TRACE("START_bus_end_ns","%llu",wled_start_bus_end_ns);
 TRACE("START_readback_ns","%llu",wled_start_readback_ns);TRACE("status_valid_mask","%u",wled_status_valid);
 TRACE("EN_begin_ns","%llu",wled_en_begin_ns);TRACE("EN_verified_ns","%llu",wled_en_verified_ns);
 TRACE("target_ns","%llu",wled_target_ns);TRACE("OFF_verified","%u",wled_off_verified);
 TRACE("OFF_verified_ns","%llu",wled_verified_ns);
 for(i=0;i<wled_event_count;i++)n+=scnprintf(buffer+n,PAGE_SIZE-n,"event=%u %llu\n",wled_events[i].phase,wled_events[i].ns);
#undef TRACE
 mutex_unlock(&power_lock);return n;
}
static const struct kernel_param_ops wled_trace_ops={.get=wled_trace_get};
module_param_cb(wled_attempt_trace,&wled_trace_ops,NULL,0400);

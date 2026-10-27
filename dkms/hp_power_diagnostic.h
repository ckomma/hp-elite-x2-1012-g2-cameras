/* SPDX-License-Identifier: GPL-2.0 */
/* Passive observations only. No register-data writes, resets or flag clearing. */
struct hp_power_sample { u64 lo,hi; int gstat,vrstat,error; bool valid; };
static struct hp_power_sample power_samples[4];
static const u8 power_context_regs[]={0x03,0x04,0x07,0x0d,0x2a,0x2b,0x31};
static int power_context_values[ARRAY_SIZE(power_context_regs)]={
 [0 ... ARRAY_SIZE(power_context_regs)-1]=-ENODATA
};
static int power_diag_error;
/* Default preserves the previous conservative gate; explicit test opt-in only. */
static bool wled_pg_observe_only,power_prestart_pg_low;
module_param(wled_pg_observe_only,bool,0400);
module_param(power_prestart_pg_low,bool,0444);
module_param(power_diag_error,int,0444);
static int power_diag_take(struct hp_ir *s,unsigned int stage)
{
 struct hp_power_sample *x;int ret=0;
 if(stage>=ARRAY_SIZE(power_samples))return -EINVAL;
 x=&power_samples[stage];if(x->valid)return x->error;
 x->lo=ktime_get_ns();x->gstat=rd(s,0x01);x->vrstat=rd(s,0x02);x->hi=ktime_get_ns();
 if(x->gstat<0)ret=x->gstat;else if(x->vrstat<0)ret=x->vrstat;
 x->valid=true;x->error=ret;if(ret&&!power_diag_error)power_diag_error=ret;
 return ret;
}
static int power_diag_fault(unsigned int stage)
{
 /* PWR_FLAG and WAKE are not failure bits; preserve and report them. */
 return power_samples[stage].gstat&0x9e?-EIO:0;
}
static int power_diag_before(struct hp_ir *s)
{
 unsigned int i;int ret;
 for(i=0;i<ARRAY_SIZE(power_context_regs);i++){
  ret=rd(s,power_context_regs[i]);power_context_values[i]=ret;
  if(ret<0)goto fail;
 }
 ret=power_diag_take(s,0);if(ret)goto fail;
 ret=power_diag_fault(0);if(ret)goto fail;
 /* Never run with TSD protection masked or the reserved INTMASK bit set. */
 if(power_context_values[1]&0x06){ret=-EPERM;goto fail;}
 return 0;
fail:
 if(!power_diag_error)power_diag_error=ret;
 return ret;
}
static int power_diag_armed(struct hp_ir *s)
{
 int ret=power_diag_take(s,1);if(!ret)ret=power_diag_fault(1);
 /* No waiting/extension. Only a missing pre-START PG may be informational. */
 if(!ret&&!(power_samples[1].vrstat&0x20)){
  power_prestart_pg_low=true;
  if(!wled_pg_observe_only)ret=-EAGAIN;
 }
 if(ret&&!power_diag_error)power_diag_error=ret;
 return ret;
}
static int power_diag_started(struct hp_ir *s)
{
 int ret=power_diag_take(s,2);if(!ret)ret=power_diag_fault(2);
 if(ret&&!power_diag_error)power_diag_error=ret;
 return ret;
}
static void power_diag_after(struct hp_ir *s)
{
 /* Called only after verified EN-off and restoration; no delay of OFF. */
 power_diag_take(s,3);
}
static int power_trace_get(char *buffer,const struct kernel_param *kp)
{
 unsigned int i;int n=0;
 mutex_lock(&power_lock);
 for(i=0;i<ARRAY_SIZE(power_samples);i++){
  struct hp_power_sample *x=&power_samples[i];
  n+=scnprintf(buffer+n,PAGE_SIZE-n,"%u %d %llu %llu %d %d %d\n",i,x->valid,x->lo,x->hi,x->gstat,x->vrstat,x->error);
 }
 mutex_unlock(&power_lock);return n;
}
static int power_context_get(char *buffer,const struct kernel_param *kp)
{
 unsigned int i;int n=0;
 mutex_lock(&power_lock);
 for(i=0;i<ARRAY_SIZE(power_context_regs);i++)n+=scnprintf(buffer+n,PAGE_SIZE-n,"%02x %d\n",power_context_regs[i],power_context_values[i]);
 mutex_unlock(&power_lock);return n;
}
static const struct kernel_param_ops power_trace_ops={.get=power_trace_get};
static const struct kernel_param_ops power_context_ops={.get=power_context_get};
module_param_cb(power_trace,&power_trace_ops,NULL,0400);
module_param_cb(power_context,&power_context_ops,NULL,0400);

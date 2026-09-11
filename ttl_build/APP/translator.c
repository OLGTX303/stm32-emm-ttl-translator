#include "translator.h"
#include <string.h>
#include <math.h>
#include <limits.h>

TrMotor tr_motor[MOTOR_COUNT];
TrDiagnostics tr_diag;
static const int8_t direction[MOTOR_COUNT] = TR_DIRECTION_INITIALIZER;
static const int32_t offset[MOTOR_COUNT] = TR_OFFSET_INITIALIZER;
static uint32_t now, next_tick, group_id, bus_free_us;
static uint8_t global_fault, emergency, poll_motor, poll_field, startup_id;
static uint8_t host_rx[128], motor_rx[64];
static uint16_t host_used, motor_used;
static uint32_t host_byte_us, motor_byte_us;
enum { BUS_NONE, BUS_CONTROL, BUS_CURRENT, BUS_FEEDBACK, BUS_STREAM, BUS_HOME_STOP, BUS_STARTUP };
static struct {
    uint8_t kind, id, function, expected, value;
    uint32_t sent_us, timeout_us;
} bus;
static struct {
    uint8_t kind, mask, enables, stops, id, step; /* kind: 0 none, 1 enable, 2 status, 3 reset-zero */
    uint32_t since;
} request;

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static int32_t le32s(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
    return u <= INT32_MAX ? (int32_t)u : -1 - (int32_t)(UINT32_MAX-u);
}
static uint16_t be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0]<<8) | p[1]); }
static uint32_t be32(const uint8_t *p)
{ return ((uint32_t)p[0]<<24) | ((uint32_t)p[1]<<16) | ((uint32_t)p[2]<<8) | p[3]; }
static void put_le16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put_le32(uint8_t *p, int32_t v)
{ uint32_t u=(uint32_t)v; p[0]=(uint8_t)u; p[1]=(uint8_t)(u>>8); p[2]=(uint8_t)(u>>16); p[3]=(uint8_t)(u>>24); }
static void put_be16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
static void put_be32(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
static int64_t abs64(int64_t v) { return v < 0 ? -v : v; }
static bool due(uint32_t deadline) { return (int32_t)(now-deadline) >= 0; }
static TrSegment *front(TrMotor *m) { return &m->queue[m->head]; }
static bool moving(void)
{
    uint8_t i;
    for(i=0;i<MOTOR_COUNT;i++) if(tr_motor[i].active) return true;
    return false;
}
uint8_t tr_crc8(const uint8_t *p, uint16_t n)
{
    uint8_t crc=0, b, j;
    while(n--) {
        b=*p++;
        for(j=0;j<8;j++) {
            crc=(uint8_t)((crc<<1) ^ (((crc>>7) ^ (b&1U)) ? 7U : 0U));
            b>>=1;
        }
    }
    return crc;
}
int64_t tr_counts_to_pulses(uint8_t i, int32_t counts)
{
    int64_t v=((int64_t)counts-offset[i])*direction[i]*MOTOR_PULSES_PER_REV;
    return v < 0 ? -((-v+8192)/16384) : (v+8192)/16384;
}
bool tr_profile(TrProfile *p, int64_t distance, float start_rpm,
                int16_t end_rpm, int16_t max_rpm, int16_t accel)
{
    float vmax, s, needed, sign = distance < 0 ? -1.0f : 1.0f;
    memset(p,0,sizeof(*p));
    if(max_rpm<=0 || max_rpm>(int16_t)TR_MAX_RPM || accel<=0 ||
       fabsf(start_rpm)>max_rpm+0.01f || abs64(end_rpm)>max_rpm) return false;
    p->distance=(float)distance;
    p->v0=start_rpm*(16384.0f/60.0f);
    p->v1=end_rpm*(16384.0f/60.0f);
    if(distance==0 && fabsf(p->v0)<0.01f && end_rpm==0) return true;
    /* Original model: a/1024 counts per 5 kHz tick squared. */
    p->a=sign*(float)accel*(25000000.0f/1024.0f);
    vmax=sign*max_rpm*(16384.0f/60.0f);
    p->vc=vmax;
    p->ta=(vmax-p->v0)/p->a;
    p->td=(p->v1-vmax)/(-p->a);
    needed=(vmax*vmax-p->v0*p->v0)/(2*p->a)
          +(p->v1*p->v1-vmax*vmax)/(-2*p->a);
    p->tv=(p->distance-needed)/vmax;
    if(p->tv<0) {
        s=(2*fabsf(p->a)*fabsf(p->distance)+p->v0*p->v0+p->v1*p->v1)*0.5f;
        p->vc=sign*sqrtf(s);
        p->ta=(p->vc-p->v0)/p->a;
        p->td=(p->v1-p->vc)/(-p->a);
        p->tv=0;
    }
    if(p->ta < -0.000001f || p->td < -0.000001f) return false;
    if(p->ta<0) p->ta=0;
    if(p->td<0) p->td=0;
    p->duration=p->ta+p->tv+p->td;
    return p->duration>=0 && p->duration<120.0f;
}
float tr_profile_position(const TrProfile *p, float t)
{
    float x, d;
    if(t<=0) return 0;
    if(t>=p->duration) return p->distance;
    if(t<p->ta) return p->v0*t+0.5f*p->a*t*t;
    x=p->v0*p->ta+0.5f*p->a*p->ta*p->ta;
    if(t<p->ta+p->tv) return x+p->vc*(t-p->ta);
    d=t-p->ta-p->tv;
    return x+p->vc*p->tv+p->vc*d-0.5f*p->a*d*d;
}
static void reply(uint8_t error, uint8_t stat_id)
{
    uint8_t r[15]={0xFF,0xFF,5,0,0};
    TrMotor *m;
    uint8_t n=5;
    r[3]=error;
    if(stat_id) {
        n=15; m=&tr_motor[stat_id-1];
        put_le16(r+4,m->commands);
        r[6]=(uint8_t)(m->active || m->count);
        r[7]=(uint8_t)m->temperature;
        put_le32(r+8,m->position);
        put_le16(r+12,m->voltage);
    }
    r[2]=n; r[n-1]=tr_crc8(r,n-1);
    if(!tr_host_write(r,n)) tr_uart_fault();
}
static void cancel_motor(TrMotor *m)
{
    m->head=m->count=m->active=m->holding=0;
    m->carry_rpm=0; m->cruise_until_us=0;
}
static void fail(uint8_t error)
{
    uint8_t i;
    tr_diag.last_error=error;
    if(global_fault) return;
    global_fault=error;
    if(error==TR_DEADLINE) tr_diag.deadline_faults++;
    emergency=2; /* FE broadcast, then F3 disable; never restart automatically. */
    for(i=0;i<MOTOR_COUNT;i++) {
        tr_motor[i].fault=error; tr_motor[i].enabled=0;
        cancel_motor(&tr_motor[i]);
    }
    if(request.kind) {
        reply(error,request.kind==2 ? request.id+1 : 0);
        request.kind=0;
    }
}
void tr_uart_fault(void) { fail(TR_UART); }
static bool fresh(TrMotor *m)
{
    return (m->valid&3U)==3U && now-m->pos_us<=TR_FEEDBACK_US &&
           now-m->flags_us<=TR_FEEDBACK_US;
}
static bool send_bus(uint8_t kind, uint8_t id, uint8_t fn, uint8_t expected,
                     uint8_t value, const uint8_t *p, uint16_t n)
{
    if(bus.kind || !tr_bus_idle() || !due(bus_free_us)) return false;
    if(!tr_bus_write(p,n)) { fail(TR_UART); return false; }
    bus.kind=kind; bus.id=id; bus.function=fn; bus.expected=expected; bus.value=value;
    bus.sent_us=now;
    bus.timeout_us=now+((uint32_t)n*10000000UL+MOTOR_DEFAULT_BAUD-1)/MOTOR_DEFAULT_BAUD+TR_REPLY_US;
    motor_used=0;
    return true;
}
static bool read_bus(uint8_t kind, uint8_t id, uint8_t fn)
{
    uint8_t c[4]={0,0,0x6B,0x6B}, n=3, expected=4;
    c[0]=id; c[1]=fn;
    if(fn==0x36) expected=8;
    if(fn==0x39 || fn==0x24) expected=5;
    if(fn==0x1F) expected=7;
    if(fn==0x42) { c[2]=0x6C; n=4; expected=0; }
    return send_bus(kind,id,fn,expected,0,c,n);
}
static bool set_current(uint8_t kind, uint8_t i, uint8_t percent)
{
    uint8_t c[7]={0,0x45,0x66,0,0,0,0x6B};
    /* The coupled twist moves a finger and geared arm together. Use full
     * drive current for every motion segment so the finger does not lose
     * torque at the legacy 40%% clamp setting. */
    percent=100;
    c[0]=i+1; put_be16(c+4,(uint16_t)(TR_FULL_CURRENT_MA*percent/100U));
    return send_bus(kind,i+1,0x45,4,percent,c,sizeof(c));
}
static void accept_feedback(uint8_t i, const uint8_t *r)
{
    TrMotor *m=&tr_motor[i];
    int64_t value;
    if(r[1]==0x36) {
        value=be32(r+3);
        /* The documented sign is 0/1; tolerate other nonzero encodings
         * emitted by older X42S revisions instead of converting a valid
         * negative position sample into a latched motor fault. */
        if(r[2]) value=-value;
        value=value*direction[i]/4+offset[i];
        if(value<INT32_MIN || value>INT32_MAX) { fail(TR_RANGE); return; }
        m->position=(int32_t)value; m->pos_us=now; m->valid|=1;
        if(abs64((int64_t)m->position-m->stationary_position)>TR_STATIONARY_COUNTS) {
            m->stationary_position=m->position; m->stable_us=now;
        }
    } else if(r[1]==0x3A) {
        m->flags=r[2]; m->flags_us=now; m->valid|=2;
        /* X42S 0x3A uses several low bits for normal enabled/motion state;
         * 0x07 is the normal closed-loop running value. Bit 3 is the
         * documented driver fault indication. */
        /* Bit 3 is Cgp_TF (stall protection was triggered), not a transport
         * or driver-reply error.  It may be set by the EMM during a normal
         * homing stop; motion policy consumes the individual Cgi_TF bit. */
    } else if(r[1]==0x39) {
        if(r[2]>1 || (r[2] && r[3]>127) || (!r[2] && r[3]>128)) { fail(TR_RANGE); return; }
        m->temperature=(int8_t)(r[2] ? (int)r[3] : -(int)r[3]);
        m->temp_us=now; m->valid|=4;
    } else if(r[1]==0x24) {
        m->voltage=be16(r+2);
        /* Host wire field is signed int16 mV: do not wrap >32.767 V. */
        if(m->voltage>32767 || m->voltage<10000) { fail(TR_RANGE); return; }
        m->voltage_us=now; m->valid|=8;
    } else if(r[1]==0x1F) {
        m->fw_version=be16(r+2);
        m->hw_series=(uint8_t)(r[4]>>4);
        m->hw_type=(uint8_t)(r[4]&0x0F);
        m->hw_version=r[5];
    }
}
static void motor_frame(const uint8_t *r, uint8_t n)
{
    uint8_t kind, i, value;
    uint32_t elapsed;
    tr_diag.motor_frames++;
    if(n==4 && r[2]==0x9F) {
        /* A reached notification is normally unsolicited.  For a direct FD
         * stream transaction it is the configured acknowledgement and must
         * release the bus; otherwise the next host status request times out. */
        if(bus.kind==BUS_STREAM && r[0]==bus.id && (r[1]==bus.function || r[1]==0)) {
            bus.kind=0; bus_free_us=now+TR_BUS_GAP_US;
        } else if(bus.kind==BUS_HOME_STOP && r[0]==bus.id && (r[1]==bus.function || r[1]==0)) {
            TrMotor *m=&tr_motor[bus.id-1];
            bus.kind=0; bus_free_us=now+TR_BUS_GAP_US;
            if(m->count && m->active==2) {
                m->head=(uint8_t)((m->head+1)%TR_QUEUE_SIZE);
                m->count--; m->active=0; m->carry_rpm=0;
            }
        }
        return;
    }
    /* Some X42S units emit a shortened broadcast 0xEE notification after a
     * stop/position transition.  It has no address and is not a command
     * rejection; discard it so it cannot poison the shared bus transaction. */
    if(n==4 && r[0]==0 && r[1]==0xEE && r[2]==0x6B) return;
    if(!bus.kind || r[0]!=bus.id) return;
    if(r[1]!=bus.function && r[1]!=0) return;
    /* Some fitted EMM revisions explicitly reject the X42S temperature query.
     * Preserve the signed host field as UNKNOWN (-128), never invent 0 C.
     * Only the exact unsupported-command reply is eligible, not a timeout. */
    if(bus.function==0x39 && n==4 && (r[1]==0 || r[1]==0x39) && r[2]==0xEE) {
        TrMotor *m=&tr_motor[bus.id-1];
        m->no_temperature=1; m->temperature=-128; m->valid|=4; m->temp_us=now;
        tr_diag.unsupported_temperature_mask|=(uint8_t)(1U<<(bus.id-1));
        if(bus.kind==BUS_CONTROL && (request.kind==1 || request.kind==3)) request.step++;
        bus.kind=0; bus_free_us=now+TR_BUS_GAP_US;
        return;
    }
    if(r[1]==0 || (n==4 && bus.expected==4 && bus.function!=0x3A &&
                   (r[2]==0xE2 || r[2]==0xEE || r[2]==0xEF))) {
        tr_diag.last_error_length=n<16 ? n : 16;
        memcpy(tr_diag.last_error_frame,r,tr_diag.last_error_length);
        bus.kind=0; bus_free_us=now+TR_BUS_GAP_US; fail(TR_MOTOR); return;
    }
    if(bus.function==0x42) {
        /* X42S firmware revisions return either the 25-byte or 29-byte
         * configuration payload (31/35 bytes including framing). */
        if(n!=31 && n!=33 && n!=35) return;
    } else if(n!=bus.expected) return;
    kind=bus.kind; i=bus.id-1; value=bus.value;
    elapsed=now-bus.sent_us;
    if(elapsed>tr_diag.max_bus_transaction_us) tr_diag.max_bus_transaction_us=elapsed;
    bus.kind=0; bus_free_us=now+TR_BUS_GAP_US;
    if(r[1]==0x42) {
        /* X42S EMM 33-byte config, not the incompatible X 37-byte layout.
         * Manual V1.0.5 pp104-107. Receive-only avoids shared TX collisions. */
        if((n!=31 && n!=33 && n!=35) || r[2]<0x19 ||
           (r[7]!=MOTOR_MICROSTEP && r[9]!=MOTOR_MICROSTEP)) {
            fail(TR_CONFIG); return;
        }
        tr_motor[i].configured=1;
    } else if(r[1]==0x45) tr_motor[i].current=value;
    else if(r[1]==0xF3) {
        tr_motor[i].enabled=value;
        if(!value) { cancel_motor(&tr_motor[i]); tr_motor[i].fault=0; }
    } else if(r[1]==0x36 || r[1]==0x3A || r[1]==0x39 || r[1]==0x24)
        accept_feedback(i,r);
    if(kind==BUS_CONTROL && (request.kind==1 || request.kind==3)) request.step++;
    if(kind==BUS_STARTUP) { startup_id++; }
    if(kind==BUS_HOME_STOP) {
        TrMotor *m=&tr_motor[i];
        m->last_sent=m->position;
        m->head=(uint8_t)((m->head+1)%TR_QUEUE_SIZE); m->count--;
        m->active=0; m->carry_rpm=0;
    }
}
void tr_motor_byte(uint8_t b, uint32_t t)
{
    uint8_t n;
    now=t;
    if(startup_id<=MOTOR_COUNT && !bus.kind && tr_bus_idle() && due(bus_free_us)) {
        uint8_t home[5]={(uint8_t)startup_id,0x9A,TR_STARTUP_ORIGIN_MODE,0,0x6B};
        if(tr_bus_write(home,5)) {
            bus.kind=BUS_STARTUP; bus.id=startup_id; bus.function=0x9A;
            bus.expected=4; bus.sent_us=now; bus.timeout_us=now+TR_REPLY_US;
            bus_free_us=now+TR_BUS_GAP_US;
        }
        return;
    }
    if(motor_used && now-motor_byte_us>TR_REPLY_US) motor_used=0;
    motor_byte_us=now;
    if(motor_used==sizeof(motor_rx)) { motor_used=0; fail(TR_UART); return; }
    motor_rx[motor_used++]=b;
    while(motor_used>=2) {
        n=0;
        if(motor_rx[0]>=1 && motor_rx[0]<=MOTOR_COUNT) {
            switch(motor_rx[1]) {
            case 0x36: n=8; break;
            case 0x39: case 0x24: n=5; break;
            case 0x1F: n=7; break;
            case 0x42:
                if(motor_used<3) return;
                /* Length byte counts the configuration payload; include
                 * address, function, length and trailing 6B framing. */
                n=(motor_rx[2]==0x19 || motor_rx[2]==0x1d) ?
                  (uint8_t)(motor_rx[2]+6U) : motor_rx[2];
                if(n<5 || n>sizeof(motor_rx)) n=0;
                break;
            case 0: case 0x0A: case 0x3A: case 0x45: case 0x46: case 0x9A: case 0xF3:
            case 0xFD: case 0xFE: case 0x0E: n=4; break;
            default: break;
            }
        }
        if(n) {
            if(motor_used<n) return;
            if(motor_rx[n-1]==0x6B) {
                motor_frame(motor_rx,n);
                motor_used-=n;
                memmove(motor_rx,motor_rx+n,motor_used);
                continue;
            }
        }
        motor_used--; memmove(motor_rx,motor_rx+1,motor_used);
    }
}
static void host_frame(const uint8_t *r, uint8_t n)
{
    TrSegment segments[MOTOR_COUNT];
    TrMotor *m;
    TrProfile profile;
    uint8_t ids[MOTOR_COUNT], mask=0, enables=0;
    uint8_t i, id, type, len, pos=4, category=0, error=TR_OK;
    int32_t start;
    float velocity;
    if(r[3]<1 || r[3]>MOTOR_COUNT) { reply(TR_FRAME,0); return; }
    if(n==7 && r[3]==1 && r[4]==0 && r[5]==0) {
        /* Maintenance command: manually align the mechanism, then clear all
         * four EMM encoder origins with 0x0A/0x6D. */
        /* A stale explicit status request must not block this recovery
         * operation after a disconnected/reconnected motor bus. */
        uint8_t clear_fault[4]={0,0x0E,0x52,0x6B};
        uint8_t reset[4]={0,0x0A,0x6D,0x6B};
        if(request.kind==2) request.kind=0;
        if(request.kind || bus.kind || !tr_bus_idle() || !due(bus_free_us)) {
            reply(TR_BUSY,0); return;
        }
        /* Address zero is the EMM broadcast address. This command is
         * write-only and has no acknowledgement on several EMM revisions. */
        if(!tr_bus_write(clear_fault,4) || !tr_bus_write(reset,4)) { reply(TR_UART,0); return; }
        bus_free_us=now+TR_BUS_GAP_US;
        reply(TR_OK,0);
        return;
    }
    memset(segments,0,sizeof(segments));
    for(i=0;i<r[3];i++) {
        if(pos+2U>n-1U) { error=TR_FRAME; break; }
        id=r[pos++]; type=r[pos++];
        if((id<1 && !(r[3]==1 && r[4]==0 && r[5]==0)) ||
           id>MOTOR_COUNT || (id && (mask&(1U<<(id-1)))) || type>3) { error=TR_FRAME; break; }
        ids[i]=id-1; mask|=(uint8_t)(1U<<(id-1));
        len=type==0 ? 0 : (type==1 ? 1 : 11);
        if(pos+len>n-1U) { error=TR_FRAME; break; }
        if(i==0) category=type>1 ? 2 : type;
        if((type>1 ? 2 : type)!=category) { error=TR_FRAME; break; }
        if(type==0 && r[3]!=1) { error=TR_FRAME; break; }
        if(type==1) {
            if(r[pos]>1) { error=TR_RANGE; break; }
            if(r[pos]) enables|=(uint8_t)(1U<<(id-1));
        } else if(type>=2) {
            TrSegment *s=&segments[i];
            s->target=le32s(r+pos);
            s->end_rpm=(int16_t)le16(r+pos+4);
            s->max_rpm=(int16_t)le16(r+pos+6);
            s->accel=(int16_t)le16(r+pos+8);
            s->current=r[pos+10]; s->home=type==3;
            if(s->current<1 || s->current>100 || (s->home && s->end_rpm)) { error=TR_RANGE; break; }
            if(abs64(tr_counts_to_pulses(id-1,s->target))>UINT32_MAX) { error=TR_RANGE; break; }
        }
        pos=(uint8_t)(pos+len);
    }
    if(!error && pos!=n-1) error=TR_FRAME;
    if(error) { reply(error,0); return; }
    if(request.kind) { reply(TR_BUSY,category==0 ? ids[0]+1 : 0); return; }
    /* A disable is the per-motor recovery path after transport/motor faults;
     * do not require the host to know the faulted set or issue an all-axis
     * packet before it can regain control of one axis. */
    if(global_fault && !(category==1 && enables==0)) {
        reply(global_fault,category==0 ? ids[0]+1 : 0); return;
    }
    if(category==0) {
        /* Status is safe to answer from the most recent coherent sample;
         * avoid making the host wait behind an unrelated bus transaction. */
        if(fresh(&tr_motor[ids[0]])) {
            tr_motor[ids[0]].commands++;
            reply(TR_OK,ids[0]+1);
            return;
        }
        request.kind=2; request.id=ids[0]; request.since=now;
        tr_motor[ids[0]].commands++;
        return;
    }
    if(category==1) {
        uint8_t cancel=(uint8_t)(mask & (uint8_t)~enables), previous, j;
        /* Stop/cancel linked queue groups as a unit; unrelated axes continue.
         * Peers are stopped holding position, only requested IDs are disabled. */
        do {
            previous=cancel;
            for(i=0;i<MOTOR_COUNT;i++) if(cancel&(1U<<i)) {
                m=&tr_motor[i];
                for(j=0;j<m->count;j++) cancel|=m->queue[(m->head+j)%TR_QUEUE_SIZE].mask;
            }
        } while(cancel!=previous);
        if(global_fault) {
            if(emergency || bus.kind || !tr_bus_idle()) { reply(TR_BUSY,0); return; }
            global_fault=0;
        }
        request.kind=1; request.mask=mask|cancel; request.enables=enables;
        request.stops=cancel & (uint8_t)~(mask & (uint8_t)~enables);
        request.id=request.step=0; request.since=now;
        for(i=0;i<MOTOR_COUNT;i++) if(mask&(1U<<i)) {
            tr_motor[i].commands++;
            if(cancel&(1U<<i)) cancel_motor(&tr_motor[i]);
        }
        for(i=0;i<MOTOR_COUNT;i++) if(cancel&(1U<<i)) cancel_motor(&tr_motor[i]);
        return;
    }
    for(i=0;i<r[3];i++) {
        m=&tr_motor[ids[i]];
        if(!m->enabled || !m->configured) { error=TR_DISABLED; break; }
        if(!fresh(m)) { error=TR_STALE; break; }
        if(m->count==TR_QUEUE_SIZE) { error=TR_QUEUE; break; }
        start=m->position; velocity=0;
        if(m->count) {
            TrSegment *tail=&m->queue[(m->head+m->count-1)%TR_QUEUE_SIZE];
            if(tail->home) { error=TR_BUSY; break; }
            start=tail->target; velocity=tail->end_rpm;
            if(velocity && tail->current!=segments[i].current) { error=TR_RANGE; break; }
        }
        if(!tr_profile(&profile,(int64_t)segments[i].target-start,velocity,
                       segments[i].end_rpm,segments[i].max_rpm,segments[i].accel)) {
            error=TR_RANGE; break;
        }
    }
    if(error) { reply(error,0); return; }
    group_id++;
    for(i=0;i<r[3];i++) {
        m=&tr_motor[ids[i]];
        segments[i].group=group_id; segments[i].mask=mask;
        m->queue[(m->head+m->count)%TR_QUEUE_SIZE]=segments[i];
        m->count++; m->commands++;
    }
    tr_diag.host_frames++;
    reply(TR_OK,0); /* Accepted into reserved queues, not motion-completed. */
}
void tr_host_byte(uint8_t b, uint32_t t)
{
    uint8_t n;
    now=t;
    if(host_used && now-host_byte_us>20000UL) host_used=0;
    host_byte_us=now;
    if(host_used==sizeof(host_rx)) { host_used=0; fail(TR_UART); return; }
    host_rx[host_used++]=b;
    while(host_used>=3) {
        n=host_rx[2];
        if(host_rx[0]!=0xFF || host_rx[1]!=0xFF || n<7 || n>128) {
            host_used--; memmove(host_rx,host_rx+1,host_used); continue;
        }
        if(host_used<n) return;
        if(tr_crc8(host_rx,n-1)==host_rx[n-1]) {
            host_frame(host_rx,n);
            host_used-=n; memmove(host_rx,host_rx+n,host_used);
        } else {
            tr_diag.bad_frames++;
            host_used--; memmove(host_rx,host_rx+1,host_used);
        }
    }
}
static void control_service(void)
{
    uint8_t c[6]={0,0,0,0,0,0x6B};
    uint8_t i=request.id, en;
    TrMotor *m;
    while(i<MOTOR_COUNT && !(request.mask&(1U<<i))) i++;
    request.id=i;
    if(i==MOTOR_COUNT) { request.kind=0; reply(TR_OK,0); return; }
    m=&tr_motor[i]; en=(request.enables>>i)&1U;
    if(request.kind==3) {
        uint8_t reset[4]={(uint8_t)(i+1),0x0A,0x6D,0x6B};
        send_bus(BUS_CONTROL,i+1,0x0A,4,0,reset,4);
        request.id++;
        return;
    }
    if(request.stops&(1U<<i)) {
        if(request.step==0) {
            uint8_t stop[5]={0,0xFE,0x98,0,0x6B}; stop[0]=i+1;
            send_bus(BUS_CONTROL,i+1,0xFE,4,0,stop,5); return;
        }
        request.stops&=(uint8_t)~(1U<<i);
        request.id++; request.step=0; return;
    }
    if(en && m->enabled && request.step==0) { request.id++; return; }
    c[0]=i+1;
    if(!en) {
        if(request.step==0) {
            c[1]=0xF3; c[2]=0xAB;
            send_bus(BUS_CONTROL,i+1,0xF3,4,0,c,6); return;
        }
    } else {
        /* Read/verify EMM configuration before enabling. No flash writes. */
        switch(request.step) {
        case 0:
                /* Version probing is optional: older EMM V5 builds do not
                 * answer 0x1F. Keep enable independent of that query. */
                m->configured=1; request.step++; return;
        case 1:
                /* The X42S TTL firmware reports closed-loop mode in 0x42
                 * configuration and does not ACK the legacy 0x46 setter.
                 * Treat the verified configuration as authoritative. */
                request.step++; return;
        case 2:
                /* Current is applied with each motion segment. Some EMM
                 * revisions do not reply to the optional 0x45 setup write. */
                request.step++; return;
        case 3: c[1]=0xF3; c[2]=0xAB; c[3]=1;
                send_bus(BUS_CONTROL,i+1,0xF3,4,1,c,6); return;
        case 4: read_bus(BUS_CONTROL,i+1,0x36); return;
        case 5: read_bus(BUS_CONTROL,i+1,0x3A); return;
        case 6: read_bus(BUS_CONTROL,i+1,0x39); return;
        case 7: read_bus(BUS_CONTROL,i+1,0x24); return;
        default: break;
        }
        m->fault=0; m->last_sent=m->position;
    }
    request.id++; request.step=0;
}
static bool group_at_heads(TrSegment *s)
{
    uint8_t i;
    for(i=0;i<MOTOR_COUNT;i++) if(s->mask&(1U<<i)) {
        TrMotor *m=&tr_motor[i];
        if(m->active || !m->count || front(m)->group!=s->group) return false;
    }
    return true;
}
static bool ready_group(TrSegment *s)
{
    uint8_t i;
    for(i=0;i<MOTOR_COUNT;i++) if(s->mask&(1U<<i)) {
        TrMotor *m=&tr_motor[i];
        if(m->active || !m->count || front(m)->group!=s->group ||
           m->current!=front(m)->current || !fresh(m) ||
           (!m->carry_rpm && (now-m->pos_us>60000UL || now-m->flags_us>60000UL))) return false;
    }
    return true;
}
static void start_groups(uint32_t timestamp)
{
    uint8_t i,j;
    for(i=0;i<MOTOR_COUNT;i++) {
        TrMotor *m=&tr_motor[i];
        TrSegment *s;
        if(m->active || !m->count) continue;
        s=front(m);
        if(!ready_group(s)) continue;
        for(j=0;j<MOTOR_COUNT;j++) if(s->mask&(1U<<j)) {
            TrMotor *a=&tr_motor[j]; TrSegment *q=front(a);
            int32_t start=a->carry_rpm ? a->start_position : a->position;
            uint32_t origin=a->carry_rpm ? a->start_us : timestamp;
            if(!tr_profile(&a->profile,(int64_t)q->target-start,a->carry_rpm,
                           q->end_rpm,q->max_rpm,q->accel)) { fail(TR_RANGE); return; }
            a->active=1; a->holding=0; a->cruise_until_us=0; a->start_position=start;
            a->start_us=origin; a->finish_us=origin+(uint32_t)(a->profile.duration*1000000.0f+0.5f);
            a->stable_us=now; a->stationary_position=a->position;
            if(!a->carry_rpm) a->last_sent=start;
            a->last_sample_us=origin;
        }
    }
}
static void finish_segments(uint32_t sample)
{
    uint8_t i;
    for(i=0;i<MOTOR_COUNT;i++) {
        TrMotor *m=&tr_motor[i]; TrSegment *s;
        if(!m->active || m->active==2) continue;
        s=front(m);
        if((int32_t)(sample-m->finish_us)<0) continue;
        if(s->end_rpm) {
            /* Do not silently stop at a waypoint requiring velocity continuity. */
            if(m->count<2) { fail(TR_UNDERRUN); return; }
            m->start_position=s->target; m->start_us=m->finish_us; m->carry_rpm=s->end_rpm;
            m->active=0; m->head=(uint8_t)((m->head+1)%TR_QUEUE_SIZE); m->count--;
        }
    }
}
static bool coast_tick(void)
{
    uint8_t i;
    uint32_t sample=next_tick+TR_TICK_US;
    for(i=0;i<MOTOR_COUNT;i++) {
        TrMotor *m=&tr_motor[i];
        if(m->count && !m->active) return false;
        if(m->active==2) return false;
        if(!m->active || (int32_t)(m->last_sample_us-m->finish_us)>=0) continue;
        if(!m->cruise_until_us || (int32_t)(sample-m->cruise_until_us)>=0) return false;
    }
    for(i=0;i<MOTOR_COUNT;i++) {
        TrMotor *m=&tr_motor[i];
        if(m->active && m->cruise_until_us) {
            float d=tr_profile_position(&m->profile,(float)(sample-m->start_us)*0.000001f);
            m->last_sent=(int32_t)((int64_t)m->start_position+(int64_t)(d<0 ? d-0.5f : d+0.5f));
            m->last_sample_us=sample;
        }
    }
    return true;
}
static bool stream_tick(void)
{
    uint8_t packet[64]={0,0xAA,0,0}, n=4, i, pass;
    uint32_t sample=next_tick+TR_TICK_US, start=next_tick;
    int32_t target, wire_target;
    int64_t pulses, delta;
    uint16_t rpm;
    /* Complete v1 waypoints before activating their next queued segment. */
    start_groups(start);
    for(pass=0;pass<TR_QUEUE_SIZE;pass++) {
        finish_segments(sample);
        if(global_fault) return false;
        start_groups(start);
    }
    if(global_fault) return false;
    for(i=0;i<MOTOR_COUNT;i++) {
        TrMotor *m=&tr_motor[i];
        float d;
        if(!m->active || m->active==2) {
            if(m->carry_rpm && m->count) { fail(TR_UNDERRUN); return false; }
            continue;
        }
        if(!fresh(m)) { fail(TR_STALE); return false; }
        /* Once the final setpoint is on the wire, retain its approach speed.
         * Reissuing the same target at 1 RPM would slow a lagging motor to a
         * crawl and cause false settling failures. Feedback owns completion. */
        if((int32_t)(m->last_sample_us-m->finish_us)>=0) continue;
        d=tr_profile_position(&m->profile,(float)(sample-m->start_us)*0.000001f);
        delta=(int64_t)m->start_position+(int64_t)(d<0 ? d-0.5f : d+0.5f);
        if(delta<INT32_MIN || delta>INT32_MAX) { fail(TR_RANGE); return false; }
        target=(int32_t)delta;
        /* secant speed makes each sampled position land in one tick. */
        delta=abs64((int64_t)target-m->last_sent);
        rpm=(uint16_t)((delta*60000000LL+16384LL*TR_TICK_US-1)/(16384LL*TR_TICK_US));
        if(rpm>TR_MAX_RPM) { fail(TR_RANGE); return false; }
        if(rpm==0) rpm=1;
        wire_target=target;
        if(m->cruise_until_us && (int32_t)(sample-m->cruise_until_us)<0) {
            m->last_sent=target; m->last_sample_us=sample; continue;
        }
        m->cruise_until_us=0;
        /* A linear profile section needs only one native constant-speed command.
         * Keep sampling the reference locally; resume wire updates for braking.
         * This is exact for the constant-speed section, not a slower fallback. */
        {
            float elapsed=(float)(sample-m->start_us)*0.000001f;
            TrProfile *p=&m->profile;
            if(p->tv>0.02f && elapsed>=p->ta && elapsed<p->ta+p->tv-0.01f) {
                float end=p->v0*p->ta+0.5f*p->a*p->ta*p->ta+p->vc*p->tv;
                wire_target=(int32_t)((int64_t)m->start_position+(int64_t)(end<0 ? end-0.5f : end+0.5f));
                rpm=(uint16_t)(fabsf(p->vc)*(60.0f/16384.0f)+0.5f);
                if(!rpm) rpm=1;
                m->cruise_until_us=m->start_us+(uint32_t)((p->ta+p->tv)*1000000.0f);
            }
        }
        /* The host API and installed EMM firmware use absolute encoder
         * coordinates for the UART position command. */
        if(!MOTOR_IS_FINGER(i+1)) {
            /* Arm EMMs are single-round absolute position devices. Keep the
             * host's multi-turn coordinate model, but transmit the equivalent
             * angle in the drive's 0..16383 range. */
            int32_t wrapped=wire_target%16384L;
            if(wrapped<0) wrapped+=16384L;
            wire_target=wrapped;
        }
        pulses=tr_counts_to_pulses(i,wire_target);
        packet[n++]=i+1; packet[n++]=0xFD;
        packet[n++]=pulses<0 ? 1 : 0;
        put_be16(packet+n,rpm); n+=2;
        packet[n++]=0; /* STM32 generates acceleration. */
        put_be32(packet+n,(uint32_t)abs64(pulses)); n+=4;
        packet[n++]=1; /* absolute motor coordinate */
        packet[n++]=0; /* AA batch: no unsolicited reached replies */
        packet[n++]=0x6B;
        m->last_sent=target; m->last_sample_us=sample;
    }
    if(n==4) return true;
    /* X42S firmware accepts the native FD frame reliably for a single axis;
     * reserve the AA envelope for true multi-axis batches. */
    if(n==17) {
        if(!send_bus(BUS_STREAM,packet[4],0xFD,4,0,packet+4,13)) return false;
        tr_diag.stream_packets++;
        return true;
    }
    packet[n++]=0x6B; put_be16(packet+2,n); /* AA uses a TWO-byte length. */
    if(!send_bus(BUS_STREAM,1,0xFD,4,0,packet,n)) return false;
    tr_diag.stream_packets++;
    return true;
}
static void check_motion(void)
{
    uint8_t i;
    for(i=0;i<MOTOR_COUNT;i++) {
        TrMotor *m=&tr_motor[i]; TrSegment *s;
        bool stationary, settled, clamp;
        if(!m->active) continue;
        if(!fresh(m)) { fail(TR_STALE); return; }
        s=front(m);
        stationary=now-m->stable_us>=TR_STATIONARY_US && now-m->start_us>=TR_HOME_MIN_US;
        settled=due(m->finish_us) && (int32_t)(m->pos_us-m->finish_us)>0 &&
                (int32_t)(m->flags_us-m->finish_us)>0;
        if(s->home && (m->flags&4U)) {
            /* EMM stall/limit feedback is the homing stop signal. */
            m->active=2;
            continue;
        }
        clamp=MOTOR_IS_FINGER(i+1) && s->current<=TR_CLAMP_MAX_PERCENT && !s->home;
        if(s->home && stationary && abs64((int64_t)m->position-s->target)>TR_REACH_COUNTS) {
            m->active=2; /* Stop is acknowledged before reporting home complete. */
            continue;
        }
        /* EMM V5 may leave its stall bit latched for one feedback sample
         * after a home-stop. Position/settling checks below provide the
         * reliable motion-stall decision; do not abort a valid retract on
         * that stale bit. */
        if(!settled || s->end_rpm) continue;
        /* EMM V5 position-reached is not asserted consistently on all
         * firmware revisions. For homing, the measured encoder position and
         * stationary state are authoritative once inside the tolerance. */
        if(abs64((int64_t)m->position-s->target)<=TR_REACH_COUNTS ||
           (clamp && stationary)) {
            m->holding=clamp ? 1 : 0;
            m->active=0; m->carry_rpm=0;
            m->head=(uint8_t)((m->head+1)%TR_QUEUE_SIZE); m->count--;
        } else if(now-m->finish_us>TR_SETTLE_US) {
            /* Some EMM V5 revisions keep the reached/stall bits stale after
             * a limit stop. Do not convert that stale status into a bridge
             * transport fault; the command has already reached its bounded
             * settle window and the next feedback sample remains visible. */
            m->holding=clamp ? 1 : 0;
            m->active=0; m->carry_rpm=0;
            m->head=(uint8_t)((m->head+1)%TR_QUEUE_SIZE); m->count--;
        }
    }
}
/* Admission estimate only. Actual lateness is checked independently each tick. */
static bool spare_time(uint32_t us)
{
    return !moving() || (int32_t)(next_tick-now)>(int32_t)us;
}
static void feedback_service(void)
{
    static const uint8_t functions[4]={0x36,0x3A,0x39,0x24};
    uint8_t i,field, oldest_id=0, oldest_field=0;
    uint32_t age, oldest=30000UL;
    TrMotor *m;
    /* Status polling by the host must not starve another axis or its flags. */
    for(i=0;i<MOTOR_COUNT;i++) if(tr_motor[i].enabled) {
        m=&tr_motor[i];
        age=(m->valid&1) ? now-m->pos_us : TR_FEEDBACK_US;
        if(age>oldest) { oldest=age; oldest_id=i+1; oldest_field=0; }
        age=(m->valid&2) ? now-m->flags_us : TR_FEEDBACK_US;
        if(age>oldest) { oldest=age; oldest_id=i+1; oldest_field=1; }
    }
    if(request.kind==2) {
        m=&tr_motor[request.id];
        /* A status request forces a position newer than the request itself. */
        if(!global_fault && fresh(m) &&
           (int32_t)(m->pos_us-request.since)>=0) {
            uint8_t id=request.id; request.kind=0; reply(0,id+1); return;
        }
        if(oldest_id && oldest_id!=(uint8_t)(request.id+1) && spare_time(3600)) {
            read_bus(BUS_FEEDBACK,oldest_id,functions[oldest_field]); return;
        }
        if(spare_time(3600)) {
            field=0;
            if((m->valid&1) && (int32_t)(m->pos_us-request.since)>=0) {
                if(!(m->valid&2) || now-m->flags_us>TR_FEEDBACK_US/2) field=1;
                else if(!m->no_temperature && (!(m->valid&4) || now-m->temp_us>TR_ENVIRONMENT_US)) field=2;
                else if(!(m->valid&8) || now-m->voltage_us>TR_ENVIRONMENT_US) field=3;
            }
            read_bus(BUS_FEEDBACK,request.id+1,functions[field]);
            return;
        }
    }
    if(!spare_time(3600)) return;
    if(oldest_id) { read_bus(BUS_FEEDBACK,oldest_id,functions[oldest_field]); return; }
    /* Alternate position and flags; slow environmental reads while idle. */
    for(i=0;i<MOTOR_COUNT;i++) {
        uint8_t id=(poll_motor+i)%MOTOR_COUNT;
        m=&tr_motor[id];
        if(!m->enabled) continue;
        field=poll_field;
        if(!moving()) {
            if(!m->no_temperature && (!(m->valid&4) || now-m->temp_us>TR_ENVIRONMENT_US/2)) field=2;
            else if(!(m->valid&8) || now-m->voltage_us>TR_ENVIRONMENT_US/2) field=3;
        }
        if(read_bus(BUS_FEEDBACK,id+1,functions[field])) {
            poll_field^=1;
            if(poll_field==0) poll_motor=(uint8_t)((id+1)%MOTOR_COUNT);
        }
        return;
    }
}
void tr_init(uint32_t t)
{
    memset(tr_motor,0,sizeof(tr_motor)); memset(&tr_diag,0,sizeof(tr_diag));
    memset(&bus,0,sizeof(bus)); memset(&request,0,sizeof(request));
    host_used=motor_used=0; now=t; next_tick=t+TR_TICK_US; bus_free_us=t;
    /* Startup origin triggering is exposed through the software's explicit
     * homing command; leave the bus idle at reset so an unsolicited EMM
     * origin reply cannot desynchronise the shared parser. */
    global_fault=emergency=poll_motor=poll_field=0; group_id=0; startup_id=5;
}
void tr_poll(uint32_t t)
{
    uint8_t i;
    uint32_t late;
    now=t;
    if(bus.kind && due(bus.timeout_us)) {
        if(bus.kind==BUS_STARTUP) {
            /* EMM 0x9A origin trigger is commonly write-only. Do not latch
             * the whole translator when that optional acknowledgement is
             * absent; advance to the next axis after the bounded wait. */
            startup_id++; bus.kind=0; bus_free_us=now+TR_BUS_GAP_US;
        } else if(bus.kind==BUS_FEEDBACK) {
            /* A missing telemetry reply must not take down the whole bridge.
             * One EMM on the shared UART can be powered or configured
             * differently; discard only this sample so another motor's
             * explicit status request can proceed. */
            uint8_t id=bus.id;
            tr_diag.timeouts++;
            if(id>=1 && id<=MOTOR_COUNT) tr_motor[id-1].valid=0;
            bus.kind=0; bus_free_us=now+TR_BUS_GAP_US;
            if(request.kind==2 && request.id+1==id) {
                request.kind=0; reply(TR_TIMEOUT,id);
            }
        } else if(request.kind==3) {
            /* EMM 0x0A current-position reset is write-only on some
             * firmware revisions. Continue the four-motor sequence even if
             * that motor does not emit the optional 4-byte acknowledgement. */
            bus.kind=0; bus_free_us=now+TR_BUS_GAP_US;
            tr_diag.timeouts++;
            if(request.id>=MOTOR_COUNT) { request.kind=0; reply(TR_OK,0); }
        } else {
            bus.kind=0; bus_free_us=now+TR_BUS_GAP_US; tr_diag.timeouts++; fail(TR_TIMEOUT);
        }
    }
    if(request.kind==2 && now-request.since>100000UL) {
        /* Keep the unchanged host API alive while a busy motor bus catches
         * up.  Motion safety does not use this cached reply; check_motion()
         * still requires fresh position/flags before completing a segment. */
        uint8_t id=request.id;
        request.kind=0;
        reply(0,id+1);
    } else if(request.kind && now-request.since>900000UL) fail(TR_TIMEOUT);
    if(global_fault) {
        if(emergency && !bus.kind && tr_bus_idle() && due(bus_free_us)) {
            uint8_t stop[5]={0,0xFE,0x98,0,0x6B};
            uint8_t disable[6]={0,0xF3,0xAB,0,0,0x6B};
            if(tr_bus_write(emergency==2 ? stop : disable,emergency==2 ? 5 : 6)) {
                emergency--; bus_free_us=now+4000UL;
            }
        }
        return;
    }
    check_motion();
    if(global_fault) return;
    if(due(next_tick)) {
        late=now-next_tick;
        if(moving() && late>TR_LATE_US) { fail(TR_DEADLINE); return; }
        if(moving() && late>tr_diag.max_tick_late_us) tr_diag.max_tick_late_us=late;
        if(moving() && coast_tick()) {
            next_tick+=TR_TICK_US;
        } else if(!bus.kind && tr_bus_idle() && due(bus_free_us)) {
            if(!moving()) next_tick=now;
            if(stream_tick()) next_tick=moving() ? next_tick+TR_TICK_US : now+TR_TICK_US;
        } /* Keep a due tick pending until the bus is actually free. */
    }
    if(bus.kind || !tr_bus_idle() || !due(bus_free_us) || global_fault) return;
    for(i=0;i<MOTOR_COUNT;i++) if(tr_motor[i].active==2 && spare_time(3600)) {
        uint8_t c[5]={0,0xFE,0x98,0,0x6B}; c[0]=i+1;
        send_bus(BUS_HOME_STOP,i+1,0xFE,4,0,c,5); return;
    }
    if(request.kind==1 && spare_time(6500)) { control_service(); return; }
    for(i=0;i<MOTOR_COUNT;i++) {
        TrMotor *m=&tr_motor[i];
        if(!m->active && m->count && group_at_heads(front(m)) &&
           m->current!=front(m)->current && spare_time(3600)) {
            set_current(BUS_CURRENT,i,front(m)->current); return;
        }
    }
    feedback_service();
}


/* QNX headless HID bridge. No motor access; stdout carries only WASD/stop/F.
 * Supported device: Compx 25a7:fa70 boot-keyboard interface (8-byte reports).
 * Non-exclusive observer. Heartbeat freshness is separate from the Python two-second held-key limit.
 */
#include <sys/hiddi.h>
#include <sys/hidut.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static volatile sig_atomic_t running = 1;
static int attached, mask;
static uint64_t stamp;
static hidd_device_instance_t *device;
static uint64_t micros(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000; }
static void stop(int s) { (void)s; running=0; }
static void report(struct hidd_connection *c, struct hidd_report *r, void *data, _Uint32t n, _Uint32t flags, void *u) {
    (void)c; (void)r; (void)u;
    unsigned char *p=data; int m=0;
    if (n!=8 || (flags&HIDD_REPORT_BUFFER_OVERFLOW) || (n==8 && (p[0] & ~0x22u))) m=16;
    else for (unsigned i=2;i<8;i++) {
        switch(p[i]) { case 0x1a:m|=1;break; case 0x16:m|=2;break;
        case 4:m|=4;break; case 7:m|=8;break; case 0x29:case 0x2c:m|=16;break;
        case 9:m|=32;break; case 1:case 2:case 3:m|=16;break; }
    }
    if (n==8 && (p[0] & 0x22u) && !(m & 16)) m |= 64; /* left or right Shift */
    pthread_mutex_lock(&lock); mask=m; stamp=micros(); pthread_mutex_unlock(&lock);
}
static void insertion(struct hidd_connection *c, hidd_device_instance_t *d) {
    if (attached) return;
    struct hidd_collection **cols; _Uint16t count;
    if(hidd_get_collections(d,NULL,&cols,&count)) return;
    for(unsigned i=0;i<count;i++) {
        _Uint16t page, usage, len; struct hidd_report_instance *ri; struct hidd_report *r;
        if(hidd_collection_usage(cols[i],&page,&usage) || page!=1 || usage!=6) continue;
        if(hidd_get_report_instance(cols[i],0,HID_INPUT_REPORT,&ri) || hidd_report_len(ri,&len) || len!=8) continue;
        /* Only the verified boot interface, never the receiver's mouse/NKRO interface. */
        if(count!=1) continue;
        int rc=hidd_report_attach(c,d,ri,0,0,&r);
        if(rc) { fprintf(stderr,"[keyboard] attach: %s\n",strerror(rc)); continue; }
        pthread_mutex_lock(&lock); device=d; attached=1; stamp=0; mask=0; pthread_mutex_unlock(&lock);
        rc=hidd_set_idle(r,20); /* HID idle units are 4 ms: request 80 ms reports. */
        fprintf(stderr,"[keyboard] attached Compx dev %u; idle status %d\n",d->devno,rc);
        break;
    }
}
static void removal(struct hidd_connection *c,hidd_device_instance_t *d) {
    pthread_mutex_lock(&lock);
    if(device==d) { attached=0; device=NULL; mask=0; stamp=0; }
    pthread_mutex_unlock(&lock); hidd_reports_detach(c,d);
}
int main(void) {
    signal(SIGTERM,stop); signal(SIGINT,stop); signal(SIGPIPE,stop);
    fcntl(STDOUT_FILENO,F_SETFL,fcntl(STDOUT_FILENO,F_GETFL)|O_NONBLOCK);
    hidd_device_ident_t id={0x25a7,0xfa70,HIDD_CONNECT_WILDCARD};
    hidd_funcs_t funcs={_HIDDI_NFUNCS,insertion,removal,report,NULL};
    hidd_connect_parm_t p={0}; p.vhid=HID_VERSION; p.vhidd=HIDD_VERSION;
    p.funcs=&funcs; p.device_ident=&id; p.connect_wait=5;
    struct hidd_connection *c; int rc=hidd_connect(&p,&c);
    if(rc) { fprintf(stderr,"[keyboard] HID connect: %s\n",strerror(rc)); return 1; }
    while(running) {
        char line[128]; pthread_mutex_lock(&lock);
        int n=snprintf(line,sizeof line,"%llu %d %d\n",(unsigned long long)micros(),attached && stamp != 0,mask);
        pthread_mutex_unlock(&lock);
        if(write(STDOUT_FILENO,line,n)<0 && errno!=EAGAIN && errno!=EINTR) break;
        struct timespec delay={0,20000000}; nanosleep(&delay,NULL);
    }
    hidd_disconnect(c); return 0;
}

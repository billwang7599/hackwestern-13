/* tars_motor: QNX motor/arm server for TARS.
 *
 * TCP line protocol (one command per line, reply "OK ..." or "ERR <why>"):
 *   D <left> <right>     drive, each in [-1, 1] (must be resent < watchdog ms)
 *   DL <epoch> <l> <r> leased drive: epoch from STATUS; a guard denial invalidates it
 *                      (guard denial: D/DL/MOVE refused + aborted; an in-place TURN still runs
 *                      while the reason is a fresh guard_blocked, so the robot can pivot away,
 *                      but only a limited one: speed clamped to PIVOT_BLOCKED_MAX_SPEED (0.40),
 *                      |deg| > PIVOT_BLOCKED_MAX_DEG (180) -> "ERR guard_blocked pivot>180".
 *                      A TURN already running when the guard turns BLOCKED continues only if
 *                      it fits both limits, else it aborts (move_why=lidar_guard). The guard
 *                      can't see the sideways/behind sweep: that relies on goto's 0.35 m map
 *                      inflation. A reverse MOVE (mm < 0) is also allowed under a fresh
 *                      guard_blocked, speed clamped to REVERSE_BLOCKED_MAX_SPEED (0.50; on DC
 *                      wheels MOVE speed is ignored), so goto can back out the way it came;
 *                      a forward MOVE or a running reverse MOVE faster than 0.50 aborts.
 *                      Stale/missing/invalid guard data still refuses both. Pending
 *                      Andrew's sign-off.)
 *   S                    stop driving NOW (no slew), cancel any gesture, halt both arms
 *   G <name>             gesture: wave point shrug nod stance fold
 *   A <arm> <deg> [dps]  arm 0=left 1=right to an absolute angle -> "OK <deg>"
 *   HOME                 both arms to 0
 *   ZERO                 the arms' current position becomes 0
 *   LEVEL                "the robot is upright now" (recapture tilt reference)
 *   RESET                clear a tip-over latch (refused while still tipped)
 *   GESTURES             -> "OK wave point ..."
 *   PING                 -> "OK PONG" (also feeds the watchdog)
 *   STATUS               -> "OK <l> <r> <ms_since_cmd> <dist_m> <estop> key=val..."
 *   MOVE <mm> [speed]    drive <mm> (+ fwd), speed in (0,1]. Stepper wheels: exact (steps).
 *                        DC wheels (Uno says WHEELS DC): timed open loop at D 1, speed ignored.
 *                        -> "OK <id> <est_ms>" at once; progress in STATUS move=...
 *   TURN <deg> [speed]   spin <deg>, + = right (clockwise); exact on steppers, timed on DC.
 *                        DC: speed < 0.99 = the Uno's SLOW turn (own duty, timed at
 *                        dc_deg_s_slow), else D 1. Goto's 0.40 turns and the
 *                        guard_blocked pivot clamp are slow turns on both wheel types.
 *                        A non-zero D, S, a new MOVE/TURN, tip, bump or the
 *                        client disconnecting aborts a running move.
 *   WHEELCAL [diam track] set (and save on the Uno) wheel diameter / track mm (steppers)
 *   SPEEDCAL [mm_s deg_s [deg_s_slow]] DC wheels: save the speeds measured at D 1 on the
 *                        Uno (V), and the slow-turn rate (omitted = the Uno keeps its own);
 *                        no args -> "OK dc <mm_s> <deg_s> [<deg_s_slow>]" or "OK stepper"
 *                        (the slow rate is shown once the Uno has reported it)
 *   LED <name|0-3>       on-robot status LED. Names (iter3-pet/vision/GESTURES.md §7): off=0,
 *                        teach=2 (slow blink), rec=1 (solid), ok=1, no=3 (fast), forget=3
 *   BEEP <n>             n short beeps on the robot's buzzer (0 = silence)
 *   VIS <mode> <fps> <cam_ms> <stop_ms> <label>
 *                        report from the vision app (iter3-pet/vision/), shown in STATUS as
 *                        vis_*; does NOT feed the watchdog
 *   MODE <gesture|follow|toggle|teach>
 *                        ask the vision app to switch mode (e.g. from iter2-autonomy/voice/;
 *                        teach = start teach-a-gesture, like the Uno button). Shown
 *                        in STATUS as mode_req= mode_seq=, and in the reply to VIS
 *                        ("OK MODE <name> <seq>") so the vision app sees it at 1 Hz.
 *   WHO <name>           label this client in logs/STATUS (voice, vision, teleop...)
 *   SIM T|B|READY [n]    stub backend only: inject a co-processor event (tests)
 *   SIM BTN TEACH|FORGET stub backend only: inject a button press
 *   POSE                 -> "OK <x_mm> <y_mm> <hdg_deg> <epoch> <moving>" dead-reckoned pose
 *   POSE RESET           pose back to 0,0,0 and a new epoch (iter2-autonomy/goto/ re-bases its memory)
 *
 * DC wheels: if the Uno sends "WHEELS DC <mm_s> <deg_s>" (Kuman SM11 firmware), D 1 is
 * taken as mm_s straight and deg_s spinning, for the pose and for MOVE/TURN estimates.
 * "WHEELS DCSLOW <deg_s>" (right after it) is the slow-turn rate: a TURN with speed < 0.99
 * is estimated with it. An Uno that never sends it ignores TURN speed, so deg_s is used.
 * STATUS has wheels=stepper|dc dc_mm_s= dc_deg_s= dc_deg_s_slow= (0 = not reported).
 *
 * Pose (best effort, no encoders): the 28BYJ-48 wheels move what they're told,
 * so the pose is integrated from the motion that actually ran: the slewed D
 * output every tick (fraction x max wheel speed), plus each exact MOVE/TURN when
 * the Uno reports DONE/ABORT with the amount done. x is forward from where the
 * server started, y to the right, hdg + = clockwise (like TURN). moving=1 while
 * an exact move runs (its motion is only added when it ends). pose_epoch changes
 * when the server restarts or on POSE RESET. Skipped steps, wheel slip and being
 * carried are invisible to it. Pose never touches the stop path.
 *
 * STATUS also has move=idle|running|done|aborted move_id= move_kind=MOVE|TURN|-
 * move_req= move_done= move_why= move_est_ms= wheel_d= wheel_t= and the on-robot
 * button counters btn_teach= btn_forget= (they only ever count up), and stops=
 * (number of S received from any client; iter2-autonomy/goto/ cancels when it goes up).
 *
 * Several clients at once (e.g. iter3-pet/vision/ + iter2-autonomy/voice/ on the Pi, teleop on a laptop):
 *   - S from ANY client stops everything at once, and for -H ms (default 1500)
 *     non-zero D from OTHER clients is refused ("ERR held ..."), so a follow-mode
 *     D stream can't undo someone else's stop. The client that sent S can drive
 *     again at once. MOVE/TURN are not held (they're one-shot, deliberate).
 *   - a client disconnecting only zeroes the wheels if it was the last one to
 *     drive, and only aborts a MOVE/TURN that it started.
 *
 * Runs unattended at boot: no client = wheels held at 0 (the watchdog starts
 * tripped), and a missing/unplugged Uno is reopened every second.
 *
 * Threads:
 *   main      accept loop, one detached thread per client
 *   control   SCHED_RR high priority, fixed-rate tick. The ONLY thread that
 *             talks to the backend: drains events (tip/bump/ready), applies
 *             the watchdog / obstacle / tip / bump stops, slews the wheels,
 *             plays gesture keyframes and keeps a model of the arm angles.
 *
 * Usage: tars_motor [-b uno|stub|pico] [-d dev] [-p port] [-w watchdog_ms]
 *                   [-r rt_priority] [-k bump_hold_ms] [-H stop_hold_ms]
 */
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "gesture.h"
#include "motor.h"
#include "../../../shared/action_log.h"
#include "obstacle.h"
#include "wheels.h"
#include "guard_gate.h"

#define TICK_HZ          50
#define SLEW_PER_TICK    0.08f   /* max change per tick: 0 -> full in ~0.25 s */
#define OBSTACLE_STOP_M  0.25f   /* block forward motion closer than this */
#define TIP_CLEAR_MS     1000    /* RESET refused if a T event came this recently */
/* In-place TURN allowed while lidar_guard reports a fresh BLOCKED (forward corridor only,
 * blind to the pivot's sideways sweep): keep that pivot slow and at most a half turn. */
#define PIVOT_BLOCKED_MAX_SPEED 0.40f
#define PIVOT_BLOCKED_MAX_DEG   180.0f
/* Reverse MOVE allowed under a fresh BLOCKED (the guard looks forward; goto backs out along
 * the trail it drove in on). Tip, bump, S, the watchdog and the deadman still stop it. */
#define REVERSE_BLOCKED_MAX_SPEED 0.50f

static const motor_backend_t *motor_backends[] = { &motor_uno_serial, &motor_stub, &motor_pico_uart };

/* ---- config (from argv) ---- */
static const char *cfg_backend = "uno";
static const char *cfg_dev     = NULL;      /* NULL = backend default */
static int cfg_port      = 5555;
static int cfg_wd_ms     = 500;
static int cfg_rt_prio   = 30;
static int cfg_bump_ms   = 1000;
static int cfg_guard_port = 0; /* service explicitly enables 5568; legacy bench tests use 0 */
static int cfg_local = 0;
static int guard_permit = 1;
static unsigned long long guard_epoch = 1;
static long guard_age = -1;
static const char *guard_reason = "disabled";
static int cfg_hold_ms   = 1500;          /* S from one client holds other clients' D */

/* ---- shared state (all under `lock`) ---- */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static float target_l, target_r;          /* what clients asked for */
static float out_l, out_r;                /* what we last sent to motors */
static float last_dist = -1.0f;
static int   estop;                       /* 1 = some stop is holding the wheels */
static const char *estop_why = "none";
static struct timespec last_cmd;
/* multi-client ownership (client ids start at 1; 0 = nobody) */
static const char *keyboard_auth;
static long long keyboard_until; /* renewed only by authenticated physical-key commands */
static int keyboard_obstacle;
static int   drive_owner;                 /* last client to set a non-zero D */
static int   mv_owner;                    /* client that started the current MOVE/TURN */
static int   hold_cid;                    /* client whose S is holding the others */
static long long hold_until_ms;
static int   n_clients;
static char  mode_req[12] = "-";          /* MODE request for the vision app */
static int   mode_seq;

/* requests from client threads, executed by the control thread */
typedef struct { int set; float deg, dps; } arm_req_t;
static arm_req_t arm_req[2];
static int   req_halt, req_zero, req_level, req_stop_gesture, req_reset;
static int   req_hard_stop;                /* S: zero the wheels this tick, no slew */
static long long stop_rx_ms;
static int   stop_count;                  /* S commands received (STATUS stops=), lets
                                           * autonomous clients (iter2-autonomy/goto/) see a stop sent by
                                           * someone else and give up */              /* when that S arrived */
static long  stop_ms = -1;                /* last S: received -> wheels commanded 0 */
/* last VIS report from the vision app */
static char  vis_mode[16] = "-", vis_see[24] = "-";
static float vis_fps;
static int   vis_cam_ms = -1, vis_stop_ms = -1;
static long long vis_at_ms;
static const gesture_t *req_gesture;
static char  req_dev_gesture[48];         /* pico: device-side gesture */
#define SIMQ 8
static motor_event_t simq[SIMQ];
static int   simq_n;

/* published by the control thread for STATUS / command checks */
static float arm_pos[2], arm_tgt[2];
static const char *gesture_name = "none";
static int   tipped, tilt_deg, bumps;
static long long last_tip_ms, bump_until_ms;
static int   ready_count;
static int   link_up;  /* protected by lock once worker threads start */

/* exact moves (MOVE/TURN). State under `lock`; uno_pending is control-thread only. */
typedef enum { MV_IDLE, MV_RUNNING, MV_DONE, MV_ABORTED } mv_state_t;
static const char *mv_names[] = { "idle", "running", "done", "aborted" };
static mv_state_t mv_state;
static int   mv_id, mv_est_ms;
static char  mv_kind;                     /* 'M' | 'R' | 0 */
static float mv_req, mv_done, mv_speed;
static char  mv_why[16] = "-";
static long long mv_deadline_ms;
static int   req_move;                    /* client asked: send mv_* to the device */
static const char *req_move_abort;        /* client/thread asked: abort, with this why */
static float wheel_d = WHEEL_DIAM_MM, wheel_t = WHEEL_TRACK_MM;
static int   req_wheelcal;
/* DC wheels (set by the Uno's WHEELS line, never by a client): speeds at D 1 */
static int   wheels_dc;
static float dc_mm_s, dc_deg_s;
static float dc_deg_s_slow;               /* WHEELS DCSLOW; 0 = the Uno didn't say (old firmware) */
#define DC_SLOW_BELOW 0.99f               /* TURN speed below this = the Uno's slow turn */
static int   req_speedcal;                /* 1 = send V (dc_req_*: 0 0 = query) */
static float dc_req_mm_s, dc_req_deg_s, dc_req_deg_s_slow;
static int   led_pattern = -1, req_led = -1, req_beep = -1;
static int   btn_teach, btn_forget;
static int   uno_pending;                 /* moves sent to the device, not yet answered */
/* dead-reckoned pose (under `lock`), see the header comment */
static double pose_x, pose_y, pose_h;     /* mm, mm, deg (+ = clockwise), h in (-180, 180] */
static unsigned pose_epoch;
static long long pose_last_ms;

static volatile sig_atomic_t running = 1;
static const motor_backend_t *motor;

static long long now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static long ms_since(const struct timespec *t)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->tv_sec) * 1000L + (now.tv_nsec - t->tv_nsec) / 1000000L;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static double wrap180(double d)
{
    d = fmod(d, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d <= -180.0) d += 360.0;
    return d;
}

/* Called with `lock` held. Differential drive, midpoint heading. */
static void pose_advance_locked(double dist_mm, double dhdg_deg)
{
    double mid = (pose_h + dhdg_deg / 2.0) * M_PI / 180.0;
    pose_x += dist_mm * cos(mid);
    pose_y += dist_mm * sin(mid);
    pose_h = wrap180(pose_h + dhdg_deg);
}

/* Called with `lock` held: wheel fractions l, r ran for dt_s seconds. */
static void pose_drive_locked(float l, float r, double dt_s)
{
    if (fabsf(l) < 0.03f) l = 0;              /* the Uno's DRIVE_DEADBAND */
    if (fabsf(r) < 0.03f) r = 0;
    if (l == 0 && r == 0) return;
    double vmax = WHEEL_MAX_RPM / 60.0 * M_PI * wheel_d;      /* mm/s at fraction 1 */
    double track = wheel_t;
    if (wheels_dc) {
        /* linear in D (rough: the Uno maps D to a min..cap duty); the effective track
         * makes D 1 -1 spin at the measured deg/s (skid steer slips, so it's wider) */
        vmax = dc_mm_s;
        track = 2.0 * dc_mm_s / (dc_deg_s * M_PI / 180.0);
    }
    double vl = l * vmax, vr = r * vmax;
    pose_advance_locked((vl + vr) / 2.0 * dt_s, (vl - vr) / track * dt_s * 180.0 / M_PI);
}

/* Called with `lock` held: how long a MOVE/TURN should take, ms. */
static int move_est_ms_locked(char kind, float amt, float spd)
{
    if (wheels_dc) {
        float rate = kind == 'M' ? dc_mm_s : dc_deg_s;
        if (kind == 'R' && spd > 0 && spd < DC_SLOW_BELOW && dc_deg_s_slow > 0) rate = dc_deg_s_slow;
        return (int)(fabsf(amt) / rate * 1000.0f + 0.5f);
    }
    return wheel_move_ms(wheel_move_steps(kind, amt, wheel_d, wheel_t), spd);
}

static unsigned new_epoch(void)
{
    struct timespec a, b;
    clock_gettime(CLOCK_REALTIME, &a);
    clock_gettime(CLOCK_MONOTONIC, &b);
    unsigned e = (unsigned)(a.tv_nsec ^ (a.tv_sec * 2654435761u) ^ (b.tv_nsec * 40503u) ^ ((unsigned)getpid() << 16));
    e %= 1000000000u;
    return e ? e : 1;
}

static float slew(float cur, float tgt)
{
    float d = clampf(tgt - cur, -SLEW_PER_TICK, SLEW_PER_TICK);
    return cur + d;
}

static void sleep_until(struct timespec *next)
{
#ifdef __APPLE__
    /* macOS has no clock_nanosleep; relative sleep is fine for a host build. */
    struct timespec now, d;
    clock_gettime(CLOCK_MONOTONIC, &now);
    d.tv_sec = next->tv_sec - now.tv_sec;
    d.tv_nsec = next->tv_nsec - now.tv_nsec;
    if (d.tv_nsec < 0) { d.tv_nsec += 1000000000L; d.tv_sec--; }
    if (d.tv_sec >= 0) nanosleep(&d, NULL);
#else
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL) == EINTR) {}
#endif
}

/* ------------------------------------------------------------------ control
 * Everything below with a ctl_ prefix runs only in the control thread. */
static float ctl_pos[2], ctl_tgt[2], ctl_dps[2] = { ARM_DEFAULT_DPS, ARM_DEFAULT_DPS };
static const gesture_t *ctl_g;
static int ctl_g_idx;
static long long ctl_g_next;

static void ctl_arm(int i, float deg, float dps)
{
    deg = clampf(deg, -ARM_LIMIT_DEG, ARM_LIMIT_DEG);
    dps = clampf(dps, 1.0f, ARM_MAX_DPS);
    ctl_tgt[i] = deg;
    ctl_dps[i] = dps;
    if (motor->arm) motor->arm(i, deg, dps);
}

static void ctl_halt_arms(void)
{
    ctl_tgt[0] = ctl_pos[0];
    ctl_tgt[1] = ctl_pos[1];
    if (motor->arm_halt) motor->arm_halt();
}

static void ctl_gesture_stop(const char *why)
{
    if (!ctl_g) return;
    fprintf(stderr, "[ctl] gesture %s %s\n", ctl_g->name, why);
    ctl_g = NULL;
}

static void ctl_gesture_start(const gesture_t *g)
{
    ctl_gesture_stop("cancelled");
    ctl_g = g;
    ctl_g_idx = -1;
    ctl_g_next = 0;
    fprintf(stderr, "[ctl] gesture %s start (~%d ms)\n", g->name, gesture_duration_ms(g, ctl_pos));
}

/* Next keyframe once the previous one's travel + hold time has passed. */
static void ctl_gesture_tick(long long now)
{
    if (!ctl_g || now < ctl_g_next) return;
    if (++ctl_g_idx >= ctl_g->n) { ctl_gesture_stop("done"); return; }
    const keyframe_t *f = &ctl_g->frames[ctl_g_idx];
    float travel = 0;
    for (int a = 0; a < 2; a++) {
        if (f->deg[a] >= ARM_KEEP) continue;
        float t = fabsf(f->deg[a] - ctl_pos[a]) / clampf(f->dps, 1.0f, ARM_MAX_DPS);
        if (t > travel) travel = t;
        ctl_arm(a, f->deg[a], f->dps);
    }
    ctl_g_next = now + (long long)(travel * 1000.0f * FRAME_MARGIN) + FRAME_PAD_MS + f->hold_ms;
}

/* Dead-reckoned arm angles: the steppers move at a known rate, so the Pi can
 * track them without feedback (until something skips steps or the Uno reboots). */
static void ctl_arm_model(float dt)
{
    for (int i = 0; i < 2; i++) {
        float step = ctl_dps[i] * dt, d = ctl_tgt[i] - ctl_pos[i];
        ctl_pos[i] = fabsf(d) <= step ? ctl_tgt[i] : ctl_pos[i] + (d > 0 ? step : -step);
    }
}

static void ctl_move_abort_locked(const char *why);
static int guard_pivot_ok(void);
static int pivot_fits(float deg, float speed);
static int blocked_exempt_fits(char kind, float req, float speed);

static void ctl_event(const motor_event_t *ev, long long now)
{
    switch (ev->kind) {
    case MOTOR_EV_TIP:
        pthread_mutex_lock(&lock);
        int was = tipped;
        tipped = 1;
        tilt_deg = ev->value;
        last_tip_ms = now;
        target_l = target_r = 0.0f;
        ctl_move_abort_locked("tipped");
        pthread_mutex_unlock(&lock);
        if (!was) {
            fprintf(stderr, "[ctl] TIPPED (%d deg) -> stop, latched until RESET\n", ev->value);
            ctl_gesture_stop("cancelled (tipped)");
            ctl_halt_arms();
        }
        break;
    case MOTOR_EV_BUMP:
        pthread_mutex_lock(&lock);
        bumps++;
        bump_until_ms = now + cfg_bump_ms;
        target_l = target_r = 0.0f;
        ctl_move_abort_locked("bump");
        pthread_mutex_unlock(&lock);
        fprintf(stderr, "[ctl] BUMP (%d mg) -> stop, drive refused for %d ms\n", ev->value, cfg_bump_ms);
        break;
    case MOTOR_EV_READY:
        pthread_mutex_lock(&lock);
        int n = ++ready_count;
        if (mv_state == MV_RUNNING) {     /* a rebooted Uno forgot the move */
            mv_state = MV_ABORTED;
            snprintf(mv_why, sizeof mv_why, "reboot");
        }
        uno_pending = 0;
        int led = led_pattern;
        pthread_mutex_unlock(&lock);
        if (led >= 0 && motor->led) motor->led(led);   /* restore the LED it forgot */
        if (n > 1)
            fprintf(stderr, "[ctl] co-processor READY again (it rebooted): arm angles reset to 0, "
                            "re-zero the arms by hand if they were not at 0\n");
        else
            fprintf(stderr, "[ctl] co-processor READY\n");
        ctl_gesture_stop("cancelled (co-processor reboot)");
        ctl_pos[0] = ctl_pos[1] = ctl_tgt[0] = ctl_tgt[1] = 0.0f;
        break;
    case MOTOR_EV_MOVE_DONE:
    case MOTOR_EV_MOVE_ABORT: {
        int ok = ev->kind == MOTOR_EV_MOVE_DONE;
        if (uno_pending > 0 && isfinite(ev->amount)) {   /* the wheels really did this much */
            pthread_mutex_lock(&lock);
            if (ev->what == 'M') pose_advance_locked(ev->amount, 0);
            else if (ev->what == 'R') pose_advance_locked(0, ev->amount);
            pthread_mutex_unlock(&lock);
        }
        if (uno_pending <= 0) {
            fprintf(stderr, "[ctl] stray %s %c %.1f ignored\n", ok ? "DONE" : "ABORT", ev->what, ev->amount);
            break;
        }
        if (--uno_pending > 0) {          /* answer to an older, replaced move */
            fprintf(stderr, "[ctl] %s %c %.1f (older move)\n", ok ? "DONE" : "ABORT", ev->what, ev->amount);
            break;
        }
        pthread_mutex_lock(&lock);
        mv_state = ok ? MV_DONE : MV_ABORTED;
        mv_done = ev->amount;
        if (ok) snprintf(mv_why, sizeof mv_why, "-");
        else if (!strcmp(mv_why, "-")) snprintf(mv_why, sizeof mv_why, "%s", ev->why[0] ? ev->why : "?");
        int id = mv_id;
        pthread_mutex_unlock(&lock);
        fprintf(stderr, "[ctl] move %d %s: %c %.1f %s\n", id, ok ? "done" : "aborted", ev->what,
                ev->amount, ok ? "" : ev->why);
        break;
    }
    case MOTOR_EV_WHEELS:
        if (ev->what == 'S') {                /* WHEELS DCSLOW: follows WHEELS DC */
            pthread_mutex_lock(&lock);
            dc_deg_s_slow = ev->amount;
            pthread_mutex_unlock(&lock);
            fprintf(stderr, "[ctl] DC slow turn: %.1f deg/s (TURN speed < %.2f)\n", ev->amount, DC_SLOW_BELOW);
            break;
        }
        pthread_mutex_lock(&lock);
        dc_deg_s_slow = 0;                    /* until DCSLOW: an old Uno ignores TURN speed */
        int was_dc = wheels_dc;
        wheels_dc = 1;
        dc_mm_s = ev->amount;
        dc_deg_s = ev->amount2;
        pthread_mutex_unlock(&lock);
        fprintf(stderr, "[ctl] %sDC wheels: %.1f mm/s, %.1f deg/s at D 1 (timed MOVE/TURN)\n",
                was_dc ? "" : "co-processor has ", ev->amount, ev->amount2);
        break;
    case MOTOR_EV_BTN:
        pthread_mutex_lock(&lock);
        if (ev->value == MOTOR_BTN_TEACH) btn_teach++;
        else if (ev->value == MOTOR_BTN_FORGET) btn_forget++;
        pthread_mutex_unlock(&lock);
        fprintf(stderr, "[ctl] button %s\n", ev->value == MOTOR_BTN_TEACH ? "TEACH" : "FORGET");
        break;
    default:
        break;
    }
}

/* Abort the running move (if any). Called with `lock` held, from the control
 * thread. The local state flips at once; the device's ABORT fills in how far it got. */
static void ctl_move_abort_locked(const char *why)
{
    if (mv_state != MV_RUNNING) return;
    mv_state = MV_ABORTED;
    snprintf(mv_why, sizeof mv_why, "%s", why);
    if (motor->move_abort) motor->move_abort();
    fprintf(stderr, "[ctl] move %d abort (%s)\n", mv_id, why);
}

static void *control_thread(void *arg)
{
    (void)arg;
    const long period_ns = 1000000000L / TICK_HZ;
    const float dt = 1.0f / TICK_HZ;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const char *was_why = "none";

    while (running) {
        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        long long now = now_ms();

        int connected = !motor->link || motor->link();
        pthread_mutex_lock(&lock);
        if (link_up && !connected) {
            ++guard_epoch;                 /* invalidate outstanding drive leases */
            target_l = target_r = out_l = out_r = 0;
            keyboard_until = 0;
            req_move = uno_pending = 0;
            ctl_move_abort_locked("uno_disconnected");
            req_gesture = NULL;
            req_dev_gesture[0] = 0;
            req_stop_gesture = req_halt = 1;
            memset(arm_req, 0, sizeof arm_req);
        }
        link_up = connected;
        pthread_mutex_unlock(&lock);

        /* 1. events: from the co-processor, then injected ones (SIM) */
        motor_event_t ev;
        for (int i = 0; i < 64 && motor->poll && motor->poll(&ev); i++) ctl_event(&ev, now);
        motor_event_t sims[SIMQ];
        pthread_mutex_lock(&lock);
        int ns = simq_n;
        memcpy(sims, simq, sizeof sims);
        simq_n = 0;
        pthread_mutex_unlock(&lock);
        for (int i = 0; i < ns; i++) ctl_event(&sims[i], now);

        /* 2. take client requests */
        float dist = obstacle_distance_m();
        int gate_ok = guard_gate_poll();
        pthread_mutex_lock(&lock);
        if (guard_permit && !gate_ok) ++guard_epoch;
        guard_permit = gate_ok; guard_age = guard_gate_age_ms(); guard_reason = guard_gate_reason();
        keyboard_obstacle = guard_gate_obstacle();
        int expired_key = keyboard_until && now >= keyboard_until;
        int key_drive = keyboard_until > now && (gate_ok || keyboard_obstacle);
        if (expired_key || (!gate_ok && !key_drive)) {
            target_l = target_r = 0;
            keyboard_until = 0;
        }
        if (!gate_ok) {
            /* lidar_guard watches a FORWARD corridor only, so an in-place pivot isn't what it
             * guards: while it reports a fresh BLOCKED (not stale/missing/invalid) an exact TURN
             * may run, so the robot can turn away from what's ahead, if it fits the pivot
             * limits (speed <= 0.40, |deg| <= 180; no mid-move speed change on the Uno, so a
             * faster or longer TURN aborts). Likewise a reverse MOVE at speed <= 0.50 keeps
             * running. Drives and forward MOVE stop. */
            if (!(guard_pivot_ok() && blocked_exempt_fits(mv_kind, mv_req, mv_speed))) {
                req_move = 0;
                ctl_move_abort_locked("lidar_guard");
            }
            req_stop_gesture = req_halt = 1;
            req_gesture = NULL; req_dev_gesture[0] = 0;
            memset(arm_req, 0, sizeof arm_req);
        }
        arm_req_t areq[2];
        memcpy(areq, arm_req, sizeof areq);
        memset(&arm_req, 0, sizeof arm_req);
        int halt = req_halt, zero = req_zero, level = req_level;
        int stop_g = req_stop_gesture, reset = req_reset;
        int hard = req_hard_stop || expired_key;
        long long hard_rx = req_hard_stop ? stop_rx_ms : now;
        req_hard_stop = 0;
        const gesture_t *start_g = req_gesture;
        char dev_g[48];
        memcpy(dev_g, req_dev_gesture, sizeof dev_g);
        req_halt = req_zero = req_level = req_stop_gesture = req_reset = 0;
        req_gesture = NULL;
        req_dev_gesture[0] = '\0';

        /* exact moves: abort first, then a new one, then the timeout */
        if (req_move_abort) { ctl_move_abort_locked(req_move_abort); req_move_abort = NULL; }
        if (req_move && mv_state == MV_RUNNING && motor->move) {
            motor->move(mv_kind, mv_req, mv_speed);
            uno_pending++;
            mv_deadline_ms = now + 2LL * mv_est_ms + 2000;
            fprintf(stderr, "[ctl] move %d: %s %.1f speed %.2f (~%d ms)\n", mv_id,
                    mv_kind == 'M' ? "MOVE" : "TURN", mv_req, mv_speed, mv_est_ms);
        }
        req_move = 0;
        if (mv_state == MV_RUNNING && now > mv_deadline_ms) {
            ctl_move_abort_locked("timeout");   /* device never answered */
            uno_pending = 0;
        }
        int moving = mv_state == MV_RUNNING;
        int calibrate_wheels = req_wheelcal;
        float calibrate_d = wheel_d, calibrate_t = wheel_t;
        req_wheelcal = 0;
        int calibrate_speed = req_speedcal;
        float cal_mm_s = dc_req_mm_s, cal_deg_s = dc_req_deg_s, cal_deg_s_slow = dc_req_deg_s_slow;
        req_speedcal = 0;
        int led = req_led, beep = req_beep;
        req_led = req_beep = -1;

        /* 3. wheels: watchdog, obstacle, tip latch, bump hold */
        float tl = target_l, tr = target_r;
        int wd_tripped = ms_since(&last_cmd) > cfg_wd_ms;
        if (wd_tripped) { tl = tr = 0.0f; target_l = target_r = 0.0f; ctl_move_abort_locked("watchdog"); }
        /* Obstacle: allow turning/reversing, block net forward motion. */
        int blocked = !key_drive && dist >= 0.0f && dist < OBSTACLE_STOP_M && (tl + tr) > 0.0f;
        if (moving && dist >= 0.0f && dist < OBSTACLE_STOP_M && mv_kind == 'M' && mv_req > 0)
            ctl_move_abort_locked("obstacle");
        int bumped = now < bump_until_ms;
        if (blocked || tipped || bumped || (!gate_ok && !key_drive)) { tl = tr = 0.0f; target_l = target_r = 0.0f; }
        estop = !link_up || wd_tripped || blocked || tipped || bumped || (!gate_ok && !key_drive);
        estop_why = !link_up ? "uno_disconnected" : tipped ? "tipped" : bumped ? "bump" : blocked ? "obstacle"
                  : (!gate_ok && !key_drive) ? guard_reason : wd_tripped ? "watchdog" : "none";
        last_dist = dist;
        /* Hard stop on estop (no slew); smooth otherwise. */
        out_l = (estop || hard) ? tl : slew(out_l, tl);
        out_r = (estop || hard) ? tr : slew(out_r, tr);
        float l = out_l, r = out_r;
        int is_tipped = tipped;
        /* pose: what we're about to send runs until the next tick */
        if (pose_last_ms) {
            long long el = now - pose_last_ms;
            if (el > 0 && el < 200) pose_drive_locked(l, r, el / 1000.0);
        }
        pose_last_ms = now;
        pthread_mutex_unlock(&lock);

        if (strcmp(estop_why, was_why) && strcmp(estop_why, "none"))
            fprintf(stderr, "[ctl] STOP (%s)\n", estop_why);
        was_why = estop_why;

        /* Commit wheel zero before arm/LED writes can flush queued serial data.
         * During exact moves D 0 0 is a keepalive; gate denial already aborts M/R. */
        motor->drive(l, r);
        if (calibrate_wheels && motor->wheel_cal) motor->wheel_cal(calibrate_d, calibrate_t);
        if (calibrate_speed && motor->speed_cal) motor->speed_cal(cal_mm_s, cal_deg_s, cal_deg_s_slow);

        /* 4. arms + gestures (backend calls happen only here) */
        if (reset) fprintf(stderr, "[ctl] RESET: tip latch cleared\n");
        if (stop_g) ctl_gesture_stop("cancelled");
        if (halt) ctl_halt_arms();
        if (zero) {
            ctl_gesture_stop("cancelled");
            ctl_pos[0] = ctl_pos[1] = ctl_tgt[0] = ctl_tgt[1] = 0.0f;
            if (motor->arm_zero) motor->arm_zero();
        }
        if (level && motor->level) motor->level();
        if (!is_tipped && gate_ok) {
            for (int i = 0; i < 2; i++)
                if (areq[i].set) ctl_arm(i, areq[i].deg, areq[i].dps);
            if (start_g) ctl_gesture_start(start_g);
            ctl_gesture_tick(now);
        }
        ctl_arm_model(dt);

        if (led >= 0 && motor->led) motor->led(led);
        if (beep >= 0 && motor->beep) motor->beep(beep);
        if (hard) {
            long ms = (long)(now_ms() - hard_rx);
            pthread_mutex_lock(&lock);
            stop_ms = ms;
            pthread_mutex_unlock(&lock);
        }
        if (gate_ok && dev_g[0] && motor->gesture) motor->gesture(dev_g);

        pthread_mutex_lock(&lock);
        for (int i = 0; i < 2; i++) { arm_pos[i] = ctl_pos[i]; arm_tgt[i] = ctl_tgt[i]; }
        gesture_name = ctl_g ? ctl_g->name : "none";
        pthread_mutex_unlock(&lock);

        sleep_until(&next);
    }
    if (mv_state == MV_RUNNING && motor->move_abort) motor->move_abort();
    motor->drive(0, 0);
    if (motor->arm_halt) motor->arm_halt();
    return NULL;
}

/* ------------------------------------------------------------------ network */
static void feed_watchdog(void) { clock_gettime(CLOCK_MONOTONIC, &last_cmd); }

/* Called with `lock` held. NULL = motion allowed, else the ERR reason. */
static int guard_pivot_ok(void)
{
    return !strcmp(guard_gate_reason(), "guard_blocked");
}

/* the limits on a TURN that runs only because of the fresh-BLOCKED exception */
static int pivot_fits(float deg, float speed)
{
    return fabsf(deg) <= PIVOT_BLOCKED_MAX_DEG + 1e-3f && speed <= PIVOT_BLOCKED_MAX_SPEED + 1e-6f;
}

/* a running/requested exact move that may continue under a fresh guard_blocked */
static int blocked_exempt_fits(char kind, float req, float speed)
{
    if (kind == 'R') return pivot_fits(req, speed);
    if (kind == 'M') return req < 0.0f && speed <= REVERSE_BLOCKED_MAX_SPEED + 1e-6f;
    return 0;
}

static const char *motion_refused(void)
{
    if (!link_up) return "uno_disconnected";
    if (tipped) return "tipped over: stand TARS up, then send RESET";
    if (!guard_permit) return guard_reason;
    return NULL;
}

static void handle_line(int cid, char *line, char *reply, size_t rlen)
{
    char cmd[16] = "";
    float a, b, c;
    char name[48];
    const char *why;

    sscanf(line, "%15s", cmd);

    int keyboard = !strcasecmp(cmd, "KD");
    if (keyboard || !strcasecmp(cmd, "D") || !strcasecmp(cmd, "DL")) {
        int leased = !strcasecmp(cmd, "DL");
        unsigned long long epoch = 0;
        int parsed;
        if (keyboard) {
            char token[33], key, extra;
            if (!cfg_local || !cfg_guard_port || !keyboard_auth ||
                sscanf(line, "%*s %32s %c %c", token, &key, &extra) != 2 ||
                strcmp(token, keyboard_auth) || !strchr("wasd", key)) {
                snprintf(reply, rlen, "ERR invalid keyboard request\n");
                return;
            }
            a = key == 'w' || key == 'd' ? .25f : -.25f;
            b = key == 'w' || key == 'a' ? .25f : -.25f;
            parsed = 2;
        } else parsed = leased ? sscanf(line, "%*s %llu %f %f", &epoch, &a, &b)
                            : sscanf(line, "%*s %f %f", &a, &b);
        if (parsed != (leased ? 3 : 2) || !isfinite(a) || !isfinite(b)) {
            snprintf(reply, rlen, "ERR usage: D <left> <right>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        feed_watchdog();
        long long left = bump_until_ms - now_ms();
        if (leased && epoch != guard_epoch) {
            target_l = target_r = 0.0f;
            snprintf(reply, rlen, "ERR guard epoch changed; new command required\n");
        } else if ((why = motion_refused()) != NULL && !(link_up && keyboard && keyboard_obstacle && !tipped)) {
            target_l = target_r = 0.0f;
            snprintf(reply, rlen, "ERR %s\n", why);
        } else if (left > 0) {
            target_l = target_r = 0.0f;
            snprintf(reply, rlen, "ERR bumped: driving paused %lld ms\n", left);
        } else if (hold_cid && hold_cid != cid && now_ms() < hold_until_ms
                   && (fabsf(a) >= 0.03f || fabsf(b) >= 0.03f)) {
            if (drive_owner == cid) target_l = target_r = 0.0f;
            snprintf(reply, rlen, "ERR held: stop from another client (%lld ms left)\n",
                     hold_until_ms - now_ms());
        } else {
            keyboard_until = keyboard ? now_ms() + 150 : 0;
            target_l = clampf(a, -1, 1);
            target_r = clampf(b, -1, 1);
            if (fabsf(target_l) >= 0.03f || fabsf(target_r) >= 0.03f) drive_owner = cid;
            /* a real drive command takes over from an exact move (the device
             * aborts it itself when it sees the non-zero D) */
            if (mv_state == MV_RUNNING && (fabsf(target_l) >= 0.03f || fabsf(target_r) >= 0.03f)) {
                mv_state = MV_ABORTED;
                snprintf(mv_why, sizeof mv_why, "drive");
            }
            snprintf(reply, rlen, "OK\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "S")) {
        pthread_mutex_lock(&lock);
        target_l = target_r = 0.0f;
        req_move = 0;
        req_stop_gesture = req_halt = 1;
        req_hard_stop = 1;
        stop_rx_ms = now_ms();
        hold_cid = cid;
        hold_until_ms = stop_rx_ms + cfg_hold_ms;
        drive_owner = 0;
        keyboard_until = 0;
        stop_count++;
        if (mv_state == MV_RUNNING) req_move_abort = "stop";
        req_gesture = NULL;
        memset(&arm_req, 0, sizeof arm_req);
        feed_watchdog();
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "G")) {
        if (sscanf(line, "%*s %47s", name) != 1) {
            snprintf(reply, rlen, "ERR usage: G <name>\n");
            return;
        }
        const gesture_t *g = gesture_find(name);
        pthread_mutex_lock(&lock);
        feed_watchdog();
        if ((why = motion_refused()) != NULL) {
            snprintf(reply, rlen, "ERR %s\n", why);
        } else if (motor->arm) {
            if (!g) {
                snprintf(reply, rlen, "ERR unknown gesture '%s' (try GESTURES)\n", name);
            } else {
                req_gesture = g;
                memset(&arm_req, 0, sizeof arm_req);
                snprintf(reply, rlen, "OK %d\n", gesture_duration_ms(g, arm_pos));
            }
        } else if (motor->gesture) {
            snprintf(req_dev_gesture, sizeof req_dev_gesture, "%s", name);
            snprintf(reply, rlen, "OK\n");
        } else {
            snprintf(reply, rlen, "ERR backend has no gestures\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "A")) {
        int arm;
        int n = sscanf(line, "%*s %d %f %f", &arm, &a, &c);
        if (n < 2 || (arm != 0 && arm != 1) || !isfinite(a) || (n == 3 && !(c > 0))) {
            snprintf(reply, rlen, "ERR usage: A <0|1> <deg> [dps>0]\n");
            return;
        }
        if (!motor->arm) { snprintf(reply, rlen, "ERR backend has no arms\n"); return; }
        float deg = clampf(a, -ARM_LIMIT_DEG, ARM_LIMIT_DEG);
        float dps = n == 3 ? clampf(c, 1.0f, ARM_MAX_DPS) : ARM_DEFAULT_DPS;
        pthread_mutex_lock(&lock);
        feed_watchdog();
        if ((why = motion_refused()) != NULL) {
            snprintf(reply, rlen, "ERR %s\n", why);
        } else {
            req_stop_gesture = 1;      /* a manual arm move overrides a gesture */
            req_gesture = NULL;
            arm_req[arm].set = 1;
            arm_req[arm].deg = deg;
            arm_req[arm].dps = dps;
            snprintf(reply, rlen, "OK %.1f\n", deg);
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "HOME")) {
        if (!motor->arm) { snprintf(reply, rlen, "ERR backend has no arms\n"); return; }
        pthread_mutex_lock(&lock);
        feed_watchdog();
        if ((why = motion_refused()) != NULL) {
            snprintf(reply, rlen, "ERR %s\n", why);
        } else {
            req_stop_gesture = 1;
            req_gesture = NULL;
            for (int i = 0; i < 2; i++) { arm_req[i].set = 1; arm_req[i].deg = 0; arm_req[i].dps = ARM_DEFAULT_DPS; }
            snprintf(reply, rlen, "OK\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "ZERO") || !strcasecmp(cmd, "LEVEL")) {
        pthread_mutex_lock(&lock);
        if (cmd[0] == 'Z' || cmd[0] == 'z') { req_zero = 1; req_gesture = NULL; }
        else req_level = 1;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "RESET")) {
        pthread_mutex_lock(&lock);
        long long ago = now_ms() - last_tip_ms;
        if (tipped && ago < TIP_CLEAR_MS) {
            snprintf(reply, rlen, "ERR still tipped (last report %lld ms ago)\n", ago);
        } else {
            if (tipped) req_reset = 1;
            tipped = 0;
            bump_until_ms = 0;
            target_l = target_r = 0.0f;
            snprintf(reply, rlen, "OK\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "MOVE") || !strcasecmp(cmd, "TURN")) {
        char kind = (cmd[0] == 'M' || cmd[0] == 'm') ? 'M' : 'R';
        int n = sscanf(line, "%*s %f %f", &a, &c);
        if (n < 1 || !isfinite(a) || (n == 2 && !(c > 0 && c <= 1))) {
            snprintf(reply, rlen, "ERR usage: %s <%s> [speed 0-1]\n", kind == 'M' ? "MOVE" : "TURN",
                     kind == 'M' ? "mm" : "deg");
            return;
        }
        if (!motor->move) { snprintf(reply, rlen, "ERR backend has no MOVE/TURN\n"); return; }
        float lim = kind == 'M' ? MOVE_MAX_MM : TURN_MAX_DEG;
        float amt = clampf(a, -lim, lim), spd = n == 2 ? c : 1.0f;
        pthread_mutex_lock(&lock);
        feed_watchdog();
        long long left = bump_until_ms - now_ms();
        /* pivot = this TURN runs only because of the fresh-BLOCKED exception: limit it.
         * reverse = the same exception for a MOVE with mm < 0 (back out the way we came). */
        int exempt = (why = motion_refused()) != NULL && link_up && !tipped && guard_pivot_ok();
        int pivot = exempt && kind == 'R';
        int reverse = exempt && kind == 'M' && amt < 0.0f;
        if (why && !pivot && !reverse) {
            snprintf(reply, rlen, "ERR %s\n", why);
        } else if (pivot && fabsf(amt) > PIVOT_BLOCKED_MAX_DEG + 1e-3f) {
            snprintf(reply, rlen, "ERR guard_blocked pivot>%.0f\n", PIVOT_BLOCKED_MAX_DEG);
        } else if (left > 0) {
            snprintf(reply, rlen, "ERR bumped: driving paused %lld ms\n", left);
        } else {
            if (pivot && spd > PIVOT_BLOCKED_MAX_SPEED) {
                fprintf(stderr, "[net] client %d: TURN %.1f under guard_blocked: speed %.2f clamped to %.2f\n",
                        cid, amt, spd, PIVOT_BLOCKED_MAX_SPEED);
                spd = PIVOT_BLOCKED_MAX_SPEED;
            }
            if (reverse && spd > REVERSE_BLOCKED_MAX_SPEED) {
                fprintf(stderr, "[net] client %d: MOVE %.0f under guard_blocked: speed %.2f clamped to %.2f\n",
                        cid, amt, spd, REVERSE_BLOCKED_MAX_SPEED);
                spd = REVERSE_BLOCKED_MAX_SPEED;
            }
            target_l = target_r = 0.0f;       /* the D stream becomes keepalives */
            mv_id++;
            mv_owner = cid;
            mv_state = MV_RUNNING;
            mv_kind = kind;
            mv_req = amt;
            mv_speed = spd;
            mv_done = 0;
            snprintf(mv_why, sizeof mv_why, "-");
            mv_est_ms = move_est_ms_locked(kind, amt, spd);
            req_move = 1;
            req_move_abort = NULL;            /* the device replaces a running move itself */
            snprintf(reply, rlen, "OK %d %d\n", mv_id, mv_est_ms);
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "WHEELCAL")) {
        int n = sscanf(line, "%*s %f %f", &a, &b);
        if (n == 2 && a >= 10 && a <= 300 && b >= 20 && b <= 500) {
            pthread_mutex_lock(&lock);
            wheel_d = a;
            wheel_t = b;
            req_wheelcal = 1;
            pthread_mutex_unlock(&lock);
        } else if (n > 0) {
            snprintf(reply, rlen, "ERR usage: WHEELCAL [diam_mm 10-300 track_mm 20-500]\n");
            return;
        }
        pthread_mutex_lock(&lock);
        snprintf(reply, rlen, "OK %.1f %.1f\n", wheel_d, wheel_t);
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "SPEEDCAL")) {
        float sl = 0;
        int nc = 0;
        int n = sscanf(line, "%*s %f %f %f %n", &a, &b, &sl, &nc);
        if (!motor->speed_cal) { snprintf(reply, rlen, "ERR backend has no DC speed calibration\n"); return; }
        int ok2 = n == 2 && sscanf(line, "%*s %*f %*f %15s", name) != 1;   /* nothing after b */
        int ok3 = n == 3 && !line[nc] && sl >= 20 && sl <= 2000;
        if ((ok2 || ok3) && a >= 20 && a <= 2000 && b >= 20 && b <= 2000) {
            pthread_mutex_lock(&lock);
            dc_req_mm_s = a;
            dc_req_deg_s = b;
            dc_req_deg_s_slow = ok3 ? sl : 0;
            req_speedcal = 1;
            pthread_mutex_unlock(&lock);
            /* the Uno answers WHEELS DC <a> <b> + DCSLOW; STATUS / SPEEDCAL show it once it has */
            if (ok3) snprintf(reply, rlen, "OK sent %.1f %.1f %.1f\n", a, b, sl);
            else snprintf(reply, rlen, "OK sent %.1f %.1f\n", a, b);
            return;
        } else if (n > 0 || sscanf(line, "%*s %15s", name) == 1) {   /* any junk argument */
            snprintf(reply, rlen, "ERR usage: SPEEDCAL [mm_s 20-2000 deg_s 20-2000 [deg_s_slow 20-2000]]\n");
            return;
        }
        pthread_mutex_lock(&lock);
        if (wheels_dc && dc_deg_s_slow > 0)
            snprintf(reply, rlen, "OK dc %.1f %.1f %.1f\n", dc_mm_s, dc_deg_s, dc_deg_s_slow);
        else if (wheels_dc) snprintf(reply, rlen, "OK dc %.1f %.1f\n", dc_mm_s, dc_deg_s);
        else snprintf(reply, rlen, "OK stepper\n");
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "LED") || !strcasecmp(cmd, "BEEP")) {
        int led = !strcasecmp(cmd, "LED"), v = -1, n2 = 0;
        char word[16] = "";
        /* iter3-pet/vision/GESTURES.md §7 names -> Uno L patterns (0 off, 1 on, 2 slow, 3 fast) */
        static const struct { const char *name; int pat; } led_names[] = {
            { "off", 0 }, { "teach", 2 }, { "rec", 1 }, { "ok", 1 }, { "no", 3 }, { "forget", 3 },
        };
        if (led && sscanf(line, "%*s %15s%n", word, &n2) == 1 && !(word[0] >= '0' && word[0] <= '9')) {
            for (size_t i = 0; i < sizeof led_names / sizeof *led_names; i++)
                if (!strcasecmp(word, led_names[i].name)) v = led_names[i].pat;
            if (line[n2 + strspn(line + n2, " \t")] != '\0') v = -1;
        } else if (sscanf(line, "%*s %d%n", &v, &n2) != 1 || line[n2 + strspn(line + n2, " \t")] != '\0') {
            v = -1;
        }
        if (v < 0 || v > (led ? 3 : 9)) {
            snprintf(reply, rlen, led ? "ERR usage: LED <off|teach|rec|ok|no|forget|0-3>\n"
                                      : "ERR usage: BEEP <0-9>\n");
            return;
        }
        if (led ? !motor->led : !motor->beep) {
            snprintf(reply, rlen, "ERR backend has no %s\n", led ? "LED" : "buzzer");
            return;
        }
        pthread_mutex_lock(&lock);
        if (!link_up) {
            pthread_mutex_unlock(&lock);
            snprintf(reply, rlen, "ERR uno_disconnected\n");
            return;
        }
        if (led) { led_pattern = v; req_led = v; } else req_beep = v;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "MODE")) {
        char m[16] = "";
        if (sscanf(line, "%*s %15s", m) != 1
            || (strcasecmp(m, "gesture") && strcasecmp(m, "follow") && strcasecmp(m, "toggle") && strcasecmp(m, "teach"))) {
            snprintf(reply, rlen, "ERR usage: MODE <gesture|follow|toggle|teach>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        for (char *p = m; *p; p++) *p = (char)(*p | 0x20);
        snprintf(mode_req, sizeof mode_req, "%s", m);
        int seq = ++mode_seq;
        pthread_mutex_unlock(&lock);
        fprintf(stderr, "[net] client %d: MODE %s (seq %d)\n", cid, m, seq);
        snprintf(reply, rlen, "OK MODE %s %d\n", m, seq);
    } else if (!strcasecmp(cmd, "WHO")) {
        char who[24] = "";
        if (sscanf(line, "%*s %23s", who) != 1) { snprintf(reply, rlen, "ERR usage: WHO <name>\n"); return; }
        fprintf(stderr, "[net] client %d is '%s'\n", cid, who);
        snprintf(reply, rlen, "OK %d\n", cid);
    } else if (!strcasecmp(cmd, "GESTURES")) {
        size_t off = (size_t)snprintf(reply, rlen, "OK");
        for (int i = 0; gesture_at(i) && off < rlen; i++)
            off += (size_t)snprintf(reply + off, rlen - off, " %s", gesture_at(i)->name);
        if (off < rlen - 1) snprintf(reply + off, rlen - off, "\n");
    } else if (!strcasecmp(cmd, "PING")) {
        pthread_mutex_lock(&lock);
        feed_watchdog();
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK PONG\n");
    } else if (!strcasecmp(cmd, "STATUS")) {
        pthread_mutex_lock(&lock);
        long long bl = bump_until_ms - now_ms();
        snprintf(reply, rlen,
                 "OK %.2f %.2f %ld %.2f %d why=%s arm0=%.1f arm1=%.1f arm0_tgt=%.1f arm1_tgt=%.1f "
                 "gesture=%s tipped=%d tilt=%d bump_ms=%lld bumps=%d ready=%d backend=%s uno_link=%s "
                 "stop_ms=%ld vis=%s vis_fps=%.1f vis_cam_ms=%d vis_stop_ms=%d vis_see=%s "
                 "vis_age_ms=%lld move=%s move_id=%d move_kind=%s move_req=%.1f move_done=%.1f "
                 "move_why=%s move_est_ms=%d wheel_d=%.1f wheel_t=%.1f btn_teach=%d btn_forget=%d "
                 "led=%d guard_enabled=%d guard_permit=%d guard_age_ms=%ld guard_reason=%s guard_epoch=%llu clients=%d hold_ms=%lld mode_req=%s mode_seq=%d stops=%d "
                 "pose=%.0f,%.0f,%.1f pose_epoch=%u wheels=%s dc_mm_s=%.1f dc_deg_s=%.1f dc_deg_s_slow=%.1f\n",
                 out_l, out_r, ms_since(&last_cmd), last_dist, estop, estop_why,
                 arm_pos[0], arm_pos[1], arm_tgt[0], arm_tgt[1], gesture_name,
                 tipped, tilt_deg, bl > 0 ? bl : 0, bumps, ready_count, motor->name,
                 !motor->link ? "na" : link_up ? "up" : "down",
                 stop_ms, vis_mode, vis_fps, vis_cam_ms, vis_stop_ms, vis_see,
                 vis_at_ms ? now_ms() - vis_at_ms : -1LL,
                 mv_names[mv_state], mv_id, mv_kind == 'M' ? "MOVE" : mv_kind == 'R' ? "TURN" : "-",
                 mv_req, mv_done, mv_why, mv_est_ms, wheel_d, wheel_t, btn_teach, btn_forget,
                 led_pattern, cfg_guard_port != 0, guard_permit, guard_age, guard_reason, guard_epoch, n_clients,
                 hold_cid && now_ms() < hold_until_ms ? hold_until_ms - now_ms() : 0LL,
                 mode_req, mode_seq, stop_count, pose_x, pose_y, pose_h, pose_epoch,
                 wheels_dc ? "dc" : "stepper", dc_mm_s, dc_deg_s, dc_deg_s_slow);
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "POSE")) {
        char sub[16] = "";
        sscanf(line, "%*s %15s", sub);
        if (sub[0] && strcasecmp(sub, "RESET")) { snprintf(reply, rlen, "ERR usage: POSE [RESET]\n"); return; }
        pthread_mutex_lock(&lock);
        if (sub[0]) {
            pose_x = pose_y = pose_h = 0;
            unsigned e;
            do e = new_epoch(); while (e == pose_epoch);
            pose_epoch = e;
            fprintf(stderr, "[net] client %d: POSE RESET (epoch %u)\n", cid, e);
        }
        snprintf(reply, rlen, "OK %.0f %.0f %.1f %u %d\n", pose_x, pose_y, pose_h, pose_epoch,
                 mv_state == MV_RUNNING);
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "VIS")) {
        char m[16], see[24];
        float fps;
        int cam, stp;
        if (sscanf(line, "%*s %15s %f %d %d %23s", m, &fps, &cam, &stp, see) != 5) {
            snprintf(reply, rlen, "ERR usage: VIS <mode> <fps> <cam_ms> <stop_ms> <label>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        memcpy(vis_mode, m, sizeof vis_mode);
        memcpy(vis_see, see, sizeof vis_see);
        vis_fps = fps;
        vis_cam_ms = cam;
        vis_stop_ms = stp;
        vis_at_ms = now_ms();
        if (mode_seq) snprintf(reply, rlen, "OK MODE %s %d\n", mode_req, mode_seq);
        else snprintf(reply, rlen, "OK\n");
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "SIM")) {
        char kind[16] = "";
        int v = 0;
        if (motor != &motor_stub) { snprintf(reply, rlen, "ERR SIM is only for -b stub\n"); return; }
        if (sscanf(line, "%*s %15s %d", kind, &v) < 1) { snprintf(reply, rlen, "ERR usage: SIM T|B|READY [n]\n"); return; }
        motor_event_t ev;
        memset(&ev, 0, sizeof ev);
        ev.value = v;
        if (!strcasecmp(kind, "T")) ev.kind = MOTOR_EV_TIP;
        else if (!strcasecmp(kind, "B")) ev.kind = MOTOR_EV_BUMP;
        else if (!strcasecmp(kind, "READY")) ev.kind = MOTOR_EV_READY;
        else if (!strcasecmp(kind, "BTN")) {
            char b2[16] = "";
            sscanf(line, "%*s %*s %15s", b2);
            ev.kind = MOTOR_EV_BTN;
            ev.value = !strcasecmp(b2, "TEACH") ? MOTOR_BTN_TEACH : !strcasecmp(b2, "FORGET") ? MOTOR_BTN_FORGET : 0;
            if (!ev.value) { snprintf(reply, rlen, "ERR usage: SIM BTN TEACH|FORGET\n"); return; }
        }
        else { snprintf(reply, rlen, "ERR usage: SIM T|B|READY [n] | SIM BTN TEACH|FORGET\n"); return; }
        pthread_mutex_lock(&lock);
        if (simq_n < SIMQ) simq[simq_n++] = ev;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (cmd[0] == '\0') {
        reply[0] = '\0';
    } else {
        snprintf(reply, rlen, "ERR unknown command '%s'\n", cmd);
    }
}

typedef struct { int fd, id; } client_arg_t;

static void *client_thread(void *arg)
{
    client_arg_t ca = *(client_arg_t *)arg;
    free(arg);
    int fd = ca.fd, cid = ca.id;
    char buf[512], reply[1024];
    size_t used = 0;

    pthread_mutex_lock(&lock);
    int nc = ++n_clients;
    pthread_mutex_unlock(&lock);
    fprintf(stderr, "[net] client connected (id %d, %d connected)\n", cid, nc);
    for (;;) {
        ssize_t n = recv(fd, buf + used, sizeof buf - 1 - used, 0);
        if (n <= 0) break;
        used += (size_t)n;
        buf[used] = '\0';

        char *start = buf, *nl;
        while ((nl = strchr(start, '\n')) != NULL) {
            *nl = '\0';
            if (nl > start && nl[-1] == '\r') nl[-1] = '\0';
            handle_line(cid, start, reply, sizeof reply);
            if (reply[0]) send(fd, reply, strlen(reply), 0);
            char verb[24] = "";
            sscanf(start, "%23s", verb);
            if (*verb && strcasecmp(verb,"STATUS") && strcasecmp(verb,"PING") &&
                strcasecmp(verb,"VIS") && strcasecmp(verb,"GESTURES") && strcasecmp(verb,"WHO") &&
                (strcasecmp(verb,"POSE") || strstr(start," ") != NULL))
                action_log("motor", cid, !strcasecmp(verb, "KD") ? "shift+WASD" : start, reply);
            start = nl + 1;
        }
        used = strlen(start);
        memmove(buf, start, used + 1);
        if (used >= sizeof buf - 1) used = 0;   /* line too long: drop it */
    }
    /* Safety: a dropped client that was driving means stop the wheels now,
     * don't wait for the watchdog. Another client's drive or move is left
     * alone (it has its own watchdog feed). Arms finish their current move. */
    pthread_mutex_lock(&lock);
    int mine = drive_owner == cid || drive_owner == 0;
    if (mine) { target_l = target_r = 0.0f; drive_owner = 0; keyboard_until = 0; }
    int abort_mv = mv_state == MV_RUNNING && mv_owner == cid;
    if (abort_mv) { req_move = 0; req_move_abort = "disconnect"; }
    if (mine || abort_mv) { req_hard_stop = 1; stop_rx_ms = now_ms(); }
    if (hold_cid == cid) hold_cid = 0;
    nc = --n_clients;
    pthread_mutex_unlock(&lock);
    if (mine || abort_mv) action_log("motor", cid, "stop", "client disconnected");
    if (mine || abort_mv)
        fprintf(stderr, "[net] client %d disconnected -> stop (%d left)\n", cid, nc);
    else
        fprintf(stderr, "[net] client %d disconnected (was not driving; %d left)\n", cid, nc);
    close(fd);
    return NULL;
}

/* ------------------------------------------------------------------ main */
static void on_signal(int sig) { (void)sig; running = 0; }

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [-b uno|stub|pico] [-d serial_dev] [-p port] [-w watchdog_ms]\n"
        "          [-r rt_prio] [-k bump_hold_ms] [-H stop_hold_ms] [-g guard_udp_port] [-l localhost_only]\n", argv0);
}

int main(int argc, char **argv)
{
    keyboard_auth = getenv("TARS_KEYBOARD_AUTH");
    if (keyboard_auth && strlen(keyboard_auth) != 32) keyboard_auth = NULL;
    int opt;
    while ((opt = getopt(argc, argv, "b:d:p:w:r:k:H:g:lh")) != -1) {
        switch (opt) {
        case 'b': cfg_backend = optarg; break;
        case 'd': cfg_dev = optarg; break;
        case 'p': cfg_port = atoi(optarg); break;
        case 'w': cfg_wd_ms = atoi(optarg); break;
        case 'r': cfg_rt_prio = atoi(optarg); break;
        case 'k': cfg_bump_ms = atoi(optarg); break;
        case 'g': cfg_guard_port = atoi(optarg); break;
        case 'l': cfg_local = 1; break;
        case 'H': cfg_hold_ms = atoi(optarg); break;
        default:  usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }

    if (cfg_port < 1 || cfg_port > 65535 || cfg_guard_port < 0 || cfg_guard_port > 65535 || cfg_wd_ms < 50 || cfg_wd_ms > 5000) {
        fprintf(stderr, "bad -p/-g port or -w watchdog (50..5000 ms)\n");
        return 2;
    }
    if (guard_gate_init(cfg_guard_port) != 0) { perror("guard UDP bind"); return 1; }
    guard_permit = cfg_guard_port == 0;
    for (size_t i = 0; i < sizeof motor_backends / sizeof *motor_backends; i++)
        if (!strcmp(motor_backends[i]->name, cfg_backend)) motor = motor_backends[i];
    if (!motor) { fprintf(stderr, "unknown backend '%s'\n", cfg_backend); return 2; }
    if (!cfg_dev) cfg_dev = motor->default_dev;
    link_up = !motor->link;
    if (motor->init(cfg_dev) != 0) { fprintf(stderr, "backend init failed\n"); return 1; }
    obstacle_init();

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;          /* no SA_RESTART: lets accept() return EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGHUP, SIG_IGN);            /* started from ssh/boot script: keep running */

    clock_gettime(CLOCK_MONOTONIC, &last_cmd);
    last_cmd.tv_sec -= 10;              /* start in watchdog-stop state */
    pose_epoch = new_epoch();

    /* Real-time control thread. */
    pthread_attr_t attr;
    struct sched_param sp = { .sched_priority = cfg_rt_prio };
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setschedparam(&attr, &sp);
    pthread_t ctl;
    if (pthread_create(&ctl, &attr, control_thread, NULL) != 0) {
        fprintf(stderr, "[ctl] can't get SCHED_RR prio %d (need root?), using default\n",
                cfg_rt_prio);
        if (pthread_create(&ctl, NULL, control_thread, NULL) != 0) {
            perror("pthread_create");
            return 1;
        }
    }
    pthread_attr_destroy(&attr);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(cfg_local ? INADDR_LOOPBACK : INADDR_ANY);
    addr.sin_port = htons((unsigned short)cfg_port);
    if (srv < 0 || bind(srv, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(srv, 4) < 0) {
        perror("socket/bind/listen");
        running = 0;
    } else {
        fprintf(stderr, "tars_motor: backend=%s dev=%s port=%d watchdog=%dms prio=%d\n",
                motor->name, cfg_dev, cfg_port, cfg_wd_ms, cfg_rt_prio);
    }

    /* QNX may deliver SIGTERM to a worker, leaving accept asleep in main. */
    struct pollfd listener = { .fd = srv, .events = POLLIN };
    while (running) {
        int readable = poll(&listener, 1, 250);
        if (readable < 0 && errno != EINTR) { perror("poll listener"); break; }
        if (readable <= 0 || !running) continue;
        int c = accept(srv, NULL, NULL);
        if (c < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        static int next_id;
        client_arg_t *ca = malloc(sizeof *ca);
        pthread_t t;
        if (ca) { ca->fd = c; ca->id = ++next_id; }
        if (ca && pthread_create(&t, NULL, client_thread, ca) == 0)
            pthread_detach(t);
        else {
            free(ca);
            close(c);
        }
    }

    running = 0;
    pthread_join(ctl, NULL);
    motor->shutdown();
    guard_gate_close();
    if (srv >= 0) close(srv);
    fprintf(stderr, "tars_motor: bye\n");
    return 0;
}

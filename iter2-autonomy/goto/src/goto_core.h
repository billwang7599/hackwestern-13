/* goto core: config, object names/aliases, detection-line parser and the
 * go-to state machine. Pure C11, no I/O except loading files, so it builds
 * the same with qcc (QNX) and clang (Mac tests). goto.c does the sockets. */
#ifndef GOTO_CORE_H
#define GOTO_CORE_H
#include <stddef.h>
#include "nav.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- config */
typedef struct {
    /* camera */
    double hfov_deg;          /* horizontal field of view of the webcam */
    double min_conf;          /* ignore detections below this */
    /* search */
    double search_step_deg;   /* TURN per search step */
    double search_max_deg;    /* give up after turning this far without a sighting */
    double search_speed;      /* TURN speed fraction (0,1] */
    double settle_s;          /* after a TURN finishes, ignore frames this long */
    int    look_frames;       /* fresh frames to look at after each step */
    int    confirm_frames;    /* ... of which this many must contain the target */
    double frame_timeout_s;   /* no frames at all for this long -> fail "no camera" */
    /* align (exact TURN toward the target) */
    double align_deg;         /* |bearing| above this -> exact TURN instead of steering */
    double align_gain;        /* TURN this fraction of the measured bearing */
    double realign_deg;       /* while driving: |bearing| above this -> stop + align */
    /* approach (closed loop D l r) */
    double fwd;               /* forward drive fraction when centred (steppers: 1.0 = ~4 cm/s) */
    double kp_per_deg;        /* steering per degree of bearing */
    double deadband_deg;      /* no steering inside this */
    double turn_max;          /* clamp on the steering term */
    double slow_deg;          /* forward speed falls to 0 at this bearing */
    double ema_alpha;         /* smoothing of box centre/height, 1 = none */
    double tick_hz;           /* D refresh rate (>= 5 for the 500 ms watchdog) */
    /* arrive / lose */
    double arrive_h_frac;     /* box height >= this * frame height -> arrived */
    double arrive_bottom_frac;/* box bottom edge >= this * frame height -> arrived */
    double arrive_bottom_min_h;/* ... only if the box is at least this tall (frac) */
    int    arrive_frames;     /* consecutive frames meeting the arrive test */
    double lost_s;            /* no target for this long while driving -> stop + search */
    double timeout_s;         /* whole GOTO gives up after this */
    double move_timeout_pad_s;/* exact move overdue by est*2 + this -> fail */
    /* object memory (goto_mem below) */
    double arrive_dist_mm;    /* camera -> object distance when arrive_h is reached: scales
                               * the rough box-height distance estimate */
    double memory_min_conf;   /* detections below this aren't remembered */
    double memory_max_age_s;  /* GOTO trusts a remembered spot this long */
    double memory_drive_mm;   /* remembered target not in view after turning to it: MOVE up to
                               * this far toward it, then look again (0 = off, just search) */
    double memory_save_s;     /* write the memory file at most this often */
    double memory_merge_mm;   /* a sighting this close to an instance is that instance */
    double memory_tries;      /* GOTO tries at most this many remembered instances, then searches */
    double cam_height_mm;     /* camera lens above the floor (floor-contact distance estimate) */
    double cam_pitch_deg;     /* camera tilted down by this much (0 = level) */
    /* map-guided GOTO (nav.h grid from the phone, feed on UDP/TCP 5572) */
    double nav_pose_fresh_s;  /* no LDR1 pose this long -> ARKit tracking treated as lost (DR takes over) */
    double nav_turn_min_deg;  /* path leg: turn first only if the heading is off by more than this */
    double nav_leg_min_mm;    /* path legs shorter than this are skipped */
    double nav_max_replans;   /* re-plans (guard denial, map change) per GOTO, then FAILED blocked */
    double nav_guard_wait_s;  /* after a guard-aborted leg: wait this long (and for CLEAR) before re-planning */
    double nav_reveal_deg;    /* robot's own footprint not mapped yet: turn this much to reveal floor, re-plan */
    /* per-object arrive override, e.g. person is tall: arrive_h.person = 0.85 */
    int    n_arrive_over;
    char   arrive_over_label[16][32];
    double arrive_over_h[16];
    /* not compared with goto.conf by the tests (strings) */
    char   memory_path[256];  /* "-" = keep it in RAM only. Relative = next to goto.conf */
} goto_cfg;

void goto_cfg_defaults(goto_cfg *c);
/* "key = value" lines, # comments. Returns 0, or -1 if the file can't be
 * opened. Unknown keys are reported in err (first one) and skipped. */
int  goto_cfg_load(goto_cfg *c, const char *path, char *err, size_t errlen);
double goto_cfg_arrive_h(const goto_cfg *c, const char *label);

/* ------------------------------------------------------------ objects */
#define GOTO_MAX_OBJ   32
#define GOTO_MAX_ALIAS 256
typedef struct {
    int  n_obj;
    char obj[GOTO_MAX_OBJ][32];              /* canonical names, file order */
    int  n_alias;
    char alias[GOTO_MAX_ALIAS][40];          /* normalized alias text */
    int  alias_obj[GOTO_MAX_ALIAS];          /* -> index into obj */
} goto_objects;

void goto_objects_defaults(goto_objects *o);  /* built-in copy of iter2-autonomy/goto/objects.txt */
int  goto_objects_load(goto_objects *o, const char *path);   /* 0 ok, -1 can't open */
/* phrase ("the big trash can", "Bin.", "dining table") -> canonical index or -1 */
int  goto_objects_resolve(const goto_objects *o, const char *phrase);
/* lowercase, punctuation -> space, collapse spaces, drop articles */
void goto_normalize(const char *in, char *out, size_t outlen);

/* ------------------------------------------------- detection JSON lines */
#define GOTO_MAX_DETS 64
typedef struct {
    char   label[40];
    double conf;
    double box[4];                            /* x1 y1 x2 y2, pixels */
} goto_det;
typedef struct {
    double t;                                 /* sender's timestamp (informational) */
    int    w, h;
    int    n;
    goto_det d[GOTO_MAX_DETS];
} goto_frame;
/* Parse one line {"t":..,"w":640,"h":480,"dets":[{"label":..,"conf":..,"box":[..]}]}.
 * Returns 0 on success, -1 if it isn't a valid frame. Extra keys are ignored. */
int goto_parse_frame(const char *line, goto_frame *f);

/* ------------------------------------------------------- object memory
 * Every detection frame (all labels, not just the target) updates the memory:
 * remembered INSTANCES, keyed label + id (trash can #1, trash can #2). A
 * sighting merges into an instance of the same label whose world x,y is within
 * memory_merge_mm, or that lies on the same ray from the robot within the
 * (noisy) range; otherwise it becomes a new instance.
 * World frame: built on tars_motor's dead-reckoned pose (x forward, y right of
 * where tars_motor started, mm; headings/bearings deg, + = clockwise like
 * TURN). tars_motor's pose restarts at 0 when it restarts (new pose_epoch); the
 * memory then re-bases on the last pose it saw or saved, i.e. it assumes the
 * robot wasn't moved meanwhile. If you carried it: FORGET (and POSE RESET).
 *
 * Status per instance:  FRESH --GOTO looked there, not seen--> STALE --again--> GONE
 *                       any sighting -> FRESH.   GOTO prefers FRESH, then STALE; never
 *                       GONE or older than memory_max_age_s.
 * (A lidar layer can later set GONE / feed distances through the same fields.) */
#define GOTO_MEM_MAX 32
typedef enum { MEM_FRESH = 0, MEM_STALE, MEM_GONE } goto_mem_status;
typedef struct {
    char   label[32];        /* canonical object name if objects.txt knows it, else the label */
    int    id;               /* instance number within the label, from 1 */
    goto_mem_status status;
    double x, y;             /* world mm (smoothed over recent sightings) */
    double bearing;          /* world bearing of the last sighting, from where the robot was */
    double dist;             /* estimated distance of the last sighting, mm */
    double conf;             /* confidence of the last sighting */
    int    seen;             /* frames it was seen in */
    double last_seen;        /* wall clock, s (survives restarts; may jump on a Pi with no RTC) */
    int    has_w;            /* map position known (seen while the nav pose was good) */
    double wx, wz;           /* ... in the nav/ARKit frame, metres */
} goto_mem_entry;

typedef struct {
    int n;
    goto_mem_entry e[GOTO_MEM_MAX];
    int have_pose;           /* robot pose known */
    double px, py, ph;       /* robot pose in the world frame */
    int have_offset;
    unsigned epoch;          /* tars_motor pose_epoch the offset belongs to */
    double ox, oy, oh;       /* world = rot(oh) * motor + (ox, oy) */
    double spx, spy, sph;    /* pose when last saved (dirty if it moves away from it) */
    int dirty;
    int rebased;             /* times it re-based on a new epoch (STATUS / tests) */
    int have_nav;            /* robot pose in the nav (ARKit) frame, for has_w/wx/wz of new sightings */
    double nx, nz, nyaw;     /* metres, rad; forward = (sin yaw, cos yaw); + yaw = left turn */
} goto_mem;

void goto_mem_init(goto_mem *m);
const char *goto_mem_status_name(goto_mem_status st);
/* tars_motor STATUS pose= / pose_epoch=: re-bases on a new epoch, then updates px/py/ph */
void goto_mem_motor_pose(goto_mem *m, unsigned epoch, double mx, double my, double mh);
/* rough distance (mm) to a box: floor contact (level camera at cam_height_mm), else box height */
double goto_mem_box_dist(const goto_cfg *c, const char *label, const goto_det *d, int fw, int fh);
/* remember every usable detection in f (needs a pose). Returns how many boxes it used. */
int  goto_mem_observe(goto_mem *m, const goto_cfg *c, const goto_objects *o, const goto_frame *f, double wall);
/* merge a sighting at world (x, y) of `label` (or add an instance); returns its index */
int  goto_mem_sight(goto_mem *m, const goto_cfg *c, const char *label, double x, double y,
                    double bearing, double dist, double conf, double wall);
int  goto_mem_find(const goto_mem *m, const char *label);          /* first instance or -1 */
int  goto_mem_find_id(const goto_mem *m, const char *label, int id);
int  goto_mem_count(const goto_mem *m, const char *label);
/* best instance to drive to: FRESH before STALE, then nearest; skips GONE, too old, and
 * the ids in skip[0..nskip). -1 = none. */
int  goto_mem_best(const goto_mem *m, const char *label, double wall, double max_age,
                   const int *skip, int nskip);
/* looked at instance i and it wasn't there: FRESH -> STALE -> GONE */
void goto_mem_miss(goto_mem *m, int i);
/* "trash can" = all its instances, "trash can#2" = one, NULL = everything; returns how many */
int  goto_mem_forget(goto_mem *m, const char *label);
/* relative bearing (deg, + = right) and distance (mm) from the robot to entry i */
void goto_mem_relative(const goto_mem *m, int i, double *bearing, double *dist);
/* atomic: path.tmp, fsync, rename. 0 ok, -1 error. */
int  goto_mem_save(goto_mem *m, const char *path, double wall);
/* 0 ok, 1 ok but some lines were bad (skipped), -1 missing/unreadable (memory left empty) */
int  goto_mem_load(goto_mem *m, const char *path);
/* one line: "n=2 pose=x,y,h; trash can#1 fresh x=.. y=.. ...; chair#1 ..." */
void goto_mem_describe(const goto_mem *m, double wall, char *out, size_t len);
/* One short spoken-style sentence, one line, for a robot with no speaker (voice prints it,
 * the phone may speak it): "I remember 2 things: a trash can about 1.2 m to my left, and a
 * chair behind me." Gone instances are left out. Returns how many objects it mentions. */
int  goto_mem_say(const goto_mem *m, double wall, double max_age, char *out, size_t len);
/* canonical name for a detector label ("dining table" -> "table"), else the normalized label */
void goto_mem_label(const goto_objects *o, const char *raw, char *out, size_t len);

/* ------------------------------------------------------ state machine */
typedef enum {
    ST_IDLE = 0,
    ST_LOOK,          /* standing still, looking at fresh frames */
    ST_SEARCH_TURN,   /* exact TURN of one search step running */
    ST_ALIGN_TURN,    /* exact TURN toward the target running */
    ST_APPROACH,      /* closed-loop D l r toward the target */
    ST_MEM_TURN,      /* exact TURN toward where memory says the target is */
    ST_MEM_MOVE,      /* exact MOVE toward the remembered spot (memory_drive_mm) */
    /* map-guided GOTO (docs/ARCHITECTURE.md §3). RECALL/OCCLUDED/CHECK_OTHERS/DETOUR/
     * NEXT_CANDIDATE are decision states: entered and left in the same call (EVENTs show them). */
    ST_RECALL,        /* picked a remembered instance, its map position is the goal */
    ST_PLAN,          /* waiting to (re-)plan: guard CLEAR + nav_guard_wait_s after a blocked leg */
    ST_FOLLOW_PATH,   /* exact leg running: TURN to the next waypoint, MOVE to it, or the final face TURN */
    ST_OCCLUDED,      /* not seen and the map has an obstacle between us and the spot */
    ST_CHECK_OTHERS,  /* not there (line of sight clear): instance fresh->stale->gone, try the next one */
    ST_DETOUR,        /* occluded: plan to a vantage cell with a clear line of sight, look again */
    ST_NEXT_CANDIDATE,/* no route to this instance: try the next one, else the blind search */
} goto_state;

typedef enum { MODE_GOTO = 0, MODE_SCAN } goto_mode;

typedef enum {
    ACT_NONE = 0,
    ACT_DRIVE,        /* D a b */
    ACT_STOP,         /* S */
    ACT_TURN,         /* TURN a [b = speed] -> reply gives move id; call goto_sm_move_started */
    ACT_MOVE,         /* MOVE a [b = speed], same reply handling as TURN */
    ACT_BEEP,         /* BEEP a */
    ACT_LED,          /* LED text */
    ACT_EVENT,        /* text for control-port watchers: "arrived trash can" */
} goto_act_kind;
/* ACT_DRIVE with text = guard epoch: send "DL <epoch> a b" (leased drive, refused once the
 * guard has denied since that epoch); empty text: plain "D a b" (old server, no guard_epoch=). */

typedef struct {
    goto_act_kind kind;
    double a, b;
    char text[200];
} goto_act;

#define GOTO_MAX_ACTS 16
typedef struct {
    goto_cfg cfg;
    const goto_objects *objs;
    goto_mem *mem;                    /* NULL = no memory */
    double wall_base;                 /* wall clock = wall_base + now */
    int pose_moving;                  /* tars_motor says the robot is turning/moving right now */
    goto_mode mode;
    goto_state st;
    int target;                       /* canonical index, -1 = none */
    char result[64];                  /* last outcome: arrived / not found / ... */
    double now;
    double started, state_since;
    /* look window */
    double fresh_after;               /* frames received after this count as fresh */
    int look_seen, look_hits;
    double look_bearing;              /* averaged bearing of the hits */
    double last_frame;                /* time of the last frame of any kind */
    /* search */
    double searched_deg;
    int search_dir;                   /* +1 right, -1 left */
    /* move tracking */
    int move_id;                      /* -1 = waiting for the TURN reply */
    double move_deadline;
    /* target track */
    int have_track;
    double ema_x, ema_h;              /* x: -1..1 left..right, h: box height / frame height */
    double last_seen;
    double last_bearing;
    int arrive_count;
    double last_tick;
    double cmd_l, cmd_r;              /* last D sent */
    /* stats for STATUS */
    double fps_ema, last_frame_for_fps;
    int frames, frames_with_target;
    /* memory-guided GOTO: 0 none, 1 turned toward the remembered spot, 2 drove toward it,
     * 3 not there -> normal search */
    int mem_phase;
    int mem_id;                       /* instance id being tried */
    int mem_tried[8], n_tried;        /* instance ids already tried in this GOTO */
    double mem_bearing, mem_dist;     /* relative, when the instance was picked */
    /* map-guided GOTO */
    nav_t *nav;                       /* NULL = no map: pure visual GOTO (unchanged fallback) */
    int map_mode;                     /* this instance is being reached over the map */
    int recall;                       /* RECALL <obj>: any remembered instance, no blind search */
    double goal_x, goal_z;            /* remembered object, nav frame (m) */
    int vantage;                      /* planning to (vx, vz) instead of the goal's approach ring */
    double vx, vz;
    nav_pt_t path[64];
    int path_n, path_i, leg;          /* leg: 0 turn, 1 move, 2 face the goal, 3 reveal turn */
    double leg_mm;
    unsigned path_ver;
    int replans, detoured_id;
    double revealed_deg;
    int plan_wait;                    /* ST_PLAN: re-plan once the guard is CLEAR */
    int exploring, explores;          /* heading to a mapped cell nearer an unmapped goal, then re-plan */
    int leg_settle;                   /* leg done, ARKit pose: wait for 2 poses taken after it ended */
    unsigned settle_seq;
    double settle_since;
    /* lidar_guard via tars_motor STATUS */
    int guard_known, guard_ok;
    unsigned long long guard_epoch, lease_epoch;
    /* SCAN */
    int scan_n;
    char scan_seen[GOTO_MEM_MAX][32];
    /* output queue */
    int n_act;
    goto_act act[GOTO_MAX_ACTS];
} goto_sm;

const char *goto_state_name(goto_state s);
void goto_sm_init(goto_sm *s, const goto_cfg *cfg, const goto_objects *objs);
/* All take the current monotonic time in seconds. After each call, the caller
 * sends s->act[0..n_act) to tars_motor in order and then clears n_act. */
void goto_sm_goto(goto_sm *s, double now, int target);
/* "go back to where the X was": any remembered instance (stale/gone/old too), routed over the
 * map when there is one; then YOLO confirms. Nothing remembered -> FAILED "not remembered". */
void goto_sm_recall(goto_sm *s, double now, int target);
/* map: attach (may be NULL). Call nav_note_dr()/nav_set_pose() on it from the I/O side. */
void goto_sm_set_nav(goto_sm *s, nav_t *nav);
/* STATUS guard_permit= / guard_epoch= (tars_motor with -g). An epoch change ends a leased
 * APPROACH leg: an old drive never resumes after the guard clears. */
void goto_sm_guard(goto_sm *s, double now, int permit, unsigned long long epoch);
/* tars_motor refused a D/DL (ERR guard epoch changed / guard_blocked / ...) */
void goto_sm_drive_refused(goto_sm *s, double now, const char *why);
/* tars_motor refused a TURN/MOVE because of the guard (ERR guard_*). Map legs wait + re-plan;
 * the final "face the spot" turn goes straight to LOOK (what blocks is usually the target).
 * Returns 1 if handled, 0 if the caller should fail the GOTO as before. */
int  goto_sm_move_refused(goto_sm *s, double now, const char *why);
/* memory: attach (may be NULL). wall clock = wall_base + now. */
void goto_sm_set_memory(goto_sm *s, goto_mem *m, double wall_base);
/* pose from tars_motor STATUS. moving = an exact move runs / spinning (skip remembering) */
void goto_sm_pose(goto_sm *s, double now, unsigned epoch, double mx, double my, double mh, int moving);
/* stand still, 12 x TURN 30, remember everything seen; EVENT "scanned a,b,c" */
void goto_sm_scan(goto_sm *s, double now);
void goto_sm_cancel(goto_sm *s, double now, const char *why);      /* sends S */
void goto_sm_fail(goto_sm *s, double now, const char *why);        /* S + BEEP 3 + LED no */
void goto_sm_frame(goto_sm *s, double now, const goto_frame *f);
void goto_sm_tick(goto_sm *s, double now);
void goto_sm_move_started(goto_sm *s, double now, int move_id, double est_s);
/* from STATUS: move_id, and move= running|done|aborted|idle, move_why= */
void goto_sm_move_status(goto_sm *s, double now, int move_id, const char *move, const char *why);
int  goto_sm_busy(const goto_sm *s);
int  goto_sm_wants_status(const goto_sm *s);   /* an exact move is being waited on */
/* "state=approach target=trash can ..." (no newline) */
void goto_sm_describe(const goto_sm *s, char *out, size_t len);

#ifdef __cplusplus
}
#endif
#endif

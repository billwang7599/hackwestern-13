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
    double nav_heading_tol_deg; /* after a path TURN: still off by more (fresh pose) -> corrective TURN first */
    double nav_heading_retries; /* ... at most this many corrective TURNs, then MOVE anyway (0 = off) */
    double nav_leg_min_mm;    /* path legs shorter than this are skipped */
    double nav_max_replans;   /* re-plans (guard denial, map change) per GOTO, then FAILED blocked */
    double nav_guard_wait_s;  /* after a guard-aborted leg: wait this long (and for CLEAR) before re-planning */
    double nav_reveal_deg;    /* robot's own footprint not mapped yet: turn this much to reveal floor, re-plan */
    /* lidar_guard's stop zone, as goto predicts it from its map (mirror of lidar_guard.cpp, NOT
     * the guard's thresholds: changing these never changes what the guard stops) */
    double nav_guard_stop_m;  /* BLOCKED if an obstacle is this close ahead ... */
    double nav_guard_half_w;  /* ... and less than this to the side ... */
    double nav_guard_hfov_deg;/* ... and inside the phone's LiDAR cone (>= 180: the whole rectangle) */
    double nav_escape_mm;     /* a path leg would start with an obstacle in that zone: pivot to the nearest
                                 heading that's clear and drive this far, then re-plan (0 = off) */
    double robot_radius_m;    /* nav footprint for inflation/fit checks (nav_set_robot_radius); MEASURE the real car */
    double nav_foot_cells;    /* > 0: nav footprint half-size in 0.1 m cells (nav_set_foot_cells); 0 = ceil(robot_radius_m / 0.1) */
    double nav_move_max_mm;   /* tars_motor's MOVE limit (wheels.h MOVE_MAX_MM): longer legs go in pieces */
    /* ONE source of truth for the car's size. With car_length_m and car_width_m both > 0,
     * goto_cfg_derive() sets robot_radius_m = half the diagonal + safety_margin_m and
     * nav_guard_half_w = car_width_m / 2 + safety_margin_m (the guard corridor), and the guard's
     * stop distance is guard_stop_m (= nav_guard_stop_m). The same numbers go to lidar_guard
     * (goto --guard-flags). Both 0 = the legacy numbers above (0.35 / 0.45 / 0.6). */
    double car_length_m, car_width_m, safety_margin_m;
    int    car_derived;       /* 1 = robot_radius_m / nav_guard_half_w came from car_* */
    double nav_guard_plan;    /* 1 = A* only plans legs the guard's box allows (nav_set_guard_box) */
    double nav_backtrack;     /* 1 = when stuck, retrace the breadcrumb trail, then re-plan */
    double nav_replan_here;   /* 1 = a guard stop of a path leg re-plans in place first (cost-to-go field); backtrack only if that fails */
    double nav_here_turn_cost;/* metres of route a 90 deg pivot costs when ranking in-place legs */
    double nav_block_s;       /* cells a guard refusal marked blocked are forgotten after this */
    /* optimistic planning: no route over observed floor -> plan through UNKNOWN (penalised) */
    double nav_optimistic;    /* 1 = on */
    double nav_unknown_penalty;/* extra cost per step on not-yet-observed floor (1 = twice the cost) */
    double nav_unknown_leg_mm;/* a leg through unknown floor drives at most this far, then re-plans */
    double nav_travel_cap_m;  /* optimistic routes: give up after driving this far (0 = 3 x first route + 2 m) */
    double nav_max_opt_replans;/* optimistic routes: re-plans because the map showed a wall on the route */
    /* heading-aware, sticky routes (all 0 = off: the robot's behaviour before them) */
    double nav_turn_cost_m;   /* A*: metres of route a 90 deg corner costs (fewer corners = fewer TURNs) */
    double nav_start_turn_cost_m; /* A*: metres per 90 deg between the car's heading and the first leg (U-turn = 2x) */
    double nav_sticky;        /* 1 = a map-change re-plan keeps the current route if it still fits, else repairs it */
    double nav_sticky_frac;   /* ... unless the fresh route is cheaper by this fraction ... */
    double nav_sticky_m;      /* ... and by at least this many metres */
    /* per-object arrive override, e.g. person is tall: arrive_h.person = 0.85 */
    int    n_arrive_over;
    char   arrive_over_label[16][32];
    double arrive_over_h[16];
    /* not compared with goto.conf by the tests (strings) */
    char   memory_path[256];  /* "-" = keep it in RAM only. Relative = next to goto.conf */
} goto_cfg;

void goto_cfg_defaults(goto_cfg *c);
void goto_cfg_derive(goto_cfg *c);            /* car_* -> radius / guard corridor (goto_cfg_load calls it) */
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
 * (noisy) range; otherwise it becomes a new instance. With a fresh ARKit pose (have_nav_arkit)
 * and an instance with a map position (has_w), that match is done in the map frame instead.
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
    int have_nav_arkit;      /* ... and it's a fresh ARKit pose (not DR-carried): sightings then
                              * match instances by map position (wx, wz), not by the DR x/y */
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
    int path_n, path_i, leg;          /* leg: 0 turn, 1 move, 2 face the goal, 3 reveal turn, 4/5 escape, 6/7 backtrack */
    double leg_mm;
    unsigned path_ver;
    int replans, detoured_id;
    double revealed_deg;
    int reveal_have_yaw;              /* reveal_yaw = ARKit yaw when the current reveal TURN started */
    double reveal_yaw;
    int mem_fallback, fallback_noroute;                 /* the search failed and we headed for the nearest-by-route memory */
    int escapes;                      /* guard-zone escape steps this GOTO (leg 4 = its TURN, 5 = its MOVE) */
    int plan_wait;                    /* ST_PLAN: re-plan once the guard is CLEAR */
    int exploring, explores;          /* heading to a mapped cell nearer an unmapped goal, then re-plan */
    int leg_settle;                   /* leg done, ARKit pose: wait for 2 poses taken after it ended */
    unsigned settle_seq;
    double settle_since;
    /* guard refusals of path legs this GOTO (fresh guard_blocked = the map didn't predict it):
     * refused = ERR when the TURN/MOVE was asked for, aborts = stopped while running */
    int guard_refusals, guard_aborts, guard_other_stops;
    /* breadcrumb trail (nav frame, m): where the robot has driven this GOTO, every ~0.1 m and
     * after each finished leg; corner = a path leg started there. Ring, newest last. */
#define GOTO_CRUMBS 200
    float crumb_x[GOTO_CRUMBS], crumb_z[GOTO_CRUMBS];
    unsigned char crumb_corner[GOTO_CRUMBS];
    int crumb_head, crumb_n;
    /* stuck recovery: backtrack along the trail (leg 6 = its TURN, 7 = its reverse MOVE) */
    int bt_rung, backtracks;          /* rung of the ladder used last (reset on progress), total this GOTO */
    nav_pt_t bt_pts[64];
    int bt_n, bt_i;
    double bt_mm;
    int ref_valid;                    /* the leg the guard just refused: start + waypoint */
    double ref_sx, ref_sz, ref_wx, ref_wz;
    double block_since;               /* nav_temp_block cells set at this time (decay nav_block_s) */
    int turn_stops;                   /* consecutive TURNs stopped by the guard */
    int settle_fresh;                 /* the last leg's settle ended on 2 new ARKit poses (not the timeout) */
    int heading_unchecked;            /* legs whose heading could not be checked (DR / stale pose) this GOTO */
    int heading_tries;                /* corrective TURNs since this leg's TURN (nav_heading_retries) */
    double esc_yaw;                   /* escape leg 4: the nav yaw (rad) the pivot is aiming for */
    int heading_fixes;                /* corrective TURNs this GOTO (STATUS heading_fixes=) */
    double plan_sx, plan_sz;          /* where the current route starts (its first leg's start corner) */
    int relaxed;                      /* the last plan_from_robot fell back to ignoring the guard box */
    int route_guard_ok;               /* the route passed the guard-box check when planned (map-change re-check) */
    int bt_tries;                     /* reverse attempts at the current backtrack point */
    int here_tries;                   /* in-place re-plans since the last finished path MOVE (<= 3) */
    double here_base;                 /* cells left on the route the guard first refused at this spot */
    double here_ref[4];               /* headings (rad) refused at this spot */
    int here_nref;
    int replans_here;                 /* in-place re-plans this GOTO (capped; STATUS replans_here=) */
    int have_goal;                    /* goal_x/z is a remembered spot of this GOTO (map frame) */
    int los_standoffs;                /* searches moved to a cell that can see the spot, this GOTO */
    int plan_depth;                   /* do_plan nesting guard */
    int search_hops, search_hop;      /* search relocations this GOTO; 1 = driving to one, spin on arrival */
    int optimistic;                   /* the current route goes through unknown floor */
    int opt_failed;                   /* the last goal plan failed even through unknown floor */
    int unblock_tried;                /* this GOTO already cleared guard temp blocks once before calling it unreachable */
    int unreachable;                  /* ... for some instance this GOTO: fail reason unreachable */
    double travel_m, opt_len0, last_move_mm;
    int opt_replans, leg_cut;
    int sticky_map;                   /* the pending re-plan is because the map changed under the route */
    int sticky_kept, sticky_repairs;  /* nav_sticky outcomes this GOTO (STATUS) */
    int backed_off;                   /* the last plan_from_robot_1 route starts with a back-off cell */
    int rt_vantage;                   /* the goal the current route was planned to */
    double rt_gx, rt_gz;
    int gone_ids[8], n_gone;          /* instances looked at this GOTO with a clear view: not there */
    /* lidar_guard via tars_motor STATUS */
    int guard_known, guard_ok;
    char guard_reason[24];            /* STATUS guard_reason= (clear|guard_blocked|guard_stale|...); "" = never reported */
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
void search_fallback_for_test(goto_sm *s);   /* tests: the search fallback from the current state */
/* STATUS guard_permit= / guard_epoch= (tars_motor with -g). An epoch change ends a leased
 * APPROACH leg: an old drive never resumes after the guard clears. */
void goto_sm_guard(goto_sm *s, double now, int permit, unsigned long long epoch);
/* STATUS guard_reason= (call before goto_sm_guard, so an epoch change sees the reason behind it).
 * Only a fresh guard_blocked lets a guard stop at a big centred target count as docked. */
void goto_sm_guard_reason(goto_sm *s, const char *reason);
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

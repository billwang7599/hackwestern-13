/* nav: QNX-native occupancy grid + A* route planner for goto.
 *
 * C99 port of iter2-autonomy/dashboard/mapping.py (Map.update + Map._plan). That Python
 * stays a desktop visualization; this module is what drives the robot.
 * No dependencies beyond libc/libm, no threads, no I/O: the caller owns the
 * sockets and feeds packets in. Not thread-safe; call from one thread.
 *
 * Frame: ARKit world, metres, y up. The grid is the x/z floor plane:
 * 200 x 200 cells of 0.1 m, cell (0,0) has its corner at world (-10,-10),
 * so it covers x,z in [-10, 10). Rows increase with world z.
 * Yaw: forward = (sin yaw, cos yaw) in (x, z), like lidar_guard and the
 * LDR1 pose. Radians.
 *
 * Cells are UNKNOWN / FREE / OCC. Planning is conservative like the Python:
 * the robot's whole footprint square (2*ceil(radius/0.1)+1 cells per side)
 * must be observed FREE, so unknown space blocks exactly like an obstacle.
 * SCAN (spin in place) is how the caller reveals more floor.
 *
 * POSE SOURCE RULE (one pose for the planner):
 *  - ARKit pose while tracking is OK: nav_set_pose(x, z, yaw, 1), or let
 *    nav_ingest_ldr1() do it from each packet.
 *  - When tracking is lost, the caller falls back to dead reckoning
 *    (tars_motor STATUS pose=): feed it with nav_note_dr() every time you
 *    read STATUS, tracking or not. nav keeps the (ARKit, DR) pair from the
 *    last tracking-OK moment as an anchor and nav_get_pose() maps the DR
 *    motion since then into the ARKit frame. With no DR fed it returns the
 *    last ARKit pose, flagged NAV_POSE_STALE.
 *  - nav_set_pose(..., 0) means "tracking lost": values are kept only as a
 *    last resort and the grid stops accepting LDR1 points.
 * Always re-observe the target with YOLO before declaring arrival: a DR or
 * relocalized pose can be off by tens of centimetres.
 */
#ifndef TARS_NAV_H
#define TARS_NAV_H

#include <stddef.h>
#include <stdint.h>

#define NAV_SIZE      200
#define NAV_RES       0.1
#define NAV_ORIGIN_X  (-10.0)
#define NAV_ORIGIN_Z  (-10.0)
#define NAV_MAX_PATH  256          /* corner limit, same as mapping.py */
#define NAV_LDR1_HEADER 44
#define NAV_LDR1_MAX_POINTS 1024
#define NAV_LDR1_MAX_BODY (NAV_LDR1_HEADER + NAV_LDR1_MAX_POINTS * 12)

/* LDR1 flags (iter2-autonomy/edge-ai/README.md) */
#define NAV_LDR1_TRACKING 1u
#define NAV_LDR1_FLOOR    2u
#define NAV_LDR1_HEADING  4u

/* Height bands above the floor, metres (mapping.py MIN/MAX_HEIGHT) */
#define NAV_FLOOR_BAND  0.06       /* |h| <= this: observed floor -> FREE */
#define NAV_OBS_MIN     0.08       /* MIN_HEIGHT <= h <= MAX_HEIGHT: OCC  */
#define NAV_OBS_MAX     1.2
#define NAV_CLEAR_FRAMES 5
#define NAV_CLEAR_SPAN_US 600000u
#define NAV_CLEAR_GAP_US 500000u
#define NAV_DEFAULT_ROBOT_RADIUS 0.35   /* mapping.py ROBOT_RADIUS; set the real one */
#define NAV_RING_MIN    0.6        /* approach ring around the target, metres */
#define NAV_RING_MAX    0.85
#define NAV_LOS_END_MARGIN 0.3     /* nav_line_clear ignores cells this close to the end (the object itself) */

typedef struct nav nav_t;                 /* opaque; nav_new()/nav_free() */
typedef struct { float x, z; } nav_pt_t;

enum { NAV_UNKNOWN = 0, NAV_FREE = 1, NAV_OCC = 2 };   /* same codes as mapping.py's grid */

enum {
    NAV_OK = 0,
    NAV_ERR_START_BLOCKED  = -1,   /* robot's footprint not observed clear (or start off-map) */
    NAV_ERR_TARGET_OUTSIDE = -2,   /* target outside the 20 m map */
    NAV_ERR_NO_GOAL        = -3,   /* no clear cell on the 0.6-0.85 m ring */
    NAV_ERR_NO_PATH        = -4,   /* no route through observed clear floor */
    NAV_ERR_NO_MAP         = -5,   /* nothing observed free yet */
    NAV_ERR_TOO_LONG       = -6,   /* > NAV_MAX_PATH corners, or > max_out */
    NAV_ERR_ARG            = -7    /* NULL / non-finite input */
};

/* nav_get_pose() sources */
enum { NAV_POSE_NONE = 0, NAV_POSE_ARKIT = 1, NAV_POSE_DR = 2, NAV_POSE_STALE = 3 };

/* LDR1 ingest results (>= 0 is a valid packet) */
enum { NAV_LDR1_MAPPED = 0, NAV_LDR1_SKIPPED = 1 /* valid, but tracking/floor not OK: no map change */,
       NAV_LDR1_OLD = 2 /* duplicate/out-of-order: no pose/map/freshness change */,
       NAV_LDR1_BAD = -1 };

nav_t *nav_new(void);                     /* ~2.2 MiB, all scratch preallocated; NULL on OOM */
void   nav_free(nav_t *n);
void   nav_reset(nav_t *n);               /* all cells UNKNOWN, pose/anchor/floor forgotten */
void   nav_set_robot_radius(nav_t *n, float metres);  /* footprint for inflation (default 0.35) */
float  nav_robot_radius(const nav_t *n);

/* Guard-aware planning (off by default, so plans match mapping.py exactly).
 * lidar_guard refuses a forward move while an obstacle is within stop_m ahead and
 * |side| < half_w (inside the phone LiDAR's hfov cone). With a box set, an A* step from a cell
 * in one of the 8 grid directions is only allowed if that box, from the cell centre along that
 * direction and stretched by the step length, holds no OCC (or temporarily blocked) cell, with
 * half a cell of slack. Corners are kept, so every route leg is a grid direction, and the route
 * is one the guard lets through as far as the map knows. For object goals (ring_min >= 0.3)
 * obstacles within NAV_GUARD_IGNORE_R of the target are ignored: stopping in front of the
 * target is docking. stop_m <= 0 turns it off. nav_set_guard_active() switches it off/on for
 * one plan (goto's relaxed fallback) without forgetting the box. */
#define NAV_GUARD_IGNORE_R 0.35
void   nav_set_guard_box(nav_t *n, float stop_m, float half_w, float hfov_deg);
void   nav_set_guard_active(nav_t *n, int on);
int    nav_guard_active(const nav_t *n);
/* The same box test for one pose (x, z, yaw; forward = (sin yaw, cos yaw)): 1 clear, 0 an
 * OCC/temporarily blocked cell in it. ignore_r > 0: cells within ignore_r of (ix, iz) don't count. */
int    nav_guard_clear(const nav_t *n, float x, float z, float yaw, float ix, float iz, float ignore_r);

/* Optimistic planning (off by default): UNKNOWN floor is traversable. Only known OCC (and
 * temporarily blocked) cells are inflated; a step onto a cell whose footprint isn't all observed
 * FREE costs (1 + penalty) x the step. The start may stand on unknown floor. A plan that fails
 * even so means the goal is enclosed by known walls (or too tight for the footprint). */
void   nav_set_optimistic(nav_t *n, int on, float penalty);
int    nav_optimistic(const nav_t *n);
/* 1 if the footprint square at (x, z) holds an OCC or temporarily blocked cell */
int    nav_footprint_blocked(const nav_t *n, float x, float z);

/* Temporarily blocked cells (a guard refusal the map can't explain): planned around like OCC
 * (inflation and guard box) but never written into the map. The caller decays them. */
void   nav_temp_block(nav_t *n, float x, float z);
void   nav_temp_clear(nav_t *n);
int    nav_temp_count(const nav_t *n);

void nav_set_pose(nav_t *n, float x, float z, float yaw_rad, int tracking_ok);
void nav_note_dr(nav_t *n, float dr_x, float dr_z, float dr_yaw_rad);   /* dead reckoning, its own frame, same yaw convention */
int  nav_get_pose(const nav_t *n, float *x, float *z, float *yaw_rad);  /* NAV_POSE_* */
/* Count of tracking-OK ARKit poses taken so far (wraps). A caller that just finished a
 * move waits for this to advance before trusting the ARKit pose: the newest packet may
 * have been measured before the move ended (link latency; DR is already current). */
unsigned nav_pose_seq(const nav_t *n);

/* Single-point fixture/debug helper, not the live ingest/recovery path. y_above_floor = point y - floor y.
 * |h| <= 0.06 -> FREE if it was UNKNOWN (never clears an obstacle);
 * 0.08 <= h <= 1.2 -> OCC; anything else is ignored. */
void nav_mark_point(nav_t *n, float x, float y_above_floor, float z);

/* Robot -> measured hit: UNKNOWN cells strictly before the hit cell become
 * FREE. Never clears OCC, never touches the hit cell. Optional: LDR1 ingest
 * does NOT carve (mapping.py only frees directly observed floor). */
void nav_carve_ray(nav_t *n, float x0, float z0, float x1, float z1);

/* One LDR1 packet, with or without the 4-byte big-endian length prefix.
 * Validated like lidar_guard: exact size, flags & ~7 == 0, count <= 1024,
 * every float finite and |v| <= 1000, uptime != 0. Points are ARKit world
 * x,y,z; height = y - floor_y. Maps only if tracking+floor flags are set;
 * sets the pose when tracking+heading are set. A floor_y jump > 0.1 m
 * clears the grid (like mapping.py). Returns NAV_LDR1_*. */
int nav_ingest_ldr1(nav_t *n, const unsigned char *buf, size_t len);
/* Explicit monotonic receive time for deterministic recovery tests. */
int nav_ingest_ldr1_at(nav_t *n, const unsigned char *buf, size_t len, uint64_t received_us);

/* A* from (sx,sz) to the approach ring around (tx,tz). Writes the corners of
 * the route (collinear cells removed, start excluded, last = approach point,
 * cell centres) to out. Returns the count (0 = already on the ring) or NAV_ERR_*. */
int nav_plan(nav_t *n, float sx, float sz, float tx, float tz, nav_pt_t *out, int max_out);
/* Same, with a custom ring [ring_min, ring_max] (metres, ring_max <= NAV_RING_MAX).
 * ring 0..0.11 = "the cell at (tx,tz) itself": goto's DETOUR uses it to reach a vantage cell. */
int nav_plan_ring(nav_t *n, float sx, float sz, float tx, float tz, float ring_min, float ring_max,
                  nav_pt_t *out, int max_out);

/* Line of sight over the grid, ignoring the start cell and cells within
 * NAV_LOS_END_MARGIN of the end (the object itself is occupied).
 * 1 all FREE, 0 an OCC cell in between (occluded), -1 unknown/off-map cells
 * but no OCC. */
int nav_line_clear(const nav_t *n, float x0, float z0, float x1, float z1);
int nav_line_clear_ex(const nav_t *n, float x0, float z0, float x1, float z1, float end_margin);

int      nav_cell(const nav_t *n, float x, float z);   /* NAV_UNKNOWN off-map */
unsigned nav_version(const nav_t *n);
unsigned nav_generation(const nav_t *n);
int nav_floor(const nav_t *n, float *floor, float *camera_y);

/* Debug / tests */
int  nav_cell_at(const nav_t *n, int ix, int iz);
void nav_set_cell(nav_t *n, int ix, int iz, int v);
int  nav_count(const nav_t *n, int v);
int  nav_world_to_cell(float x, float z, int *ix, int *iz);   /* 1 if on the map */

#endif

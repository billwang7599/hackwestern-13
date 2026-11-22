/* nav: occupancy grid + A* planner. See nav.h. Port of iter2-autonomy/dashboard/mapping.py.
 *
 * Parity notes (tests/nav_parity.py checks these against the Python):
 *  - the same doubles as Python: cell = floor((x - ORIGIN) / RES), cell centre
 *    = round(ORIGIN + (c + 0.5) * RES, 3);
 *  - A* pops by the same total order as heapq on (f, g, (x, z)), so ties break
 *    identically and the route is the same cell for cell. We keep one live entry
 *    per cell (indexed heap with decrease-key) instead of heapq's stale copies;
 *    the stale copies have a larger key and are skipped there, so the pop order
 *    of live entries is the same.
 */
#include "nav.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define N_CELLS (NAV_SIZE * NAV_SIZE)
#define STRIDE  (NAV_SIZE + 1)
#define RING_SPAN 9               /* mapping.py scans dx,dz in [-9, 9] */
#define MAX_ABS 1000.0            /* lidar_guard's scalar() bound */

struct nav {
    unsigned char grid[N_CELLS];
    unsigned version;
    unsigned generation;
    uint64_t source_us;
    float camera_y;
    unsigned char frame[N_CELLS], clear_hits[N_CELLS];
    uint64_t clear_since[N_CELLS], clear_last[N_CELLS];
    double robot_radius;
    /* guard-aware planning (nav_set_guard_box) */
    double guard_stop, guard_half_w, guard_hfov;
    int    guard_on;
    int    guard_n[8];
    int16_t guard_off[8][640][2];
    unsigned short gmemo[N_CELLS];  /* bit k: dir k computed, bit 8+k: dir k blocked */
    unsigned char tblock[N_CELLS];
    int    n_tblock;
    int    ign_on;
    double ign_x, ign_z, ign_r;
    int    have_floor;
    double floor_y;

    /* pose */
    int   ark_ok, have_ark, have_dr, have_anchor, have_lost;
    unsigned pose_seq;                  /* tracking-OK ARKit poses received (wraps) */
    float ark_x, ark_z, ark_yaw;
    float dr_x, dr_z, dr_yaw;
    float a_ark_x, a_ark_z, a_ark_yaw, a_dr_x, a_dr_z, a_dr_yaw;
    float lost_x, lost_z, lost_yaw;

    /* planner scratch, preallocated so planning never mallocs */
    int32_t prefix[STRIDE * STRIDE];
    unsigned char clear[N_CELLS];
    unsigned char goal[N_CELLS];
    double  g[N_CELLS];
    double  f[N_CELLS];
    int32_t parent[N_CELLS];
    int32_t heap[N_CELLS];
    int32_t pos[N_CELLS];       /* index in heap, -1 = not in heap */
    int32_t route[N_CELLS];
};

/* ---- basics ------------------------------------------------------------ */

static int finite_ok(double v) { return isfinite(v) && fabs(v) <= MAX_ABS; }

int nav_world_to_cell(float x, float z, int *ix, int *iz)
{
    if (!finite_ok(x) || !finite_ok(z)) return 0;
    double cx = floor(((double)x - NAV_ORIGIN_X) / NAV_RES);
    double cz = floor(((double)z - NAV_ORIGIN_Z) / NAV_RES);
    if (cx < 0 || cz < 0 || cx >= NAV_SIZE || cz >= NAV_SIZE) return 0;
    if (ix) *ix = (int)cx;
    if (iz) *iz = (int)cz;
    return 1;
}

/* Python round(v, 3). Values here are k*0.05 +- 1e-15, never a tie. */
static double round3(double v) { return round(v * 1000.0) / 1000.0; }
static double centre_x(int ix) { return round3(NAV_ORIGIN_X + (ix + 0.5) * NAV_RES); }
static double centre_z(int iz) { return round3(NAV_ORIGIN_Z + (iz + 0.5) * NAV_RES); }

static void set_idx(nav_t *n, int idx, unsigned char v)
{
    if (n->grid[idx] != v) { n->grid[idx] = v; n->version++; }
}

nav_t *nav_new(void)
{
    nav_t *n = malloc(sizeof *n);
    if (!n) return NULL;
    memset(n, 0, sizeof *n);
    n->robot_radius = NAV_DEFAULT_ROBOT_RADIUS;
    n->version = 1;
    return n;
}

void nav_free(nav_t *n) { free(n); }

void nav_reset(nav_t *n)
{
    if (!n) return;
    memset(n->grid, NAV_UNKNOWN, sizeof n->grid);
    memset(n->clear_hits, 0, sizeof n->clear_hits);
    memset(n->clear_since, 0, sizeof n->clear_since);
    memset(n->clear_last, 0, sizeof n->clear_last);
    n->source_us = 0;
    n->generation++;
    n->have_floor = 0;
    n->ark_ok = n->have_ark = n->have_dr = n->have_anchor = n->have_lost = 0;
    memset(n->tblock, 0, sizeof n->tblock);
    n->n_tblock = 0;
    n->version++;
}

void nav_set_robot_radius(nav_t *n, float metres)
{
    if (!n || !isfinite(metres) || metres < 0 || metres > 2) return;
    /* back to the decimal the caller meant: float 0.2f is 0.2000000030, whose
     * ceil(r / 0.1) would be 3 cells where mapping.py's 0.2 gives 2 */
    double r = round((double)metres * 1e6) / 1e6;
    if (n->robot_radius != r) { n->robot_radius = r; n->version++; }
}

float nav_robot_radius(const nav_t *n) { return n ? (float)n->robot_radius : (float)NAV_DEFAULT_ROBOT_RADIUS; }

/* the 8 A* directions, same order as the planner */
static const int GDX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
static const int GDZ[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

/* Offsets (cells) of lidar_guard's box from a cell centre along grid direction k, stretched by
 * one step: an obstacle in any of them stops a robot driving that step. Half a cell of slack on
 * every test (an obstacle is anywhere inside its cell). */
static void guard_offsets(nav_t *n)
{
    const double m = 0.5 * NAV_RES;
    double ta = n->guard_hfov >= 180 ? 1e9 : tan(n->guard_hfov / 2 * M_PI / 180.0);
    for (int k = 0; k < 8; k++) {
        double L = hypot(GDX[k], GDZ[k]), ux = GDX[k] / L, uz = GDZ[k] / L;
        double reach = n->guard_stop + L * NAV_RES + m;
        int R = (int)ceil(reach / NAV_RES) + 1, c = 0;
        for (int i = -R; i <= R; i++)
            for (int j = -R; j <= R; j++) {
                double dx = i * NAV_RES, dz = j * NAV_RES;
                double f = ux * dx + uz * dz, side = fabs(uz * dx - ux * dz);
                if (f <= 0 || f > reach || side >= n->guard_half_w + m || side >= f * ta + m) continue;
                if (c < 640) { n->guard_off[k][c][0] = (int16_t)i; n->guard_off[k][c][1] = (int16_t)j; c++; }
            }
        n->guard_n[k] = c;
    }
}

void nav_set_guard_box(nav_t *n, float stop_m, float half_w, float hfov_deg)
{
    if (!n) return;
    if (!isfinite(stop_m) || !isfinite(half_w) || !isfinite(hfov_deg) || stop_m <= 0 || half_w <= 0 ||
        stop_m > 1.5f || half_w > 1.0f) {
        n->guard_stop = 0; n->guard_on = 0; return;
    }
    n->guard_stop = round((double)stop_m * 1e6) / 1e6;
    n->guard_half_w = round((double)half_w * 1e6) / 1e6;
    n->guard_hfov = hfov_deg > 0 ? hfov_deg : 180;
    n->guard_on = 1;
    guard_offsets(n);
    n->version++;
}

void nav_set_guard_active(nav_t *n, int on) { if (n) n->guard_on = on && n->guard_stop > 0; }
int  nav_guard_active(const nav_t *n) { return n && n->guard_on; }

static int blocked_idx(const nav_t *n, int idx) { return n->grid[idx] == NAV_OCC || n->tblock[idx]; }

/* 1 if stepping from (x, z) in direction k keeps the guard's box clear */
static int guard_step_ok(nav_t *n, int x, int z, int k)
{
    int idx = z * NAV_SIZE + x;
    unsigned short g = n->gmemo[idx];
    if (g & (1u << k)) return !(g & (1u << (8 + k)));
    int ok = 1;
    for (int i = 0; i < n->guard_n[k] && ok; i++) {
        int cx = x + n->guard_off[k][i][0], cz = z + n->guard_off[k][i][1];
        if (cx < 0 || cz < 0 || cx >= NAV_SIZE || cz >= NAV_SIZE) continue;
        if (!blocked_idx(n, cz * NAV_SIZE + cx)) continue;
        if (n->ign_on) {
            double wx = NAV_ORIGIN_X + (cx + 0.5) * NAV_RES, wz = NAV_ORIGIN_Z + (cz + 0.5) * NAV_RES;
            if (hypot(wx - n->ign_x, wz - n->ign_z) <= n->ign_r) continue;
        }
        ok = 0;
    }
    n->gmemo[idx] = (unsigned short)(g | (1u << k) | (ok ? 0 : 1u << (8 + k)));
    return ok;
}

int nav_guard_clear(const nav_t *n, float x, float z, float yaw, float ix, float iz, float ignore_r)
{
    int cx, cz;
    if (!n || n->guard_stop <= 0 || !nav_world_to_cell(x, z, &cx, &cz)) return 1;
    const double m = 0.5 * NAV_RES;
    double ta = n->guard_hfov >= 180 ? 1e9 : tan(n->guard_hfov / 2 * M_PI / 180.0);
    double fx = sin(yaw), fz = cos(yaw);
    int r = (int)ceil((n->guard_stop + m) / NAV_RES) + 1;
    for (int i = -r; i <= r; i++)
        for (int k = -r; k <= r; k++) {
            int ax = cx + i, az = cz + k;
            if (ax < 0 || az < 0 || ax >= NAV_SIZE || az >= NAV_SIZE || !blocked_idx(n, az * NAV_SIZE + ax)) continue;
            double wx = NAV_ORIGIN_X + (ax + 0.5) * NAV_RES, wz = NAV_ORIGIN_Z + (az + 0.5) * NAV_RES;
            if (ignore_r > 0 && hypot(wx - ix, wz - iz) <= ignore_r) continue;
            double dx = wx - x, dz = wz - z, f = fx * dx + fz * dz, side = fabs(fz * dx - fx * dz);
            if (f > -m && f <= n->guard_stop + m && side < n->guard_half_w + m && side < f * ta + m) return 0;
        }
    return 1;
}

void nav_temp_block(nav_t *n, float x, float z)
{
    int ix, iz;
    if (!n || !nav_world_to_cell(x, z, &ix, &iz)) return;
    int idx = iz * NAV_SIZE + ix;
    if (!n->tblock[idx]) { n->tblock[idx] = 1; n->n_tblock++; n->version++; }
}

void nav_temp_clear(nav_t *n)
{
    if (!n || !n->n_tblock) return;
    memset(n->tblock, 0, sizeof n->tblock);
    n->n_tblock = 0;
    n->version++;
}

int nav_temp_count(const nav_t *n) { return n ? n->n_tblock : 0; }

int nav_cell_at(const nav_t *n, int ix, int iz)
{
    if (!n || ix < 0 || iz < 0 || ix >= NAV_SIZE || iz >= NAV_SIZE) return NAV_UNKNOWN;
    return n->grid[iz * NAV_SIZE + ix];
}

void nav_set_cell(nav_t *n, int ix, int iz, int v)
{
    if (!n || ix < 0 || iz < 0 || ix >= NAV_SIZE || iz >= NAV_SIZE) return;
    if (v != NAV_UNKNOWN && v != NAV_FREE && v != NAV_OCC) return;
    set_idx(n, iz * NAV_SIZE + ix, (unsigned char)v);
}

int nav_count(const nav_t *n, int v)
{
    int c = 0;
    if (!n) return 0;
    for (int i = 0; i < N_CELLS; i++) c += n->grid[i] == v;
    return c;
}

int nav_cell(const nav_t *n, float x, float z)
{
    int ix, iz;
    if (!n || !nav_world_to_cell(x, z, &ix, &iz)) return NAV_UNKNOWN;
    return n->grid[iz * NAV_SIZE + ix];
}

unsigned nav_version(const nav_t *n) { return n ? n->version : 0; }
unsigned nav_generation(const nav_t *n) { return n ? n->generation : 0; }
int nav_floor(const nav_t *n, float *floor, float *camera_y) {
    if (floor) *floor = n ? (float)n->floor_y : 0;
    if (camera_y) *camera_y = n ? n->camera_y : 0;
    return n && n->have_floor;
}
unsigned nav_pose_seq(const nav_t *n) { return n ? n->pose_seq : 0; }

/* ---- pose -------------------------------------------------------------- */

static float wrap_pi(double a)
{
    a = fmod(a, 2 * M_PI);
    if (a > M_PI) a -= 2 * M_PI;
    else if (a <= -M_PI) a += 2 * M_PI;
    return (float)a;
}

static void take_anchor(nav_t *n)
{
    n->a_ark_x = n->ark_x; n->a_ark_z = n->ark_z; n->a_ark_yaw = n->ark_yaw;
    n->a_dr_x = n->dr_x;   n->a_dr_z = n->dr_z;   n->a_dr_yaw = n->dr_yaw;
    n->have_anchor = 1;
}

void nav_set_pose(nav_t *n, float x, float z, float yaw_rad, int tracking_ok)
{
    if (!n) return;
    if (!finite_ok(x) || !finite_ok(z) || !finite_ok(yaw_rad)) tracking_ok = 0;
    if (tracking_ok) {
        n->ark_x = x; n->ark_z = z; n->ark_yaw = yaw_rad;
        n->ark_ok = n->have_ark = 1;
        n->pose_seq++;
        if (n->have_dr) take_anchor(n);
    } else {
        n->ark_ok = 0;
        if (finite_ok(x) && finite_ok(z) && finite_ok(yaw_rad)) {
            n->lost_x = x; n->lost_z = z; n->lost_yaw = yaw_rad; n->have_lost = 1;
        }
    }
}

void nav_note_dr(nav_t *n, float dr_x, float dr_z, float dr_yaw_rad)
{
    if (!n || !finite_ok(dr_x) || !finite_ok(dr_z) || !finite_ok(dr_yaw_rad)) return;
    n->dr_x = dr_x; n->dr_z = dr_z; n->dr_yaw = dr_yaw_rad; n->have_dr = 1;
    if (n->ark_ok) take_anchor(n);
}

int nav_get_pose(const nav_t *n, float *x, float *z, float *yaw)
{
    float px, pz, pyaw;
    int src;
    if (!n) return NAV_POSE_NONE;
    if (n->ark_ok) {
        px = n->ark_x; pz = n->ark_z; pyaw = n->ark_yaw; src = NAV_POSE_ARKIT;
    } else if (n->have_anchor && n->have_dr) {
        /* DR motion since the anchor, rotated into the ARKit frame.
         * Rotating a heading by t maps (sin y, cos y) to (sin(y+t), cos(y+t)). */
        double t = (double)n->a_ark_yaw - n->a_dr_yaw;
        double vx = (double)n->dr_x - n->a_dr_x, vz = (double)n->dr_z - n->a_dr_z;
        double c = cos(t), s = sin(t);
        px = (float)(n->a_ark_x + vx * c + vz * s);
        pz = (float)(n->a_ark_z + vz * c - vx * s);
        pyaw = wrap_pi((double)n->dr_yaw + t);
        src = NAV_POSE_DR;
    } else if (n->have_ark) {
        px = n->ark_x; pz = n->ark_z; pyaw = n->ark_yaw; src = NAV_POSE_STALE;
    } else if (n->have_lost) {
        px = n->lost_x; pz = n->lost_z; pyaw = n->lost_yaw; src = NAV_POSE_STALE;
    } else {
        return NAV_POSE_NONE;
    }
    if (x) *x = px;
    if (z) *z = pz;
    if (yaw) *yaw = pyaw;
    return src;
}

/* ---- marking ----------------------------------------------------------- */

void nav_mark_point(nav_t *n, float x, float h, float z)
{
    int ix, iz;
    if (!n || !finite_ok(h) || !nav_world_to_cell(x, z, &ix, &iz)) return;
    int idx = iz * NAV_SIZE + ix;
    double hd = h;
    if (fabs(hd) <= NAV_FLOOR_BAND) {
        if (n->grid[idx] == NAV_UNKNOWN) set_idx(n, idx, NAV_FREE);
    } else if (hd >= NAV_OBS_MIN && hd <= NAV_OBS_MAX) {
        set_idx(n, idx, NAV_OCC);
    }
}

/* Grid traversal (Amanatides & Woo), 4-connected: every cell the segment
 * passes through, in order. Cells may be off the map (caller checks).
 * Returns the cell count, or -1 if the input is bad or the line is absurdly long. */
#define WALK_MAX (8 * NAV_SIZE)
static int walk(double x0, double z0, double x1, double z1, int *cx, int *cz)
{
    if (!finite_ok(x0) || !finite_ok(z0) || !finite_ok(x1) || !finite_ok(z1)) return -1;
    double gx0 = (x0 - NAV_ORIGIN_X) / NAV_RES, gz0 = (z0 - NAV_ORIGIN_Z) / NAV_RES;
    double gx1 = (x1 - NAV_ORIGIN_X) / NAV_RES, gz1 = (z1 - NAV_ORIGIN_Z) / NAV_RES;
    int ix = (int)floor(gx0), iz = (int)floor(gz0);
    int ex = (int)floor(gx1), ez = (int)floor(gz1);
    int nx = abs(ex - ix), nz = abs(ez - iz);
    int count = 1 + nx + nz;
    if (count > WALK_MAX) return -1;
    double dx = gx1 - gx0, dz = gz1 - gz0;
    int sx = ex > ix ? 1 : -1, sz = ez > iz ? 1 : -1;
    double tmx = nx ? ((sx > 0 ? ix + 1 : ix) - gx0) / dx : INFINITY;
    double tmz = nz ? ((sz > 0 ? iz + 1 : iz) - gz0) / dz : INFINITY;
    double tdx = nx ? fabs(1.0 / dx) : INFINITY, tdz = nz ? fabs(1.0 / dz) : INFINITY;
    for (int i = 0; i < count; i++) {
        cx[i] = ix; cz[i] = iz;
        if (i == count - 1) break;
        /* the remaining-step counts keep us exact even if t rounds oddly */
        if (nz == 0 || (nx > 0 && tmx < tmz)) { ix += sx; tmx += tdx; nx--; }
        else                                  { iz += sz; tmz += tdz; nz--; }
    }
    return count;
}

void nav_carve_ray(nav_t *n, float x0, float z0, float x1, float z1)
{
    int cx[WALK_MAX], cz[WALK_MAX];
    if (!n) return;
    int k = walk(x0, z0, x1, z1, cx, cz);
    for (int i = 0; i < k - 1; i++) {          /* not the hit cell */
        if (cx[i] < 0 || cz[i] < 0 || cx[i] >= NAV_SIZE || cz[i] >= NAV_SIZE) continue;
        int idx = cz[i] * NAV_SIZE + cx[i];
        if (n->grid[idx] == NAV_UNKNOWN) set_idx(n, idx, NAV_FREE);
    }
}

int nav_line_clear_ex(const nav_t *n, float x0, float z0, float x1, float z1, float end_margin)
{
    int cx[WALK_MAX], cz[WALK_MAX];
    if (!n || !isfinite(end_margin)) return -1;
    int k = walk(x0, z0, x1, z1, cx, cz);
    if (k < 0) return -1;
    int unknown = 0;
    for (int i = 1; i < k - 1; i++) {          /* skip the robot's cell and the end cell */
        int in = cx[i] >= 0 && cz[i] >= 0 && cx[i] < NAV_SIZE && cz[i] < NAV_SIZE;
        double wx = NAV_ORIGIN_X + (cx[i] + 0.5) * NAV_RES, wz = NAV_ORIGIN_Z + (cz[i] + 0.5) * NAV_RES;
        if (hypot(wx - x1, wz - z1) <= end_margin) continue;
        int v = in ? n->grid[cz[i] * NAV_SIZE + cx[i]] : NAV_UNKNOWN;
        if (v == NAV_OCC) return 0;
        if (v == NAV_UNKNOWN) unknown = 1;
    }
    return unknown ? -1 : 1;
}

int nav_line_clear(const nav_t *n, float x0, float z0, float x1, float z1)
{
    return nav_line_clear_ex(n, x0, z0, x1, z1, (float)NAV_LOS_END_MARGIN);
}

/* ---- LDR1 -------------------------------------------------------------- */

static uint32_t be32(const unsigned char *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static int befloat(const unsigned char *p, float *out)
{
    uint32_t bits = be32(p);
    float v;
    memcpy(&v, &bits, sizeof v);
    if (!isfinite(v) || fabsf(v) > (float)MAX_ABS) return 0;
    *out = v;
    return 1;
}

int nav_ingest_ldr1(nav_t *n, const unsigned char *buf, size_t len)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return nav_ingest_ldr1_at(n, buf, len, (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}

int nav_ingest_ldr1_at(nav_t *n, const unsigned char *buf, size_t len, uint64_t received_us)
{
    if (!n || !buf) return NAV_LDR1_BAD;
    const unsigned char *p = buf;
    size_t size = len;
    if (len >= 8 && memcmp(buf + 4, "LDR1", 4) == 0) {        /* length-prefixed */
        if ((size_t)be32(buf) != len - 4) return NAV_LDR1_BAD;
        p = buf + 4; size = len - 4;
    }
    if (size < NAV_LDR1_HEADER || size > NAV_LDR1_MAX_BODY || memcmp(p, "LDR1", 4) != 0) return NAV_LDR1_BAD;
    uint32_t flags = be32(p + 16), count = be32(p + 20);
    if ((flags & ~7u) != 0 || count > NAV_LDR1_MAX_POINTS
        || size != NAV_LDR1_HEADER + (size_t)count * 12) return NAV_LDR1_BAD;
    if (be32(p + 8) == 0 && be32(p + 12) == 0) return NAV_LDR1_BAD;   /* source uptime 0 */
    float pose[4], floor_y;
    for (int i = 0; i < 4; i++) if (!befloat(p + 24 + i * 4, &pose[i])) return NAV_LDR1_BAD;
    if (!befloat(p + 40, &floor_y)) return NAV_LDR1_BAD;
    for (uint32_t i = 0; i < count * 3; i++) {                 /* validate all before mutating */
        float v;
        if (!befloat(p + NAV_LDR1_HEADER + i * 4, &v)) return NAV_LDR1_BAD;
    }

    uint64_t source_us = (uint64_t)be32(p + 8) << 32 | be32(p + 12);
    if (source_us <= n->source_us) return NAV_LDR1_OLD;
    n->source_us = source_us;
    n->camera_y = pose[1];
    int pose_ok = (flags & (NAV_LDR1_TRACKING | NAV_LDR1_HEADING)) == (NAV_LDR1_TRACKING | NAV_LDR1_HEADING);
    nav_set_pose(n, pose[0], pose[2], pose[3], pose_ok);

    if ((flags & (NAV_LDR1_TRACKING | NAV_LDR1_FLOOR)) != (NAV_LDR1_TRACKING | NAV_LDR1_FLOOR))
    {
        memset(n->clear_hits, 0, sizeof n->clear_hits);
        return NAV_LDR1_SKIPPED;
    }
    if (n->have_floor && fabs((double)floor_y - n->floor_y) > 0.1) {
        memset(n->grid, NAV_UNKNOWN, sizeof n->grid);           /* floor recalibrated */
        memset(n->clear_hits, 0, sizeof n->clear_hits);
        n->generation++;
        n->version++;
    }
    n->have_floor = 1;
    n->floor_y = floor_y;
    memset(n->frame, 0, sizeof n->frame);
    for (uint32_t i = 0; i < count; i++) {
        float x, y, z;
        const unsigned char *q = p + NAV_LDR1_HEADER + i * 12;
        befloat(q, &x); befloat(q + 4, &y); befloat(q + 8, &z);
        int ix, iz;
        if (!nav_world_to_cell(x, z, &ix, &iz)) continue;
        int k = iz * NAV_SIZE + ix;
        double h = (double)y - floor_y;
        if (fabs(h) <= NAV_FLOOR_BAND) n->frame[k] |= 1;
        else if (h >= NAV_OBS_MIN && h <= NAV_OBS_MAX) n->frame[k] |= 2;
    }
    /* One vote per cell per whole frame. Obstacle wins regardless of point order.
     * Never clear on silence, high rays, duplicated packets or a burst of backlog. */
    for (int k = 0; k < N_CELLS; k++) {
        if (n->frame[k] & 2) {
            set_idx(n, k, NAV_OCC);
            n->clear_hits[k] = 0;
        } else if (n->frame[k] & 1) {
            if (n->grid[k] == NAV_UNKNOWN) set_idx(n, k, NAV_FREE);
            if (n->grid[k] != NAV_OCC) continue;
            if (!n->clear_hits[k] || received_us < n->clear_last[k] ||
                    received_us - n->clear_last[k] > NAV_CLEAR_GAP_US) {
                n->clear_hits[k] = 0;
                n->clear_since[k] = received_us;
            }
            n->clear_last[k] = received_us;
            if (n->clear_hits[k] < NAV_CLEAR_FRAMES) n->clear_hits[k]++;
            if (n->clear_hits[k] >= NAV_CLEAR_FRAMES &&
                    received_us - n->clear_since[k] >= NAV_CLEAR_SPAN_US) {
                set_idx(n, k, NAV_FREE);
                n->clear_hits[k] = 0;
            }
        }
    }
    return NAV_LDR1_MAPPED;
}

/* ---- A* ---------------------------------------------------------------- */

/* heapq order on (f, g, (x, z)) */
static int less(const nav_t *n, int a, int b)
{
    if (n->f[a] != n->f[b]) return n->f[a] < n->f[b];
    if (n->g[a] != n->g[b]) return n->g[a] < n->g[b];
    int ax = a % NAV_SIZE, bx = b % NAV_SIZE;
    if (ax != bx) return ax < bx;
    return a / NAV_SIZE < b / NAV_SIZE;
}

static void heap_swap(nav_t *n, int i, int j)
{
    int32_t t = n->heap[i]; n->heap[i] = n->heap[j]; n->heap[j] = t;
    n->pos[n->heap[i]] = i; n->pos[n->heap[j]] = j;
}
static void sift_up(nav_t *n, int i)
{
    while (i > 0) {
        int p = (i - 1) / 2;
        if (!less(n, n->heap[i], n->heap[p])) break;
        heap_swap(n, i, p); i = p;
    }
}
static void sift_down(nav_t *n, int i, int size)
{
    for (;;) {
        int l = 2 * i + 1, r = l + 1, m = i;
        if (l < size && less(n, n->heap[l], n->heap[m])) m = l;
        if (r < size && less(n, n->heap[r], n->heap[m])) m = r;
        if (m == i) break;
        heap_swap(n, i, m); i = m;
    }
}

static double heuristic(int idx, int tx, int tz, int span)
{
    int dx = idx % NAV_SIZE - tx, dz = idx / NAV_SIZE - tz;
    double d = sqrt((double)(dx * dx + dz * dz)) - span;
    return d > 0 ? d : 0.0;
}

static int is_clear(const nav_t *n, int x, int z)
{
    return x >= 0 && z >= 0 && x < NAV_SIZE && z < NAV_SIZE && n->clear[z * NAV_SIZE + x];
}

static int nav_plan_ring_d(nav_t *n, float sx, float sz, float tx, float tz, double ring_min, double ring_max,
                           nav_pt_t *out, int max_out);

int nav_plan(nav_t *n, float sx, float sz, float tx, float tz, nav_pt_t *out, int max_out)
{
    return nav_plan_ring_d(n, sx, sz, tx, tz, NAV_RING_MIN, NAV_RING_MAX, out, max_out);
}

int nav_plan_ring(nav_t *n, float sx, float sz, float tx, float tz, float ring_min, float ring_max,
                  nav_pt_t *out, int max_out)
{
    return nav_plan_ring_d(n, sx, sz, tx, tz, ring_min, ring_max, out, max_out);
}

static int nav_plan_ring_d(nav_t *n, float sx, float sz, float tx, float tz, double ring_min, double ring_max,
                           nav_pt_t *out, int max_out)
{
    if (!n || (max_out > 0 && !out) || max_out < 0) return NAV_ERR_ARG;
    if (!isfinite(sx) || !isfinite(sz) || !isfinite(tx) || !isfinite(tz)) return NAV_ERR_ARG;
    if (!isfinite(ring_min) || !isfinite(ring_max) || ring_min < 0 || ring_max < ring_min || ring_max > NAV_RING_MAX + 1e-5)
        return NAV_ERR_ARG;
    /* mapping.py's fixed [-9, 9] window for the default ring (parity); tighter for smaller rings */
    int span = ring_max >= NAV_RING_MAX - 1e-5 ? RING_SPAN : (int)ceil(ring_max / NAV_RES) + 1;
    if (nav_count(n, NAV_FREE) == 0) return NAV_ERR_NO_MAP;

    /* footprint inflation: the (2r+1)^2 square must be all observed FREE */
    int radius = (int)ceil(n->robot_radius / NAV_RES);
    memset(n->clear, 0, sizeof n->clear);
    memset(n->prefix, 0, sizeof n->prefix);
    for (int z = 0; z < NAV_SIZE; z++) {
        int32_t row = 0;
        for (int x = 0; x < NAV_SIZE; x++) {
            row += n->grid[z * NAV_SIZE + x] == NAV_FREE && !n->tblock[z * NAV_SIZE + x];
            n->prefix[(z + 1) * STRIDE + x + 1] = n->prefix[z * STRIDE + x + 1] + row;
        }
    }
    int32_t required = (2 * radius + 1) * (2 * radius + 1);
    for (int z = radius; z < NAV_SIZE - radius; z++) {
        int top = (z - radius) * STRIDE, bottom = (z + radius + 1) * STRIDE;
        for (int x = radius; x < NAV_SIZE - radius; x++) {
            int left = x - radius, right = x + radius + 1;
            int32_t c = n->prefix[bottom + right] - n->prefix[top + right]
                      - n->prefix[bottom + left] + n->prefix[top + left];
            if (c == required) n->clear[z * NAV_SIZE + x] = 1;
        }
    }

    int s_x, s_z, t_x, t_z;
    if (!nav_world_to_cell(sx, sz, &s_x, &s_z) || !is_clear(n, s_x, s_z)) return NAV_ERR_START_BLOCKED;
    if (!nav_world_to_cell(tx, tz, &t_x, &t_z)) return NAV_ERR_TARGET_OUTSIDE;

    memset(n->goal, 0, sizeof n->goal);
    int n_goals = 0;
    for (int dx = -span; dx <= span; dx++) {
        for (int dz = -span; dz <= span; dz++) {
            int cx = t_x + dx, cz = t_z + dz;
            double d = hypot(centre_x(cx) - (double)tx, centre_z(cz) - (double)tz);
            if (d >= ring_min && d <= ring_max && is_clear(n, cx, cz)) {
                n->goal[cz * NAV_SIZE + cx] = 1;
                n_goals++;
            }
        }
    }
    if (!n_goals) return NAV_ERR_NO_GOAL;

    for (int i = 0; i < N_CELLS; i++) { n->g[i] = INFINITY; n->pos[i] = -1; n->parent[i] = -1; }
    int guard = n->guard_on && n->guard_stop > 0;
    if (guard) {
        memset(n->gmemo, 0, sizeof n->gmemo);
        n->ign_on = ring_min >= 0.3;          /* an object goal: stopping in front of it is docking */
        n->ign_x = tx; n->ign_z = tz; n->ign_r = NAV_GUARD_IGNORE_R;
    }
    int start = s_z * NAV_SIZE + s_x, size = 0, end = -1;
    n->g[start] = 0.0;
    n->f[start] = heuristic(start, t_x, t_z, span);
    n->heap[0] = start; n->pos[start] = 0; size = 1;
    static const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const int DZ[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    while (size > 0) {
        int cur = n->heap[0];
        n->pos[cur] = -1;
        if (--size > 0) { n->heap[0] = n->heap[size]; n->pos[n->heap[0]] = 0; sift_down(n, 0, size); }
        if (n->goal[cur]) { end = cur; break; }
        int x = cur % NAV_SIZE, z = cur / NAV_SIZE;
        for (int k = 0; k < 8; k++) {
            int nx = x + DX[k], nz = z + DZ[k];
            if (!is_clear(n, nx, nz)) continue;
            /* never cut diagonally between blocked or unknown cells */
            if (DX[k] && DZ[k] && (!is_clear(n, x + DX[k], z) || !is_clear(n, x, z + DZ[k]))) continue;
            if (guard && !guard_step_ok(n, x, z, k)) continue;
            double ng = n->g[cur] + (DX[k] && DZ[k] ? sqrt(2.0) : 1.0);
            int nxt = nz * NAV_SIZE + nx;
            if (ng < n->g[nxt]) {
                n->g[nxt] = ng;
                n->f[nxt] = ng + heuristic(nxt, t_x, t_z, span);
                n->parent[nxt] = cur;
                if (n->pos[nxt] < 0) { n->heap[size] = nxt; n->pos[nxt] = size; size++; }
                sift_up(n, n->pos[nxt]);
            }
        }
    }
    if (end < 0) return NAV_ERR_NO_PATH;

    /* route end -> start, then reverse */
    int len = 0;
    for (int c = end; c >= 0; c = (c == start) ? -1 : n->parent[c]) n->route[len++] = c;
    for (int i = 0, j = len - 1; i < j; i++, j--) { int32_t t = n->route[i]; n->route[i] = n->route[j]; n->route[j] = t; }

    /* keep every turn, drop only collinear cells; start (route[0]) excluded from out */
    int corners = 1, n_out = 0;
    for (int pass = 0; pass < 2; pass++) {
        corners = 1; n_out = 0;
        for (int i = 1; i < len; i++) {
            int keep = i == len - 1;
            if (!keep) {
                int a = n->route[i - 1], b = n->route[i], c = n->route[i + 1];
                int bx = b % NAV_SIZE - a % NAV_SIZE, bz = b / NAV_SIZE - a / NAV_SIZE;
                int ax = c % NAV_SIZE - b % NAV_SIZE, az = c / NAV_SIZE - b / NAV_SIZE;
                keep = bx != ax || bz != az;
            }
            if (!keep) continue;
            corners++;
            if (pass == 1) {
                out[n_out].x = (float)centre_x(n->route[i] % NAV_SIZE);
                out[n_out].z = (float)centre_z(n->route[i] / NAV_SIZE);
            }
            n_out++;
        }
        if (pass == 0 && (corners > NAV_MAX_PATH || n_out > max_out)) return NAV_ERR_TOO_LONG;
    }
    return n_out;
}

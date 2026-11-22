/* Host unit tests for iter2-autonomy/goto/src/nav.c: grid marking, A* planner, line of sight,
 * pose source rule, LDR1 parser (+ a deterministic fuzz loop; run this binary
 * under ASan/UBSan with `make test-nav-asan`). */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/nav.h"

static int passed, failed;
#define CHECK(c, ...) do { if (c) passed++; else { failed++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static nav_t *N;

static float cw(int i) { return (float)(-10.0 + (i + 0.5) * 0.1); }   /* cell centre, world */

static void fill(int x0, int z0, int x1, int z1, int v)
{
    for (int z = z0; z < z1; z++) for (int x = x0; x < x1; x++) nav_set_cell(N, x, z, v);
}
static void room(void) { nav_reset(N); nav_set_robot_radius(N, 0.35f); fill(60, 60, 140, 140, NAV_FREE); }

/* every leg between consecutive corners must be observed free */
static int legs_free(float sx, float sz, const nav_pt_t *p, int k)
{
    float x = sx, z = sz;
    for (int i = 0; i < k; i++) {
        if (nav_line_clear_ex(N, x, z, p[i].x, p[i].z, 0) != 1) return 0;
        x = p[i].x; z = p[i].z;
    }
    return 1;
}

/* ---- LDR1 builder ----------------------------------------------------- */
static size_t put32(unsigned char *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; return 4; }
static size_t putf(unsigned char *p, float f) { uint32_t b; memcpy(&b, &f, 4); return put32(p, b); }
static size_t ldr1(unsigned char *buf, int framed, uint32_t flags, float px, float pz, float yaw, float floor_y,
                   const float (*pts)[3], uint32_t count)
{
    unsigned char *p = buf + (framed ? 4 : 0);
    size_t o = 0;
    memcpy(p, "LDR1", 4); o = 4;
    o += put32(p + o, 7);                 /* seq */
    static uint64_t source_us = 1000000;
    source_us += 200000;
    o += put32(p + o, (uint32_t)(source_us >> 32)); o += put32(p + o, (uint32_t)source_us);
    o += put32(p + o, flags);
    o += put32(p + o, count);
    o += putf(p + o, px); o += putf(p + o, 1.0f); o += putf(p + o, pz); o += putf(p + o, yaw);
    o += putf(p + o, floor_y);
    for (uint32_t i = 0; i < count; i++) for (int j = 0; j < 3; j++) o += putf(p + o, pts[i][j]);
    if (framed) { put32(buf, (uint32_t)o); o += 4; }
    return o;
}

/* ---- tests ------------------------------------------------------------ */

static void test_cells(void)
{
    nav_reset(N);
    int ix, iz;
    CHECK(nav_world_to_cell(0.0f, 0.0f, &ix, &iz) && ix == 100 && iz == 100, "origin cell %d %d", ix, iz);
    CHECK(nav_world_to_cell(-10.0f, -10.0f, &ix, &iz) && ix == 0 && iz == 0, "corner");
    CHECK(!nav_world_to_cell(10.0f, 0.0f, NULL, NULL), "x=10 is off map");
    CHECK(!nav_world_to_cell(NAN, 0.0f, NULL, NULL), "nan");
    CHECK(nav_cell(N, 50.0f, 0.0f) == NAV_UNKNOWN, "off map is unknown");

    unsigned v0 = nav_version(N);
    nav_mark_point(N, 0.05f, 0.03f, 0.05f);          /* floor */
    CHECK(nav_cell(N, 0.05f, 0.05f) == NAV_FREE, "floor point -> FREE");
    CHECK(nav_version(N) != v0, "version bumps");
    nav_mark_point(N, 0.05f, 0.5f, 0.05f);           /* obstacle */
    CHECK(nav_cell(N, 0.05f, 0.05f) == NAV_OCC, "band point -> OCC");
    unsigned v1 = nav_version(N);
    nav_mark_point(N, 0.05f, 0.0f, 0.05f);           /* floor never clears an obstacle */
    CHECK(nav_cell(N, 0.05f, 0.05f) == NAV_OCC, "floor doesn't clear OCC");
    CHECK(nav_version(N) == v1, "no change -> no bump");
    nav_mark_point(N, 0.25f, 0.07f, 0.25f);          /* gap between bands */
    nav_mark_point(N, 0.25f, 1.5f, 0.25f);           /* above the band */
    nav_mark_point(N, 0.25f, -0.2f, 0.25f);          /* below the floor */
    CHECK(nav_cell(N, 0.25f, 0.25f) == NAV_UNKNOWN, "out-of-band heights ignored");
    nav_mark_point(N, 0.25f, 0.081f, 0.25f);
    CHECK(nav_cell(N, 0.25f, 0.25f) == NAV_OCC, "just above 0.08 is an obstacle");
}

static void test_carve(void)
{
    nav_reset(N);
    nav_set_cell(N, 105, 100, NAV_OCC);
    nav_carve_ray(N, cw(100), cw(100), cw(110), cw(100));
    CHECK(nav_cell_at(N, 100, 100) == NAV_FREE && nav_cell_at(N, 104, 100) == NAV_FREE, "carve frees unknown");
    CHECK(nav_cell_at(N, 105, 100) == NAV_OCC, "carve never clears OCC");
    CHECK(nav_cell_at(N, 109, 100) == NAV_FREE && nav_cell_at(N, 110, 100) == NAV_UNKNOWN, "hit cell untouched");
    nav_carve_ray(N, 0, 0, 1e30f, 0);                /* absurd input: no crash, no change */
    nav_carve_ray(N, NAN, 0, 1, 1);
    CHECK(nav_cell_at(N, 120, 100) == NAV_UNKNOWN, "absurd ray ignored");
}

static void test_route_around_wall(void)
{
    room();
    fill(99, 60, 101, 125, NAV_OCC);                /* wall, open at the top */
    nav_pt_t p[NAV_MAX_PATH];
    float sx = cw(75), sz = cw(100), tx = cw(125), tz = cw(100);
    int k = nav_plan(N, sx, sz, tx, tz, p, NAV_MAX_PATH);
    CHECK(k > 1, "route around wall, k=%d", k);
    if (k > 0) {
        double d = hypot(p[k - 1].x - tx, p[k - 1].z - tz);
        CHECK(d >= 0.6 - 1e-6 && d <= 0.85 + 1e-6, "ends on the approach ring (%.3f)", d);
        int above = 0;
        for (int i = 0; i < k; i++) above |= p[i].z > cw(125);
        CHECK(above, "goes over the top of the wall");
        CHECK(legs_free(sx, sz, p, k), "every leg is over observed free floor");
    }
    /* too small an output buffer */
    CHECK(nav_plan(N, sx, sz, tx, tz, p, 1) == NAV_ERR_TOO_LONG, "max_out respected");
}

static void test_errors(void)
{
    nav_pt_t p[NAV_MAX_PATH];
    nav_reset(N);
    CHECK(nav_plan(N, 0, 0, 1, 1, p, NAV_MAX_PATH) == NAV_ERR_NO_MAP, "empty map -> NO_MAP");

    room();
    fill(70, 60, 72, 140, NAV_OCC);
    CHECK(nav_plan(N, cw(74), cw(100), cw(120), cw(100), p, NAV_MAX_PATH) == NAV_ERR_START_BLOCKED, "start inside the inflation");
    CHECK(nav_plan(N, 30.0f, 0, cw(120), cw(100), p, NAV_MAX_PATH) == NAV_ERR_START_BLOCKED, "start off map");

    room();
    CHECK(nav_plan(N, cw(100), cw(100), 10.5f, 0, p, NAV_MAX_PATH) == NAV_ERR_TARGET_OUTSIDE, "target outside");

    room(); fill(110, 90, 130, 110, NAV_OCC); fill(115, 95, 125, 105, NAV_FREE);
    CHECK(nav_plan(N, cw(75), cw(100), cw(120), cw(100), p, NAV_MAX_PATH) == NAV_ERR_NO_GOAL, "ring all blocked -> NO_GOAL");

    room(); fill(60, 99, 140, 101, NAV_OCC);
    CHECK(nav_plan(N, cw(100), cw(75), cw(100), cw(125), p, NAV_MAX_PATH) == NAV_ERR_NO_PATH, "sealed wall -> NO_PATH");

    room(); fill(60, 99, 140, 101, NAV_UNKNOWN);     /* never observed: blocks like a wall */
    CHECK(nav_plan(N, cw(100), cw(75), cw(100), cw(125), p, NAV_MAX_PATH) == NAV_ERR_NO_PATH, "unknown strip blocks");
    fill(60, 99, 140, 101, NAV_FREE);
    CHECK(nav_plan(N, cw(100), cw(75), cw(100), cw(125), p, NAV_MAX_PATH) > 0, "observed strip opens the route");

    room();
    CHECK(nav_plan(N, cw(100), cw(100), cw(107), cw(100), p, NAV_MAX_PATH) == 0, "already on the ring -> 0 waypoints");
    CHECK(nav_plan(N, NAN, 0, 0, 0, p, NAV_MAX_PATH) == NAV_ERR_ARG, "nan arg");
}

static void test_inflation(void)
{
    nav_pt_t p[NAV_MAX_PATH];
    /* footprint is 2*ceil(r/0.1)+1 cells: 0.35 m -> 9, 0.2 m -> 5 */
    room(); fill(99, 60, 101, 140, NAV_OCC); fill(99, 97, 101, 106, NAV_FREE);   /* 9-cell door */
    nav_set_robot_radius(N, 0.35f);                  /* 9x9 footprint fits exactly */
    CHECK(nav_plan(N, cw(75), cw(100), cw(125), cw(100), p, NAV_MAX_PATH) > 0, "9-cell door fits 0.35 m");
    fill(99, 97, 101, 98, NAV_OCC);                  /* 8-cell door */
    CHECK(nav_plan(N, cw(75), cw(100), cw(125), cw(100), p, NAV_MAX_PATH) == NAV_ERR_NO_PATH, "8-cell door too narrow for 0.35 m");
    unsigned v = nav_version(N);
    nav_set_robot_radius(N, 0.2f);                   /* 5x5 */
    CHECK(nav_version(N) != v, "radius change bumps version");
    CHECK(nav_plan(N, cw(75), cw(100), cw(125), cw(100), p, NAV_MAX_PATH) > 0, "8-cell door fits 0.2 m");
}

static void test_diagonal(void)
{
    nav_pt_t p[NAV_MAX_PATH];
    nav_reset(N);
    nav_set_robot_radius(N, 0.0f);                   /* 1-cell footprint */
    fill(90, 100, 101, 101, NAV_FREE);               /* row z=100, x 90..100 */
    fill(101, 101, 102, 120, NAV_FREE);              /* column x=101, z 101..119 */
    /* the only link is the diagonal (100,100)->(101,101) between two unknown cells */
    float tx = cw(101), tz = cw(126);
    CHECK(nav_plan(N, cw(92), cw(100), tx, tz, p, NAV_MAX_PATH) == NAV_ERR_NO_PATH, "no diagonal corner-cutting");
    nav_set_cell(N, 101, 100, NAV_OCC);
    CHECK(nav_plan(N, cw(92), cw(100), tx, tz, p, NAV_MAX_PATH) == NAV_ERR_NO_PATH, "not past one occupied side either");
    nav_set_cell(N, 101, 100, NAV_FREE);
    int k = nav_plan(N, cw(92), cw(100), tx, tz, p, NAV_MAX_PATH);
    CHECK(k == 2, "open corner -> L-shaped route, k=%d", k);
    if (k == 2) CHECK(p[0].x == cw(101) && p[0].z == cw(100), "turns at (101,100): %.2f %.2f", p[0].x, p[0].z);
}

static void test_line_clear(void)
{
    room();
    CHECK(nav_line_clear(N, cw(70), cw(100), cw(130), cw(100)) == 1, "clear line");
    nav_set_cell(N, 100, 100, NAV_OCC);
    CHECK(nav_line_clear(N, cw(70), cw(100), cw(130), cw(100)) == 0, "occluded");
    CHECK(nav_line_clear(N, cw(70), cw(110), cw(130), cw(110)) == 1, "parallel line is clear");
    /* the target itself is occupied: cells within 0.3 m of the end are ignored */
    fill(127, 99, 131, 102, NAV_OCC);
    CHECK(nav_line_clear(N, cw(70), cw(120), cw(129), cw(100)) == 1, "object's own cells don't occlude");
    CHECK(nav_line_clear_ex(N, cw(70), cw(120), cw(129), cw(100), 0) == 0, "margin 0 sees them");
    room(); fill(95, 95, 105, 105, NAV_UNKNOWN);
    CHECK(nav_line_clear(N, cw(70), cw(100), cw(130), cw(100)) == -1, "unknown -> -1");
    nav_set_cell(N, 110, 100, NAV_OCC);
    CHECK(nav_line_clear(N, cw(70), cw(100), cw(130), cw(100)) == 0, "OCC beats unknown");
    CHECK(nav_line_clear(N, cw(70), cw(130), 50.0f, cw(130)) == -1, "off map -> -1");
    CHECK(nav_line_clear(N, NAN, 0, 0, 0) == -1, "nan -> -1");
}

static void test_pose(void)
{
    float x, z, yaw;
    nav_reset(N);
    CHECK(nav_get_pose(N, &x, &z, &yaw) == NAV_POSE_NONE, "no pose yet");
    nav_set_pose(N, 1.0f, 2.0f, 0.5f, 1);
    CHECK(nav_get_pose(N, &x, &z, &yaw) == NAV_POSE_ARKIT && x == 1.0f && z == 2.0f, "ARKit while tracking");
    nav_set_pose(N, 9.0f, 9.0f, 0, 0);
    CHECK(nav_get_pose(N, &x, &z, &yaw) == NAV_POSE_STALE && x == 1.0f, "lost, no DR -> last ARKit, stale");

    /* ARKit facing +x (yaw pi/2) at (1,2); DR frame facing +z (yaw 0) at (0,0) */
    nav_reset(N);
    nav_note_dr(N, 0, 0, 0);
    nav_set_pose(N, 1.0f, 2.0f, (float)(M_PI / 2), 1);
    nav_set_pose(N, 0, 0, 0, 0);                     /* tracking lost */
    nav_note_dr(N, 0, 0.5f, 0);                      /* DR: drove 0.5 m forward */
    CHECK(nav_get_pose(N, &x, &z, &yaw) == NAV_POSE_DR, "DR fallback");
    CHECK(fabsf(x - 1.5f) < 1e-5f && fabsf(z - 2.0f) < 1e-5f, "forward in DR = +x in ARKit: %f %f", x, z);
    CHECK(fabsf(yaw - (float)(M_PI / 2)) < 1e-5f, "yaw %f", yaw);
    nav_note_dr(N, 0, 0.5f, (float)(M_PI / 2));      /* turned right 90 deg */
    nav_get_pose(N, &x, &z, &yaw);
    CHECK(fabsf(fabsf(yaw) - (float)M_PI) < 1e-5f, "yaw wraps to pi: %f", yaw);
    nav_set_pose(N, 3.0f, 3.0f, 0, 1);               /* tracking back */
    CHECK(nav_get_pose(N, &x, &z, &yaw) == NAV_POSE_ARKIT && x == 3.0f, "ARKit wins when back");
}

static void test_ldr1(void)
{
    static unsigned char buf[4 + NAV_LDR1_MAX_BODY + 64];
    float pts[4][3] = {
        { 0.05f, 0.52f, 0.05f },   /* floor y=0.5 -> floor point */
        { 0.25f, 1.00f, 0.05f },   /* 0.5 above floor -> OCC */
        { 0.45f, 2.50f, 0.05f },   /* 2 m above: ignored */
        { 30.0f, 0.50f, 0.05f },   /* off map: ignored */
    };
    nav_reset(N);
    size_t len = ldr1(buf, 0, 7, 0.1f, 0.2f, 0.3f, 0.5f, pts, 4);
    CHECK(len == NAV_LDR1_HEADER + 48, "builder size");
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_MAPPED, "good packet");
    CHECK(nav_cell(N, 0.05f, 0.05f) == NAV_FREE && nav_cell(N, 0.25f, 0.05f) == NAV_OCC
          && nav_cell(N, 0.45f, 0.05f) == NAV_UNKNOWN, "points mapped by height above floor");
    float x, z, yaw;
    CHECK(nav_get_pose(N, &x, &z, &yaw) == NAV_POSE_ARKIT && x == 0.1f && z == 0.2f && yaw == 0.3f, "pose from packet");

    len = ldr1(buf, 1, 7, 0.1f, 0.2f, 0.3f, 0.5f, pts, 4);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_MAPPED, "length-prefixed packet");
    CHECK(nav_ingest_ldr1(N, buf, len - 1) == NAV_LDR1_BAD, "prefix mismatch");

    len = ldr1(buf, 0, 7, 0, 0, 0, 0.5f, pts, 4);
    CHECK(nav_ingest_ldr1(N, buf, len - 1) == NAV_LDR1_BAD, "truncated");
    CHECK(nav_ingest_ldr1(N, buf, 20) == NAV_LDR1_BAD, "shorter than header");
    CHECK(nav_ingest_ldr1(N, buf, len + 12) == NAV_LDR1_BAD, "trailing bytes");
    CHECK(nav_ingest_ldr1(N, NULL, 10) == NAV_LDR1_BAD, "null");

    put32(buf + 20, NAV_LDR1_MAX_POINTS + 1);      /* oversized count */
    CHECK(nav_ingest_ldr1(N, buf, NAV_LDR1_HEADER + (NAV_LDR1_MAX_POINTS + 1) * 12) == NAV_LDR1_BAD, "count > 1024");
    put32(buf + 20, 0xffffffffu);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_BAD, "count overflow");

    len = ldr1(buf, 0, 15, 0, 0, 0, 0.5f, pts, 4);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_BAD, "unknown flag bits");
    len = ldr1(buf, 0, 7, 0, 0, 0, 0.5f, pts, 4); buf[0] = 'X';
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_BAD, "bad magic");
    len = ldr1(buf, 0, 7, 0, 0, 0, 0.5f, pts, 4); put32(buf + 8, 0); put32(buf + 12, 0);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_BAD, "uptime 0");

    /* NaN in the last point rejects the whole packet before any cell changes */
    nav_reset(N);
    float bad[2][3] = { { 1.05f, 0.5f, 1.05f }, { 1.25f, NAN, 1.05f } };
    len = ldr1(buf, 0, 7, 0, 0, 0, 0.5f, bad, 2);
    unsigned v = nav_version(N);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_BAD && nav_version(N) == v && nav_count(N, NAV_FREE) == 0, "NaN rejected atomically");
    float big[1][3] = { { 1.05f, 2000.0f, 1.05f } };
    len = ldr1(buf, 0, 7, 0, 0, 0, 0.5f, big, 1);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_BAD, "|v| > 1000 rejected");

    /* tracking/floor not OK: valid, pose flagged, no mapping */
    nav_reset(N);
    len = ldr1(buf, 0, 1 | 4, 0, 0, 0, 0.5f, pts, 4);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_SKIPPED && nav_count(N, NAV_FREE) == 0, "no floor -> skipped");
    len = ldr1(buf, 0, 2, 5, 5, 0, 0.5f, pts, 4);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_SKIPPED, "no tracking -> skipped");
    CHECK(nav_get_pose(N, &x, &z, &yaw) == NAV_POSE_STALE, "tracking lost -> stale pose");

    /* empty packet is valid */
    len = ldr1(buf, 0, 7, 0, 0, 0, 0.5f, pts, 0);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_MAPPED, "zero points ok");

    /* floor recalibration > 0.1 m clears the grid */
    nav_reset(N);
    len = ldr1(buf, 0, 7, 0, 0, 0, 0.5f, pts, 2);
    nav_ingest_ldr1(N, buf, len);
    CHECK(nav_count(N, NAV_OCC) == 1, "mapped before floor change");
    len = ldr1(buf, 0, 7, 0, 0, 0, 0.75f, pts, 0);
    nav_ingest_ldr1(N, buf, len);
    CHECK(nav_count(N, NAV_OCC) == 0 && nav_count(N, NAV_FREE) == 0, "floor jump cleared the grid");

    /* max size */
    static float many[NAV_LDR1_MAX_POINTS][3];
    for (int i = 0; i < NAV_LDR1_MAX_POINTS; i++) { many[i][0] = (i % 32) * 0.1f; many[i][1] = 0.5f; many[i][2] = (i / 32) * 0.1f; }
    len = ldr1(buf, 1, 7, 0, 0, 0, 0.5f, (const float (*)[3])many, NAV_LDR1_MAX_POINTS);
    CHECK(nav_ingest_ldr1(N, buf, len) == NAV_LDR1_MAPPED, "1024 points, framed");
}

/* Dynamic occupancy: distinct whole frames, bounded gaps, real elapsed time. */
static void test_recovery(void)
{
    unsigned char b[NAV_LDR1_MAX_BODY];
    float floor_pts[80][3];
    for (int i = 0; i < 80; i++) {
        floor_pts[i][0] = cw(100); floor_pts[i][1] = 0; floor_pts[i][2] = cw(60 + i);
    }
    room();
    nav_set_robot_radius(N, 0.1f);
    fill(100, 60, 101, 140, NAV_OCC);
    nav_pt_t route[NAV_MAX_PATH];
    CHECK(nav_plan(N, cw(90), cw(100), cw(115), cw(100), route, NAV_MAX_PATH) < 0,
          "temporary wall blocks A*");
    size_t len = ldr1(b, 0, 7, 0, 0, 0, 0, floor_pts, 80);
    nav_ingest_ldr1_at(N, b, len, 1000000);
    for (int i = 0; i < 20; i++)
        CHECK(nav_ingest_ldr1_at(N, b, len, 1100000 + i * 100000) == NAV_LDR1_OLD,
              "replayed packet not clearance evidence");
    CHECK(nav_cell_at(N, 100, 100) == NAV_OCC, "replay/silence cannot clear a wall");
    for (int i = 0; i < 5; i++) {
        len = ldr1(b, 0, 7, 0, 0, 0, 0, floor_pts, 80);
        nav_ingest_ldr1_at(N, b, len, 4000000 + i * 150000);
        if (i < 4) CHECK(nav_cell_at(N, 100, 100) == NAV_OCC, "hysteresis retains obstacle");
    }
    CHECK(nav_cell_at(N, 100, 100) == NAV_FREE, "five floor frames over 600ms clear temporary wall");
    CHECK(nav_plan(N, cw(90), cw(100), cw(115), cw(100), route, NAV_MAX_PATH) > 0,
          "A* route reopens using the recovered grid");
    float both[2][3] = {{cw(100), 0, cw(100)}, {cw(100), .5f, cw(100)}};
    len = ldr1(b, 0, 7, 0, 0, 0, 0, both, 2);
    nav_ingest_ldr1_at(N, b, len, 5000000);
    CHECK(nav_cell_at(N, 100, 100) == NAV_OCC, "new obstacle blocks immediately even with floor in frame");
    float one[1][3] = {{cw(100), 0, cw(100)}};
    for (int i = 0; i < 8; i++) {
        len = ldr1(b, 0, 7, 0, 0, 0, 0, one, 1);
        nav_ingest_ldr1_at(N, b, len, 5100000 + i);  /* burst, not elapsed observations */
    }
    CHECK(nav_cell_at(N, 100, 100) == NAV_OCC, "backlog burst cannot clear obstacle");
    for (int i = 0; i < 5; i++) {
        len = ldr1(b, 0, 7, 0, 0, 0, 0, one, 1);
        nav_ingest_ldr1_at(N, b, len, 6000000 + i * 600000); /* gaps too long */
    }
    CHECK(nav_cell_at(N, 100, 100) == NAV_OCC, "disconnected observations cannot accumulate");
    for (int i = 0; i < 4; i++) {
        len = ldr1(b, 0, 7, 0, 0, 0, 0, one, 1);
        nav_ingest_ldr1_at(N, b, len, 10000000 + i * 150000);
    }
    len = ldr1(b, 0, 0, 0, 0, 0, 0, one, 1);
    nav_ingest_ldr1_at(N, b, len, 10600000);
    len = ldr1(b, 0, 7, 0, 0, 0, 0, one, 1);
    nav_ingest_ldr1_at(N, b, len, 10750000);
    CHECK(nav_cell_at(N, 100, 100) == NAV_OCC, "tracking loss cancels pending recovery");
    for (int i = 0; i < 8; i++) {
        len = ldr1(b, 0, 7, 0, 0, 0, 0, both, 2);
        nav_ingest_ldr1_at(N, b, len, 11000000 + i * 150000);
    }
    CHECK(nav_cell_at(N, 100, 100) == NAV_OCC, "persistent person/wall is not cleared by adjacent floor");
}

/* deterministic fuzz: mutated good packets + random bytes; must never crash
 * (ASan/UBSan build) and must only ever return a documented code */
static uint64_t rng = 0x9e3779b97f4a7c15ull;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)rng; }
static void test_fuzz(int iters)
{
    static unsigned char good[4 + NAV_LDR1_MAX_BODY], buf[4 + NAV_LDR1_MAX_BODY + 16];
    static float pts[64][3];
    int bad_rc = 0, mapped = 0;
    nav_reset(N);
    for (int it = 0; it < iters; it++) {
        int cnt = rnd() % 64;
        for (int i = 0; i < cnt; i++) {
            pts[i][0] = (float)((int)(rnd() % 4000) - 2000) / 100.0f;
            pts[i][1] = (float)((int)(rnd() % 300) - 50) / 100.0f;
            pts[i][2] = (float)((int)(rnd() % 4000) - 2000) / 100.0f;
        }
        size_t len = ldr1(good, rnd() & 1, rnd() & 7, 0, 0, 0, 0, pts, (uint32_t)cnt);
        memcpy(buf, good, len);
        int muts = rnd() % 4;
        for (int m = 0; m < muts; m++) {
            switch (rnd() % 4) {
            case 0: buf[rnd() % (len ? len : 1)] ^= (unsigned char)(1u << (rnd() % 8)); break;
            case 1: buf[rnd() % (len ? len : 1)] = (unsigned char)rnd(); break;
            case 2: len = rnd() % (len + 1); break;
            case 3: { size_t extra = rnd() % 16; for (size_t k = 0; k < extra; k++) buf[len + k] = (unsigned char)rnd(); len += extra; } break;
            }
        }
        if (rnd() % 10 == 0) { len = rnd() % sizeof buf; for (size_t k = 0; k < len; k++) buf[k] = (unsigned char)rnd(); }
        /* exact-size heap copy so ASan catches any read past len */
        unsigned char *h = malloc(len ? len : 1);
        memcpy(h, buf, len);
        int rc = nav_ingest_ldr1(N, h, len);
        free(h);
        if (rc != NAV_LDR1_MAPPED && rc != NAV_LDR1_SKIPPED && rc != NAV_LDR1_OLD && rc != NAV_LDR1_BAD) bad_rc++;
        mapped += rc == NAV_LDR1_MAPPED;
        if (it % 5000 == 0) {        /* plan on whatever the fuzz produced */
            nav_pt_t p[NAV_MAX_PATH];
            nav_set_robot_radius(N, 0.1f);
            int k = nav_plan(N, 0, 0, (float)((int)(rnd() % 20) - 10), 1.0f, p, NAV_MAX_PATH);
            if (k < NAV_ERR_ARG || k > NAV_MAX_PATH) bad_rc++;
            nav_line_clear(N, 0, 0, (float)((int)(rnd() % 40) - 20), 3.0f);
        }
    }
    CHECK(bad_rc == 0, "fuzz: only documented return codes (%d bad)", bad_rc);
    CHECK(mapped > 0, "fuzz: some packets still valid (%d)", mapped);
    printf("fuzz: %d LDR1 packets, %d mapped\n", iters, mapped);
}

static void test_big_map(void)
{
    /* the whole 20 x 20 m map free: corner to corner, worst-case search */
    nav_reset(N);
    nav_set_robot_radius(N, 0.35f);
    fill(0, 0, NAV_SIZE, NAV_SIZE, NAV_FREE);
    for (int z = 20; z < 180; z += 20) fill(z % 40 ? 10 : 30, z, z % 40 ? 170 : 190, z + 2, NAV_OCC);   /* serpentine */
    nav_pt_t p[NAV_MAX_PATH];
    int k = nav_plan(N, cw(8), cw(8), cw(190), cw(192), p, NAV_MAX_PATH);
    CHECK(k > 8, "serpentine route, k=%d", k);
    if (k > 0) CHECK(legs_free(cw(8), cw(8), p, k), "serpentine legs free");
}


/* cost-to-go field (goto's in-place recovery): agrees with A*, descends to the ring, honours
 * temp marks, the guard box and the error cases */
static void test_field(void)
{
    room();
    fill(99, 60, 101, 125, NAV_OCC);                /* wall, open at the top */
    nav_pt_t p[NAV_MAX_PATH], q[NAV_MAX_PATH];
    float sx = cw(75), sz = cw(100), tx = cw(125), tz = cw(100);
    int k = nav_plan(N, sx, sz, tx, tz, p, NAV_MAX_PATH);
    double alen = 0; float ax = sx, az = sz;
    for (int i = 0; i < k; i++) { alen += hypot(p[i].x - ax, p[i].z - az); ax = p[i].x; az = p[i].z; }
    int r = nav_field_build(N, tx, tz, (float)NAV_RING_MIN, (float)NAV_RING_MAX);
    CHECK(r > 100, "field reaches many cells (%d)", r);
    double c = nav_field_cost(N, sx, sz);
    CHECK(isfinite(c) && fabs(c * NAV_RES - alen) < 0.05, "cost-to-go = A* length (%.2f vs %.2f m)", c * NAV_RES, alen);
    int unk = -1, m = nav_field_path(N, sx, sz, q, NAV_MAX_PATH, &unk);
    CHECK(m > 1 && unk == 0, "descent route (%d)", m);
    if (m > 0) {
        double d = hypot(q[m - 1].x - tx, q[m - 1].z - tz);
        CHECK(d >= 0.6 - 1e-6 && d <= 0.85 + 1e-6, "descent ends on the ring (%.3f)", d);
        CHECK(legs_free(sx, sz, q, m), "descent legs over observed free floor");
        CHECK(fabs(nav_field_cost(N, q[m - 1].x, q[m - 1].z)) < 1e-9, "ring cost 0");
    }
    CHECK(nav_field_clear(N, sx, sz) == 1 && nav_field_clear(N, cw(100), cw(100)) == 0, "field clear flags");
    CHECK(!isfinite(nav_field_cost(N, cw(100), cw(100))), "a wall cell can't reach the goal");
    /* a plan invalidates the field */
    nav_plan(N, sx, sz, tx, tz, p, NAV_MAX_PATH);
    CHECK(!isfinite(nav_field_cost(N, sx, sz)) && nav_field_path(N, sx, sz, q, NAV_MAX_PATH, NULL) == NAV_ERR_ARG,
          "nav_plan invalidates the field");
    /* temp marks close the gap over the wall: no route; cleared: back */
    for (int x = 95; x < 106; x++) for (int z = 125; z < 140; z++) nav_temp_block(N, cw(x), cw(z));
    nav_field_build(N, tx, tz, (float)NAV_RING_MIN, (float)NAV_RING_MAX);
    CHECK(!isfinite(nav_field_cost(N, sx, sz)), "marks block the only way: unreachable");
    CHECK(nav_field_path(N, sx, sz, q, NAV_MAX_PATH, NULL) == NAV_ERR_NO_PATH, "no descent");
    nav_temp_clear(N);
    /* goal enclosed in a wall box: NO_GOAL */
    fill(120, 95, 131, 106, NAV_OCC);
    CHECK(nav_field_build(N, tx, tz, (float)NAV_RING_MIN, (float)NAV_RING_MAX) == NAV_ERR_NO_GOAL, "goal ring all inflated: NO_GOAL");
    room();
    /* guard box on: the field from right next to a wall facing it is never cheaper than A* */
    nav_set_guard_box(N, 0.6f, 0.45f, 60.0f);
    fill(99, 60, 101, 125, NAV_OCC);
    k = nav_plan(N, sx, sz, tx, tz, p, NAV_MAX_PATH);
    alen = 0; ax = sx; az = sz;
    for (int i = 0; i < k; i++) { alen += hypot(p[i].x - ax, p[i].z - az); ax = p[i].x; az = p[i].z; }
    nav_field_build(N, tx, tz, (float)NAV_RING_MIN, (float)NAV_RING_MAX);
    c = nav_field_cost(N, sx, sz);
    CHECK(k > 0 && isfinite(c) && fabs(c * NAV_RES - alen) < 0.05, "guard-aware: field = A* (%.2f vs %.2f)", c * NAV_RES, alen);
    nav_set_guard_box(N, 0, 0, 0);
    /* bad args */
    CHECK(nav_field_build(NULL, 0, 0, 0.6f, 0.85f) == NAV_ERR_ARG && nav_field_build(N, NAN, 0, 0.6f, 0.85f) == NAV_ERR_ARG &&
          nav_field_build(N, 0, 0, 0.9f, 0.6f) == NAV_ERR_ARG, "bad args");
    nav_reset(N);
    CHECK(nav_field_build(N, 0, 0, 0.6f, 0.85f) == NAV_ERR_NO_MAP, "empty map");
}

/* nav_foot_cells: a corridor 6 free cells wide (walls 2 cells thick, as noisy LiDAR draws them).
 * Radius 0.27 -> ceil = 3 cells -> needs 7: closed. nav_set_foot_cells(2): open. 0: back to closed. */
static void test_foot_cells(void)
{
    nav_t *n = nav_new();
    nav_set_robot_radius(n, 0.27f);
    CHECK(nav_foot_cells(n) == 3, "0.27 m -> 3 cells (%d)", nav_foot_cells(n));
    for (int iz = 70; iz < 130; iz++)
        for (int ix = 70; ix < 130; ix++) nav_set_cell(n, ix, iz, NAV_FREE);
    /* a wall across z = 100..101 except a gap of 6 cells at x 97..102 */
    for (int ix = 70; ix < 130; ix++)
        if (ix < 97 || ix > 102) { nav_set_cell(n, ix, 100, NAV_OCC); nav_set_cell(n, ix, 101, NAV_OCC); }
    nav_pt_t out[64];
    float sx = -0.05f + 0.1f * 0, sz = -1.5f, tx = 0.0f, tz = 1.8f;   /* through the gap to past it */
    int k = nav_plan(n, sx, sz, tx, tz, out, 64);
    CHECK(k == NAV_ERR_NO_PATH, "6-cell gap closed at 3 cells (%d)", k);
    unsigned v = nav_version(n);
    nav_set_foot_cells(n, 2);
    CHECK(nav_foot_cells(n) == 2 && nav_version(n) != v, "foot cells 2, version bumped");
    k = nav_plan(n, sx, sz, tx, tz, out, 64);
    CHECK(k > 0, "6-cell gap open at 2 cells (%d)", k);
    nav_set_foot_cells(n, 0);
    CHECK(nav_foot_cells(n) == 3, "0 = back to ceil(radius)");
    CHECK(nav_plan(n, sx, sz, tx, tz, out, 64) == NAV_ERR_NO_PATH, "closed again");
    nav_set_foot_cells(n, -1);
    nav_set_foot_cells(n, 11);
    CHECK(nav_foot_cells(n) == 3, "out-of-range values ignored");
    nav_free(n);
}

/* Heading-aware A* (nav_set_turn_cost): fewer corners on an open diagonal, the forward route over a
 * U-turn when the two are near-equal, nav_route_cost. Off (0, 0) must leave plans as they were. */
static int count_corners(int k) { return k - 1; }   /* corners between legs (the last point is the goal) */
static void test_turn_cost(void)
{
    nav_pt_t a[256], b[256];
    /* open room, goal off-axis: 8-connected A* has many equal-length routes */
    room();
    nav_set_turn_cost(N, 0, 0, 0, 0);
    int ka = nav_plan_ring(N, cw(70), cw(70), cw(100), cw(82), 0.0f, 0.11f, a, 256);
    nav_set_turn_cost(N, 0.3f, 0.0f, 0, 0);
    int kb = nav_plan_ring(N, cw(70), cw(70), cw(100), cw(82), 0.0f, 0.11f, b, 256);
    nav_set_turn_cost(N, 0, 0, 0, 0);
    double la = 0, lb = 0;
    { float x = cw(70), z = cw(70); for (int i = 0; i < ka; i++) { la += hypot(a[i].x - x, a[i].z - z); x = a[i].x; z = a[i].z; } }
    { float x = cw(70), z = cw(70); for (int i = 0; i < kb; i++) { lb += hypot(b[i].x - x, b[i].z - z); x = b[i].x; z = b[i].z; } }
    CHECK(ka > 0 && kb > 0 && count_corners(kb) <= 1 && count_corners(kb) <= count_corners(ka),
          "turn cost: open diagonal in <= 1 corner (off: %d corners, on: %d)", count_corners(ka), count_corners(kb));
    CHECK(fabs(la - lb) < 0.01, "turn cost: same length (%.3f vs %.3f m)", la, lb);
    CHECK(legs_free(cw(70), cw(70), b, kb), "turn cost: legs over free floor");

    /* forward vs U-turn. Robot at x=100 facing +z (yaw 0), goal 3 m to +x, a wall between them from
     * z=86 to z=114: around its top (+z, ahead) is 0.2 m longer than around its bottom (behind). */
    room();
    fill(108, 86, 111, 115, NAV_OCC);
    nav_set_turn_cost(N, 0, 0, 0, 0);
    int k0 = nav_plan(N, cw(100), cw(99), cw(130), cw(99), a, 256);
    CHECK(k0 > 0 && a[0].z < cw(99), "turn cost off: the shorter way, behind the robot (first corner z %.2f)", k0 > 0 ? a[0].z : 0.0f);
    nav_set_turn_cost(N, 0.3f, 0.5f, 0.0f, 1);
    int k1 = nav_plan(N, cw(100), cw(99), cw(130), cw(99), b, 256);
    CHECK(k1 > 0 && b[0].z > cw(99), "start turn cost: forward (+z) over a U-turn when near-equal (first corner z %.2f)",
          k1 > 0 ? b[0].z : 0.0f);
    nav_set_turn_cost(N, 0.3f, 0.5f, (float)M_PI, 1);       /* facing -z: the short way is ahead now */
    k1 = nav_plan(N, cw(100), cw(99), cw(130), cw(99), b, 256);
    CHECK(k1 > 0 && b[0].z < cw(99), "start turn cost, facing -z: the shorter way (first corner z %.2f)", k1 > 0 ? b[0].z : 0.0f);
    /* a much shorter way behind still wins: the penalty is a tie-breaker, not a ban */
    fill(108, 86, 111, 125, NAV_OCC);                        /* top end now 1.1 m further away */
    nav_set_turn_cost(N, 0.3f, 0.5f, 0.0f, 1);
    k1 = nav_plan(N, cw(100), cw(99), cw(130), cw(99), b, 256);
    CHECK(k1 > 0 && b[0].z < cw(99), "start turn cost: a clearly shorter way behind still wins (first corner z %.2f)",
          k1 > 0 ? b[0].z : 0.0f);

    /* nav_route_cost */
    room();
    nav_set_turn_cost(N, 0, 0, 0, 0);
    nav_pt_t r[2] = {{cw(90), cw(80)}, {cw(90), cw(90)}};
    double c = nav_route_cost(N, cw(80), cw(80), r, 2);
    CHECK(fabs(c - 20.0) < 0.2, "route cost: 2 x 1 m legs = 20 cells (%.2f)", c);
    nav_set_turn_cost(N, 0.3f, 0.5f, 0.0f, 1);               /* facing +z: first leg is +x (90), then a 90 corner */
    double ct = nav_route_cost(N, cw(80), cw(80), r, 2);
    CHECK(fabs(ct - (20.0 + 5.0 + 3.0)) < 0.2, "route cost with turns: 20 + 5 (start 90) + 3 (corner) (%.2f)", ct);
    fill(85, 75, 86, 86, NAV_OCC);
    nav_set_turn_cost(N, 0, 0, 0, 0);
    CHECK(isinf(nav_route_cost(N, cw(80), cw(80), r, 2)), "route cost: a wall on a leg = INFINITY");
    nav_reset(N);
}

int main(int argc, char **argv)
{
    int iters = argc > 1 ? atoi(argv[1]) : 200000;
    N = nav_new();
    if (!N) { printf("nav_new failed\n"); return 1; }
    test_cells();
    test_carve();
    test_route_around_wall();
    test_errors();
    test_inflation();
    test_diagonal();
    test_line_clear();
    test_pose();
    test_ldr1();
    test_recovery();
    test_field();
    test_big_map();
    test_foot_cells();
    test_turn_cost();
    test_fuzz(iters);
    nav_free(N);
    printf("test_nav: %d/%d passed\n", passed, passed + failed);
    return failed ? 1 : 0;
}

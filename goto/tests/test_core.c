/* Host unit tests for goto_core: objects/aliases, JSON parser, config and
 * the state machine driven by scripted detection streams (fake clock). */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/goto_core.h"

static int passed, failed;
#define CHECK(c, ...) do { if (c) passed++; else { failed++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static goto_objects O;
static goto_cfg C;
static goto_sm S;
static double T;

/* --- helpers ------------------------------------------------------------ */
static int count(goto_act_kind k) { int n = 0; for (int i = 0; i < S.n_act; i++) n += S.act[i].kind == k; return n; }
static const goto_act *find(goto_act_kind k) { for (int i = 0; i < S.n_act; i++) if (S.act[i].kind == k) return &S.act[i]; return NULL; }
static int has_event(const char *prefix) { for (int i = 0; i < S.n_act; i++) if (S.act[i].kind == ACT_EVENT && !strncmp(S.act[i].text, prefix, strlen(prefix))) return 1; return 0; }
static void clear(void) { S.n_act = 0; }

/* frame with one box of `label`, centre x (0..1 of width), height frac, bottom frac */
static void mkframe(goto_frame *f, const char *label, double cx, double hfrac, double bottom)
{
    memset(f, 0, sizeof *f);
    f->w = 640; f->h = 480;
    if (!label) return;
    f->n = 1;
    snprintf(f->d[0].label, sizeof f->d[0].label, "%s", label);
    f->d[0].conf = 0.8;
    double bw = 80, bh = hfrac * 480, y2 = bottom * 480;
    f->d[0].box[0] = cx * 640 - bw / 2; f->d[0].box[2] = cx * 640 + bw / 2;
    f->d[0].box[1] = y2 - bh; f->d[0].box[3] = y2;
}
static void frame(const char *label, double cx, double hfrac, double bottom)
{
    goto_frame f;
    mkframe(&f, label, cx, hfrac, bottom);
    goto_sm_frame(&S, T, &f);
}
static double bearing_x(double cx) { return atan((cx - 0.5) * 2 * tan(M_PI / 6)) * 180 / M_PI; }
static void advance(double dt) { double end = T + dt; while (T < end - 1e-9) { T += 0.02; goto_sm_tick(&S, T); } }
static void reset_sm(void) { goto_sm_init(&S, &C, &O); T = 100.0; clear(); }
static void start(const char *what) { goto_sm_goto(&S, T, goto_objects_resolve(&O, what)); }
/* acknowledge the TURN in the queue and finish it */
static double ack_turn(int id)
{
    const goto_act *a = find(ACT_TURN);
    double deg = a ? a->a : NAN;
    clear();
    goto_sm_move_started(&S, T, id, 0.8);
    advance(0.5);
    goto_sm_move_status(&S, T, id, "running", "-");
    advance(0.4);
    goto_sm_move_status(&S, T, id, "done", "-");
    return deg;
}

/* --- tests -------------------------------------------------------------- */
static void test_objects(void)
{
    goto_objects_defaults(&O);
    goto_objects F;
    CHECK(goto_objects_load(&F, "objects.txt") == 0 || goto_objects_load(&F, "../objects.txt") == 0, "objects.txt loads");
    CHECK(F.n_obj == O.n_obj && F.n_alias == O.n_alias, "built-in default matches objects.txt (%d/%d vs %d/%d)",
          O.n_obj, O.n_alias, F.n_obj, F.n_alias);
    for (int i = 0; i < O.n_alias && i < F.n_alias; i++)
        if (strcmp(O.alias[i], F.alias[i])) { CHECK(0, "alias %d differs: %s vs %s", i, O.alias[i], F.alias[i]); break; }
    struct { const char *in, *want; } cases[] = {
        {"trash can", "trash can"}, {"the trash can", "trash can"}, {"Bin.", "trash can"},
        {"the garbage", "trash can"}, {"go to the big trash can over there", "trash can"},
        {"trash cans", "trash can"}, {"Garbage-Can", "trash can"}, {"recycling bin", "trash can"},
        {"me", "person"}, {"chairs", "chair"}, {"dining table", "table"}, {"bottle", "water bottle"},
        {"my backpack", "backpack"}, {"boxes", "box"}, {"the door please", "door"}, {"a mug", "cup"},
        {"that desk", "table"}, {"tars go to the laptop", "laptop"},
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        int r = goto_objects_resolve(&O, cases[i].in);
        CHECK(r >= 0 && !strcmp(O.obj[r], cases[i].want), "resolve '%s' -> %s (want %s)", cases[i].in,
              r >= 0 ? O.obj[r] : "-", cases[i].want);
    }
    const char *unk[] = {"", "the", "banana", "pizza", "spaceship", "binary", NULL};
    for (int i = 0; unk[i]; i++) CHECK(goto_objects_resolve(&O, unk[i]) < 0, "'%s' is unknown", unk[i]);
}

static void test_json(void)
{
    goto_frame f;
    CHECK(goto_parse_frame("{\"t\": 12.5, \"w\":640, \"h\":480, \"dets\":[{\"label\":\"trash can\",\"conf\":0.71,\"box\":[10,20,110.5,220]}]}", &f) == 0, "parse basic");
    CHECK(f.n == 1 && f.w == 640 && f.h == 480 && fabs(f.t - 12.5) < 1e-9, "fields");
    CHECK(!strcmp(f.d[0].label, "trash can") && fabs(f.d[0].conf - 0.71) < 1e-9 && fabs(f.d[0].box[2] - 110.5) < 1e-9, "det fields");
    CHECK(goto_parse_frame("{\"w\":320,\"h\":240,\"dets\":[]}", &f) == 0 && f.n == 0, "empty dets");
    CHECK(goto_parse_frame("{\"src\":\"yolo\",\"meta\":{\"a\":[1,2,{\"b\":null}],\"ok\":true},\"w\":320,\"h\":240,"
                           "\"dets\":[{\"cls\":3,\"label\":\"cup\",\"conf\":5e-1,\"box\":[1,2,3,4],\"track_id\":7}]}", &f) == 0
          && f.n == 1 && !strcmp(f.d[0].label, "cup") && fabs(f.d[0].conf - 0.5) < 1e-9, "extra keys skipped");
    CHECK(goto_parse_frame("{\"w\":320,\"h\":240,\"dets\":[{\"label\":\"a\\\"b\",\"box\":[1,2,3,4]}]}", &f) == 0
          && !strcmp(f.d[0].label, "a\"b") && f.d[0].conf == 1.0, "escapes, default conf");
    CHECK(goto_parse_frame("{\"w\":320,\"h\":240,\"dets\":[{\"label\":\"x\"}]}", &f) == 0 && f.n == 0, "det without box skipped");
    CHECK(goto_parse_frame("{\"w\":320,\"h\":240}", &f) < 0, "no dets -> invalid");
    CHECK(goto_parse_frame("{\"h\":240,\"dets\":[]}", &f) < 0, "no w -> invalid");
    CHECK(goto_parse_frame("not json", &f) < 0, "garbage");
    CHECK(goto_parse_frame("{\"w\":320,\"h\":240,\"dets\":[{\"label\":\"x\",\"box\":[1,2,3]}]}", &f) < 0, "short box");
    CHECK(goto_parse_frame("{\"w\":320,\"h\":240,\"dets\":[", &f) < 0, "truncated");
    CHECK(goto_parse_frame("{\"w\":1e80,\"h\":240,\"dets\":[]}", &f) < 0, "absurd w rejected");
    CHECK(goto_parse_frame("{\"w\":320,\"h\":240,\"dets\":[{\"label\":\"x\",\"box\":[1,2,1e999,4]}]}", &f) == 0 && f.n == 0,
          "non-finite box dropped");
}

static void test_cfg(void)
{
    const char *p = "build/test.conf";
    FILE *f = fopen(p, "w");
    fprintf(f, "# test\nhfov_deg = 70\nlook_frames=3 # inline\narrive_h.person = 0.9\narrive_h.trash can = 0.5\nbogus = 1\ntick_hz = 2\n");
    fclose(f);
    goto_cfg c;
    goto_cfg_defaults(&c);
    char err[64];
    CHECK(goto_cfg_load(&c, p, err, sizeof err) == 0, "cfg loads");
    CHECK(c.hfov_deg == 70 && c.look_frames == 3, "values");
    CHECK(strstr(err, "bogus") != NULL, "unknown key reported (%s)", err);
    CHECK(c.tick_hz == 5, "tick_hz clamped to >= 5 (watchdog)");
    CHECK(goto_cfg_arrive_h(&c, "person") == 0.9 && goto_cfg_arrive_h(&c, "trash can") == 0.5
          && goto_cfg_arrive_h(&c, "chair") == c.arrive_h_frac, "per-object arrive overrides");
    CHECK(goto_cfg_load(&c, "build/nope.conf", err, sizeof err) < 0, "missing file -> -1");
    goto_cfg d;
    goto_cfg_defaults(&d);
    CHECK(goto_cfg_load(&d, "goto.conf", err, sizeof err) == 0 || goto_cfg_load(&d, "../goto.conf", err, sizeof err) == 0, "goto/goto.conf loads");
    CHECK(!err[0], "goto.conf has no unknown keys (%s)", err);
    goto_cfg_defaults(&c);
    CHECK(memcmp(&c, &d, offsetof(goto_cfg, n_arrive_over)) == 0, "goto.conf matches the built-in defaults");
}

static void test_visible_at_start(void)
{
    reset_sm();
    start("trash can");
    CHECK(S.st == ST_LOOK && count(ACT_BEEP) == 1 && has_event("searching"), "GOTO -> look first, beep, event");
    clear();
    frame("trash can", 0.56, 0.15, 0.6);
    CHECK(S.st == ST_LOOK, "one hit isn't enough");
    frame("trash can", 0.56, 0.15, 0.6);
    CHECK(S.st == ST_APPROACH, "confirmed small bearing -> approach directly (no turn)");
    clear();
    advance(0.11);
    const goto_act *d = find(ACT_DRIVE);
    CHECK(d && d->a > 0.5 && d->b > 0.5, "drives forward (%.2f %.2f)", d ? d->a : 0, d ? d->b : 0);
    CHECK(d && d->a > d->b, "target right of centre -> left wheel faster");
    /* a second of driving: D at >= 5 Hz */
    clear();
    for (int i = 0; i < 10; i++) { advance(0.1); frame("trash can", 0.5, 0.2, 0.7); }
    CHECK(count(ACT_DRIVE) >= 8, "D refreshed at ~10 Hz while approaching (%d in 1 s)", count(ACT_DRIVE));
    clear();
    frame("trash can", 0.5, 0.75, 0.9);
    CHECK(S.st == ST_APPROACH, "arrive needs 2 frames");
    frame("trash can", 0.5, 0.76, 0.9);
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived") && count(ACT_STOP) == 1 && count(ACT_BEEP) == 1
          && has_event("arrived trash can"), "arrived: S, BEEP 1, event");
    clear();
    advance(1.0);
    CHECK(S.n_act == 0, "idle sends nothing (teleop isn't fought)");
}

static void test_search_and_align(void)
{
    reset_sm();
    start("bin");
    clear();
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    const goto_act *t = find(ACT_TURN);
    CHECK(S.st == ST_SEARCH_TURN && t && t->a == 30, "4 empty frames -> TURN 30");
    /* frames during the turn are ignored; keepalive D 0 0 goes out */
    clear();
    goto_sm_move_started(&S, T, 7, 0.8);
    frame("trash can", 0.5, 0.2, 0.6);
    frame("trash can", 0.5, 0.2, 0.6);
    advance(0.5);
    CHECK(S.st == ST_SEARCH_TURN, "frames during the turn don't count");
    const goto_act *k = find(ACT_DRIVE);
    CHECK(k && k->a == 0 && k->b == 0, "D 0 0 keepalive during TURN");
    goto_sm_move_status(&S, T, 6, "done", "-");
    CHECK(S.st == ST_SEARCH_TURN, "status of another move id is ignored");
    goto_sm_move_status(&S, T, 7, "done", "-");
    CHECK(S.st == ST_LOOK, "TURN done -> look");
    /* frames inside the settle window don't count */
    clear();
    frame("trash can", 0.9, 0.2, 0.6);
    frame("trash can", 0.9, 0.2, 0.6);
    CHECK(S.st == ST_LOOK && S.look_seen == 0, "settle window ignores frames");
    advance(0.4);
    frame("trash can", 0.9, 0.2, 0.6);
    frame("trash can", 0.9, 0.2, 0.6);
    t = find(ACT_TURN);
    /* x = 0.8 -> atan(0.8 * tan 30) = 24.8 deg, * 0.9 gain = 22.3 */
    CHECK(S.st == ST_ALIGN_TURN && t && fabs(t->a - 22.3) < 0.3, "big bearing -> exact TURN %.1f", t ? t->a : 0);
    ack_turn(8);
    advance(0.4);
    frame("trash can", 0.48, 0.2, 0.6);
    frame("trash can", 0.48, 0.2, 0.6);
    CHECK(S.st == ST_APPROACH, "aligned -> approach");
    t = find(ACT_TURN);
    CHECK(!t, "no turn for a small bearing");
}

static void test_left_target_and_lost(void)
{
    reset_sm();
    start("chair");
    clear();
    advance(0.1);
    frame("chair", 0.2, 0.2, 0.6);
    frame("chair", 0.2, 0.2, 0.6);
    const goto_act *t = find(ACT_TURN);
    CHECK(t && t->a < -15, "target on the left -> negative TURN (%.1f)", t ? t->a : 0);
    ack_turn(1);
    advance(0.4);
    frame("chair", 0.47, 0.2, 0.6);
    frame("chair", 0.47, 0.2, 0.6);
    CHECK(S.st == ST_APPROACH, "approach");
    clear();
    /* frames keep coming but the chair is gone */
    for (int i = 0; i < 10; i++) { advance(0.1); frame("person", 0.5, 0.5, 0.9); }
    CHECK(S.st == ST_APPROACH, "still approaching after 1.0 s");
    clear();
    for (int i = 0; i < 4; i++) { advance(0.1); frame(NULL, 0, 0, 0); }
    CHECK(S.st == ST_LOOK && count(ACT_STOP) == 1 && has_event("lost"), "lost after 1.2 s -> S + look");
    CHECK(S.search_dir == -1, "searches toward where it was last seen (left)");
    clear();
    advance(0.4);
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    t = find(ACT_TURN);
    CHECK(t && t->a == -30, "search turns left first");
}

static void test_give_up(void)
{
    reset_sm();
    start("door");
    int turns = 0;
    for (int step = 0; step < 20 && S.st != ST_IDLE; step++) {
        clear();
        for (int i = 0; i < 4 && S.st == ST_LOOK; i++) { advance(0.12); frame("chair", 0.5, 0.3, 0.6); }
        if (S.st == ST_SEARCH_TURN) { turns++; ack_turn(100 + step); advance(0.4); }
    }
    CHECK(S.st == ST_IDLE && turns == 12, "full circle (12 x 30) then give up (turns=%d)", turns);
    CHECK(!strcmp(S.result, "failed:not_found") && count(ACT_STOP) >= 1 && find(ACT_BEEP) && find(ACT_BEEP)->a == 3,
          "not found: S + BEEP 3 (%s)", S.result);
}

static void test_cancel_and_failures(void)
{
    reset_sm();
    start("cup");
    clear();
    goto_sm_cancel(&S, T, "cancel");
    CHECK(S.st == ST_IDLE && S.n_act >= 1 && S.act[0].kind == ACT_STOP, "CANCEL: S first");
    clear();
    goto_sm_cancel(&S, T, "cancel");
    CHECK(count(ACT_STOP) == 1, "CANCEL when idle still sends S (stop always wins)");

    reset_sm();
    start("cup");
    clear();
    advance(3.1);
    CHECK(S.st == ST_IDLE && strstr(S.result, "no_camera"), "no frames for 3 s -> fail (%s)", S.result);

    reset_sm();
    start("cup");
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    clear();
    goto_sm_move_started(&S, T, 3, 0.7);
    advance(0.3);
    goto_sm_move_status(&S, T, 3, "aborted", "obstacle");
    CHECK(S.st == ST_IDLE && strstr(S.result, "obstacle"), "TURN aborted (obstacle) -> fail (%s)", S.result);

    reset_sm();
    start("cup");
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    goto_sm_move_started(&S, T, 4, 0.7);
    goto_sm_move_status(&S, T, 4, "aborted", "stop");
    CHECK(S.st == ST_IDLE && strstr(S.result, "cancelled"), "TURN aborted by someone's S -> cancelled (%s)", S.result);

    reset_sm();
    start("cup");
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    goto_sm_move_started(&S, T, 5, 0.7);
    clear();
    for (int i = 0; i < 10; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    advance(4.0);
    CHECK(S.st == ST_IDLE && strstr(S.result, "timed_out"), "TURN that never finishes -> fail (%s)", S.result);

    reset_sm();
    C.timeout_s = 5;
    goto_sm_init(&S, &C, &O);
    start("cup");
    for (int i = 0; i < 60 && S.st != ST_IDLE; i++) { advance(0.1); frame("cup", 0.5, 0.1, 0.5); }
    CHECK(S.st == ST_IDLE && strstr(S.result, "timeout"), "overall timeout (%s)", S.result);
    goto_cfg_defaults(&C);
}

static void test_retarget_and_filters(void)
{
    reset_sm();
    start("cup");
    advance(0.1);
    frame("cup", 0.5, 0.2, 0.6);
    frame("cup", 0.5, 0.2, 0.6);
    CHECK(S.st == ST_APPROACH, "approaching the cup");
    clear();
    start("chair");
    CHECK(S.st == ST_LOOK && S.n_act && S.act[0].kind == ACT_STOP && !strcmp(O.obj[S.target], "chair"),
          "new GOTO while driving: S, then search for the new object");
    /* low confidence and COCO label mapping */
    reset_sm();
    start("table");
    advance(0.1);
    goto_frame f;
    mkframe(&f, "dining table", 0.5, 0.2, 0.6);
    f.d[0].conf = 0.2;
    goto_sm_frame(&S, T, &f);
    goto_sm_frame(&S, T, &f);
    CHECK(S.look_hits == 0, "below min_conf ignored");
    f.d[0].conf = 0.6;
    goto_sm_frame(&S, T, &f);
    goto_sm_frame(&S, T, &f);
    CHECK(S.st == ST_APPROACH, "COCO 'dining table' counts as table");
    /* highest confidence of two boxes */
    reset_sm();
    start("cup");
    advance(0.1);
    mkframe(&f, "cup", 0.1, 0.2, 0.6);
    f.n = 2;
    f.d[1] = f.d[0];
    f.d[0].conf = 0.4;
    f.d[1].conf = 0.9;
    f.d[1].box[0] = 0.5 * 640 - 40; f.d[1].box[2] = 0.5 * 640 + 40;
    goto_sm_frame(&S, T, &f);
    goto_sm_frame(&S, T, &f);
    CHECK(S.st == ST_APPROACH, "picks the confident centred cup, not the left one");
    /* person needs a much bigger box */
    reset_sm();
    start("me");
    advance(0.1);
    frame("person", 0.5, 0.5, 0.8);
    frame("person", 0.5, 0.5, 0.8);
    CHECK(S.st == ST_APPROACH, "person at 50%% height: keep going");
    frame("person", 0.5, 0.9, 1.0);
    frame("person", 0.5, 0.9, 1.0);
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived"), "person at 90%%: arrived");
    /* bottom edge rule */
    reset_sm();
    start("box");
    advance(0.1);
    frame("box", 0.5, 0.2, 0.99);
    frame("box", 0.5, 0.2, 0.99);
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived"), "box touching the frame bottom: arrived (already there)");
    /* steering saturation and re-align while driving */
    reset_sm();
    start("cup");
    advance(0.1);
    frame("cup", 0.5, 0.1, 0.5);
    frame("cup", 0.5, 0.1, 0.5);
    clear();
    for (int i = 0; i < 6; i++) { advance(0.1); frame("cup", 0.95, 0.1, 0.5); }
    CHECK(S.st == ST_ALIGN_TURN && find(ACT_STOP) && find(ACT_TURN) && find(ACT_TURN)->a > 15,
          "target drifts far right while driving -> S + exact TURN");
}

/* --- object memory ------------------------------------------------------ */
static goto_mem M;
static void pose(double x, double y, double h) { goto_sm_pose(&S, T, 7, x, y, h, 0); }
static void reset_mem_sm(void) { reset_sm(); goto_mem_init(&M); goto_sm_set_memory(&S, &M, 1000.0); pose(0, 0, 0); }
static int count_turns_until_idle(double *first)
{
    int turns = 0;
    for (int step = 0; step < 30 && S.st != ST_IDLE; step++) {
        for (int i = 0; i < 4 && S.st == ST_LOOK; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
        if (S.st == ST_SEARCH_TURN || S.st == ST_MEM_TURN) {
            double d = ack_turn(200 + step);
            if (!turns && first) *first = d;
            turns++;
            advance(0.4);
        } else if (S.st == ST_MEM_MOVE) { ack_turn(300 + step); advance(0.4); }
    }
    return turns;
}

static void test_memory_observe(void)
{
    reset_mem_sm();
    CHECK(M.have_pose && M.px == 0 && M.ph == 0, "pose from tars_motor");
    /* idle: frames are still remembered. cx 0.75 -> x 0.5 -> bearing atan(0.5 tan 30) = 16.1 deg;
     * bottom 0.6 of 480 = 48 px below centre, f = 554 px, camera 120 mm up, level:
     * 1385.6 mm ahead, range 1442 mm */
    frame("trash can", 0.75, 0.35, 0.6);
    int i = goto_mem_find(&M, "trash can");
    CHECK(i >= 0 && M.n == 1, "idle frame remembered (n=%d)", M.n);
    if (i >= 0) {
        goto_mem_entry *e = &M.e[i];
        CHECK(fabs(e->bearing - 16.1) < 0.2 && fabs(e->dist - 1442) < 3, "bearing %.1f dist %.0f (floor contact)", e->bearing, e->dist);
        CHECK(fabs(e->x - 1386) < 3 && fabs(e->y - 400) < 3, "world position %.0f,%.0f (1386,400)", e->x, e->y);
        CHECK(e->id == 1 && e->status == MEM_FRESH, "instance #1, fresh");
        CHECK(e->seen == 1 && fabs(e->last_seen - (1000 + T)) < 1e-6 && M.dirty, "seen/last_seen/dirty");
    }
    goto_frame f;
    mkframe(&f, "dining table", 0.5, 0.2, 0.6);
    f.d[0].conf = 0.35;                     /* below memory_min_conf 0.40 */
    goto_sm_frame(&S, T, &f);
    CHECK(goto_mem_find(&M, "table") < 0, "weak detection not remembered");
    f.d[0].conf = 0.7;
    goto_sm_frame(&S, T, &f);
    CHECK(goto_mem_find(&M, "table") >= 0 && goto_mem_find(&M, "dining table") < 0, "COCO label stored as canonical 'table'");
    mkframe(&f, "frisbee", 0.5, 0.2, 0.6);
    goto_sm_frame(&S, T, &f);
    CHECK(goto_mem_find(&M, "frisbee") >= 0, "labels objects.txt doesn't know are kept too");
    /* two cups in one frame, far apart: two instances */
    mkframe(&f, "cup", 0.1, 0.2, 0.6);
    f.n = 2; f.d[1] = f.d[0]; f.d[0].conf = 0.5; f.d[1].conf = 0.9;
    f.d[1].box[0] = 0.5 * 640 - 40; f.d[1].box[2] = 0.5 * 640 + 40;
    goto_sm_frame(&S, T, &f);
    CHECK(goto_mem_count(&M, "cup") == 2 && goto_mem_find_id(&M, "cup", 2) >= 0, "two cups in a frame -> cup #1 and cup #2");
    /* the same cup twice (overlapping boxes, NMS leftover): one */
    mkframe(&f, "mug", 0.9, 0.2, 0.6);
    f.n = 2; f.d[1] = f.d[0]; f.d[1].conf = 0.6; f.d[1].box[0] += 10; f.d[1].box[2] += 10;
    goto_sm_frame(&S, T, &f);
    CHECK(goto_mem_count(&M, "cup") == 3, "overlapping duplicate boxes -> one new instance, not two (%d)", goto_mem_count(&M, "cup"));
    /* turning: nothing remembered */
    int n = M.e[goto_mem_find(&M, "trash can")].seen;
    goto_sm_pose(&S, T, 7, 0, 0, 0, 1);
    frame("trash can", 0.5, 0.35, 0.6);
    CHECK(M.e[goto_mem_find(&M, "trash can")].seen == n, "pose says moving -> frame not remembered");
    goto_sm_pose(&S, T, 7, 0, 0, 90, 0);
    advance(10);
    frame("trash can", 0.5, 0.35, 0.6);     /* now straight ahead at hdg 90 */
    i = goto_mem_find_id(&M, "trash can", 2);
    CHECK(i >= 0 && fabs(M.e[i].bearing - 90) < 0.5 && M.e[i].seen == 1 && M.e[goto_mem_find(&M, "trash can")].seen == n,
          "heading 90 -> a different place: trash can #2 at world bearing 90");
    /* the same can seen again from 600 mm closer: same instance */
    reset_mem_sm();
    frame("trash can", 0.5, 0.2, 0.6);              /* 1386 mm ahead */
    pose(600, 0, 0);
    advance(1);
    frame("trash can", 0.5, 0.2, 0.68);             /* closer: bottom lower in the frame */
    CHECK(goto_mem_count(&M, "trash can") == 1 && M.e[0].seen == 2, "same can from closer -> still one instance");
    /* noisy range along the same ray: same instance */
    pose(0, 0, 0);
    frame("trash can", 0.5, 0.2, 0.57);             /* range estimate ~2.0 m vs ~1.4 m, same bearing */
    CHECK(goto_mem_count(&M, "trash can") == 1, "same ray, range off by 45%% -> same instance");
    /* a second can elsewhere */
    pose(0, 0, 90);
    frame("trash can", 0.5, 0.2, 0.6);
    CHECK(goto_mem_count(&M, "trash can") == 2, "a can to the right -> trash can #2");
    char dd[2000];
    goto_mem_describe(&M, 1000 + T, dd, sizeof dd);
    CHECK(strstr(dd, "; trash can#1 fresh x=") && strstr(dd, "; trash can#2 fresh x=") && strstr(dd, "est_dist="), "describe: %.120s", dd);
    /* no pose (old tars_motor): nothing */
    goto_mem_init(&M);
    frame("trash can", 0.5, 0.35, 0.6);
    CHECK(M.n == 0, "no pose from tars_motor -> no memory");
    /* full: oldest replaced */
    reset_mem_sm();
    char lab[32];
    for (int k = 0; k < GOTO_MEM_MAX + 3; k++) {
        advance(0.1);
        snprintf(lab, sizeof lab, "thing%d", k);
        frame(lab, 0.5, 0.3, 0.6);
    }
    CHECK(M.n == GOTO_MEM_MAX && goto_mem_find(&M, "thing0") < 0 && goto_mem_find(&M, "thing34") >= 0, "full: oldest replaced");
    CHECK(goto_mem_forget(&M, "thing34") == 1 && goto_mem_find(&M, "thing34") < 0 && M.n == GOTO_MEM_MAX - 1, "FORGET one");
    CHECK(goto_mem_forget(&M, "nothing") == 0, "FORGET unknown -> 0");
    CHECK(goto_mem_forget(&M, NULL) == GOTO_MEM_MAX - 1 && M.n == 0, "FORGET all");
}

static void test_memory_goto_hit(void)
{
    reset_mem_sm();
    /* earlier: trash can seen behind-left: world bearing -150, 1.5 m */
    pose(0, 0, -150);
    frame("trash can", 0.5, 0.257, 0.6);
    pose(0, 0, 0);
    clear();
    advance(1.0);
    start("bin");
    const goto_act *t = find(ACT_TURN);
    CHECK(S.st == ST_MEM_TURN && t && fabs(t->a + 150) < 1 && has_event("remembered trash can"),
          "remembered: first an exact TURN to its bearing (%.1f)", t ? t->a : 0);
    CHECK(count(ACT_BEEP) == 1 && S.search_dir == -1, "beeps once, searches left if needed");
    ack_turn(1);
    pose(0, 0, -150);
    advance(0.4);
    frame("trash can", 0.52, 0.26, 0.6);
    frame("trash can", 0.52, 0.26, 0.6);
    CHECK(S.st == ST_APPROACH && S.mem_phase == 4, "there it is -> approach (mem=hit)");
    char d[600];
    goto_sm_describe(&S, d, sizeof d);
    CHECK(strstr(d, "mem=hit") && strstr(d, "mode=goto"), "describe: %s", d);
    for (int i = 0; i < 4 && S.st != ST_IDLE; i++) { advance(0.1); frame("trash can", 0.5, 0.75, 0.9); }
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived"), "arrived (%s)", S.result);
    int i = goto_mem_find(&M, "trash can");
    CHECK(i >= 0 && fabs(M.e[i].dist - 550) < 1 && fabs(M.e[i].bearing + 150) < 2, "arrival refines the entry (dist %.0f)", M.e[i].dist);

    /* small bearing: no turn, look straight away */
    reset_mem_sm();
    frame("chair", 0.55, 0.3, 0.6);
    clear();
    start("chair");
    CHECK(S.st == ST_LOOK && !find(ACT_TURN) && S.mem_phase == 1, "remembered almost straight ahead -> just look");

    /* remembered spot from another place: bearing from where the robot is NOW */
    reset_mem_sm();
    frame("box", 0.5, 0.35, 0.6);           /* 1386 mm straight ahead */
    goto_sm_pose(&S, T, 7, 1386, -1386, 0, 0); /* robot drove 1.39 m forward, 1.39 m left */
    clear();
    start("box");
    t = find(ACT_TURN);
    CHECK(t && fabs(t->a - 90) < 1, "object is now to the right: TURN 90 (%.1f)", t ? t->a : 0);
}

static void test_memory_goto_miss(void)
{
    reset_mem_sm();
    pose(0, 0, 120);
    frame("backpack", 0.5, 0.3, 0.6);
    pose(0, 0, 0);
    clear();
    start("backpack");
    CHECK(S.st == ST_MEM_TURN && fabs(find(ACT_TURN)->a - 120) < 1, "turn to remembered backpack");
    ack_turn(1);
    pose(0, 0, 120);
    advance(0.4);
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    const goto_act *t = find(ACT_TURN);
    CHECK(S.st == ST_SEARCH_TURN && t && t->a == 30 && has_event("not_there") && S.mem_phase == 3,
          "not there -> normal search, starting toward the remembered side");
    int turns = count_turns_until_idle(NULL);   /* includes the queued first one */
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:not_found") && turns == 12, "full circle then give up (%d turns)", turns);
    int i = goto_mem_find(&M, "backpack");
    CHECK(i >= 0 && M.e[i].status == MEM_STALE, "not there -> instance STALE");
    clear();
    pose(0, 0, 0);
    start("backpack");
    CHECK(S.st == ST_MEM_TURN && S.mem_phase == 1, "a STALE instance is still tried (nothing better)");
    ack_turn(50);
    pose(0, 0, 120);
    advance(0.4);
    for (int k = 0; k < 4; k++) { advance(0.12); frame(NULL, 0, 0, 0); }
    CHECK(M.e[goto_mem_find(&M, "backpack")].status == MEM_GONE && S.st == ST_SEARCH_TURN, "missed again -> GONE, search");
    goto_sm_cancel(&S, T, "x");
    clear();
    start("backpack");
    CHECK(S.st == ST_LOOK && !find(ACT_TURN) && S.mem_phase == 0, "GONE instance is never used");
    goto_sm_cancel(&S, T, "x");
    frame("backpack", 0.5, 0.3, 0.6);
    CHECK(goto_mem_count(&M, "backpack") == 1 && M.e[goto_mem_find(&M, "backpack")].status == MEM_FRESH,
          "seen again where it was -> FRESH again");
    pose(0, 0, -60);
    frame("backpack", 0.5, 0.3, 0.6);
    CHECK(goto_mem_count(&M, "backpack") == 2, "seen somewhere else -> backpack #2");

    /* two instances: nearest fresh first; not there -> the other one */
    reset_mem_sm();
    pose(0, 0, 90);
    frame("chair", 0.5, 0.2, 0.6);           /* chair #1: 1386 mm to the right */
    pose(0, 0, -90);
    frame("chair", 0.5, 0.2, 0.52);          /* chair #2: farther, to the left */
    pose(0, 0, 0);
    CHECK(goto_mem_count(&M, "chair") == 2, "two chairs remembered");
    clear();
    start("chair");
    const goto_act *t2 = find(ACT_TURN);
    CHECK(S.st == ST_MEM_TURN && S.mem_id == 1 && t2 && fabs(t2->a - 90) < 1 && has_event("remembered chair #1"),
          "nearest instance first (chair #1, TURN 90)");
    ack_turn(60);
    pose(0, 0, 90);
    advance(0.4);
    for (int k = 0; k < 4; k++) { advance(0.12); frame(NULL, 0, 0, 0); }
    t2 = find(ACT_TURN);
    CHECK(S.st == ST_MEM_TURN && S.mem_id == 2 && t2 && fabs(t2->a - 180) < 1 && has_event("not_there chair #1")
          && has_event("remembered chair #2"), "chair #1 not there -> straight to chair #2 (TURN %.0f)", t2 ? t2->a : 0);
    CHECK(M.e[goto_mem_find_id(&M, "chair", 1)].status == MEM_STALE, "chair #1 now STALE");
    ack_turn(61);
    pose(0, 0, -90);
    advance(0.4);
    frame("chair", 0.5, 0.2, 0.52);
    frame("chair", 0.5, 0.2, 0.52);
    CHECK(S.st == ST_APPROACH && S.mem_phase == 4, "found at chair #2 -> approach (mem=hit)");
    goto_sm_cancel(&S, T, "x");
    CHECK(goto_mem_forget(&M, "chair#2") == 1 && goto_mem_count(&M, "chair") == 1 && goto_mem_find_id(&M, "chair", 1) >= 0,
          "FORGET chair#2 forgets just that one");
    CHECK(goto_mem_forget(&M, "chair #9") == 0 && goto_mem_forget(&M, "chair") == 1, "FORGET chair forgets all chairs");

    /* too old */
    reset_mem_sm();
    pose(0, 0, 90);
    frame("door", 0.5, 0.3, 0.6);
    pose(0, 0, 0);
    advance(10);
    T += C.memory_max_age_s;
    clear();
    start("door");
    CHECK(S.st == ST_LOOK && !find(ACT_TURN), "older than memory_max_age_s -> ignored, blind search");
    /* clock went backwards (Pi without RTC) */
    reset_mem_sm();
    pose(0, 0, 90);
    frame("door", 0.5, 0.3, 0.6);
    pose(0, 0, 0);
    goto_sm_set_memory(&S, &M, -1e6);
    clear();
    start("door");
    CHECK(S.st == ST_MEM_TURN, "wall clock went backwards -> age 0, still used");

    /* memory_drive_mm: get closer before searching */
    C.memory_drive_mm = 300;
    reset_mem_sm();
    frame("laptop", 0.5, 0.15, 0.6);        /* 550*0.7/0.15 = 2567 mm ahead */
    clear();
    start("laptop");
    CHECK(S.st == ST_LOOK, "straight ahead: look");
    for (int k = 0; k < 4; k++) { advance(0.12); frame(NULL, 0, 0, 0); }
    const goto_act *mv = find(ACT_MOVE);
    CHECK(S.st == ST_MEM_MOVE && mv && mv->a == 300, "not in view -> MOVE 300 toward the remembered spot");
    ack_turn(5);
    CHECK(S.st == ST_LOOK, "MOVE done -> look again");
    advance(0.4);
    for (int k = 0; k < 4; k++) { advance(0.12); frame(NULL, 0, 0, 0); }
    CHECK(S.st == ST_SEARCH_TURN && S.mem_phase == 3, "still nothing -> search");
    reset_mem_sm();
    frame("laptop", 0.5, 0.15, 0.6);
    start("laptop");
    for (int k = 0; k < 4; k++) { advance(0.12); frame(NULL, 0, 0, 0); }
    goto_sm_move_started(&S, T, 9, 7.0);
    goto_sm_move_status(&S, T, 9, "aborted", "obstacle");
    CHECK(S.st == ST_IDLE && strstr(S.result, "move_obstacle"), "MOVE blocked -> fail (%s)", S.result);
    C.memory_drive_mm = 0;
    goto_cfg_defaults(&C);
}

static void test_memory_scan(void)
{
    reset_mem_sm();
    goto_sm_scan(&S, T);
    CHECK(S.st == ST_LOOK && S.mode == MODE_SCAN && has_event("scanning"), "SCAN: look first");
    int turns = 0;
    double h = 0;
    for (int step = 0; step < 20 && S.st != ST_IDLE; step++) {
        clear();
        for (int i = 0; i < 4 && S.st == ST_LOOK; i++) {
            advance(0.12);
            if (step == 0) frame("chair", 0.3, 0.3, 0.6);
            else if (step == 5) frame("bin", 0.6, 0.2, 0.6);
            else if (step == 7) frame("person", 0.5, 0.6, 0.9);
            else frame(NULL, 0, 0, 0);
        }
        if (S.st == ST_SEARCH_TURN) {
            const goto_act *t = find(ACT_TURN);
            CHECK(t && t->a == 30, "scan step is TURN 30");
            turns++;
            ack_turn(400 + step);
            h += 30;
            pose(0, 0, h);
            advance(0.4);
        }
    }
    CHECK(S.st == ST_IDLE && turns == 12 && !strcmp(S.result, "scanned:3"), "12 turns then done (%d, %s)", turns, S.result);
    CHECK(has_event("scanned chair,trash can,person"), "EVENT scanned chair,trash can,person");
    int i = goto_mem_find(&M, "trash can");
    CHECK(M.n == 3 && i >= 0 && fabs(M.e[i].bearing - 150 - bearing_x(0.6)) < 1, "scan filled memory (trash can at %.0f)", i >= 0 ? M.e[i].bearing : 0);
    clear();
    advance(1.0);
    CHECK(S.n_act == 0, "idle after scan");
    /* cancel mid-scan */
    goto_sm_scan(&S, T);
    for (int k = 0; k < 4; k++) { advance(0.12); frame(NULL, 0, 0, 0); }
    clear();
    goto_sm_cancel(&S, T, "stop");
    CHECK(S.st == ST_IDLE && S.mode == MODE_GOTO && S.act[0].kind == ACT_STOP, "CANCEL during scan: S, idle");
    /* empty room */
    reset_mem_sm();
    goto_sm_scan(&S, T);
    turns = count_turns_until_idle(NULL);
    CHECK(turns == 12 && has_event("scanned nothing"), "empty room: EVENT scanned nothing");
}

static void test_memory_file(void)
{
    reset_mem_sm();
    goto_sm_pose(&S, T, 7, 250, -40, 33, 0);
    frame("trash can", 0.7, 0.3, 0.6);
    frame("dining table", 0.2, 0.2, 0.6);
    M.e[1].status = MEM_STALE;
    const char *p = "build/mem_test.txt";
    remove(p);
    CHECK(goto_mem_save(&M, p, 1234.5) == 0 && !M.dirty, "save");
    FILE *f = fopen("build/mem_test.txt.tmp", "r");
    CHECK(!f, "no .tmp left behind");
    if (f) fclose(f);
    goto_mem L;
    CHECK(goto_mem_load(&L, p) == 0, "load");
    CHECK(L.n == 2 && !strcmp(L.e[0].label, "trash can") && !strcmp(L.e[1].label, "table") && L.e[1].status == MEM_STALE
          && L.e[0].status == MEM_FRESH && L.e[0].id == 1, "entries round-trip (labels with spaces, id, status)");
    CHECK(fabs(L.e[0].x - M.e[0].x) < 0.1 && fabs(L.e[0].y - M.e[0].y) < 0.1 && fabs(L.e[0].bearing - M.e[0].bearing) < 0.01
          && L.e[0].seen == 1 && fabs(L.e[0].last_seen - M.e[0].last_seen) < 0.1, "values round-trip");
    CHECK(L.have_pose && L.epoch == 7 && fabs(L.px - 250) < 0.1 && fabs(L.ph - 33) < 0.01, "pose + epoch saved");
    /* goto restarted, same tars_motor (same epoch): keep the offset */
    goto_mem_motor_pose(&L, 7, 260, -40, 33);
    CHECK(L.rebased == 0 && fabs(L.px - 260) < 0.1, "same epoch -> no re-base");
    /* tars_motor restarted (pose 0,0,0, new epoch), robot not moved: re-base */
    goto_mem_motor_pose(&L, 99, 0, 0, 0);
    CHECK(L.rebased == 1 && fabs(L.px - 260) < 0.1 && fabs(L.py + 40) < 0.1 && fabs(L.ph - 33) < 0.01,
          "new epoch -> same world pose (%.0f,%.0f,%.1f)", L.px, L.py, L.ph);
    goto_mem_motor_pose(&L, 99, 100, 0, 0);  /* 100 mm forward in the new motor frame */
    CHECK(fabs(L.px - (260 + 100 * cos(33 * M_PI / 180))) < 0.1 && fabs(L.py - (-40 + 100 * sin(33 * M_PI / 180))) < 0.1
          && fabs(L.ph - 33) < 0.01, "motion after the re-base lands in the old frame");
    double b, d;
    goto_mem_relative(&L, 0, &b, &d);
    double b0, d0;
    goto_mem_relative(&M, 0, &b0, &d0);
    CHECK(d < d0 && fabs(b - b0) < 10, "relative bearing still sensible after re-base (%.0f vs %.0f)", b, b0);
    /* memory loaded at startup is used for a GOTO */
    reset_sm();
    goto_mem_load(&M, p);
    goto_sm_set_memory(&S, &M, 1000.0);
    goto_sm_pose(&S, T, 7, 250, -40, 33, 0);
    clear();
    start("trash can");
    CHECK(S.mem_phase == 1, "memory from the file used by GOTO");

    /* missing / corrupt files */
    CHECK(goto_mem_load(&L, "build/nope_mem.txt") == -1 && L.n == 0 && !L.have_pose, "missing file -> empty, -1");
    f = fopen(p, "w");
    fprintf(f, "v 2\npose 7 1 2 3 0 0 0 9\ninst 1 fresh 1 2 3 400 0.5 2 100 cup\ninst 1 fresh nan 2 3 4 0.5 1 1 chair\n"
               "inst 1 fresh 1 2\ninst 2 fresh 1 2 3 4 0.5 1 1\n\x01\x02\xff garbage\npose x\ninst 1 fresh 1e300 2 3 4 0.5 1 1 box\n"
               "inst 1 maybe 1 2 3 4 0.5 1 1 box\ninst 0 fresh 1 2 3 4 0.5 1 1 box\ninst 1 fresh 1 2 3 400 0.5 2 100 cup\n"
               "inst 3 gone 5 6 7 400 0.6 1 100 water bottle\n");
    fclose(f);
    CHECK(goto_mem_load(&L, p) == 1 && L.n == 2 && !strcmp(L.e[1].label, "water bottle") && L.e[1].status == MEM_GONE
          && L.e[1].id == 3 && L.have_pose, "corrupt lines / bad status / id 0 / duplicate id skipped, good ones kept (n=%d)", L.n);
    f = fopen(p, "w");
    fprintf(f, "inst 1 fresh 1 2 3 400 0.5 2 100 cup\n");
    fclose(f);
    CHECK(goto_mem_load(&L, p) == 1 && L.n == 0, "no version line -> ignored");
    f = fopen(p, "w");
    fprintf(f, "v 1\nobj 1 2 3 400 0.5 2 100 0 cup\n");
    fclose(f);
    CHECK(goto_mem_load(&L, p) == 1 && L.n == 0, "unknown version -> ignored");
    /* truncations of a good file never crash and never invent entries */
    reset_mem_sm();
    for (int k = 0; k < 6; k++) { char l[16]; snprintf(l, sizeof l, "obj%d", k); frame(l, 0.1 + k * 0.15, 0.3, 0.6); }
    goto_mem_save(&M, p, 1.0);
    f = fopen(p, "rb");
    char buf[4096];
    size_t len = fread(buf, 1, sizeof buf, f);
    fclose(f);
    int ok = 1;
    srand(3);
    for (size_t cut = 0; cut < len; cut += 3) {
        f = fopen(p, "wb");
        fwrite(buf, 1, cut, f);
        if (cut % 2) { char junk = (char)(rand() & 0xff); fwrite(&junk, 1, 1, f); }
        fclose(f);
        int r = goto_mem_load(&L, p);
        if (L.n > 6 || (r == -1)) ok = 0;
    }
    CHECK(ok, "truncated/junked files load without crashing");
    M.dirty = 1;
    CHECK(goto_mem_save(&M, "build/no/such/dir/m.txt", 1.0) == -1 && M.dirty, "unwritable path -> -1, stays dirty");
    CHECK(goto_mem_save(&M, "-", 1.0) == 0, "\"-\" = RAM only");
    remove(p);
}

static void test_memory_cfg(void)
{
    const char *p = "build/mem.conf";
    FILE *f = fopen(p, "w");
    fprintf(f, "memory_path = /data/home/qnxuser/goto/memory.txt\nmemory_drive_mm = 5000\nmemory_max_age_s = 60\nmemory_save_s = 0\n");
    fclose(f);
    goto_cfg c;
    goto_cfg_defaults(&c);
    char err[64];
    CHECK(goto_cfg_load(&c, p, err, sizeof err) == 0 && !err[0], "memory keys known (%s)", err);
    CHECK(!strcmp(c.memory_path, "/data/home/qnxuser/goto/memory.txt") && c.memory_drive_mm == 1000 && c.memory_max_age_s == 60
          && c.memory_save_s == 0.5, "memory_path, clamps (drive <= 1000, save >= 0.5)");
    goto_cfg_defaults(&c);
    CHECK(!strcmp(c.memory_path, "memory.txt") && c.memory_drive_mm == 0, "defaults: memory.txt, no blind driving");
}

static void test_describe(void)
{
    reset_sm();
    start("trash can");
    char d[600];
    goto_sm_describe(&S, d, sizeof d);
    CHECK(strstr(d, "state=look") && strstr(d, "target=trash_can") && strstr(d, "mem=-"), "describe: %s", d);
}

int main(void)
{
    goto_cfg_defaults(&C);
    test_objects();
    test_json();
    test_cfg();
    test_visible_at_start();
    test_search_and_align();
    test_left_target_and_lost();
    test_give_up();
    test_cancel_and_failures();
    test_retarget_and_filters();
    test_describe();
    test_memory_observe();
    test_memory_goto_hit();
    test_memory_goto_miss();
    test_memory_scan();
    test_memory_file();
    test_memory_cfg();
    printf("%d/%d checks passed\n", passed, passed + failed);
    return failed ? 1 : 0;
}

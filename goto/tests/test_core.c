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

static void test_describe(void)
{
    reset_sm();
    start("trash can");
    char d[400];
    goto_sm_describe(&S, d, sizeof d);
    CHECK(strstr(d, "state=look") && strstr(d, "target=trash_can"), "describe: %s", d);
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
    printf("%d/%d checks passed\n", passed, passed + failed);
    return failed ? 1 : 0;
}

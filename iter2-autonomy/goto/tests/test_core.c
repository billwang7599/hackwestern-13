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
    CHECK(goto_cfg_load(&d, "goto.conf", err, sizeof err) == 0 || goto_cfg_load(&d, "../goto.conf", err, sizeof err) == 0, "iter2-autonomy/goto/goto.conf loads");
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
    CHECK(!find(ACT_DRIVE) && goto_sm_wants_status(&S),
          "no D during a TURN (goto.c PINGs the deadman while wants_status; a zero D is refused under guard)");
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
    CHECK(!strcmp(S.result, "failed:not_found_no_memory") && count(ACT_STOP) >= 1 && find(ACT_BEEP) && find(ACT_BEEP)->a == 3,
          "not found: S + BEEP 3 (%s)", S.result);
}

/* Regression (full-stack sim, "moved" ~2 runs in 15 on a loaded Mac): a guard hiccup (lidar_guard
 * stale for 100 ms) aborts a search TURN in tars_motor, as it should. goto used to end the whole
 * GOTO on it (failed:turn_lidar_guard). Like a route leg: stop, wait, LOOK again, resume the search;
 * the interrupted step isn't counted. Bounded by nav_max_replans; obstacle/stop unchanged. */
static void test_turn_guard_hiccup(void)
{
    reset_sm();
    start("door");
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    CHECK(S.st == ST_SEARCH_TURN && S.searched_deg == 30, "nothing in view -> search TURN (%s)", goto_state_name(S.st));
    clear();
    goto_sm_move_started(&S, T, 61, 0.7);
    advance(0.2);
    goto_sm_move_status(&S, T, 61, "aborted", "lidar_guard");
    CHECK(S.st == ST_LOOK && has_event("guard_stop door"), "search TURN aborted by the guard -> S, LOOK again, not failed (%s, %s)",
          goto_state_name(S.st), S.result);
    CHECK(S.searched_deg == 0, "the interrupted step doesn't count toward the 360 (%.0f)", S.searched_deg);
    /* the search carries on: still a full circle before not_found */
    int turns = 0;
    for (int step = 0; step < 30 && S.st != ST_IDLE; step++) {
        clear();
        for (int i = 0; i < 8 && S.st == ST_LOOK; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
        if (S.st == ST_SEARCH_TURN) { turns++; ack_turn(200 + step); advance(0.4); }
    }
    CHECK(S.st == ST_IDLE && turns == 12 && !strcmp(S.result, "failed:not_found_no_memory"), "then a full 12 x 30 search (%d, %s)", turns, S.result);

    /* refused at the start (guard stale right then): same */
    reset_sm();
    start("door");
    for (int i = 0; i < 4; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
    clear();
    CHECK(goto_sm_move_refused(&S, T, "guard_stale") == 1 && S.st == ST_LOOK, "search TURN refused (guard_stale) -> LOOK again (%s)",
          goto_state_name(S.st));

    /* a guard that never comes back: bounded, failed:blocked */
    reset_sm();
    start("door");
    int n = 0;
    for (int step = 0; step < 40 && S.st != ST_IDLE; step++) {
        for (int i = 0; i < 8 && S.st == ST_LOOK; i++) { advance(0.12); frame(NULL, 0, 0, 0); }
        if (S.st == ST_SEARCH_TURN) {
            clear();
            goto_sm_move_started(&S, T, 300 + step, 0.7);
            advance(0.1);
            goto_sm_move_status(&S, T, 300 + step, "aborted", "lidar_guard");
            n++;
        }
    }
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:blocked") && n == (int)C.nav_max_replans + 1,
          "guard keeps aborting -> failed:blocked after nav_max_replans (%d aborts, %s)", n, S.result);

    /* an align TURN aborted by the guard: LOOK again (then re-align on the next sighting) */
    reset_sm();
    start("cup");
    for (int i = 0; i < 4; i++) { advance(0.12); frame("cup", 0.9, 0.2, 0.6); }
    CHECK(S.st == ST_ALIGN_TURN, "off to the side -> ALIGN TURN (%s)", goto_state_name(S.st));
    clear();
    goto_sm_move_started(&S, T, 71, 0.7);
    goto_sm_move_status(&S, T, 71, "aborted", "lidar_guard");
    CHECK(S.st == ST_LOOK, "align TURN aborted by the guard -> LOOK again (%s)", goto_state_name(S.st));
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
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:not_found_no_map") && turns == 12, "full circle then give up (%d turns)", turns);
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


/* ===================================================== map-guided GOTO (nav) */
static nav_t *N;
static goto_mem MM;
static double PX, PZ, PYAW;                   /* the fake robot, nav frame */
static int crossed_occ, reverse_moves, legs;
static double TURN_SCALE = 1.0;                /* fake robot's TURN error (DC car: timed turns) */

static void set_free_rect(double x0, double z0, double x1, double z1, int v)
{
    for (double x = x0 + 0.05; x < x1; x += 0.1)
        for (double z = z0 + 0.05; z < z1; z += 0.1) {
            int ix, iz;
            if (nav_world_to_cell((float)x, (float)z, &ix, &iz)) nav_set_cell(N, ix, iz, v);
        }
}
static void put_robot(double x, double z, double yaw) { PX = x; PZ = z; PYAW = yaw; nav_set_pose(N, (float)x, (float)z, (float)yaw, 1); }

static int add_inst(double wx, double wz, int status)
{
    int i = goto_mem_sight(&MM, &C, "trash can", wx * 1000, wz * 1000, 0, 1000, 0.8, S.wall_base + T);
    MM.e[i].has_w = 1; MM.e[i].wx = wx; MM.e[i].wz = wz; MM.e[i].status = status;
    return i;
}

static void map_setup(void)
{
    reset_sm();
    if (!N) N = nav_new();
    nav_reset(N);
    nav_set_robot_radius(N, 0.35f);
    S.cfg.robot_radius_m = 0.35;              /* map tests were written for the 0.35 m TARS footprint */
    goto_mem_init(&MM);
    goto_sm_set_memory(&S, &MM, 1e6);
    goto_sm_set_nav(&S, N);
    goto_sm_pose(&S, T, 1, 0, 0, 0, 0);       /* memory has a (motor) pose */
    crossed_occ = reverse_moves = legs = 0;
}

/* tars_motor says the leg is done; the phone's newest pose is still from BEFORE the move
 * (link latency), the post-move poses arrive 50 ms apart afterwards, like the real LDR1 feed */
static void leg_done_phone_lags(int id)
{
    goto_sm_move_status(&S, T, id, "done", "-");
    for (int k = 0; k < 2; k++) {
        T += 0.05;
        nav_set_pose(N, (float)PX, (float)PZ, (float)PYAW, 1);
        goto_sm_tick(&S, T);
    }
}

/* execute queued TURN/MOVE legs on the fake robot until the SM stops issuing them */
static int run_legs(int max)
{
    int id = 100;
    for (int n = 0; n < max; n++) {
        const goto_act *a = find(ACT_TURN);
        const goto_act *m = find(ACT_MOVE);
        if (!a && !m) return n;
        if (a) PYAW -= a->a * M_PI / 180;
        else {
            if (m->a < 0) reverse_moves++;
            double nx = PX + m->a / 1000 * sin(PYAW), nz = PZ + m->a / 1000 * cos(PYAW);
            if (nav_line_clear_ex(N, (float)PX, (float)PZ, (float)nx, (float)nz, 0.0f) == 0) crossed_occ++;
            PX = nx; PZ = nz;
        }
        legs++;
        clear();
        goto_sm_move_started(&S, T, ++id, 0.5);
        advance(0.3);
        frame(NULL, 0, 0, 0);                         /* the camera keeps streaming */
        leg_done_phone_lags(id);
    }
    return max;
}
static int had_event(const char *prefix, const char *log) { return strstr(log, prefix) != NULL; }
static char EV[4000];
static void grab_events(void)
{
    for (int i = 0; i < S.n_act; i++)
        if (S.act[i].kind == ACT_EVENT && strlen(EV) + strlen(S.act[i].text) + 2 < sizeof EV) {
            strcat(EV, S.act[i].text);
            strcat(EV, "|");
        }
}
static int run_legs_ev(int max)
{
    int id = 500;
    for (int n = 0; n < max; n++) {
        grab_events();
        const goto_act *a = find(ACT_TURN);
        const goto_act *m = find(ACT_MOVE);
        if (!a && !m) return n;
        if (a) PYAW -= a->a * TURN_SCALE * M_PI / 180;
        else {
            if (m->a < 0) reverse_moves++;
            double nx = PX + m->a / 1000 * sin(PYAW), nz = PZ + m->a / 1000 * cos(PYAW);
            if (nav_line_clear_ex(N, (float)PX, (float)PZ, (float)nx, (float)nz, 0.0f) == 0) crossed_occ++;
            PX = nx; PZ = nz;
        }
        legs++;
        clear();
        goto_sm_move_started(&S, T, ++id, 0.5);
        advance(0.3);
        frame(NULL, 0, 0, 0);                         /* the camera keeps streaming */
        leg_done_phone_lags(id);
    }
    return max;
}
static void look_empty(int n) { for (int i = 0; i < n; i++) { advance(0.12); frame(NULL, 0, 0, 0); } }
/* run every TURN/MOVE (path legs, reveal, search) on the fake robot, camera sees nothing, until idle */
static void drive_until_idle(int max)
{
    int id = 3000;
    for (int k = 0; k < max && S.st != ST_IDLE; k++) {
        grab_events();
        const goto_act *a = find(ACT_TURN), *m = find(ACT_MOVE);
        if (a) PYAW -= a->a * M_PI / 180;
        else if (m) {
            double nx = PX + fmin(m->a, 1000.0) / 1000 * sin(PYAW), nz = PZ + fmin(m->a, 1000.0) / 1000 * cos(PYAW);
            if (nav_line_clear_ex(N, (float)PX, (float)PZ, (float)nx, (float)nz, 0.0f) == 0) crossed_occ++;
            PX = nx; PZ = nz;
        }
        if (a || m) { clear(); goto_sm_move_started(&S, T, ++id, 0.5); advance(0.3); leg_done_phone_lags(id); }
        else { advance(0.3); nav_set_pose(N, (float)PX, (float)PZ, (float)PYAW, 1); }
        look_empty(3);
    }
    grab_events();
}

/* Regression (full-stack sim flake): when tars_motor says a leg is done, the phone's newest
 * ARKit pose is still from before the move. Planning the next leg from it turned the robot by a
 * stale heading (one whole 30 deg reveal step) and sent it ~25 deg off course. Every leg must
 * start from the post-move pose; if no pose comes at all, carry on after nav_pose_fresh_s. */
static void test_map_leg_waits_for_post_move_pose(void)
{
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    set_free_rect(0, 1.9, 3.0, 2.1, NAV_OCC);           /* wall, gap on the right: an L route */
    put_robot(0.55, 0.55, 0);
    add_inst(0.55, 3.45, MEM_FRESH);
    start("trash can");
    CHECK(S.st == ST_FOLLOW_PATH && S.path_n >= 2, "route with corners (%d)", S.path_n);
    int id = 700, n_legs = 0, early = 0, bad_turn = 0, bad_move = 0;
    for (int n = 0; n < 40; n++) {
        const goto_act *a = find(ACT_TURN), *m = find(ACT_MOVE);
        if (!a && !m) break;
        if (S.leg <= 1 && S.path_i < S.path_n) {          /* what the TRUE pose calls for */
            double dx = S.path[S.path_i].x - PX, dz = S.path[S.path_i].z - PZ;
            double b = -(atan2(dx, dz) - PYAW) * 180 / M_PI;
            while (b > 180) b -= 360;
            while (b < -180) b += 360;
            if (a && fabs(a->a - b) > 1.0) bad_turn++;
            if (m && fabs(m->a - fmin(hypot(dx, dz) * 1000, C.nav_move_max_mm)) > 10) bad_move++;   /* tars_motor's MOVE limit */
        }
        if (a) PYAW -= a->a * M_PI / 180;
        else { PX += m->a / 1000 * sin(PYAW); PZ += m->a / 1000 * cos(PYAW); }
        clear();
        goto_sm_move_started(&S, T, ++id, 0.5);
        advance(0.3);
        frame(NULL, 0, 0, 0);
        goto_sm_move_status(&S, T, id, "done", "-");     /* phone pose still pre-move */
        goto_sm_move_status(&S, T, id, "done", "-");     /* the next poll says done again */
        if (find(ACT_TURN) || find(ACT_MOVE)) early++;
        leg_done_phone_lags(id);
        n_legs++;
    }
    CHECK(n_legs >= 4, "drove the route leg by leg (%d legs)", n_legs);
    CHECK(early == 0, "no leg starts from the pre-move ARKit pose (%d did)", early);
    CHECK(bad_turn == 0 && bad_move == 0, "every TURN/MOVE matches the true post-move pose (%d turns, %d moves off)",
          bad_turn, bad_move);
    CHECK(S.st == ST_LOOK, "route done -> LOOK (%s)", goto_state_name(S.st));

    /* the phone goes quiet right as a leg ends: don't hang, carry on after nav_pose_fresh_s */
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    set_free_rect(0, 1.9, 3.0, 2.1, NAV_OCC);
    put_robot(0.55, 0.55, 0);
    add_inst(0.55, 3.45, MEM_FRESH);
    start("trash can");
    const goto_act *a = find(ACT_TURN), *m = find(ACT_MOVE);
    if (a) PYAW -= a->a * M_PI / 180;
    else if (m) { PX += m->a / 1000 * sin(PYAW); PZ += m->a / 1000 * cos(PYAW); }
    clear();
    goto_sm_move_started(&S, T, 800, 0.5);
    advance(0.3);
    goto_sm_move_status(&S, T, 800, "done", "-");
    advance(C.nav_pose_fresh_s * 0.8);
    int waited = !find(ACT_TURN) && !find(ACT_MOVE) && S.st == ST_FOLLOW_PATH;
    advance(C.nav_pose_fresh_s * 0.4);
    CHECK(waited && (find(ACT_TURN) || find(ACT_MOVE)), "no post-move pose: waits, then the next leg after nav_pose_fresh_s (%s)",
          goto_state_name(S.st));
}

/* Regression (full-stack sim, obstacle scenario): wall between the robot and a bin in the wall's
 * LiDAR shadow. The first explore hop went straight at the middle of the wall: from there the
 * goal stays hidden, nothing is >= 0.4 m nearer and routable, so -> unmapped -> blind search ->
 * not_found. An explore hop must be able to see the goal when such a cell exists. */
static void test_map_explore_sees_the_goal(void)
{
    map_setup();
    for (double x = -1.95; x < 2.0; x += 0.1)            /* room seen from (0,0): wall + its shadow */
        for (double z = -0.95; z < 3.0; z += 0.1) {
            int ix, iz, v = NAV_FREE;
            if (z > 1.3 && z < 1.4 && fabs(x) < 0.6) v = NAV_OCC;
            else if (z > 1.4 && fabs(x * 1.3 / z) < 0.62) v = NAV_UNKNOWN;
            if (nav_world_to_cell((float)x, (float)z, &ix, &iz)) nav_set_cell(N, ix, iz, v);
        }
    put_robot(0, 0, 0);
    add_inst(0, 2.4, MEM_FRESH);                          /* the bin, behind the wall */
    EV[0] = 0;
    start("trash can");
    int id = 900;
    for (int n = 0; n < 20 && !S.exploring; n++) {        /* the 360 reveal, then explore */
        grab_events();
        const goto_act *a = find(ACT_TURN);
        if (!a) break;
        PYAW -= a->a * M_PI / 180;
        clear();
        goto_sm_move_started(&S, T, ++id, 0.5);
        advance(0.3);
        frame(NULL, 0, 0, 0);
        leg_done_phone_lags(id);
    }
    grab_events();
    CHECK(had_event("reveal trash can #1", EV) && S.exploring && S.st == ST_FOLLOW_PATH,
          "goal in the wall's shadow: reveal, then an explore hop (%s, %s)", goto_state_name(S.st), EV);
    int los = nav_line_clear(N, (float)S.vx, (float)S.vz, 0.0f, 2.4f);
    CHECK(los != 0, "the explore hop (%.2f, %.2f) can see the goal (line_clear %d), not the wall's middle", S.vx, S.vz, los);
}

static void test_map_route_around_wall(void)
{
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    set_free_rect(0, 1.9, 3.0, 2.1, NAV_OCC);           /* wall across the room, gap on the right */
    put_robot(0.55, 0.55, 0);                           /* facing +z, toward the wall */
    add_inst(0.55, 3.45, MEM_FRESH);                    /* the bin is behind the wall */
    EV[0] = 0;
    start("trash can");
    grab_events();
    CHECK(had_event("remembered trash can #1", EV) && had_event("route trash can #1", EV), "GOTO -> RECALL -> PLAN: %s", EV);
    CHECK(S.st == ST_FOLLOW_PATH && S.map_mode && S.path_n >= 2, "follows an A* path (%d corners)", S.path_n);
    run_legs_ev(60);
    CHECK(S.st == ST_LOOK, "path done + facing the spot -> LOOK (%s)", goto_state_name(S.st));
    double d = hypot(PX - 0.55, PZ - 3.45);
    CHECK(d >= 0.5 && d <= 0.95, "stopped on the approach ring (%.2f m)", d);
    CHECK(PZ > 2.1, "went around the wall (z %.2f)", PZ);
    CHECK(crossed_occ == 0, "no leg crossed an occupied cell (%d)", crossed_occ);
    CHECK(reverse_moves == 0, "no reverse moves (guard only watches ahead)");
    /* YOLO confirms and docks */
    clear();
    advance(0.4);
    for (int i = 0; i < 2; i++) { advance(0.12); frame("trash can", 0.5, 0.3, 0.8); }
    CHECK(S.st == ST_APPROACH, "seen at the spot -> APPROACH (%s)", goto_state_name(S.st));
    for (int i = 0; i < 3; i++) { advance(0.12); frame("trash can", 0.5, 0.75, 0.95); }
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived"), "ARRIVED (%s)", S.result);
}

static void test_map_gone_then_check_others(void)
{
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(1.05, 1.05, 0);
    add_inst(1.05, 1.85, MEM_FRESH);                    /* #1 right ahead, on the ring */
    add_inst(3.2, 3.2, MEM_FRESH);                      /* #2 further away */
    EV[0] = 0;
    start("trash can");
    grab_events();
    run_legs_ev(20);
    CHECK(S.st == ST_LOOK && S.mem_id == 1, "nearest instance #1 first, LOOK at it (%s #%d)", goto_state_name(S.st), S.mem_id);
    advance(0.4);
    clear();
    look_empty(4);
    grab_events();
    CHECK(had_event("not_there trash can #1", EV), "clear line of sight + nothing -> not_there (%s)", EV);
    CHECK(MM.e[goto_mem_find_id(&MM, "trash can", 1)].status == MEM_STALE, "#1 fresh -> stale");
    CHECK(had_event("remembered trash can #2", EV) && S.mem_id == 2 && S.st == ST_FOLLOW_PATH,
          "CHECK_OTHERS -> routes to #2 (%s)", goto_state_name(S.st));
    run_legs_ev(60);
    CHECK(S.st == ST_LOOK && hypot(PX - 3.2, PZ - 3.2) < 0.95, "at #2 (%.2f m)", hypot(PX - 3.2, PZ - 3.2));
}

static void test_map_occluded_detour(void)
{
    map_setup();
    nav_set_robot_radius(N, 0.25f);                     /* small footprint so the occluder fits in the test room */
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(2.05, 0.85, 0);
    add_inst(2.05, 1.65, MEM_FRESH);                    /* on the ring, straight ahead */
    EV[0] = 0;
    start("trash can");
    grab_events();
    run_legs_ev(10);
    CHECK(S.st == ST_LOOK, "at the spot, looking (%s)", goto_state_name(S.st));
    set_free_rect(1.9, 1.2, 2.2, 1.3, NAV_OCC);         /* something now stands between us and it */
    advance(0.4);
    clear();
    look_empty(4);
    grab_events();
    CHECK(had_event("occluded trash can #1", EV) && had_event("detour trash can #1", EV), "LOOK fails, map occupied -> OCCLUDED -> DETOUR (%s)", EV);
    CHECK(MM.e[goto_mem_find_id(&MM, "trash can", 1)].status == MEM_FRESH, "occluded: instance NOT downgraded");
    CHECK(S.st == ST_FOLLOW_PATH && S.vantage, "routes to a vantage cell (%s)", goto_state_name(S.st));
    run_legs_ev(40);
    CHECK(S.st == ST_LOOK, "vantage reached, facing it, LOOK (%s)", goto_state_name(S.st));
    CHECK(nav_line_clear(N, (float)PX, (float)PZ, 2.05f, 1.65f) == 1, "clear line of sight from the vantage cell");
    CHECK(crossed_occ == 0, "detour legs avoid the occluder");
    /* still nothing: already detoured -> now it's not there */
    advance(0.4);
    clear();
    look_empty(4);
    grab_events();
    CHECK(had_event("not_there trash can #1", EV), "nothing from the vantage either -> not_there");
}

/* Regression (bench L-pocket): the guard stopped a leg of the DETOUR to a vantage cell. Main's goto
 * re-planned to the vantage, failed and went straight to next_candidate + a 360 search from behind the
 * occluder. A guard stop is never "not there": the refused zone is marked, the robot backs out and
 * re-plans (another stand-off cell / the approach ring); only a clear line of sight that sees nothing is. */
static void test_map_detour_guard_stop(void)
{
    map_setup();
    nav_set_robot_radius(N, 0.25f);
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(2.05, 0.85, 0);
    add_inst(2.05, 1.65, MEM_FRESH);
    EV[0] = 0;
    start("trash can");
    grab_events();
    run_legs_ev(10);
    set_free_rect(1.9, 1.2, 2.2, 1.3, NAV_OCC);
    advance(0.4);
    clear();
    look_empty(4);
    grab_events();
    CHECK(S.st == ST_FOLLOW_PATH && S.vantage, "detour to a vantage cell (%s)", goto_state_name(S.st));
    int k = 0;
    while (k++ < 6 && !find(ACT_MOVE) && find(ACT_TURN)) run_legs_ev(1);
    CHECK(find(ACT_MOVE) != NULL, "a detour MOVE leg is asked for");
    clear();
    goto_sm_move_started(&S, T, 990, 1.0);
    goto_sm_guard_reason(&S, "guard_blocked");          /* the guard saw something the map doesn't have */
    goto_sm_move_status(&S, T, 990, "aborted", "lidar_guard");
    grab_events();
    CHECK(S.guard_aborts == 1, "counted as a guard abort on a path leg (%d)", S.guard_aborts);
    goto_sm_guard_reason(&S, "clear");
    for (int i = 0; i < 12 && S.st == ST_PLAN; i++) { advance(0.1); frame(NULL, 0, 0, 0); }
    run_legs_ev(60);
    grab_events();
    CHECK(!had_event("next_candidate", EV) && !had_event("not_there", EV),
          "guard stop on the detour: no next_candidate / not_there (%.300s)", EV);
    CHECK(S.st == ST_LOOK, "re-planned and reached a view of the spot (%s)", goto_state_name(S.st));
    CHECK(MM.e[goto_mem_find_id(&MM, "trash can", 1)].status == MEM_FRESH, "instance not downgraded");
    goto_sm_cancel(&S, T, "x");
}

/* the search must not spin where a mapped wall hides the spot: it moves to a cell that sees it */
static void test_map_search_needs_line_of_sight(void)
{
    map_setup();
    set_free_rect(0, 0, 5, 5, NAV_FREE);
    set_free_rect(1.0, 2.0, 4.0, 2.1, NAV_OCC);           /* a wall between the robot and the spot */
    put_robot(2.55, 1.05, 0);
    add_inst(2.55, 3.4, MEM_FRESH);
    S.have_goal = 1; S.goal_x = 2.55; S.goal_z = 3.4;
    EV[0] = 0;
    start("trash can");
    S.have_goal = 1; S.goal_x = 2.55; S.goal_z = 3.4;
    S.map_mode = 1; S.mem_id = 1;
    clear();
    search_fallback_for_test(&S);
    grab_events();
    CHECK(had_event("standoff_search", EV) && S.st == ST_FOLLOW_PATH,
          "wall hides the spot: go to a stand-off cell with a line of sight, don't spin here (%s, %.200s)", goto_state_name(S.st), EV);
    goto_sm_cancel(&S, T, "x");
}

/* distance from (px,pz) to an axis-aligned rectangle */
static double rect_dist(double px, double pz, double x0, double z0, double x1, double z1)
{
    double dx = px < x0 ? x0 - px : px > x1 ? px - x1 : 0, dz = pz < z0 ? z0 - pz : pz > z1 ? pz - z1 : 0;
    return hypot(dx, dz);
}

/* Regression (full-stack sim, rare OCCLUSION flake): something appears ~0.4 m in front of the robot
 * at the spot, inside its own 9x9-cell footprint square (and lidar_guard's stop zone). Every plan
 * from here was START_BLOCKED, so no vantage cell "qualified": OCCLUDED -> next_candidate -> a blind
 * 360 from behind the board -> not_found. It must still DETOUR: pivot away (allowed while the guard
 * is BLOCKED), drive FORWARD a little away from the board (never reverse), then on to a vantage cell. */
static void test_map_occluder_inside_footprint(void)
{
    map_setup();                                        /* default 0.35 m radius, like the robot */
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(2.05, 0.45, 0);                           /* facing +z */
    add_inst(2.05, 2.05, MEM_FRESH);                    /* the bin, 1.6 m ahead */
    EV[0] = 0;
    start("trash can");
    grab_events();
    run_legs_ev(10);
    CHECK(S.st == ST_LOOK && hypot(PX - 2.05, PZ - 2.05) < 0.95, "at the spot, looking (%s, %.2f m)",
          goto_state_name(S.st), hypot(PX - 2.05, PZ - 2.05));
    /* a 0.6 m board 0.39 m ahead of the robot, between it and the bin: one row of OCC cells
     * (sampled densely: 0.1 m steps from a cell edge skip cells to float rounding) */
    int ix0, ix1, izb;
    nav_world_to_cell((float)(PX - 0.3), (float)(PZ + 0.39), &ix0, &izb);
    nav_world_to_cell((float)(PX + 0.3), (float)(PZ + 0.39), &ix1, &izb);
    for (int ix = ix0; ix <= ix1; ix++) nav_set_cell(N, ix, izb, NAV_OCC);
    double bx0 = NAV_ORIGIN_X + ix0 * NAV_RES, bx1 = NAV_ORIGIN_X + (ix1 + 1) * NAV_RES;   /* the cells' extent */
    double bz0 = NAV_ORIGIN_Z + izb * NAV_RES + 0.05, bz1 = bz0 + 0.05;                    /* (bz0 - 0.05 .. bz1) */
    double d0 = rect_dist(PX, PZ, bx0, bz0 - 0.05, bx1, bz1);
    CHECK(d0 < 0.45 && ix1 - ix0 >= 5, "board (%d cells) %.2f m ahead: inside the footprint square", ix1 - ix0 + 1, d0);
    advance(0.4);
    clear();
    look_empty(4);
    grab_events();
    CHECK(had_event("occluded trash can #1", EV) && had_event("detour trash can #1", EV),
          "occluder inside the footprint: OCCLUDED -> DETOUR, not next_candidate (%s)", EV);
    CHECK(!had_event("next_candidate", EV) && S.st == ST_FOLLOW_PATH && S.vantage,
          "routes to a vantage cell (%s)", goto_state_name(S.st));
    /* drive it: the robot centre must never get closer to the board than it was */
    int id = 1200, first_turn = 1;
    double first_deg = 0, closest = d0;
    for (int n = 0; n < 40; n++) {
        grab_events();
        const goto_act *a = find(ACT_TURN), *m = find(ACT_MOVE);
        if (!a && !m) break;
        if (a) { if (first_turn) first_deg = a->a; PYAW -= a->a * M_PI / 180; }
        else {
            if (m->a < 0) reverse_moves++;
            double nx = PX + m->a / 1000 * sin(PYAW), nz = PZ + m->a / 1000 * cos(PYAW);
            if (nav_line_clear_ex(N, (float)PX, (float)PZ, (float)nx, (float)nz, 0.0f) == 0) crossed_occ++;
            for (int k = 1; k <= 20; k++) {
                double qx = PX + (nx - PX) * k / 20, qz = PZ + (nz - PZ) * k / 20;
                if (fabs(qx - 2.05) < 0.9 && qz < 1.4) closest = fmin(closest, rect_dist(qx, qz, bx0, bz0 - 0.05, bx1, bz1));
            }
            PX = nx; PZ = nz;
        }
        first_turn = 0;
        clear();
        goto_sm_move_started(&S, T, ++id, 0.5);
        advance(0.3);
        frame(NULL, 0, 0, 0);
        leg_done_phone_lags(id);
    }
    CHECK(fabs(first_deg) >= 90, "first it pivots away from the board (TURN %.0f)", first_deg);
    CHECK(reverse_moves == 0 && crossed_occ == 0, "no reverse moves (%d), no leg through the board (%d)", reverse_moves, crossed_occ);
    CHECK(closest >= d0 - 0.02, "never drove closer to the board (start %.2f m, closest %.2f m)", d0, closest);
    CHECK(S.st == ST_LOOK && nav_line_clear(N, (float)PX, (float)PZ, 2.05f, 2.05f) == 1,
          "vantage reached with a clear view of the spot, LOOK (%s)", goto_state_name(S.st));
    /* the visual approach drives straight at the bin from here: the body (and lidar_guard's
     * corridor) must not clip the board on the way to the docking distance */
    double L = hypot(2.05 - PX, 2.05 - PZ), lane = 9;
    for (int k = 0; k <= 40; k++) {
        double t = (L - 0.55) * k / 40 / L;
        lane = fmin(lane, rect_dist(PX + (2.05 - PX) * t, PZ + (2.05 - PZ) * t, bx0, bz0 - 0.05, bx1, bz1));
    }
    CHECK(lane >= 0.25, "straight approach from the vantage keeps the body clear of the board (%.2f m)", lane);
    /* YOLO sees it from there and docks */
    clear();
    advance(0.4);
    for (int i = 0; i < 2; i++) { advance(0.12); frame("trash can", 0.5, 0.3, 0.8); }
    for (int i = 0; i < 3; i++) { advance(0.12); frame("trash can", 0.5, 0.75, 0.95); }
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived"), "ARRIVED after the detour (%s)", S.result);
}

static void test_map_no_route_fails(void)
{
    map_setup();
    C.search_max_deg = 60;                              /* short blind search at the end */
    goto_sm_init(&S, &C, &O); goto_sm_set_memory(&S, &MM, 1e6); goto_sm_set_nav(&S, N); goto_sm_pose(&S, T, 1, 0, 0, 0, 0);
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    set_free_rect(2.5, 2.5, 4.0, 4.0, NAV_OCC);         /* the bin's corner is solid: no clear ring cell */
    set_free_rect(3.2, 3.2, 3.3, 3.3, NAV_FREE);
    put_robot(0.55, 0.55, 0);
    add_inst(3.25, 3.25, MEM_FRESH);
    EV[0] = 0;
    start("trash can");
    grab_events();
    CHECK(had_event("reveal trash can #1", EV) && S.st == ST_FOLLOW_PATH, "no route yet -> reveal spin first (%s)", EV);
    int spins = 0;
    while (S.st == ST_FOLLOW_PATH && S.leg == 3 && spins < 20) { run_legs_ev(1); spins++; }
    grab_events();
    CHECK(spins == 12, "a full 360 of reveal turns, re-planning after each (%d)", spins);
    /* still none: stand-off cell (none: walled in), explore hops toward it, then NO_ROUTE ->
     * blind search -> memory fallback (same walled-in spot, explores again) -> failed:no_route */
    drive_until_idle(400);
    grab_events();
    CHECK(had_event("no_route trash can #1", EV), "gave up with no_route only after exploring (%s)", EV + (strlen(EV) > 200 ? strlen(EV) - 200 : 0));
    CHECK(crossed_occ == 0, "never through the solid corner");
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:no_route"), "then FAILED (%s)", S.result);
    C = (goto_cfg){0};
    goto_cfg_defaults(&C);
}

/* Regression (DC full-stack sim, obstacle with TURN x0.75): the reveal spin counted commanded
 * degrees, so twelve 30 deg steps of a car that turns 75% covered 270 deg and part of the room
 * stayed unmapped. With ARKit, the measured yaw counts. Also: a new GOTO may reveal again. */
static int reveal_spins(double scale)
{
    map_setup();
    C.search_max_deg = 60;
    goto_sm_init(&S, &C, &O); goto_sm_set_memory(&S, &MM, 1e6); goto_sm_set_nav(&S, N); goto_sm_pose(&S, T, 1, 0, 0, 0, 0);
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    set_free_rect(2.5, 2.5, 4.0, 4.0, NAV_OCC);
    set_free_rect(3.2, 3.2, 3.3, 3.3, NAV_FREE);
    put_robot(0.55, 0.55, 0);
    add_inst(3.25, 3.25, MEM_FRESH);
    TURN_SCALE = scale;
    double yaw0 = PYAW;
    start("trash can");
    int spins = 0;
    while (S.st == ST_FOLLOW_PATH && S.leg == 3 && spins < 40) { run_legs_ev(1); spins++; }
    double turned = fabs((PYAW - yaw0) * 180 / M_PI);
    CHECK(turned >= 359.0 && turned < 360.0 + 2.0 * 30 * scale, "TURN x%.2f: reveal really turned %.0f deg in %d steps", scale, turned, spins);
    TURN_SCALE = 1.0;
    return spins;
}

static void test_map_reveal_measured(void)
{
    CHECK(reveal_spins(1.0) == 12, "exact turns: 12 x 30 as before");
    int n = reveal_spins(0.75);
    CHECK(n == 16, "turns at 75%%: 16 steps to really cover 360 (%d)", n);
    n = reveal_spins(1.25);
    CHECK(n == 10, "turns at 125%%: 10 steps (%d)", n);
    /* a second GOTO reveals again (revealed_deg used to carry over between GOTOs) */
    reveal_spins(1.0);
    goto_sm_cancel(&S, T, "x");
    EV[0] = 0;
    start("trash can");
    grab_events();
    CHECK(had_event("reveal trash can #1", EV) && S.st == ST_FOLLOW_PATH && S.leg == 3, "next GOTO: reveal again (%s)", EV);
    C = (goto_cfg){0};
    goto_cfg_defaults(&C);
}

/* the fake robot's lidar_guard (lidar_guard.cpp, no slack): an OCC cell centre at 0 < forward
 * <= 0.6 m, |side| < 0.45 m, inside the phone's 60 deg cone -> BLOCKED */
static int fake_guard_blocked(void)
{
    for (int ix = 0; ix < NAV_SIZE; ix++)
        for (int iz = 0; iz < NAV_SIZE; iz++) {
            if (nav_cell_at(N, ix, iz) != NAV_OCC) continue;
            double dx = NAV_ORIGIN_X + (ix + 0.5) * NAV_RES - PX, dz = NAV_ORIGIN_Z + (iz + 0.5) * NAV_RES - PZ;
            double f = sin(PYAW) * dx + cos(PYAW) * dz, side = fabs(cos(PYAW) * dx - sin(PYAW) * dz);
            if (f > 0 && f <= 0.6 && side < 0.45 && side < f * tan(M_PI / 6)) return 1;
        }
    return 0;
}

/* run legs; a MOVE that starts with something in the guard's zone is refused like tars_motor
 * does (ERR guard_blocked). Returns how many MOVEs were refused. */
static int run_legs_guarded(int max, int *moves)
{
    int id = 900, refused = 0;
    for (int n = 0; n < max; n++) {
        grab_events();
        const goto_act *a = find(ACT_TURN), *m = find(ACT_MOVE);
        if (!a && !m) break;
        if (m && !a && fake_guard_blocked()) {
            refused++;
            clear();
            goto_sm_move_refused(&S, T, "guard_blocked");
            goto_sm_guard_reason(&S, "guard_blocked");
            for (int k = 0; k < 10 && !find(ACT_TURN) && !find(ACT_MOVE); k++) {
                advance(0.5);
                nav_set_pose(N, (float)PX, (float)PZ, (float)PYAW, 1);
            }
            continue;
        }
        if (a) PYAW -= a->a * M_PI / 180;
        else {
            if (moves) (*moves)++;
            double nx = PX + m->a / 1000 * sin(PYAW), nz = PZ + m->a / 1000 * cos(PYAW);
            if (nav_line_clear_ex(N, (float)PX, (float)PZ, (float)nx, (float)nz, 0.0f) == 0) crossed_occ++;
            PX = nx; PZ = nz;
        }
        clear();
        goto_sm_move_started(&S, T, ++id, 0.5);
        advance(0.3);
        frame(NULL, 0, 0, 0);
        leg_done_phone_lags(id);
    }
    grab_events();
    return refused;
}

/* Regression (bench demo_detour with lidar_guard's real zone): the guard stopped the robot 0.6 m
 * from a box across the route. A* hugs the box with short legs, each facing it, so every MOVE
 * was refused (ERR guard_blocked) and 7 re-plans later: failed:blocked. Now goto pivots to a
 * heading whose guard zone is clear and steps out first. */
static void box_ahead_setup(void)
{
    map_setup();
    set_free_rect(0, 0, 4, 4.4, NAV_FREE);
    set_free_rect(0.8, 1.6, 2.2, 1.9, NAV_OCC);        /* the box: face 0.6 m ahead, 1.4 m wide */
    put_robot(1.45, 1.0, 0);
    add_inst(1.45, 3.6, MEM_FRESH);
}

static void test_map_guard_escape(void)
{
    /* before: escape off and A* blind to the guard's box = the old behaviour */
    C.nav_escape_mm = 0;
    C.nav_guard_plan = 0;
    C.nav_backtrack = 0;
    box_ahead_setup();
    EV[0] = 0;
    start("trash can");
    int moves = 0, refused = run_legs_guarded(40, &moves);
    CHECK(refused >= 2, "escape off: MOVEs refused by the guard again and again (%d refused, events %.160s)", refused, EV);
    goto_sm_cancel(&S, T, "x");
    C = (goto_cfg){0};
    goto_cfg_defaults(&C);

    /* the escape alone (A* still blind to the guard's box) */
    C.nav_guard_plan = 0;
    box_ahead_setup();
    EV[0] = 0;
    start("trash can");
    moves = 0;
    refused = run_legs_guarded(60, &moves);
    CHECK(had_event("escape trash can #1", EV), "escape: pivot clear of the guard zone first (%.160s)", EV);
    CHECK(refused == 0, "escape: no MOVE ever refused by the guard (%d)", refused);
    CHECK(S.st == ST_LOOK && PZ > 2.4, "escape: got round the box (%s, z=%.2f)", goto_state_name(S.st), PZ);
    goto_sm_cancel(&S, T, "x");
    C = (goto_cfg){0};
    goto_cfg_defaults(&C);

    /* after: the default, A* only plans legs the guard's box allows (no escape needed) */
    box_ahead_setup();
    EV[0] = 0;
    start("trash can");
    moves = 0;
    refused = run_legs_guarded(60, &moves);
    CHECK(refused == 0, "no MOVE ever refused by the guard (%d)", refused);
    CHECK(crossed_occ == 0, "never drove through the box");
    CHECK(S.st == ST_LOOK && PZ > 2.4, "got round the box and reached the spot (%s, z=%.2f)", goto_state_name(S.st), PZ);
    CHECK(S.escapes <= 8, "escapes bounded (%d)", S.escapes);
    goto_sm_cancel(&S, T, "x");
    C = (goto_cfg){0};
    goto_cfg_defaults(&C);
}

/* tars_motor clamps MOVE to 1000 mm (wheels.h MOVE_MAX_MM) and reports DONE: a 2.5 m leg used to
 * count as reached after 1 m, and the robot LOOKed from 1.5 m short */
static void test_map_long_leg_in_pieces(void)
{
    map_setup();
    set_free_rect(0, 0, 4, 4.4, NAV_FREE);
    put_robot(1.0, 0.5, 0);
    add_inst(1.0, 3.9, MEM_FRESH);
    start("trash can");
    int id = 950, big = 0, n = 0;
    for (; n < 30; n++) {
        const goto_act *a = find(ACT_TURN), *m = find(ACT_MOVE);
        if (!a && !m) break;
        if (m && m->a > 1000.5) big++;
        double amt = m ? fmin(m->a, 1000.0) : 0;          /* what tars_motor really drives */
        if (a) PYAW -= a->a * M_PI / 180;
        else { PX += amt / 1000 * sin(PYAW); PZ += amt / 1000 * cos(PYAW); }
        clear();
        goto_sm_move_started(&S, T, ++id, 0.5);
        advance(0.3);
        frame(NULL, 0, 0, 0);
        leg_done_phone_lags(id);
    }
    CHECK(big == 0, "no MOVE over tars_motor's 1000 mm limit (%d)", big);
    CHECK(S.st == ST_LOOK && fabs(PZ - 3.9) < 1.0 && PZ > 2.8, "LOOKs from the end of the route, not 1.5 m short (z=%.2f)", PZ);
}

/* run the blind search to its end (no camera hits) */
static void search_to_end(int max_steps)
{
    for (int k = 0; k < max_steps && S.st != ST_IDLE && S.st != ST_FOLLOW_PATH && S.st != ST_RECALL; k++) {
        const goto_act *a = find(ACT_TURN);
        if (a) PYAW -= a->a * M_PI / 180;
        if (S.st == ST_SEARCH_TURN) { clear(); goto_sm_move_started(&S, T, 400 + k, 0.5); leg_done_phone_lags(400 + k); }
        advance(0.4);
        nav_set_pose(N, (float)PX, (float)PZ, (float)PYAW, 1);
        look_empty(4);
    }
}

/* User's bench case: the bin was seen before (memory), then a wall was put between the robot and
 * it. Memory is too old to try first, the 360 search sees nothing: instead of failed:not_found,
 * route over the map to the remembered spot, around the wall, and look there. */
static void test_map_search_fails_then_memory_route(void)
{
    map_setup();
    set_free_rect(0, -1, 4, 4, NAV_FREE);
    set_free_rect(1.2, 0.8, 2.8, 1.1, NAV_OCC);        /* the wall, between the robot and the bin */
    put_robot(2.0, 0.0, 0);
    int i = add_inst(2.0, 1.8, MEM_FRESH);             /* bin 0.7 m behind the wall */
    MM.e[i].last_seen -= 10 * C.memory_max_age_s;       /* too old for the normal memory GOTO */
    EV[0] = 0;
    start("trash can");
    grab_events();
    CHECK(S.st == ST_LOOK && !had_event("remembered", EV), "old memory: search first (%s, %s)", goto_state_name(S.st), EV);
    search_to_end(40);
    grab_events();
    CHECK(had_event("search_failed_memory trash can #1", EV) && S.map_mode, "search failed -> route to the remembered spot (%s)", EV);
    run_legs_ev(60);
    grab_events();
    CHECK(crossed_occ == 0, "never through the wall");
    CHECK(S.st == ST_LOOK && hypot(PX - 2.0, PZ - 1.8) < 1.0, "at the spot behind the wall, looking (%s, %.2f m)",
          goto_state_name(S.st), hypot(PX - 2.0, PZ - 1.8));
    /* the bin isn't there either: fail now (no second fallback loop) */
    for (int k = 0; k < 80 && S.st != ST_IDLE; k++) {
        const goto_act *a = find(ACT_TURN), *m = find(ACT_MOVE);
        if (a || m) run_legs_ev(1);
        advance(0.4);
        look_empty(4);
    }
    CHECK(S.st == ST_IDLE && !strncmp(S.result, "failed:", 7), "nothing there either: fails, no loop (%s)", S.result);

    /* no memory at all: failed:not_found_no_memory */
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(2.0, 0.3, 0);
    start("trash can");
    search_to_end(40);
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:not_found_no_memory"), "no memory -> %s", S.result);

    /* remembered spot walled in: failed:no_route */
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    set_free_rect(1.0, 1.0, 3.0, 3.0, NAV_OCC);
    put_robot(0.5, 0.5, 0);
    i = add_inst(2.0, 2.0, MEM_GONE);
    start("trash can");
    drive_until_idle(400);
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:no_route"), "remembered but unreachable -> %s", S.result);
}

/* Two remembered bins: A is nearer in a straight line but behind a long wall, B is further but in
 * the open. The fallback picks the shorter ROUTE (B). Also: a bin 0.3 m from a wall (target cell
 * inside the wall's inflation) still gets a route, to the approach ring on its open side. */
static void test_map_memory_route_by_route_length(void)
{
    map_setup();
    set_free_rect(0, 0, 5, 5, NAV_FREE);
    set_free_rect(0, 1.5, 4.4, 1.8, NAV_OCC);          /* long wall, gap only at the far right */
    put_robot(0.5, 0.5, 0);
    int a = add_inst(0.6, 2.6, MEM_GONE);               /* A: 2.1 m straight, ~8 m by route */
    int b = add_inst(3.2, 0.6, MEM_GONE);               /* B: 2.7 m straight and by route */
    EV[0] = 0;
    start("trash can");
    search_to_end(40);
    grab_events();
    char want[64];
    snprintf(want, sizeof want, "search_failed_memory trash can #%d ", MM.e[b].id);
    CHECK(had_event(want, EV), "picked B, the shorter route, not A (ids A=%d B=%d: %s)", MM.e[a].id, MM.e[b].id, EV);

    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    set_free_rect(1.25, 1.2, 1.55, 2.4, NAV_OCC);       /* wall; the bin 0.3 m past its face */
    put_robot(0.5, 1.8, M_PI / 2);
    add_inst(1.85, 1.8, MEM_GONE);
    EV[0] = 0;
    start("trash can");
    search_to_end(40);
    run_legs_ev(60);
    grab_events();
    CHECK(had_event("route trash can", EV) && crossed_occ == 0 && S.st == ST_LOOK,
          "bin hugging the wall: routed round to its open side (%s, %s)", goto_state_name(S.st), EV);
}

static void test_map_stop_mid_path(void)
{
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(0.55, 0.55, 0);
    add_inst(3.0, 3.0, MEM_FRESH);
    start("trash can");
    CHECK(S.st == ST_FOLLOW_PATH, "following");
    clear();
    goto_sm_move_started(&S, T, 77, 2.0);
    goto_sm_move_status(&S, T, 77, "aborted", "stop");      /* voice "stop" -> S from another client */
    CHECK(S.st == ST_IDLE && !strncmp(S.result, "cancelled", 9), "S mid-path wins (%s)", S.result);
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(0.55, 0.55, 0);
    add_inst(3.0, 3.0, MEM_FRESH);
    start("trash can");
    clear();
    goto_sm_cancel(&S, T, "cancel");
    CHECK(S.st == ST_IDLE && count(ACT_STOP) == 1 && !find(ACT_TURN) && !find(ACT_MOVE), "CANCEL mid-path: S, nothing else");
}

static void test_map_guard_and_replan(void)
{
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(0.55, 0.55, 0);
    add_inst(3.0, 3.0, MEM_FRESH);
    goto_sm_guard(&S, T, 1, 5);
    start("trash can");
    run_legs(1);                                        /* one leg done */
    CHECK(S.st == ST_FOLLOW_PATH, "following");
    clear();
    goto_sm_move_started(&S, T, 900, 2.0);
    goto_sm_guard_reason(&S, "guard_stale");            /* a guard hiccup, not an obstacle (that backtracks) */
    goto_sm_guard(&S, T, 0, 6);                         /* lidar_guard denies ... */
    goto_sm_move_status(&S, T, 900, "aborted", "lidar_guard");   /* ... and tars_motor aborts the leg */
    CHECK(S.st == ST_PLAN && S.replans == 1, "guard-aborted leg -> PLAN, waiting (%s)", goto_state_name(S.st));
    for (int i = 0; i < 10; i++) { advance(0.1); frame(NULL, 0, 0, 0); }
    CHECK(S.st == ST_PLAN, "no re-plan while the guard still denies");
    goto_sm_guard(&S, T, 1, 6);
    for (int i = 0; i < 7; i++) { advance(0.1); frame(NULL, 0, 0, 0); }
    CHECK(S.st == ST_FOLLOW_PATH, "guard CLEAR -> re-plan from the current pose (%s)", goto_state_name(S.st));
    /* the map changes under the route -> S + re-plan */
    clear();
    goto_sm_move_started(&S, T, 901, 3.0);
    nav_pt_t w = S.path[S.path_i];
    double mx = (PX + w.x) / 2, mz = (PZ + w.z) / 2;
    set_free_rect(mx - 0.15, mz - 0.15, mx + 0.15, mz + 0.15, NAV_OCC);
    advance(0.1);
    CHECK(count(ACT_STOP) >= 1 && (S.st == ST_PLAN || S.st == ST_FOLLOW_PATH) && S.replans == 2,
          "new obstacle on the route -> S + re-plan (%s, replans %d)", goto_state_name(S.st), S.replans);
    /* too many: FAILED blocked */
    for (int k = 0; k < 10 && S.st != ST_IDLE; k++) {
        clear();
        goto_sm_move_started(&S, T, 950 + k, 2.0);
        goto_sm_move_status(&S, T, 950 + k, "aborted", "lidar_guard");
        for (int i = 0; i < 7 && S.st == ST_PLAN; i++) { advance(0.1); frame(NULL, 0, 0, 0); }
    }
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:blocked"), "re-plan limit -> FAILED blocked (%s)", S.result);
}

static void test_leased_approach(void)
{
    reset_sm();
    goto_sm_guard(&S, T, 1, 41);
    start("trash can");
    advance(0.1);
    for (int i = 0; i < 2; i++) { advance(0.12); frame("trash can", 0.5, 0.2, 0.6); }
    CHECK(S.st == ST_APPROACH, "approach");
    clear();
    advance(0.15);
    const goto_act *d = find(ACT_DRIVE);
    CHECK(d && !strcmp(d->text, "41"), "APPROACH drives with DL lease = guard epoch (%s)", d ? d->text : "-");
    clear();
    goto_sm_guard(&S, T, 0, 42);                        /* guard denied: epoch moved on */
    CHECK(count(ACT_STOP) == 1 && S.st == ST_LOOK, "epoch change ends the leg: S, LOOK (%s)", goto_state_name(S.st));
    goto_sm_guard(&S, T, 1, 42);
    clear();
    for (int i = 0; i < 12; i++) { advance(0.12); frame("trash can", 0.5, 0.2, 0.6); }
    d = find(ACT_DRIVE);
    CHECK(S.st == ST_APPROACH && d && !strcmp(d->text, "42"), "a NEW leg after a fresh look gets the new lease (%s)", d ? d->text : "-");
    /* DL refused while the target is close + centred = the guard stopped us at the target */
    for (int i = 0; i < 3; i++) { advance(0.12); frame("trash can", 0.5, 0.5, 0.9); }
    clear();
    goto_sm_drive_refused(&S, T, "guard epoch changed");
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived"), "guard stop at a big centred target = docked (%s)", S.result);
    /* Regression (full-stack visual-only): a STALE guard during the final approach counted as
     * docked short of the bin. Only a fresh guard_blocked may; stale/missing/invalid -> S, LOOK
     * again, approach again, bounded by nav_max_replans -> failed:blocked. */
    static const char *not_obstacle[] = {"guard_stale", "guard_missing", "guard_invalid", "clear"};
    for (int k = 0; k < 4; k++) {
        reset_sm();
        goto_sm_guard_reason(&S, "clear");
        goto_sm_guard(&S, T, 1, 50);
        start("trash can");
        advance(0.1);
        for (int i = 0; i < 2; i++) { advance(0.12); frame("trash can", 0.5, 0.2, 0.6); }
        for (int i = 0; i < 3; i++) { advance(0.12); frame("trash can", 0.5, 0.5, 0.9); }
        CHECK(S.st == ST_APPROACH, "approach, close + centred");
        clear();
        if (k < 3) {                                     /* DL refused naming the reason */
            char why[64];
            snprintf(why, sizeof why, "%s", not_obstacle[k]);
            goto_sm_drive_refused(&S, T, why);
        } else {                                         /* epoch moved on; STATUS now says clear */
            goto_sm_guard_reason(&S, "clear");
            goto_sm_guard(&S, T, 1, 51);
        }
        CHECK(count(ACT_STOP) >= 1 && S.st == ST_LOOK && strcmp(S.result, "arrived") && S.replans == 1,
              "%s guard stop at the target is NOT docked: S, look again (%s, %s)", not_obstacle[k],
              goto_state_name(S.st), S.result);
        goto_sm_guard(&S, T, 1, 51);
        for (int i = 0; i < 12 && S.st != ST_APPROACH; i++) { advance(0.12); frame("trash can", 0.5, 0.5, 0.9); }
        CHECK(S.st == ST_APPROACH, "... and approaches again (%s)", goto_state_name(S.st));
    }
    /* a stale guard every time: bounded, then failed:blocked */
    reset_sm();
    goto_sm_guard_reason(&S, "clear");
    goto_sm_guard(&S, T, 1, 60);
    start("trash can");
    for (int r = 0; r < 12 && S.st != ST_IDLE; r++) {
        for (int i = 0; i < 12 && S.st != ST_APPROACH && S.st != ST_IDLE; i++) { advance(0.12); frame("trash can", 0.5, 0.5, 0.9); }
        clear();
        goto_sm_drive_refused(&S, T, "guard_stale");
    }
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:blocked"), "stale guard every time -> failed:blocked (%s)", S.result);
    /* a fresh guard_blocked (DL refused, or STATUS behind an epoch change) is still docked */
    for (int k = 0; k < 2; k++) {
        reset_sm();
        goto_sm_guard_reason(&S, "clear");
        goto_sm_guard(&S, T, 1, 70);
        start("trash can");
        advance(0.1);
        for (int i = 0; i < 2; i++) { advance(0.12); frame("trash can", 0.5, 0.2, 0.6); }
        for (int i = 0; i < 3; i++) { advance(0.12); frame("trash can", 0.5, 0.5, 0.9); }
        clear();
        if (k == 0) goto_sm_drive_refused(&S, T, "guard_blocked");
        else { goto_sm_guard_reason(&S, "guard_blocked"); goto_sm_guard(&S, T, 0, 71); }
        CHECK(S.st == ST_IDLE && !strcmp(S.result, "arrived"), "fresh guard_blocked at the target = docked (%s)", S.result);
    }
    /* stop still wins mid-retry */
    reset_sm();
    goto_sm_guard_reason(&S, "clear");
    goto_sm_guard(&S, T, 1, 80);
    start("trash can");
    advance(0.1);
    for (int i = 0; i < 5; i++) { advance(0.12); frame("trash can", 0.5, 0.5, 0.9); }
    goto_sm_drive_refused(&S, T, "guard_stale");
    clear();
    goto_sm_cancel(&S, T, "stop");
    advance(1.0);
    for (int i = 0; i < 6; i++) { advance(0.12); frame("trash can", 0.5, 0.5, 0.9); }
    CHECK(S.st == ST_IDLE && count(ACT_STOP) >= 1 && !find(ACT_DRIVE), "CANCEL during the stale-guard wait: idle, S, no drive");
    /* no guard on the server: plain D */
    reset_sm();
    start("trash can");
    advance(0.1);
    for (int i = 0; i < 2; i++) { advance(0.12); frame("trash can", 0.5, 0.2, 0.6); }
    clear();
    advance(0.15);
    d = find(ACT_DRIVE);
    CHECK(d && !d->text[0], "no guard_epoch from tars_motor -> plain D");
}

static void test_map_fallbacks(void)
{
    /* nav attached but no pose yet: the visual memory turn, unchanged */
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    goto_mem_sight(&MM, &C, "trash can", 1000, 1000, 45, 1414, 0.8, S.wall_base + T);
    start("trash can");
    CHECK(S.st == ST_MEM_TURN && !S.map_mode, "no nav pose -> visual memory turn (%s)", goto_state_name(S.st));
    /* pose but nothing mapped: NO_MAP -> visual */
    map_setup();
    put_robot(0, 0, 0);
    add_inst(1.0, 1.0, MEM_FRESH);
    start("trash can");
    CHECK((S.st == ST_MEM_TURN || S.st == ST_LOOK) && !S.map_mode, "empty map -> visual fallback (%s)", goto_state_name(S.st));
    /* RECALL: nothing remembered -> FAILED, a gone one is still used */
    map_setup();
    goto_sm_recall(&S, T, goto_objects_resolve(&O, "trash can"));
    CHECK(S.st == ST_IDLE && !strcmp(S.result, "failed:not_remembered"), "RECALL with no memory (%s)", S.result);
    map_setup();
    set_free_rect(0, 0, 4, 4, NAV_FREE);
    put_robot(0.55, 0.55, 0);
    add_inst(3.0, 3.0, MEM_GONE);
    start("trash can");
    CHECK(!S.map_mode, "GOTO skips a gone instance");
    goto_sm_cancel(&S, T, "x");
    goto_sm_recall(&S, T, goto_objects_resolve(&O, "trash can"));
    CHECK(S.map_mode && S.st == ST_FOLLOW_PATH, "RECALL goes back to where it was, even gone (%s)", goto_state_name(S.st));
    /* map position survives save/load */
    CHECK(goto_mem_save(&MM, "build/test_mem_nav.txt", 1e6) == 0, "save");
    goto_mem L;
    CHECK(goto_mem_load(&L, "build/test_mem_nav.txt") == 0 && L.n == 1 && L.e[0].has_w && fabs(L.e[0].wx - 3.0) < 1e-3,
          "navw map position round-trips");
}

/* Regression (DC full-stack sim, "old spot A downgraded" failed): sightings were merged by
 * tars_motor's dead-reckoned x/y. On a DC car that was ~2 m off, so a bin that moved from A to B
 * refreshed A's instance. With a fresh ARKit pose, the map position decides. */
static void test_memory_merge_by_map(void)
{
    map_setup();
    put_robot(0, 0, 0);                                  /* phone: at the origin, facing +z */
    goto_sm_pose(&S, T, 1, 0, 0, 0, 0);                  /* DR: also 0,0,0 ... */
    /* #1 = spot A: on the map 2 m to the robot's LEFT-side x (2, 0); but DR believes it is 2 m
     * straight ahead (DR was 90 deg off when it was seen). STALE: GOTO already looked there. */
    int a = goto_mem_sight(&MM, &C, "trash can", 2000, 0, 0, 2000, 0.8, S.wall_base + T);
    MM.e[a].has_w = 1; MM.e[a].wx = 2.0; MM.e[a].wz = 0.0; MM.e[a].status = MEM_STALE;
    /* the bin, now at B = 2 m straight ahead on the map (0, 2) */
    advance(0.1);
    frame("trash can", 0.5, 0.25, 0.569);
    a = goto_mem_find_id(&MM, "trash can", 1);
    CHECK(goto_mem_count(&MM, "trash can") == 2, "a bin at B is a new instance, not A refreshed by DR (%d instances)",
          goto_mem_count(&MM, "trash can"));
    CHECK(a >= 0 && MM.e[a].status == MEM_STALE && fabs(MM.e[a].wx - 2.0) < 1e-6 && fabs(MM.e[a].wz) < 1e-6,
          "A keeps its map spot and stays stale (%s, %.2f,%.2f)", a >= 0 ? goto_mem_status_name(MM.e[a].status) : "-",
          a >= 0 ? MM.e[a].wx : 0, a >= 0 ? MM.e[a].wz : 0);
    int b = goto_mem_find_id(&MM, "trash can", 2);
    CHECK(b >= 0 && MM.e[b].has_w && fabs(MM.e[b].wx) < 0.2 && fabs(MM.e[b].wz - 2.0) < 0.4,
          "B's map spot is ahead (%.2f,%.2f)", b >= 0 ? MM.e[b].wx : 0, b >= 0 ? MM.e[b].wz : 0);
    /* the other way round: same map spot, DR far off -> still the same instance */
    put_robot(0, 0, M_PI / 2);                           /* phone: facing +x, i.e. at A */
    goto_sm_pose(&S, T, 1, 0, 0, 180, 0);                /* DR: facing the other way */
    advance(0.1);
    frame("trash can", 0.5, 0.25, 0.569);
    CHECK(goto_mem_count(&MM, "trash can") == 2 && MM.e[a].status == MEM_FRESH,
          "seen again at A on the map (DR says elsewhere) -> A refreshed, no third instance (%d)",
          goto_mem_count(&MM, "trash can"));
    /* without the phone: DR decides, as before */
    goto_mem_init(&MM);
    goto_sm_set_nav(&S, NULL);
    goto_sm_pose(&S, T, 1, 0, 0, 0, 0);
    a = goto_mem_sight(&MM, &C, "trash can", 2000, 0, 0, 2000, 0.8, S.wall_base + T);
    MM.e[a].status = MEM_STALE;
    advance(0.1);
    frame("trash can", 0.5, 0.25, 0.569);
    CHECK(goto_mem_count(&MM, "trash can") == 1 && MM.e[a].status == MEM_FRESH, "no phone: DR merge as before");
    goto_sm_set_nav(&S, N);

    /* arrival: docking at B after looking for A must not rewrite A (DR 1.45 m from B's dock) */
    map_setup();
    put_robot(0, 0, 0);
    a = goto_mem_sight(&MM, &C, "trash can", 2000, 0, 0, 2000, 0.8, S.wall_base + T);
    MM.e[a].has_w = 1; MM.e[a].wx = 2.0; MM.e[a].wz = 0.0; MM.e[a].status = MEM_STALE;
    start("trash can");                                  /* empty map -> visual memory turn to #1 */
    CHECK(S.mem_id == 1, "GOTO tries #1 (%d)", S.mem_id);
    for (int i = 0; i < 40 && S.st != ST_LOOK && S.st != ST_IDLE; i++) { ack_turn(300 + i); advance(0.3); }
    advance(0.5);
    for (int i = 0; i < 6 && S.st != ST_IDLE; i++) { advance(0.12); frame("trash can", 0.5, 0.75, 0.95); }
    a = goto_mem_find_id(&MM, "trash can", 1);
    CHECK(!strcmp(S.result, "arrived"), "docked (%s)", S.result);
    CHECK(a >= 0 && MM.e[a].status == MEM_STALE && fabs(MM.e[a].wx - 2.0) < 1e-6,
          "arrival at B didn't overwrite A (%s, map %.2f,%.2f)", a >= 0 ? goto_mem_status_name(MM.e[a].status) : "-",
          a >= 0 ? MM.e[a].wx : 0, a >= 0 ? MM.e[a].wz : 0);
}

static void test_memory_say(void)
{
    goto_mem m;
    goto_mem_init(&m);
    char out[600];
    CHECK(goto_mem_say(&m, 1000, 1800, out, sizeof out) == 0 && !strcmp(out, "I don't remember anything yet."), "%s", out);
    goto_mem_motor_pose(&m, 1, 0, 0, 0);
    goto_mem_sight(&m, &C, "trash can", 0, -1200, -90, 1200, 0.8, 1000);     /* 1.2 m to the left */
    goto_mem_sight(&m, &C, "chair", -2000, 0, 180, 2000, 0.8, 1000);        /* behind */
    int i = goto_mem_sight(&m, &C, "cup", 1000, 0, 0, 1000, 0.8, 1000);
    m.e[i].status = MEM_GONE;
    int n = goto_mem_say(&m, 1000, 1800, out, sizeof out);
    CHECK(n == 2 && !strcmp(out, "I remember 2 things: a trash can about 1.2 m to my left, and a chair about 2.0 m behind me."),
          "MEMORY SAY sentence: %s", out);
    CHECK(!strchr(out, '\n'), "one line");
    for (int k = 0; k < 6; k++) goto_mem_sight(&m, &C, "box", 3000 + k * 1000, 3000, 45, 4000, 0.8, 1000);
    n = goto_mem_say(&m, 1000, 1800, out, sizeof out);
    char want[32];
    snprintf(want, sizeof want, "and %d more.", n - 4);
    CHECK(n > 4 && strstr(out, want), "caps at 4 named + 'and N more' (%s)", out);
    goto_mem_sight(&m, &C, "apple", 0, 500, 90, 500, 0.8, 1000);
    goto_mem_say(&m, 1000, 1800, out, sizeof out);
    CHECK(strstr(out, "an apple about 0.5 m to my right"), "'an' before a vowel, nearest first (%s)", out);
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
    test_turn_guard_hiccup();
    test_cancel_and_failures();
    test_retarget_and_filters();
    test_describe();
    test_memory_observe();
    test_memory_goto_hit();
    test_memory_goto_miss();
    test_memory_scan();
    test_memory_file();
    test_memory_cfg();
    test_map_route_around_wall();
    test_map_leg_waits_for_post_move_pose();
    test_map_explore_sees_the_goal();
    test_map_gone_then_check_others();
    test_map_occluded_detour();
    test_map_detour_guard_stop();
    test_map_search_needs_line_of_sight();
    test_map_occluder_inside_footprint();
    test_map_no_route_fails();
    test_map_reveal_measured();
    test_map_guard_escape();
    test_map_long_leg_in_pieces();
    test_map_search_fails_then_memory_route();
    test_map_memory_route_by_route_length();
    test_map_stop_mid_path();
    test_map_guard_and_replan();
    test_leased_approach();
    test_map_fallbacks();
    test_memory_merge_by_map();
    test_memory_say();
    printf("%d/%d checks passed\n", passed, passed + failed);
    return failed ? 1 : 0;
}

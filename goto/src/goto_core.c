/* goto core: see goto_core.h. */
#include "goto_core.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

/* ================================================================ config */
void goto_cfg_defaults(goto_cfg *c)
{
    memset(c, 0, sizeof *c);
    c->hfov_deg = 60.0;            /* Logitech C270 is ~60 deg */
    c->min_conf = 0.30;
    c->search_step_deg = 30.0;
    c->search_max_deg = 360.0;
    c->search_speed = 1.0;
    c->settle_s = 0.35;            /* stepper stops dead; covers camera exposure + latency */
    c->look_frames = 4;
    c->confirm_frames = 2;
    c->frame_timeout_s = 3.0;
    c->align_deg = 12.0;
    c->align_gain = 0.9;
    c->realign_deg = 20.0;
    c->fwd = 1.0;                  /* 28BYJ-48 full speed is only ~41 mm/s */
    c->kp_per_deg = 0.035;
    c->deadband_deg = 2.0;
    c->turn_max = 0.45;
    c->slow_deg = 25.0;
    c->ema_alpha = 0.5;
    c->tick_hz = 10.0;
    c->arrive_h_frac = 0.70;           /* camera ~12 cm up: ~0.5-0.7 m from a 0.4-0.8 m object */
    c->arrive_bottom_frac = 0.97;
    c->arrive_bottom_min_h = 0.15;
    c->arrive_frames = 2;
    c->lost_s = 1.2;
    c->timeout_s = 240.0;          /* 3 m at 4 cm/s is 75 s, plus searching */
    c->move_timeout_pad_s = 3.0;
    c->n_arrive_over = 1;
    snprintf(c->arrive_over_label[0], sizeof c->arrive_over_label[0], "person");
    c->arrive_over_h[0] = 0.80;    /* standing person: ~0.45 m from the legs (top clipped) */
}

double goto_cfg_arrive_h(const goto_cfg *c, const char *label)
{
    for (int i = 0; i < c->n_arrive_over; i++)
        if (label && !strcmp(c->arrive_over_label[i], label)) return c->arrive_over_h[i];
    return c->arrive_h_frac;
}

static char *trim(char *s)
{
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = '\0';
    return s;
}

int goto_cfg_load(goto_cfg *c, const char *path, char *err, size_t errlen)
{
    FILE *f = fopen(path, "r");
    if (err && errlen) err[0] = '\0';
    if (!f) return -1;
    struct { const char *k; double *d; int *i; } keys[] = {
        {"hfov_deg", &c->hfov_deg, 0}, {"min_conf", &c->min_conf, 0},
        {"search_step_deg", &c->search_step_deg, 0}, {"search_max_deg", &c->search_max_deg, 0},
        {"search_speed", &c->search_speed, 0}, {"settle_s", &c->settle_s, 0},
        {"look_frames", 0, &c->look_frames}, {"confirm_frames", 0, &c->confirm_frames},
        {"frame_timeout_s", &c->frame_timeout_s, 0}, {"align_deg", &c->align_deg, 0},
        {"align_gain", &c->align_gain, 0}, {"realign_deg", &c->realign_deg, 0},
        {"fwd", &c->fwd, 0}, {"kp_per_deg", &c->kp_per_deg, 0},
        {"deadband_deg", &c->deadband_deg, 0}, {"turn_max", &c->turn_max, 0},
        {"slow_deg", &c->slow_deg, 0}, {"ema_alpha", &c->ema_alpha, 0},
        {"tick_hz", &c->tick_hz, 0}, {"arrive_h_frac", &c->arrive_h_frac, 0},
        {"arrive_bottom_frac", &c->arrive_bottom_frac, 0},
        {"arrive_bottom_min_h", &c->arrive_bottom_min_h, 0},
        {"arrive_frames", 0, &c->arrive_frames}, {"lost_s", &c->lost_s, 0},
        {"timeout_s", &c->timeout_s, 0}, {"move_timeout_pad_s", &c->move_timeout_pad_s, 0},
    };
    char line[256];
    while (fgets(line, sizeof line, f)) {
        char *h = strchr(line, '#');
        if (h) *h = '\0';
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *k = trim(line), *v = trim(eq + 1);
        if (!*k) continue;
        int found = 0;
        if (!strncmp(k, "arrive_h.", 9)) {
            char lab[64];
            goto_normalize(k + 9, lab, sizeof lab);
            int i;
            for (i = 0; i < c->n_arrive_over; i++)
                if (!strcmp(c->arrive_over_label[i], lab)) break;
            if (i < 16) {
                snprintf(c->arrive_over_label[i], sizeof c->arrive_over_label[i], "%s", lab);
                c->arrive_over_h[i] = atof(v);
                if (i == c->n_arrive_over) c->n_arrive_over++;
            }
            found = 1;
        }
        for (size_t i = 0; !found && i < sizeof keys / sizeof keys[0]; i++) {
            if (strcmp(keys[i].k, k)) continue;
            if (keys[i].d) *keys[i].d = atof(v);
            else *keys[i].i = atoi(v);
            found = 1;
        }
        if (!found && err && errlen && !err[0]) snprintf(err, errlen, "unknown key '%s'", k);
    }
    fclose(f);
    if (c->tick_hz < 5) c->tick_hz = 5;            /* the watchdog is 500 ms */
    if (c->confirm_frames > c->look_frames) c->confirm_frames = c->look_frames;
    if (c->confirm_frames < 1) c->confirm_frames = 1;
    if (c->arrive_frames < 1) c->arrive_frames = 1;
    if (c->ema_alpha <= 0 || c->ema_alpha > 1) c->ema_alpha = 1;
    return 0;
}

/* =============================================================== objects */
static const char *ARTICLES[] = {"the", "a", "an", "my", "that", "this", "those", "these", "your",
                                 "our", "some", "go", "to", "towards", "toward", "over", "there",
                                 "please", "tars", "find", NULL};

void goto_normalize(const char *in, char *out, size_t outlen)
{
    char tmp[256];
    size_t n = 0;
    int sp = 1;
    for (const char *p = in; *p && n + 1 < sizeof tmp; p++) {
        unsigned char ch = (unsigned char)*p;
        if (isalnum(ch)) { tmp[n++] = (char)tolower(ch); sp = 0; }
        else if (ch == '\'') continue;              /* "o'clock", "tars's" */
        else if (!sp) { tmp[n++] = ' '; sp = 1; }
    }
    while (n && tmp[n - 1] == ' ') n--;
    tmp[n] = '\0';
    /* drop filler words */
    size_t o = 0;
    out[0] = '\0';
    char *save = NULL;
    for (char *w = strtok_r(tmp, " ", &save); w; w = strtok_r(NULL, " ", &save)) {
        int skip = 0;
        for (int i = 0; ARTICLES[i]; i++) if (!strcmp(w, ARTICLES[i])) { skip = 1; break; }
        if (skip) continue;
        int k = snprintf(out + o, outlen - o, "%s%s", o ? " " : "", w);
        if (k < 0 || (size_t)k >= outlen - o) break;
        o += (size_t)k;
    }
}

static void add_alias(goto_objects *o, const char *text, int obj)
{
    if (o->n_alias >= GOTO_MAX_ALIAS) return;
    char norm[64];
    goto_normalize(text, norm, sizeof norm);
    if (!norm[0]) return;
    for (int i = 0; i < o->n_alias; i++) if (!strcmp(o->alias[i], norm)) return;   /* first wins */
    snprintf(o->alias[o->n_alias], sizeof o->alias[0], "%s", norm);
    o->alias_obj[o->n_alias++] = obj;
}

static void parse_obj_line(goto_objects *o, char *line)
{
    char *h = strchr(line, '#');
    if (h) *h = '\0';
    char *colon = strchr(line, ':');
    if (colon) *colon = '\0';
    char *name = trim(line);
    if (!*name || o->n_obj >= GOTO_MAX_OBJ) return;
    int idx = o->n_obj++;
    char norm[64];
    goto_normalize(name, norm, sizeof norm);
    snprintf(o->obj[idx], sizeof o->obj[0], "%s", norm);
    add_alias(o, norm, idx);
    if (!colon) return;
    char *save = NULL;
    for (char *a = strtok_r(colon + 1, ",", &save); a; a = strtok_r(NULL, ",", &save))
        add_alias(o, trim(a), idx);
}

/* Keep in sync with goto/objects.txt (used when the file isn't found). */
static const char *DEFAULT_OBJECTS[] = {
    "trash can: bin, trash, garbage, garbage can, trash bin, garbage bin, waste bin, wastebasket, "
        "waste basket, rubbish bin, recycling bin, recycle bin, dustbin, garbage pail",
    "chair: seat, stool, office chair",
    "person: me, human, someone, somebody, man, woman, guy, him, her, people, teammate, judge",
    "backpack: bag, back pack, rucksack, knapsack, school bag",
    "water bottle: bottle, flask, drink, water",
    "door: doorway, door way, exit, entrance",
    "table: desk, dining table, workbench, bench",
    "laptop: computer, notebook, macbook, pc",
    "cup: mug, coffee cup, glass, coffee",
    "box: cardboard box, package, parcel, carton, crate",
    NULL,
};

void goto_objects_defaults(goto_objects *o)
{
    memset(o, 0, sizeof *o);
    char buf[512];
    for (int i = 0; DEFAULT_OBJECTS[i]; i++) {
        snprintf(buf, sizeof buf, "%s", DEFAULT_OBJECTS[i]);
        parse_obj_line(o, buf);
    }
}

int goto_objects_load(goto_objects *o, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    memset(o, 0, sizeof *o);
    char buf[1024];
    while (fgets(buf, sizeof buf, f)) parse_obj_line(o, buf);
    fclose(f);
    return o->n_obj > 0 ? 0 : -1;
}

static int lookup_exact(const goto_objects *o, const char *norm)
{
    for (int i = 0; i < o->n_alias; i++) if (!strcmp(o->alias[i], norm)) return o->alias_obj[i];
    return -1;
}

/* is `needle` inside `hay` on word boundaries? */
static int contains_words(const char *hay, const char *needle)
{
    size_t nl = strlen(needle);
    for (const char *p = hay; (p = strstr(p, needle)) != NULL; p++) {
        int left = p == hay || p[-1] == ' ';
        char r = p[nl];
        if (left && (r == '\0' || r == ' ' || (r == 's' && (p[nl + 1] == '\0' || p[nl + 1] == ' '))))
            return 1;
    }
    return 0;
}

int goto_objects_resolve(const goto_objects *o, const char *phrase)
{
    char norm[256];
    goto_normalize(phrase, norm, sizeof norm);
    if (!norm[0]) return -1;
    int r = lookup_exact(o, norm);
    if (r >= 0) return r;
    size_t n = strlen(norm);
    if (n > 2 && norm[n - 1] == 's') {           /* "chairs", "boxes" */
        char sing[256];
        snprintf(sing, sizeof sing, "%s", norm);
        sing[n - 1] = '\0';
        if ((r = lookup_exact(o, sing)) >= 0) return r;
        if (n > 3 && sing[n - 2] == 'e') { sing[n - 2] = '\0'; if ((r = lookup_exact(o, sing)) >= 0) return r; }
    }
    /* longest alias contained in the phrase */
    size_t best_len = 0;
    int best = -1;
    for (int i = 0; i < o->n_alias; i++) {
        size_t al = strlen(o->alias[i]);
        if (al > best_len && contains_words(norm, o->alias[i])) { best_len = al; best = o->alias_obj[i]; }
    }
    return best;
}

/* ======================================================== JSON frame parser
 * A small recursive-descent parser for exactly what we need. Strings are
 * decoded for the common escapes; \uXXXX becomes '?'. */
typedef struct { const char *p; } jp;

static void jws(jp *j) { while (*j->p == ' ' || *j->p == '\t' || *j->p == '\r' || *j->p == '\n') j->p++; }

static int jstring(jp *j, char *out, size_t len)
{
    jws(j);
    if (*j->p != '"') return -1;
    j->p++;
    size_t n = 0;
    while (*j->p && *j->p != '"') {
        char ch = *j->p++;
        if (ch == '\\') {
            char e = *j->p++;
            switch (e) {
            case 'n': ch = '\n'; break;
            case 't': ch = '\t'; break;
            case 'u': ch = '?'; for (int i = 0; i < 4 && *j->p; i++) j->p++; break;
            case '\0': return -1;
            default: ch = e;
            }
        }
        if (out && n + 1 < len) out[n++] = ch;
    }
    if (*j->p != '"') return -1;
    j->p++;
    if (out && len) out[n] = '\0';
    return 0;
}

static int jnumber(jp *j, double *v)
{
    jws(j);
    char *end;
    double d = strtod(j->p, &end);
    if (end == j->p) return -1;
    j->p = end;
    if (v) *v = d;
    return 0;
}

static int jskip(jp *j, int depth);

static int jskip_container(jp *j, char open, char close, int depth)
{
    if (depth > 16) return -1;
    j->p++;                                 /* open */
    jws(j);
    if (*j->p == close) { j->p++; return 0; }
    for (;;) {
        if (open == '{') {
            if (jstring(j, NULL, 0)) return -1;
            jws(j);
            if (*j->p++ != ':') return -1;
        }
        if (jskip(j, depth + 1)) return -1;
        jws(j);
        if (*j->p == ',') { j->p++; continue; }
        if (*j->p == close) { j->p++; return 0; }
        return -1;
    }
}

static int jskip(jp *j, int depth)
{
    jws(j);
    char ch = *j->p;
    if (ch == '"') return jstring(j, NULL, 0);
    if (ch == '{') return jskip_container(j, '{', '}', depth);
    if (ch == '[') return jskip_container(j, '[', ']', depth);
    if (!strncmp(j->p, "true", 4)) { j->p += 4; return 0; }
    if (!strncmp(j->p, "false", 5)) { j->p += 5; return 0; }
    if (!strncmp(j->p, "null", 4)) { j->p += 4; return 0; }
    return jnumber(j, NULL);
}

static int jdet(jp *j, goto_det *d)
{
    memset(d, 0, sizeof *d);
    d->conf = 1.0;
    int have_box = 0, have_label = 0;
    jws(j);
    if (*j->p != '{') return -1;
    j->p++;
    jws(j);
    if (*j->p == '}') { j->p++; return -1; }
    for (;;) {
        char key[32];
        if (jstring(j, key, sizeof key)) return -1;
        jws(j);
        if (*j->p++ != ':') return -1;
        if (!strcmp(key, "label") || !strcmp(key, "name")) {
            if (jstring(j, d->label, sizeof d->label)) return -1;
            have_label = 1;
        } else if (!strcmp(key, "conf") || !strcmp(key, "score")) {
            if (jnumber(j, &d->conf)) return -1;
        } else if (!strcmp(key, "box") || !strcmp(key, "xyxy")) {
            jws(j);
            if (*j->p++ != '[') return -1;
            for (int i = 0; i < 4; i++) {
                if (jnumber(j, &d->box[i])) return -1;
                jws(j);
                if (i < 3 && *j->p++ != ',') return -1;
            }
            jws(j);
            if (*j->p++ != ']') return -1;
            have_box = 1;
        } else if (jskip(j, 1)) {
            return -1;
        }
        jws(j);
        if (*j->p == ',') { j->p++; continue; }
        if (*j->p == '}') { j->p++; break; }
        return -1;
    }
    return have_box && have_label ? 0 : 1;   /* 1 = well-formed but unusable: skip it */
}

int goto_parse_frame(const char *line, goto_frame *f)
{
    memset(f, 0, sizeof *f);
    jp j = {line};
    int have_dets = 0;
    jws(&j);
    if (*j.p != '{') return -1;
    j.p++;
    jws(&j);
    if (*j.p == '}') return -1;
    for (;;) {
        char key[32];
        double v;
        if (jstring(&j, key, sizeof key)) return -1;
        jws(&j);
        if (*j.p++ != ':') return -1;
        if (!strcmp(key, "t")) { if (jnumber(&j, &f->t)) return -1; }
        else if (!strcmp(key, "w")) { if (jnumber(&j, &v)) return -1; f->w = v >= 1 && v <= 20000 ? (int)v : 0; }
        else if (!strcmp(key, "h")) { if (jnumber(&j, &v)) return -1; f->h = v >= 1 && v <= 20000 ? (int)v : 0; }
        else if (!strcmp(key, "dets")) {
            jws(&j);
            if (*j.p++ != '[') return -1;
            jws(&j);
            if (*j.p == ']') j.p++;
            else for (;;) {
                goto_det d;
                int r = jdet(&j, &d);
                if (r < 0) return -1;
                if (r == 0 && f->n < GOTO_MAX_DETS && isfinite(d.conf) && isfinite(d.box[0]) &&
                    isfinite(d.box[1]) && isfinite(d.box[2]) && isfinite(d.box[3]))
                    f->d[f->n++] = d;
                jws(&j);
                if (*j.p == ',') { j.p++; continue; }
                if (*j.p == ']') { j.p++; break; }
                return -1;
            }
            have_dets = 1;
        } else if (jskip(&j, 1)) {
            return -1;
        }
        jws(&j);
        if (*j.p == ',') { j.p++; continue; }
        if (*j.p == '}') break;
        return -1;
    }
    return have_dets && f->w > 0 && f->h > 0 ? 0 : -1;
}

/* ========================================================= state machine */
const char *goto_state_name(goto_state s)
{
    static const char *n[] = {"idle", "look", "search_turn", "align_turn", "approach"};
    return (unsigned)s < sizeof n / sizeof n[0] ? n[s] : "?";
}

static void emit(goto_sm *s, goto_act_kind k, double a, double b, const char *text)
{
    if (s->n_act >= GOTO_MAX_ACTS) {
        /* never drop a stop: overwrite the last slot */
        if (k != ACT_STOP) return;
        s->n_act = GOTO_MAX_ACTS - 1;
    }
    goto_act *x = &s->act[s->n_act++];
    x->kind = k;
    x->a = a;
    x->b = b;
    snprintf(x->text, sizeof x->text, "%s", text ? text : "");
}

static void emit_event(goto_sm *s, const char *what)
{
    char t[64];
    snprintf(t, sizeof t, "%s %s", what, s->target >= 0 ? s->objs->obj[s->target] : "-");
    emit(s, ACT_EVENT, 0, 0, t);
}

static void set_state(goto_sm *s, goto_state st)
{
    s->st = st;
    s->state_since = s->now;
}

static const char *target_name(const goto_sm *s)
{
    return s->target >= 0 ? s->objs->obj[s->target] : "-";
}

void goto_sm_init(goto_sm *s, const goto_cfg *cfg, const goto_objects *objs)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    s->objs = objs;
    s->target = -1;
    s->move_id = -1;
    snprintf(s->result, sizeof s->result, "-");
}

int goto_sm_busy(const goto_sm *s) { return s->st != ST_IDLE; }

int goto_sm_wants_status(const goto_sm *s)
{
    return s->st == ST_SEARCH_TURN || s->st == ST_ALIGN_TURN;
}

static void start_look(goto_sm *s, double after)
{
    set_state(s, ST_LOOK);
    s->fresh_after = after;
    s->look_seen = s->look_hits = 0;
    s->look_bearing = 0;
}

static void start_turn(goto_sm *s, goto_state st, double deg)
{
    set_state(s, st);
    s->move_id = -1;
    s->move_deadline = s->now + 5.0;           /* until the reply gives the estimate */
    s->cmd_l = s->cmd_r = 0;
    emit(s, ACT_TURN, deg, s->cfg.search_speed, NULL);
}

static void finish(goto_sm *s, const char *result)
{
    snprintf(s->result, sizeof s->result, "%s", result);
    s->cmd_l = s->cmd_r = 0;
    s->have_track = 0;
    set_state(s, ST_IDLE);
}

void goto_sm_goto(goto_sm *s, double now, int target)
{
    s->now = now;
    if (s->st == ST_APPROACH || goto_sm_wants_status(s)) emit(s, ACT_STOP, 0, 0, NULL);
    s->target = target;
    s->started = now;
    s->searched_deg = 0;
    s->search_dir = 1;
    s->have_track = 0;
    s->arrive_count = 0;
    s->last_frame = now;                       /* frame timeout counts from here */
    s->frames = s->frames_with_target = 0;
    snprintf(s->result, sizeof s->result, "running");
    emit(s, ACT_LED, 0, 0, "rec");
    emit(s, ACT_BEEP, 1, 0, NULL);
    emit_event(s, "searching");
    start_look(s, now);                        /* look before turning: it may be in view */
}

void goto_sm_cancel(goto_sm *s, double now, const char *why)
{
    s->now = now;
    emit(s, ACT_STOP, 0, 0, NULL);
    if (s->st == ST_IDLE) return;
    emit(s, ACT_LED, 0, 0, "off");
    char r[64];
    snprintf(r, sizeof r, "cancelled:%s", why ? why : "-");
    emit_event(s, "cancelled");
    finish(s, r);
}

void goto_sm_fail(goto_sm *s, double now, const char *why)
{
    s->now = now;
    emit(s, ACT_STOP, 0, 0, NULL);
    if (s->st == ST_IDLE) return;
    emit(s, ACT_LED, 0, 0, "no");
    emit(s, ACT_BEEP, 3, 0, NULL);
    char r[64];
    snprintf(r, sizeof r, "failed:%s", why ? why : "-");
    for (char *p = r; *p; p++) if (*p == ' ') *p = '_';
    emit_event(s, r);
    finish(s, r);
}

static double bearing_deg(const goto_sm *s, double x)   /* x in -1..1 */
{
    double half = s->cfg.hfov_deg * M_PI / 360.0;
    return atan(x * tan(half)) * 180.0 / M_PI;
}

/* best (highest-confidence) box of the target in this frame, or NULL */
static const goto_det *pick(const goto_sm *s, const goto_frame *f)
{
    const goto_det *best = NULL;
    for (int i = 0; i < f->n; i++) {
        const goto_det *d = &f->d[i];
        if (d->conf < s->cfg.min_conf) continue;
        if (d->box[2] <= d->box[0] || d->box[3] <= d->box[1]) continue;
        if (goto_objects_resolve(s->objs, d->label) != s->target) continue;
        if (!best || d->conf > best->conf) best = d;
    }
    return best;
}

static void begin_approach(goto_sm *s)
{
    set_state(s, ST_APPROACH);
    s->arrive_count = 0;
    s->last_seen = s->now;
    s->last_tick = 0;                         /* send D on the next tick */
}

static void after_sighting(goto_sm *s, double bearing)
{
    s->searched_deg = 0;                      /* a fresh 360 if we lose it later */
    s->search_dir = bearing >= 0 ? 1 : -1;
    if (fabs(bearing) > s->cfg.align_deg) {
        start_turn(s, ST_ALIGN_TURN, bearing * s->cfg.align_gain);
    } else {
        begin_approach(s);
    }
}

static int arrive_test(const goto_sm *s, const goto_det *d, const goto_frame *f)
{
    double h = (d->box[3] - d->box[1]) / f->h, bottom = d->box[3] / f->h;
    double need = goto_cfg_arrive_h(&s->cfg, target_name(s));
    if (h >= need || (s->have_track && s->ema_h >= need)) return 1;
    if (bottom >= s->cfg.arrive_bottom_frac && h >= s->cfg.arrive_bottom_min_h) return 1;
    return 0;
}

void goto_sm_frame(goto_sm *s, double now, const goto_frame *f)
{
    s->now = now;
    double dt = now - s->last_frame_for_fps;
    if (s->last_frame_for_fps > 0 && dt > 0 && dt < 5)
        s->fps_ema = s->fps_ema ? 0.8 * s->fps_ema + 0.2 / dt : 1.0 / dt;
    s->last_frame_for_fps = now;
    s->last_frame = now;
    if (s->st == ST_IDLE) return;
    s->frames++;
    const goto_det *d = pick(s, f);
    if (d) s->frames_with_target++;
    double x = d ? ((d->box[0] + d->box[2]) / 2.0 / f->w - 0.5) * 2.0 : 0;
    double h = d ? (d->box[3] - d->box[1]) / f->h : 0;

    switch (s->st) {
    case ST_LOOK:
        if (now < s->fresh_after) return;
        s->look_seen++;
        if (d) {
            s->look_hits++;
            s->look_bearing += bearing_deg(s, x);
            s->ema_x = x;                    /* seed the track */
            s->ema_h = h;
            s->have_track = 1;
            if (arrive_test(s, d, f) && s->look_hits >= s->cfg.confirm_frames) {
                emit(s, ACT_STOP, 0, 0, NULL);
                emit(s, ACT_LED, 0, 0, "ok");
                emit(s, ACT_BEEP, 1, 0, NULL);
                emit_event(s, "arrived");
                finish(s, "arrived");
                return;
            }
        }
        if (s->look_hits >= s->cfg.confirm_frames) {
            double b = s->look_bearing / s->look_hits;
            s->last_bearing = b;
            after_sighting(s, b);
        } else if (s->look_seen >= s->cfg.look_frames) {
            if (s->searched_deg + 1e-6 >= s->cfg.search_max_deg) {
                goto_sm_fail(s, now, "not found");
                return;
            }
            double step = s->cfg.search_step_deg;
            if (s->searched_deg + step > s->cfg.search_max_deg) step = s->cfg.search_max_deg - s->searched_deg;
            s->searched_deg += step;
            start_turn(s, ST_SEARCH_TURN, step * s->search_dir);
        }
        return;
    case ST_APPROACH:
        if (!d) return;                      /* lost is handled by the tick */
        s->last_seen = now;
        if (!s->have_track) { s->ema_x = x; s->ema_h = h; s->have_track = 1; }
        else {
            double a = s->cfg.ema_alpha;
            s->ema_x = a * x + (1 - a) * s->ema_x;
            s->ema_h = a * h + (1 - a) * s->ema_h;
        }
        s->last_bearing = bearing_deg(s, s->ema_x);
        s->search_dir = s->last_bearing >= 0 ? 1 : -1;
        if (arrive_test(s, d, f)) {
            if (++s->arrive_count >= s->cfg.arrive_frames) {
                emit(s, ACT_STOP, 0, 0, NULL);
                emit(s, ACT_LED, 0, 0, "ok");
                emit(s, ACT_BEEP, 1, 0, NULL);
                emit_event(s, "arrived");
                finish(s, "arrived");
            }
            return;
        }
        s->arrive_count = 0;
        if (fabs(s->last_bearing) > s->cfg.realign_deg) {
            emit(s, ACT_STOP, 0, 0, NULL);
            start_turn(s, ST_ALIGN_TURN, s->last_bearing * s->cfg.align_gain);
        }
        return;
    default:
        return;                              /* turning: frames are blurred/stale */
    }
}

void goto_sm_tick(goto_sm *s, double now)
{
    s->now = now;
    if (s->st == ST_IDLE) return;
    if (now - s->started > s->cfg.timeout_s) { goto_sm_fail(s, now, "timeout"); return; }
    if (s->st != ST_SEARCH_TURN && s->st != ST_ALIGN_TURN && now - s->last_frame > s->cfg.frame_timeout_s) {
        goto_sm_fail(s, now, "no camera frames");
        return;
    }
    double period = 1.0 / s->cfg.tick_hz;
    int due = now - s->last_tick >= period * 0.95;
    switch (s->st) {
    case ST_SEARCH_TURN:
    case ST_ALIGN_TURN:
        if (now > s->move_deadline) { goto_sm_fail(s, now, "turn timed out"); return; }
        if (due) { emit(s, ACT_DRIVE, 0, 0, NULL); s->last_tick = now; }   /* keepalive, no abort */
        return;
    case ST_APPROACH: {
        if (now - s->last_seen > s->cfg.lost_s) {
            emit(s, ACT_STOP, 0, 0, NULL);
            emit_event(s, "lost");
            s->have_track = 0;
            s->searched_deg = 0;
            s->cmd_l = s->cmd_r = 0;
            start_look(s, now + s->cfg.settle_s);
            return;
        }
        if (!due) return;
        s->last_tick = now;
        double b = s->last_bearing;
        double turn = fabs(b) <= s->cfg.deadband_deg ? 0.0
                    : clampd(s->cfg.kp_per_deg * (b - (b > 0 ? s->cfg.deadband_deg : -s->cfg.deadband_deg)),
                             -s->cfg.turn_max, s->cfg.turn_max);
        double fwd = s->cfg.fwd * clampd(1.0 - fabs(b) / s->cfg.slow_deg, 0.0, 1.0);
        double l = fwd + turn, r = fwd - turn;
        double m = fmax(fabs(l), fabs(r));
        if (m > 1.0) { l /= m; r /= m; }
        s->cmd_l = l;
        s->cmd_r = r;
        emit(s, ACT_DRIVE, l, r, NULL);
        return;
    }
    default:
        return;
    }
}

void goto_sm_move_started(goto_sm *s, double now, int move_id, double est_s)
{
    s->now = now;
    if (!goto_sm_wants_status(s)) return;
    s->move_id = move_id;
    s->move_deadline = now + est_s * 2.0 + s->cfg.move_timeout_pad_s;
}

void goto_sm_move_status(goto_sm *s, double now, int move_id, const char *move, const char *why)
{
    s->now = now;
    if (!goto_sm_wants_status(s) || s->move_id < 0 || move_id != s->move_id) return;
    if (!strcmp(move, "running")) return;
    if (!strcmp(move, "done")) {
        start_look(s, now + s->cfg.settle_s);
        return;
    }
    /* aborted / idle (replaced): obstacle, bump, tip, stop from someone else */
    char r[48];
    snprintf(r, sizeof r, "turn %s", why && *why ? why : move);
    if (why && !strcmp(why, "stop")) goto_sm_cancel(s, now, "stop");
    else goto_sm_fail(s, now, r);
}

void goto_sm_describe(const goto_sm *s, char *out, size_t len)
{
    char tn[40];                              /* "trash can" -> "trash_can" keeps key=val parsable */
    snprintf(tn, sizeof tn, "%s", target_name(s));
    for (char *p = tn; *p; p++) if (*p == ' ') *p = '_';
    snprintf(out, len,
             "state=%s target=%s result=%s seen=%d bearing=%.1f h=%.2f searched=%.0f fps=%.1f "
             "elapsed=%.1f frames=%d hits=%d l=%.2f r=%.2f",
             goto_state_name(s->st), tn, s->result,
             s->st == ST_APPROACH && s->now - s->last_seen < 0.5, s->last_bearing,
             s->have_track ? s->ema_h : 0.0, s->searched_deg, s->fps_ema,
             s->st != ST_IDLE ? s->now - s->started : 0.0, s->frames, s->frames_with_target,
             s->cmd_l, s->cmd_r);
}

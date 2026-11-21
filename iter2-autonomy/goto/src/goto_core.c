/* goto core: see goto_core.h. */
#include "goto_core.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
static double wrap180(double d)
{
    d = fmod(d, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d <= -180.0) d += 360.0;
    return d;
}
#define RAD(d) ((d) * M_PI / 180.0)
#define DEG(r) ((r) * 180.0 / M_PI)

/* ================================================================ config */
void goto_cfg_defaults(goto_cfg *c)
{
    memset(c, 0, sizeof *c);
    c->hfov_deg = 60.0;            /* Logitech C270 is ~60 deg */
    c->min_conf = 0.30;
    c->search_step_deg = 30.0;
    c->search_max_deg = 360.0;
    c->search_speed = 0.40;  /* <= tars_motor PIVOT_BLOCKED_MAX_SPEED: survives a mid-turn BLOCKED */
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
    c->arrive_dist_mm = 550.0;     /* matches the arrive_h_frac comment: ~0.5-0.7 m */
    c->memory_min_conf = 0.40;
    c->memory_max_age_s = 1800.0;  /* 30 min */
    c->memory_drive_mm = 0.0;      /* off: drive blind toward a remembered spot only if you set it */
    c->memory_save_s = 2.0;
    c->memory_merge_mm = 500.0;
    c->memory_tries = 3;
    c->cam_height_mm = 120.0;      /* README: webcam ~12 cm up, level */
    c->cam_pitch_deg = 0.0;
    c->nav_pose_fresh_s = 0.5;
    c->nav_turn_min_deg = 4.0;
    c->nav_leg_min_mm = 60.0;
    c->nav_max_replans = 6;
    c->nav_guard_wait_s = 0.5;
    c->nav_reveal_deg = 30.0;
    snprintf(c->memory_path, sizeof c->memory_path, "memory.txt");
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
        {"arrive_dist_mm", &c->arrive_dist_mm, 0}, {"memory_min_conf", &c->memory_min_conf, 0},
        {"memory_max_age_s", &c->memory_max_age_s, 0}, {"memory_drive_mm", &c->memory_drive_mm, 0},
        {"memory_save_s", &c->memory_save_s, 0}, {"memory_merge_mm", &c->memory_merge_mm, 0},
        {"memory_tries", &c->memory_tries, 0}, {"cam_height_mm", &c->cam_height_mm, 0},
        {"cam_pitch_deg", &c->cam_pitch_deg, 0},
        {"nav_pose_fresh_s", &c->nav_pose_fresh_s, 0}, {"nav_turn_min_deg", &c->nav_turn_min_deg, 0},
        {"nav_leg_min_mm", &c->nav_leg_min_mm, 0}, {"nav_max_replans", &c->nav_max_replans, 0},
        {"nav_guard_wait_s", &c->nav_guard_wait_s, 0}, {"nav_reveal_deg", &c->nav_reveal_deg, 0},
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
        if (!strcmp(k, "memory_path")) {
            snprintf(c->memory_path, sizeof c->memory_path, "%s", v);
            found = 1;
        } else if (!strncmp(k, "arrive_h.", 9)) {
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
    if (c->memory_save_s < 0.5) c->memory_save_s = 0.5;
    if (c->memory_drive_mm < 0) c->memory_drive_mm = 0;
    if (c->memory_drive_mm > 1000) c->memory_drive_mm = 1000;     /* tars_motor's MOVE limit */
    if (!c->memory_path[0]) snprintf(c->memory_path, sizeof c->memory_path, "-");
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

/* Keep in sync with iter2-autonomy/goto/objects.txt (used when the file isn't found). */
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

/* ========================================================= object memory */
static double bearing_of(const goto_cfg *c, double x)    /* x in -1..1 -> deg, + = right */
{
    double half = c->hfov_deg * M_PI / 360.0;
    return atan(x * tan(half)) * 180.0 / M_PI;
}

void goto_mem_init(goto_mem *m) { memset(m, 0, sizeof *m); }

void goto_mem_label(const goto_objects *o, const char *raw, char *out, size_t len)
{
    int k = o ? goto_objects_resolve(o, raw) : -1;
    if (k >= 0) snprintf(out, len, "%s", o->obj[k]);
    else goto_normalize(raw, out, len);
}

void goto_mem_motor_pose(goto_mem *m, unsigned epoch, double mx, double my, double mh)
{
    if (!isfinite(mx) || !isfinite(my) || !isfinite(mh)) return;
    if (!m->have_offset || epoch != m->epoch) {
        /* new tars_motor process (or POSE RESET): assume the robot is still where
         * we last knew it was, so the old memory stays valid */
        if (m->have_pose) {
            m->oh = wrap180(m->ph - mh);
            double c = cos(RAD(m->oh)), sn = sin(RAD(m->oh));
            m->ox = m->px - (c * mx - sn * my);
            m->oy = m->py - (sn * mx + c * my);
            if (m->have_offset) m->rebased++;
        } else {
            m->ox = m->oy = m->oh = 0;
        }
        m->epoch = epoch;
        m->have_offset = 1;
        m->dirty = 1;
    }
    double c = cos(RAD(m->oh)), sn = sin(RAD(m->oh));
    m->px = c * mx - sn * my + m->ox;
    m->py = sn * mx + c * my + m->oy;
    m->ph = wrap180(mh + m->oh);
    m->have_pose = 1;
    if (hypot(m->px - m->spx, m->py - m->spy) > 20.0 || fabs(wrap180(m->ph - m->sph)) > 2.0) m->dirty = 1;
}

const char *goto_mem_status_name(goto_mem_status st)
{
    return st == MEM_FRESH ? "fresh" : st == MEM_STALE ? "stale" : st == MEM_GONE ? "gone" : "?";
}

int goto_mem_find(const goto_mem *m, const char *label)
{
    for (int i = 0; i < m->n; i++) if (!strcmp(m->e[i].label, label)) return i;
    return -1;
}

int goto_mem_find_id(const goto_mem *m, const char *label, int id)
{
    for (int i = 0; i < m->n; i++) if (m->e[i].id == id && !strcmp(m->e[i].label, label)) return i;
    return -1;
}

int goto_mem_count(const goto_mem *m, const char *label)
{
    int n = 0;
    for (int i = 0; i < m->n; i++) n += !strcmp(m->e[i].label, label);
    return n;
}

/* a new instance of `label` (evicts GONE first, then the oldest when full) */
static goto_mem_entry *mem_new(goto_mem *m, const char *label)
{
    int i;
    if (m->n < GOTO_MEM_MAX) i = m->n++;
    else {
        i = 0;
        for (int k = 1; k < m->n; k++) {
            int gk = m->e[k].status == MEM_GONE, gi = m->e[i].status == MEM_GONE;
            if (gk > gi || (gk == gi && m->e[k].last_seen < m->e[i].last_seen)) i = k;
        }
    }
    int id = 1;
    for (int k = 0; k < m->n; k++)
        if (k != i && !strcmp(m->e[k].label, label) && m->e[k].id >= id) id = m->e[k].id + 1;
    memset(&m->e[i], 0, sizeof m->e[i]);
    snprintf(m->e[i].label, sizeof m->e[i].label, "%s", label);
    m->e[i].id = id;
    return &m->e[i];
}

double goto_mem_box_dist(const goto_cfg *c, const char *label, const goto_det *d, int fw, int fh)
{
    double f = (fw / 2.0) / tan(c->hfov_deg * M_PI / 360.0);   /* px, square pixels */
    double x = (d->box[0] + d->box[2]) / 2.0 - fw / 2.0;
    double below = atan((d->box[3] - fh / 2.0) / f) + RAD(c->cam_pitch_deg);
    /* floor contact: object stands on the floor and the bottom edge isn't cut by the frame */
    if (c->cam_height_mm > 0 && d->box[3] < 0.98 * fh && below > RAD(1.5)) {
        double fwd = c->cam_height_mm / tan(below);
        return clampd(fwd / cos(atan(x / f)), 150.0, 6000.0);
    }
    /* fallback: box height vs the arrival calibration (worse: top often cut off) */
    double h = (d->box[3] - d->box[1]) / fh;
    return clampd(c->arrive_dist_mm * goto_cfg_arrive_h(c, label) / fmax(h, 1e-3), 150.0, 6000.0);
}

/* Where a sighting is in the phone's map (metres), for matching it against remembered
 * instances. Only with a FRESH ARKit pose (have_nav_arkit): dead reckoning on a DC car can be
 * metres off, and a nav pose carried by DR is no better. rel_deg + = right, dist mm. */
typedef struct { int ok; double wx, wz, rel_deg, dist; } mem_wpos;

static mem_wpos mem_wpos_of(const goto_mem *m, double rel_deg, double dist_mm)
{
    mem_wpos w = {0, 0, 0, rel_deg, dist_mm};
    if (!m->have_nav || !m->have_nav_arkit) return w;
    double yaw = m->nyaw - RAD(rel_deg);
    w.wx = m->nx + dist_mm / 1000.0 * sin(yaw);
    w.wz = m->nz + dist_mm / 1000.0 * cos(yaw);
    w.ok = 1;
    return w;
}

/* Is entry e this sighting? Same label assumed. Returns the match distance (mm) or -1.
 * With a map position on both sides, the map decides (DR x/y is ignored: on a DC car it's off
 * by metres, and a bin that moved from A to B would refresh A). Otherwise the DR world x/y. */
static double mem_match(const goto_mem *m, const goto_cfg *c, const goto_mem_entry *e, double x, double y,
                        double bearing, double dist, const mem_wpos *w)
{
    if (w && w->ok && e->has_w) {
        double dw = hypot(e->wx - w->wx, e->wz - w->wz) * 1000.0;
        if (dw <= c->memory_merge_mm) return dw;
        double ex = e->wx - m->nx, ez = e->wz - m->nz, er = hypot(ex, ez) * 1000.0;
        double eb = wrap180(-DEG(atan2(ex, ez) - m->nyaw));          /* relative, + = right */
        int ray = er > 1 && fabs(wrap180(eb - w->rel_deg)) < 6.0 && er / w->dist > 0.65 && er / w->dist < 1.55;
        return ray ? dw : -1;
    }
    double dw = hypot(e->x - x, e->y - y);
    if (dw <= c->memory_merge_mm) return dw;
    if (m->have_pose) {
        double ex = e->x - m->px, ey = e->y - m->py, er = hypot(ex, ey);
        double eb = DEG(atan2(ey, ex));
        if (er > 1 && fabs(wrap180(eb - bearing)) < 6.0 && er / dist > 0.65 && er / dist < 1.55) return dw;
    }
    return -1;
}

static int mem_sight_w(goto_mem *m, const goto_cfg *c, const char *label, double x, double y,
                       double bearing, double dist, double conf, double wall, const mem_wpos *w)
{
    /* match: same label, and close in the world OR on the same ray within the range noise */
    int best = -1;
    double best_d = 1e18;
    for (int i = 0; i < m->n; i++) {
        goto_mem_entry *e = &m->e[i];
        if (strcmp(e->label, label)) continue;
        double dw = mem_match(m, c, e, x, y, bearing, dist, w);
        if (dw >= 0 && dw < best_d) { best_d = dw; best = i; }
    }
    goto_mem_entry *e;
    if (best >= 0) {
        e = &m->e[best];
        double recent = wall - e->last_seen;
        double a = recent >= 0 && recent < 60.0 && e->status == MEM_FRESH ? 0.4 : 0.7;
        e->x = (1 - a) * e->x + a * x;        /* smooth (the distance is noisy) */
        e->y = (1 - a) * e->y + a * y;
    } else {
        e = mem_new(m, label);
        e->x = x;
        e->y = y;
    }
    e->bearing = bearing;
    e->dist = dist;
    e->conf = conf;
    e->seen++;
    e->last_seen = wall;
    e->status = MEM_FRESH;
    m->dirty = 1;
    return (int)(e - m->e);
}

int goto_mem_sight(goto_mem *m, const goto_cfg *c, const char *label, double x, double y,
                   double bearing, double dist, double conf, double wall)
{
    return mem_sight_w(m, c, label, x, y, bearing, dist, conf, wall, NULL);
}

/* map position of a sighting at relative bearing rel_deg (+ right), dist mm, from the nav pose */
static void mem_set_w(goto_mem *m, goto_mem_entry *e, double rel_deg, double dist_mm, int smooth)
{
    if (!m->have_nav) return;
    double yaw = m->nyaw - RAD(rel_deg);      /* + yaw = left turn, + bearing = right */
    double wx = m->nx + dist_mm / 1000.0 * sin(yaw), wz = m->nz + dist_mm / 1000.0 * cos(yaw);
    if (smooth && e->has_w) { e->wx = 0.6 * e->wx + 0.4 * wx; e->wz = 0.6 * e->wz + 0.4 * wz; }
    else { e->wx = wx; e->wz = wz; }
    e->has_w = 1;
}

int goto_mem_observe(goto_mem *m, const goto_cfg *c, const goto_objects *o, const goto_frame *f, double wall)
{
    if (!m->have_pose || f->w <= 0 || f->h <= 0) return 0;
    int used = 0, taken[GOTO_MEM_MAX] = {0};
    for (int i = 0; i < f->n; i++) {
        const goto_det *d = &f->d[i];
        if (d->conf < c->memory_min_conf || d->box[2] <= d->box[0] || d->box[3] <= d->box[1]) continue;
        char l[32];
        goto_mem_label(o, d->label, l, sizeof l);
        if (!l[0]) continue;
        /* the same box twice (NMS leftovers): skip if a stronger same-label box overlaps it */
        int dup = 0;
        for (int k = 0; k < f->n && !dup; k++) {
            const goto_det *q = &f->d[k];
            if (k == i || q->conf < d->conf || (q->conf == d->conf && k > i)) continue;
            char lq[32];
            goto_mem_label(o, q->label, lq, sizeof lq);
            double ix = fmin(d->box[2], q->box[2]) - fmax(d->box[0], q->box[0]);
            dup = !strcmp(l, lq) && ix > 0.5 * (d->box[2] - d->box[0]);
        }
        if (dup) continue;
        double x = ((d->box[0] + d->box[2]) / 2.0 / f->w - 0.5) * 2.0;
        double dist = goto_mem_box_dist(c, l, d, f->w, f->h);
        double wb = wrap180(m->ph + bearing_of(c, x));
        double ox = m->px + dist * cos(RAD(wb)), oy = m->py + dist * sin(RAD(wb));
        mem_wpos w = mem_wpos_of(m, bearing_of(c, x), dist);
        int k = mem_sight_w(m, c, l, ox, oy, wb, dist, d->conf, wall, &w);
        /* two boxes of one label in one frame are two instances */
        if (k >= 0 && k < GOTO_MEM_MAX && taken[k]) {
            goto_mem_entry *e = mem_new(m, l);
            e->x = ox; e->y = oy; e->bearing = wb; e->dist = dist; e->conf = d->conf;
            e->seen = 1; e->last_seen = wall; e->status = MEM_FRESH;
            k = (int)(e - m->e);
        }
        if (k >= 0 && k < GOTO_MEM_MAX) {
            mem_set_w(m, &m->e[k], bearing_of(c, x), dist, !taken[k] && m->e[k].seen > 1);
            taken[k] = 1;
        }
        used++;
    }
    return used;
}

int goto_mem_best(const goto_mem *m, const char *label, double wall, double max_age,
                  const int *skip, int nskip)
{
    int best = -1;
    double best_score = 1e18;
    for (int i = 0; i < m->n; i++) {
        const goto_mem_entry *e = &m->e[i];
        if (strcmp(e->label, label) || e->status == MEM_GONE) continue;
        double age = wall - e->last_seen;
        if (age < 0) age = 0;                  /* clock went backwards (no RTC): trust it */
        if (age > max_age) continue;
        int sk = 0;
        for (int k = 0; k < nskip; k++) sk |= skip[k] == e->id;
        if (sk) continue;
        double d = m->have_pose ? hypot(e->x - m->px, e->y - m->py) : 0;
        double score = (e->status == MEM_STALE ? 1e9 : 0) + d;
        if (score < best_score) { best_score = score; best = i; }
    }
    return best;
}

void goto_mem_miss(goto_mem *m, int i)
{
    if (i < 0 || i >= m->n) return;
    m->e[i].status = m->e[i].status == MEM_FRESH ? MEM_STALE : MEM_GONE;
    m->dirty = 1;
}

int goto_mem_forget(goto_mem *m, const char *label)
{
    if (!label) { int n = m->n; m->n = 0; m->dirty = 1; return n; }
    char lab[40];
    snprintf(lab, sizeof lab, "%s", label);
    int id = 0;
    char *h = strchr(lab, '#');
    if (h) {
        id = atoi(h + 1);
        while (h > lab && h[-1] == ' ') h--;
        *h = '\0';
        if (id <= 0) return 0;
    }
    int n = 0;
    for (int i = 0; i < m->n;) {
        if (!strcmp(m->e[i].label, lab) && (!id || m->e[i].id == id)) {
            memmove(&m->e[i], &m->e[i + 1], (size_t)(m->n - i - 1) * sizeof m->e[0]);
            m->n--;
            n++;
        } else i++;
    }
    if (n) m->dirty = 1;
    return n;
}

void goto_mem_relative(const goto_mem *m, int i, double *bearing, double *dist)
{
    const goto_mem_entry *e = &m->e[i];
    double dx = e->x - m->px, dy = e->y - m->py, d = hypot(dx, dy);
    /* right on top of it: the direction to the point means nothing, use the sighting's */
    double wb = d < 250.0 ? e->bearing : DEG(atan2(dy, dx));
    if (bearing) *bearing = wrap180(wb - m->ph);
    if (dist) *dist = d;
}

int goto_mem_save(goto_mem *m, const char *path, double wall)
{
    if (!path || !path[0] || !strcmp(path, "-")) { m->dirty = 0; return 0; }
    char tmp[300];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return -1;
    fprintf(f, "# goto object memory (goto/README.md). Safe to delete. mm, deg (+ = clockwise)\n");
    fprintf(f, "# inst <id> <fresh|stale|gone> <x> <y> <bearing> <dist> <conf> <seen> <last_seen> <label>\n");
    fprintf(f, "# navw <id> <x> <z> <label>   map (ARKit) position, metres, if it was seen with a good nav pose\n");
    fprintf(f, "v 2\n");
    if (m->have_offset)
        fprintf(f, "pose %u %.1f %.1f %.2f %.1f %.1f %.3f %.1f\n", m->epoch, m->px, m->py, m->ph,
                m->ox, m->oy, m->oh, wall);
    for (int i = 0; i < m->n; i++) {
        const goto_mem_entry *e = &m->e[i];
        fprintf(f, "inst %d %s %.1f %.1f %.2f %.0f %.3f %d %.1f %s\n", e->id, goto_mem_status_name(e->status),
                e->x, e->y, e->bearing, e->dist, e->conf, e->seen, e->last_seen, e->label);
        if (e->has_w) fprintf(f, "navw %d %.3f %.3f %s\n", e->id, e->wx, e->wz, e->label);
    }
    int ok = fflush(f) == 0 && !ferror(f);
    if (ok) fsync(fileno(f));
    if (fclose(f) != 0) ok = 0;
    if (!ok || rename(tmp, path) != 0) { remove(tmp); return -1; }
    m->dirty = 0;
    m->spx = m->px; m->spy = m->py; m->sph = m->ph;
    return 0;
}

int goto_mem_load(goto_mem *m, const char *path)
{
    goto_mem_init(m);
    if (!path || !path[0] || !strcmp(path, "-")) return -1;
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[512];
    int bad = 0, version_ok = 0;
    while (fgets(line, sizeof line, f)) {
        char *l = trim(line);
        if (!*l || *l == '#') continue;
        int n = 0;
        if (!strncmp(l, "v ", 2)) { version_ok = atoi(l + 2) == 2; if (!version_ok) bad++; continue; }
        if (!version_ok) { bad++; continue; }
        if (!strncmp(l, "pose ", 5)) {
            unsigned ep;
            double v[7];
            if (sscanf(l, "pose %u %lf %lf %lf %lf %lf %lf %lf%n", &ep, &v[0], &v[1], &v[2], &v[3], &v[4],
                       &v[5], &v[6], &n) == 8 && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]) &&
                isfinite(v[3]) && isfinite(v[4]) && isfinite(v[5]) && fabs(v[0]) < 1e7 && fabs(v[1]) < 1e7 &&
                fabs(v[3]) < 1e7 && fabs(v[4]) < 1e7) {
                m->epoch = ep;
                m->px = v[0]; m->py = v[1]; m->ph = wrap180(v[2]);
                m->ox = v[3]; m->oy = v[4]; m->oh = wrap180(v[5]);
                m->spx = m->px; m->spy = m->py; m->sph = m->ph;
                m->have_pose = m->have_offset = 1;
            } else bad++;
            continue;
        }
        if (!strncmp(l, "navw ", 5)) {
            int id;
            double wx, wz;
            if (sscanf(l, "navw %d %lf %lf %n", &id, &wx, &wz, &n) == 3 && n > 0 && l[n] && isfinite(wx) &&
                isfinite(wz) && fabs(wx) < 1000 && fabs(wz) < 1000) {
                int i = goto_mem_find_id(m, l + n, id);
                if (i >= 0) { m->e[i].has_w = 1; m->e[i].wx = wx; m->e[i].wz = wz; }
                else bad++;
            } else bad++;
            continue;
        }
        if (!strncmp(l, "inst ", 5)) {
            goto_mem_entry e;
            char st[16];
            memset(&e, 0, sizeof e);
            if (sscanf(l, "inst %d %15s %lf %lf %lf %lf %lf %d %lf %n", &e.id, st, &e.x, &e.y, &e.bearing, &e.dist,
                       &e.conf, &e.seen, &e.last_seen, &n) == 9 && n > 0 && l[n] && e.id > 0 &&
                isfinite(e.x) && isfinite(e.y) && isfinite(e.bearing) && isfinite(e.dist) &&
                isfinite(e.conf) && isfinite(e.last_seen) && fabs(e.x) < 1e7 && fabs(e.y) < 1e7 &&
                e.seen >= 0 && m->n < GOTO_MEM_MAX &&
                (!strcmp(st, "fresh") || !strcmp(st, "stale") || !strcmp(st, "gone"))) {
                snprintf(e.label, sizeof e.label, "%s", l + n);
                e.status = !strcmp(st, "fresh") ? MEM_FRESH : !strcmp(st, "stale") ? MEM_STALE : MEM_GONE;
                e.bearing = wrap180(e.bearing);
                if (goto_mem_find_id(m, e.label, e.id) < 0) m->e[m->n++] = e;
                else bad++;
            } else bad++;
            continue;
        }
        bad++;
    }
    fclose(f);
    m->dirty = 0;
    return bad ? 1 : 0;
}

void goto_mem_describe(const goto_mem *m, double wall, char *out, size_t len)
{
    size_t o = (size_t)snprintf(out, len, "n=%d pose=", m->n);
    if (o >= len) return;
    if (m->have_pose) o += (size_t)snprintf(out + o, len - o, "%.0f,%.0f,%.1f", m->px, m->py, m->ph);
    else o += (size_t)snprintf(out + o, len - o, "-");
    for (int i = 0; i < m->n && o < len; i++) {
        const goto_mem_entry *e = &m->e[i];
        double b = 0, d = 0;
        if (m->have_pose) goto_mem_relative(m, i, &b, &d);
        o += (size_t)snprintf(out + o, len - o,
                              "; %s#%d %s x=%.0f y=%.0f rel_bearing=%.0f rel_dist=%.0f est_dist=%.0f age=%.0f seen=%d conf=%.2f",
                              e->label, e->id, goto_mem_status_name(e->status), e->x, e->y, b, d, e->dist,
                              fmax(0, wall - e->last_seen), e->seen, e->conf);
        if (e->has_w && o < len) o += (size_t)snprintf(out + o, len - o, " map=%.2f,%.2f", e->wx, e->wz);
    }
}

static const char *say_dir(double b)          /* relative bearing, + = right */
{
    double a = fabs(b);
    if (a <= 20) return "ahead of me";
    if (a <= 65) return b > 0 ? "ahead on my right" : "ahead on my left";
    if (a <= 115) return b > 0 ? "to my right" : "to my left";
    if (a <= 160) return b > 0 ? "behind me on the right" : "behind me on the left";
    return "behind me";
}

int goto_mem_say(const goto_mem *m, double wall, double max_age, char *out, size_t len)
{
    int idx[GOTO_MEM_MAX], n = 0;
    for (int i = 0; i < m->n; i++) {
        double age = wall - m->e[i].last_seen;
        if (m->e[i].status != MEM_GONE && age <= max_age) idx[n++] = i;
    }
    if (!n) { snprintf(out, len, "I don't remember anything yet."); return 0; }
    for (int i = 1; i < n; i++)                     /* nearest first */
        for (int j = i; j > 0; j--) {
            double dj = 0, dk = 0;
            if (m->have_pose) { goto_mem_relative(m, idx[j], NULL, &dj); goto_mem_relative(m, idx[j - 1], NULL, &dk); }
            if (dj >= dk) break;
            int t = idx[j]; idx[j] = idx[j - 1]; idx[j - 1] = t;
        }
    int said = n < 4 ? n : 4;
    size_t o = (size_t)snprintf(out, len, "I remember %d thing%s: ", n, n == 1 ? "" : "s");
    for (int k = 0; k < said && o < len; k++) {
        const goto_mem_entry *e = &m->e[idx[k]];
        const char *sep = k == 0 ? "" : (k == said - 1 && said == n) ? ", and " : ", ";
        const char *art = strchr("aeiou", e->label[0]) ? "an" : "a";
        if (m->have_pose) {
            double b, d;
            goto_mem_relative(m, idx[k], &b, &d);
            o += (size_t)snprintf(out + o, len - o, "%s%s %s%s about %.1f m %s", sep, art, e->label,
                                  e->status == MEM_STALE ? " (not sure)" : "", d / 1000.0, say_dir(b));
        } else {
            o += (size_t)snprintf(out + o, len - o, "%s%s %s", sep, art, e->label);
        }
    }
    if (o < len) {
        if (n > said) snprintf(out + o, len - o, ", and %d more.", n - said);
        else snprintf(out + o, len - o, ".");
    }
    return n;
}

/* ========================================================= state machine */
const char *goto_state_name(goto_state s)
{
    static const char *n[] = {"idle", "look", "search_turn", "align_turn", "approach", "mem_turn", "mem_move",
                              "recall", "plan", "follow_path", "occluded", "check_others", "detour",
                              "next_candidate"};
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
    s->leg_settle = 0;
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
    return s->st == ST_SEARCH_TURN || s->st == ST_ALIGN_TURN || s->st == ST_MEM_TURN || s->st == ST_MEM_MOVE ||
           s->st == ST_FOLLOW_PATH;
}

static double wall_now(const goto_sm *s) { return s->wall_base + s->now; }

void goto_sm_set_memory(goto_sm *s, goto_mem *m, double wall_base)
{
    s->mem = m;
    s->wall_base = wall_base;
}

void goto_sm_pose(goto_sm *s, double now, unsigned epoch, double mx, double my, double mh, int moving)
{
    s->now = now;
    s->pose_moving = moving;
    if (s->mem) goto_mem_motor_pose(s->mem, epoch, mx, my, mh);
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

static void start_move(goto_sm *s, double mm)
{
    set_state(s, ST_MEM_MOVE);
    s->move_id = -1;
    s->move_deadline = s->now + 5.0;
    s->cmd_l = s->cmd_r = 0;
    emit(s, ACT_MOVE, mm, 1.0, NULL);
}

static void finish(goto_sm *s, const char *result)
{
    snprintf(s->result, sizeof s->result, "%s", result);
    s->cmd_l = s->cmd_r = 0;
    s->have_track = 0;
    s->mode = MODE_GOTO;
    s->mem_phase = 0;
    set_state(s, ST_IDLE);
}

/* ============================================== map-guided GOTO helpers
 * Frames: the nav/ARKit frame is metres, x/z, forward = (sin yaw, cos yaw), + yaw = LEFT.
 * goto's bearings are degrees, + = RIGHT (like TURN). So heading to a point is
 * atan2(dx, dz) and the TURN for it is -(that - yaw). */
static int nav_pose(const goto_sm *s, double *x, double *z, double *yaw)
{
    float fx = 0, fz = 0, fy = 0;
    if (!s->nav) return 0;
    int src = nav_get_pose(s->nav, &fx, &fz, &fy);
    if (src != NAV_POSE_ARKIT && src != NAV_POSE_DR) return 0;   /* stale/none: no map driving */
    if (x) *x = fx;
    if (z) *z = fz;
    if (yaw) *yaw = fy;
    return src;
}

static void sync_nav(goto_sm *s)
{
    if (!s->mem) return;
    double x, z, yaw;
    int src = nav_pose(s, &x, &z, &yaw);
    s->mem->have_nav = src != 0;
    s->mem->have_nav_arkit = src == NAV_POSE_ARKIT;
    if (s->mem->have_nav) { s->mem->nx = x; s->mem->nz = z; s->mem->nyaw = yaw; }
}

/* relative bearing (deg, + right) and distance (mm) from the robot to a map point */
static void rel_to(const goto_sm *s, double px, double pz, double *b, double *d)
{
    double x = 0, z = 0, yaw = 0;
    nav_pose(s, &x, &z, &yaw);
    double dx = px - x, dz = pz - z;
    if (d) *d = hypot(dx, dz) * 1000.0;
    if (b) *b = wrap180(-DEG(atan2(dx, dz) - yaw));
}

static void start_turn(goto_sm *s, goto_state st, double deg);
static void start_look(goto_sm *s, double after);
static void do_plan(goto_sm *s);
static void search_step(goto_sm *s, double now);
static int  mem_try_next(goto_sm *s);
static void arrived(goto_sm *s);

static void map_event(goto_sm *s, const char *what)
{
    char t[96];
    snprintf(t, sizeof t, "%s %s #%d", what, target_name(s), s->mem_id);
    emit(s, ACT_EVENT, 0, 0, t);
}

static void start_leg(goto_sm *s, goto_act_kind k, double amount)
{
    set_state(s, ST_FOLLOW_PATH);
    s->move_id = -1;
    s->move_deadline = s->now + 5.0;
    s->cmd_l = s->cmd_r = 0;
    if (k == ACT_TURN) emit(s, ACT_TURN, amount, s->cfg.search_speed, NULL);
    else emit(s, ACT_MOVE, amount, 1.0, NULL);   /* always forward: we turn to face each waypoint */
}

/* at the end of the route: turn to face the remembered spot, then LOOK (YOLO confirms) */
static void face_goal(goto_sm *s)
{
    double b, d;
    rel_to(s, s->goal_x, s->goal_z, &b, &d);
    s->mem_bearing = s->last_bearing = b;
    s->mem_dist = d;
    s->search_dir = b >= 0 ? 1 : -1;
    s->mem_phase = 1;
    s->leg = 2;
    if (fabs(b) > s->cfg.align_deg) start_leg(s, ACT_TURN, b);
    else start_look(s, s->now);
}

static void next_leg(goto_sm *s)
{
    while (s->path_i < s->path_n) {
        double b, d;
        rel_to(s, s->path[s->path_i].x, s->path[s->path_i].z, &b, &d);
        if (d < s->cfg.nav_leg_min_mm) { s->path_i++; continue; }
        s->leg_mm = d;
        if (fabs(b) > s->cfg.nav_turn_min_deg) { s->leg = 0; start_leg(s, ACT_TURN, b); }
        else { s->leg = 1; start_leg(s, ACT_MOVE, d); }
        return;
    }
    if (s->exploring) {                         /* explore hop done: the phone has seen more, re-plan */
        s->exploring = 0;
        s->vantage = 0;
        s->revealed_deg = 0;                    /* new spot, new view: allow another look around */
        do_plan(s);
        return;
    }
    face_goal(s);
}

/* no usable map/pose after all: the memory turn of the visual-only GOTO */
static void map_fallback(goto_sm *s, const char *why)
{
    map_event(s, why);
    s->map_mode = 0;
    int i = goto_mem_find_id(s->mem, target_name(s), s->mem_id);
    if (i >= 0 && s->mem->have_pose) goto_mem_relative(s->mem, i, &s->mem_bearing, &s->mem_dist);
    s->search_dir = s->mem_bearing >= 0 ? 1 : -1;
    s->last_bearing = s->mem_bearing;
    s->mem_phase = 1;
    if (fabs(s->mem_bearing) > s->cfg.align_deg) start_turn(s, ST_MEM_TURN, s->mem_bearing);
    else start_look(s, s->now);
}

static void search_fallback(goto_sm *s)
{
    s->map_mode = 0;
    s->mem_phase = 3;
    s->searched_deg = 0;
    search_step(s, s->now);
}

static void next_candidate(goto_sm *s, const char *why)
{
    set_state(s, ST_NEXT_CANDIDATE);
    map_event(s, why);
    int dir = s->search_dir;
    if (mem_try_next(s)) return;
    s->search_dir = dir;
    search_fallback(s);
}

/* the spot was looked at: downgrade = line of sight was clear and YOLO saw nothing */
static void check_others(goto_sm *s, int downgrade)
{
    set_state(s, ST_CHECK_OTHERS);
    if (downgrade) {
        goto_mem_miss(s->mem, goto_mem_find_id(s->mem, target_name(s), s->mem_id));
        map_event(s, "not_there");
    }
    int dir = s->search_dir;
    if (mem_try_next(s)) return;
    s->search_dir = dir;
    set_state(s, ST_NEXT_CANDIDATE);
    map_event(s, "next_candidate");
    search_fallback(s);
}

static void do_plan(goto_sm *s);

/* the robot is standing here, so its own footprint is floor: the phone can't see under it.
 * Same square nav_plan checks (ceil(0.35 / 0.1) = 4 cells each side); never clears OCC. */
static void mark_footprint_free(goto_sm *s, double x, double z)
{
    int cx, cz;
    if (nav_world_to_cell((float)x, (float)z, &cx, &cz))
        for (int i = -4; i <= 4; i++)
            for (int k = -4; k <= 4; k++)
                if (nav_cell_at(s->nav, cx + i, cz + k) == NAV_UNKNOWN) nav_set_cell(s->nav, cx + i, cz + k, NAV_FREE);
}

/* the robot's footprint square at (px,pz) is all observed free (nav_plan's start rule) */
static int foot_clear(goto_sm *s, double px, double pz)
{
    nav_pt_t tmp[2];
    return nav_plan_ring(s->nav, (float)px, (float)pz, (float)px, (float)pz, 0.0f, 0.11f, tmp, 2) >= 0;
}

/* Something stands inside the robot's own footprint square (< ~0.45 m away: a person or a board
 * stepping in right in front, also inside lidar_guard's stop zone), so every plan from here is
 * START_BLOCKED, though the robot isn't touching it. A nearby cell 0.2-0.4 m away to back off to,
 * WITHOUT reversing: its footprint is clear, the straight line to it is observed free, and the
 * direction points away from every obstacle in the footprint, so a pivot (tars_motor allows it
 * under a fresh BLOCKED) and a forward MOVE only ever increase the distance to them. */
static int backoff_cell(goto_sm *s, double x, double z, double *bx, double *bz)
{
    int cx, cz;
    if (!nav_world_to_cell((float)x, (float)z, &cx, &cz)) return 0;
    double ox[81], oz[81], mx = 0, mz = 0;
    int no = 0;
    for (int i = -4; i <= 4; i++)
        for (int k = -4; k <= 4; k++)
            if (nav_cell_at(s->nav, cx + i, cz + k) == NAV_OCC) {
                ox[no] = NAV_ORIGIN_X + (cx + i + 0.5) * NAV_RES - x;
                oz[no] = NAV_ORIGIN_Z + (cz + k + 0.5) * NAV_RES - z;
                mx += ox[no]; mz += oz[no]; no++;
            }
    if (!no) return 0;
    double away = atan2(-mx, -mz);              /* straight away from the obstacles' middle */
    for (double r = 0.2; r <= 0.41; r += 0.1)
        for (int j = 0; j < 16; j++) {          /* 0, +1, -1, +2, ... steps of 22.5 deg from "away" */
            double a = away + ((j + 1) / 2) * (j % 2 ? 1 : -1) * (M_PI / 8), dx = sin(a), dz = cos(a);
            int ok = 1;
            for (int o = 0; o < no && ok; o++) ok = dx * ox[o] + dz * oz[o] <= 0;
            if (!ok) continue;
            double px = x + r * dx, pz = z + r * dz;
            if (nav_line_clear_ex(s->nav, (float)x, (float)z, (float)px, (float)pz, 0.0f) != 1) continue;
            if (!foot_clear(s, px, pz)) continue;
            *bx = px;
            *bz = pz;
            return 1;
        }
    return 0;
}

/* nav_plan(_ring) from the robot into s->path; if the start is blocked by something in the
 * footprint, back off first: path[0] = the back-off cell, then the route from there */
static int plan_from_robot(goto_sm *s, double x, double z, double tx, double tz, int ring_default)
{
    int n = ring_default ? nav_plan(s->nav, (float)x, (float)z, (float)tx, (float)tz, s->path, 64)
                         : nav_plan_ring(s->nav, (float)x, (float)z, (float)tx, (float)tz, 0.0f, 0.11f, s->path, 64);
    double bx, bz;
    if (n != NAV_ERR_START_BLOCKED || !backoff_cell(s, x, z, &bx, &bz)) return n;
    int m = ring_default ? nav_plan(s->nav, (float)bx, (float)bz, (float)tx, (float)tz, s->path + 1, 63)
                         : nav_plan_ring(s->nav, (float)bx, (float)bz, (float)tx, (float)tz, 0.0f, 0.11f, s->path + 1, 63);
    if (m < 0) return m == NAV_ERR_START_BLOCKED ? n : m;
    s->path[0].x = (float)bx;
    s->path[0].z = (float)bz;
    return m + 1;
}

/* the goal's surroundings are unmapped (LiDAR shadow): a reachable, mapped free cell that is
 * clearly closer to the goal, so the phone can see more from there. Frontier-lite. */
static int pick_explore(goto_sm *s)
{
    double x, z, yaw;
    if (!nav_pose(s, &x, &z, &yaw)) return 0;
    double now_d = hypot(s->goal_x - x, s->goal_z - z);
    double cx[96], cz[96], cs[96];
    int n = 0;
    for (double r = 0.9; r <= 3.01 && n < 96; r += 0.3)
        for (int k = 0; k < 16 && n < 96; k++) {
            double a = k * 2 * M_PI / 16, px = s->goal_x + r * sin(a), pz = s->goal_z + r * cos(a);
            if (r > now_d - 0.4 || nav_cell(s->nav, (float)px, (float)pz) != NAV_FREE) continue;
            double score = r + 0.5 * hypot(px - x, pz - z);
            /* a hop whose view of the goal is known to be blocked (straight at the middle of
             * the wall in between) can't map the goal's ring and leaves the robot in the
             * wall's shadow with nothing nearer to try: last resort only */
            if (nav_line_clear(s->nav, (float)px, (float)pz, (float)s->goal_x, (float)s->goal_z) == 0) score += 100.0;
            int j = n++;
            while (j > 0 && cs[j - 1] > score) { cx[j] = cx[j - 1]; cz[j] = cz[j - 1]; cs[j] = cs[j - 1]; j--; }
            cx[j] = px; cz[j] = pz; cs[j] = score;
        }
    for (int i = 0; i < n && i < 10; i++)
        if (nav_plan_ring(s->nav, (float)x, (float)z, (float)cx[i], (float)cz[i], 0.0f, 0.11f, s->path, 64) > 0) {
            s->vx = cx[i];
            s->vz = cz[i];
            return 1;
        }
    return 0;
}

/* From a vantage cell the robot LOOKs, then drives straight at the target (visual APPROACH), so a
 * one-cell line of sight isn't enough: the lane the body sweeps up to the target must hold no
 * obstacle either, or lidar_guard stops the approach on the occluder's edge (seen in the full-stack
 * sim: a board end 0.41 m ahead, "docked" 0.88 m from the bin). Lines beside the centre line out to
 * +-0.4 m (the footprint square nav_plan inflates by; it also absorbs the robot stopping up to a
 * cell away from the vantage point), up to NAV_LOS_END_MARGIN from the target (the object itself
 * is occupied). Unknown cells are allowed beside the lane. */
static int lane_clear(const goto_sm *s, double px, double pz, double gx, double gz)
{
    double dx = gx - px, dz = gz - pz, L = hypot(dx, dz);
    if (L <= NAV_LOS_END_MARGIN) return 1;
    dx /= L; dz /= L;
    double ex = gx - NAV_LOS_END_MARGIN * dx, ez = gz - NAV_LOS_END_MARGIN * dz;
    for (int i = 1; i <= 4; i++)
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            double off = sgn * 0.1 * i, ox = off * dz, oz = -off * dx;      /* perpendicular */
            if (nav_line_clear_ex(s->nav, (float)(px + ox), (float)(pz + oz), (float)(ex + ox), (float)(ez + oz), 0.0f) == 0)
                return 0;
        }
    return 1;
}

/* a free cell 0.7-1.3 m from the goal with a clear line of sight (and drive lane) to it, reachable */
static int pick_vantage(goto_sm *s)
{
    double x, z, yaw;
    if (!nav_pose(s, &x, &z, &yaw)) return 0;
    double cx[72], cz[72], cd[72];
    int n = 0;
    static const double R[3] = {0.7, 1.0, 1.3};
    mark_footprint_free(s, x, z);
    for (int r = 0; r < 3; r++)
        for (int k = 0; k < 24; k++) {
            double a = k * 2 * M_PI / 24, px = s->goal_x + R[r] * sin(a), pz = s->goal_z + R[r] * cos(a);
            if (nav_cell(s->nav, (float)px, (float)pz) != NAV_FREE) continue;
            if (nav_line_clear(s->nav, (float)px, (float)pz, (float)s->goal_x, (float)s->goal_z) != 1) continue;
            if (!lane_clear(s, px, pz, s->goal_x, s->goal_z)) continue;
            if (!foot_clear(s, px, pz)) continue;   /* the robot must fit there: don't spend a try on it */
            double d = hypot(px - x, pz - z);
            int j = n++;
            while (j > 0 && cd[j - 1] > d) { cx[j] = cx[j - 1]; cz[j] = cz[j - 1]; cd[j] = cd[j - 1]; j--; }
            cx[j] = px; cz[j] = pz; cd[j] = d;
        }
    for (int i = 0; i < n && i < 8; i++)
        if (plan_from_robot(s, x, z, cx[i], cz[i], 0) >= 0) {
            s->vx = cx[i];
            s->vz = cz[i];
            return 1;
        }
    return 0;
}

/* the map doesn't reach the goal yet (most of its approach ring unseen): go visual instead */
static int ring_unmapped(const goto_sm *s)
{
    int unknown = 0, total = 0;
    for (int k = 0; k < 24; k++) {
        double a = k * 2 * M_PI / 24;
        unknown += nav_cell(s->nav, (float)(s->goal_x + 0.7 * sin(a)), (float)(s->goal_z + 0.7 * cos(a))) == NAV_UNKNOWN;
        total++;
    }
    return unknown * 2 > total;
}

static void do_plan(goto_sm *s)
{
    double x, z, yaw;
    if (!nav_pose(s, &x, &z, &yaw)) { map_fallback(s, "no_pose"); return; }
    set_state(s, ST_PLAN);
    s->plan_wait = 0;
    if (nav_count(s->nav, NAV_FREE) == 0) { map_fallback(s, "no_map"); return; }
    mark_footprint_free(s, x, z);
    int n = s->vantage ? plan_from_robot(s, x, z, s->vx, s->vz, 0)
                       : plan_from_robot(s, x, z, s->goal_x, s->goal_z, 1);
    s->path_ver = nav_version(s->nav);
    if (n >= 0) {
        s->path_n = n;
        s->path_i = 0;
        char t[96];
        snprintf(t, sizeof t, "route %s #%d %d", target_name(s), s->mem_id, n);
        emit(s, ACT_EVENT, 0, 0, t);
        next_leg(s);
        return;
    }
    if (n == NAV_ERR_START_BLOCKED && s->revealed_deg < 360 && s->cfg.nav_reveal_deg > 0) {
        s->revealed_deg += s->cfg.nav_reveal_deg;
        s->leg = 3;
        if (s->revealed_deg <= s->cfg.nav_reveal_deg) map_event(s, "reveal");
        start_leg(s, ACT_TURN, s->cfg.nav_reveal_deg * s->search_dir);
        return;
    }
    /* unknown floor blocks like an obstacle and the phone sees a 60 deg cone: before calling a
     * goal unreachable, spin in place (re-planning every step) to map around the robot */
    if ((n == NAV_ERR_NO_GOAL || n == NAV_ERR_NO_PATH) && !s->vantage && !s->exploring &&
        s->revealed_deg < 360 && s->cfg.nav_reveal_deg > 0) {
        s->revealed_deg += s->cfg.nav_reveal_deg;
        s->leg = 3;
        if (s->revealed_deg <= s->cfg.nav_reveal_deg) map_event(s, "reveal");
        start_leg(s, ACT_TURN, s->cfg.nav_reveal_deg * s->search_dir);
        return;
    }
    if (n == NAV_ERR_NO_GOAL && !s->vantage && ring_unmapped(s)) {
        /* shadowed / never seen: spin to map around us, then hop closer, then give up to visual */
        if (s->revealed_deg < 360 && s->cfg.nav_reveal_deg > 0) {
            s->revealed_deg += s->cfg.nav_reveal_deg;
            s->leg = 3;
            if (s->revealed_deg <= s->cfg.nav_reveal_deg) map_event(s, "reveal");
            start_leg(s, ACT_TURN, s->cfg.nav_reveal_deg * s->search_dir);
            return;
        }
        if (s->explores < 4 && pick_explore(s)) {
            s->explores++;
            s->exploring = s->vantage = 1;
            map_event(s, "explore");
            do_plan(s);
            return;
        }
        map_fallback(s, "unmapped");
        return;
    }
    if (n == NAV_ERR_NO_GOAL || n == NAV_ERR_NO_PATH || n == NAV_ERR_TOO_LONG) {
        if (s->vantage) { s->vantage = 0; check_others(s, 0); return; }   /* can't reach a better view */
        next_candidate(s, "no_route");
        return;
    }
    map_fallback(s, n == NAV_ERR_NO_MAP ? "no_map" : n == NAV_ERR_START_BLOCKED ? "start_blocked" : "no_plan");
}

/* a leg was aborted by the guard / obstacle / deadman: re-plan once it's clear */
static void leg_blocked(goto_sm *s, const char *why)
{
    char t[96];
    snprintf(t, sizeof t, "blocked:%s %s #%d", why, target_name(s), s->mem_id);
    emit(s, ACT_EVENT, 0, 0, t);
    if (++s->replans > (int)s->cfg.nav_max_replans) { goto_sm_fail(s, s->now, "blocked"); return; }
    set_state(s, ST_PLAN);
    s->plan_wait = 1;
}

static int path_blocked(const goto_sm *s)
{
    double x, z;
    if (!nav_pose(s, &x, &z, NULL)) return 0;
    for (int i = s->path_i; i < s->path_n; i++) {
        if (nav_line_clear_ex(s->nav, (float)x, (float)z, s->path[i].x, s->path[i].z, 0.0f) == 0) return 1;
        x = s->path[i].x;
        z = s->path[i].z;
    }
    return 0;
}

static void leg_done(goto_sm *s)
{
    switch (s->leg) {
    case 0: {
        double d;
        rel_to(s, s->path[s->path_i].x, s->path[s->path_i].z, NULL, &d);
        if (d < s->cfg.nav_leg_min_mm) { s->path_i++; next_leg(s); return; }
        s->leg = 1;
        s->leg_mm = d;
        start_leg(s, ACT_MOVE, d);
        return;
    }
    case 1: s->path_i++; next_leg(s); return;
    case 2: start_look(s, s->now + s->cfg.settle_s); return;
    default: do_plan(s); return;                    /* 3: reveal turn done */
    }
}

/* LOOK at the remembered spot (reached over the map) found nothing */
static void map_look_failed(goto_sm *s)
{
    double x, z;
    int los = nav_pose(s, &x, &z, NULL)
            ? nav_line_clear(s->nav, (float)x, (float)z, (float)s->goal_x, (float)s->goal_z) : -1;
    if (los == 0) {
        set_state(s, ST_OCCLUDED);
        map_event(s, "occluded");               /* may still be there: don't downgrade it */
        if (s->detoured_id != s->mem_id && pick_vantage(s)) {
            s->detoured_id = s->mem_id;
            set_state(s, ST_DETOUR);
            map_event(s, "detour");
            s->vantage = 1;
            do_plan(s);
            return;
        }
        check_others(s, 0);
        return;
    }
    check_others(s, 1);
}

/* the stop + beep + event of an arrival, and a precise memory entry for the target */
static void arrived(goto_sm *s)
{
    emit(s, ACT_STOP, 0, 0, NULL);
    emit(s, ACT_LED, 0, 0, "ok");
    emit(s, ACT_BEEP, 1, 0, NULL);
    emit_event(s, "arrived");
    if (s->mem && s->mem->have_pose && s->target >= 0) {
        /* we're standing in front of it: the best position estimate we'll ever get */
        goto_mem *m = s->mem;
        const char *lab = s->objs->obj[s->target];
        double wb = wrap180(m->ph + s->last_bearing), d = s->cfg.arrive_dist_mm;
        double ax = m->px + d * cos(RAD(wb)), ay = m->py + d * sin(RAD(wb));
        /* the instance we drove to, if it's plausibly this one; else match by position.
         * Positions in the phone's map when both sides have one (DR can be metres off). */
        sync_nav(s);
        mem_wpos w = mem_wpos_of(m, s->last_bearing, d);
        int i = s->mem_phase ? goto_mem_find_id(m, lab, s->mem_id) : -1;
        if (i >= 0) {
            const goto_mem_entry *e = &m->e[i];
            double off = w.ok && e->has_w ? hypot(e->wx - w.wx, e->wz - w.wz) * 1000.0 : hypot(e->x - ax, e->y - ay);
            if (off > 3 * s->cfg.memory_merge_mm) i = -1;
        }
        if (i < 0) i = mem_sight_w(m, &s->cfg, lab, ax, ay, wb, d, 0.5, wall_now(s), &w);
        goto_mem_entry *e = &m->e[i];
        e->x = ax; e->y = ay; e->bearing = wb; e->dist = d;
        sync_nav(s);
        mem_set_w(m, e, s->last_bearing, d, 0);
        e->status = MEM_FRESH;
        e->seen++;
        e->last_seen = wall_now(s);
        m->dirty = 1;
    }
    finish(s, "arrived");
}

/* Memory-guided GOTO: pick the next remembered instance of the target that we
 * haven't tried and head for it (exact TURN, then LOOK). Returns 0 if there is
 * none (or memory is off) and the caller carries on with a normal look/search. */
static int mem_try_next(goto_sm *s)
{
    goto_mem *m = s->mem;
    if (!m || !m->have_pose || s->target < 0 || s->n_tried >= (int)s->cfg.memory_tries ||
        s->n_tried >= (int)(sizeof s->mem_tried / sizeof s->mem_tried[0]))
        return 0;
    int i = goto_mem_best(m, s->objs->obj[s->target], wall_now(s), s->recall ? 1e18 : s->cfg.memory_max_age_s,
                          s->mem_tried, s->n_tried);
    if (i < 0 && s->recall) {                  /* "where it was": even a gone one, newest first */
        for (int k = 0; k < m->n; k++) {
            int sk = 0;
            for (int j = 0; j < s->n_tried; j++) sk |= s->mem_tried[j] == m->e[k].id;
            if (!sk && !strcmp(m->e[k].label, s->objs->obj[s->target]) && (i < 0 || m->e[k].last_seen > m->e[i].last_seen))
                i = k;
        }
    }
    if (i < 0) return 0;
    s->mem_id = m->e[i].id;
    s->mem_tried[s->n_tried++] = s->mem_id;
    s->mem_phase = 1;
    goto_mem_relative(m, i, &s->mem_bearing, &s->mem_dist);
    s->search_dir = s->mem_bearing >= 0 ? 1 : -1;
    s->last_bearing = s->mem_bearing;
    char t[80];
    snprintf(t, sizeof t, "remembered %s #%d", s->objs->obj[s->target], s->mem_id);
    emit(s, ACT_EVENT, 0, 0, t);
    s->map_mode = 0;
    s->vantage = s->exploring = s->explores = 0;
    if (s->nav && nav_pose(s, NULL, NULL, NULL)) {
        const goto_mem_entry *e = &m->e[i];
        double b, d, x, z, yaw;
        if (e->has_w) { s->goal_x = e->wx; s->goal_z = e->wz; }
        else {                                  /* seen before the phone: project from the memory frame */
            nav_pose(s, &x, &z, &yaw);
            goto_mem_relative(m, i, &b, &d);
            s->goal_x = x + d / 1000.0 * sin(yaw - RAD(b));
            s->goal_z = z + d / 1000.0 * cos(yaw - RAD(b));
        }
        s->map_mode = 1;
        set_state(s, ST_RECALL);
        do_plan(s);
        return 1;
    }
    if (fabs(s->mem_bearing) > s->cfg.align_deg) start_turn(s, ST_MEM_TURN, s->mem_bearing);
    else start_look(s, s->now);
    return 1;
}

/* A LOOK window ended without the target. The ONE place that decides what's
 * next, in order; new strategies (lidar OCCLUDED check, DETOUR around a wall,
 * ...) slot in here as more steps / states:
 *   1. remembered spot, maybe just out of range -> MOVE closer (memory_drive_mm)
 *   2. not there -> mark the instance (fresh->stale->gone), try the next instance
 *   3. normal 360 search in search_step_deg steps, then give up */
static void look_failed(goto_sm *s, double now)
{
    if (s->map_mode && s->mem_phase == 1) { map_look_failed(s); return; }
    if (s->mem_phase == 1 && s->cfg.memory_drive_mm > 0) {
        double mm = fmin(s->cfg.memory_drive_mm, s->mem_dist - s->cfg.arrive_dist_mm);
        if (mm >= 50.0) {
            s->mem_phase = 2;
            start_move(s, mm);
            return;
        }
    }
    if (s->mem_phase == 1 || s->mem_phase == 2) {
        int i = goto_mem_find_id(s->mem, s->objs->obj[s->target], s->mem_id);
        goto_mem_miss(s->mem, i);
        char t[80];
        snprintf(t, sizeof t, "not_there %s #%d", s->objs->obj[s->target], s->mem_id);
        emit(s, ACT_EVENT, 0, 0, t);
        int dir = s->search_dir;
        if (mem_try_next(s)) return;
        s->search_dir = dir;
        s->mem_phase = 3;                     /* full search from here, toward the last remembered side */
        s->searched_deg = 0;
    }
    search_step(s, now);
}

static void search_step(goto_sm *s, double now)
{
    if (s->searched_deg + 1e-6 >= s->cfg.search_max_deg) {
        goto_sm_fail(s, now, "not found");
        return;
    }
    double step = s->cfg.search_step_deg;
    if (s->searched_deg + step > s->cfg.search_max_deg) step = s->cfg.search_max_deg - s->searched_deg;
    s->searched_deg += step;
    start_turn(s, ST_SEARCH_TURN, step * s->search_dir);
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
    s->mode = MODE_GOTO;
    s->mem_phase = 0;
    s->n_tried = 0;
    s->map_mode = s->vantage = 0;
    s->replans = 0;
    s->detoured_id = 0;
    s->path_n = s->path_i = 0;
    if (mem_try_next(s)) return;
    if (s->recall) { start_look(s, now); goto_sm_fail(s, now, "not remembered"); return; }
    start_look(s, now);                        /* look before turning: it may be in view */
}

void goto_sm_recall(goto_sm *s, double now, int target)
{
    s->recall = 1;
    goto_sm_goto(s, now, target);
    s->recall = 0;                            /* only widens the first pick + no blind start */
}

void goto_sm_set_nav(goto_sm *s, nav_t *nav) { s->nav = nav; }

void goto_sm_guard(goto_sm *s, double now, int permit, unsigned long long epoch)
{
    s->now = now;
    s->guard_ok = permit;
    if (!s->guard_known) { s->guard_known = 1; s->guard_epoch = s->lease_epoch = epoch; return; }
    if (epoch == s->guard_epoch) return;
    s->guard_epoch = epoch;
    /* a denial happened since this APPROACH leg started: the lease is void, end the leg */
    if (s->st == ST_APPROACH && s->lease_epoch != epoch) goto_sm_drive_refused(s, now, "guard epoch changed");
}

static int is_guard_why(const char *why)
{
    return why && (!strcmp(why, "lidar_guard") || !strncmp(why, "guard", 5));
}

/* A search / align / memory TURN aborted or refused by the guard. tars_motor lets an in-place TURN
 * run under a fresh BLOCKED, so this is a guard that went stale/missing for a moment (a CPU hiccup;
 * seen on a loaded Mac in the full-stack sim). Like a route leg: stop, wait, LOOK again and carry
 * on, bounded by nav_max_replans. The interrupted search step isn't counted toward the 360. */
static void turn_guard_retry(goto_sm *s, const char *why)
{
    char t[96];
    snprintf(t, sizeof t, "guard_stop %s %.40s", target_name(s), why);
    emit(s, ACT_STOP, 0, 0, NULL);
    emit(s, ACT_EVENT, 0, 0, t);
    if (++s->replans > (int)s->cfg.nav_max_replans) { goto_sm_fail(s, s->now, "blocked"); return; }
    if (s->st == ST_SEARCH_TURN) s->searched_deg = fmax(0.0, s->searched_deg - s->cfg.search_step_deg);
    s->have_track = 0;
    start_look(s, s->now + s->cfg.nav_guard_wait_s);
}

int goto_sm_move_refused(goto_sm *s, double now, const char *why)
{
    s->now = now;
    if ((s->st == ST_SEARCH_TURN || s->st == ST_ALIGN_TURN || s->st == ST_MEM_TURN) && is_guard_why(why)) {
        turn_guard_retry(s, why);
        return 1;
    }
    if (s->st != ST_FOLLOW_PATH) return 0;
    if (s->leg == 2) {
        char t[96];
        snprintf(t, sizeof t, "guard_face %s #%d %.40s", target_name(s), s->mem_id, why ? why : "-");
        emit(s, ACT_EVENT, 0, 0, t);
        start_look(s, now);
        return 1;
    }
    leg_blocked(s, "guard");
    return 1;
}

void goto_sm_drive_refused(goto_sm *s, double now, const char *why)
{
    s->now = now;
    if (s->st != ST_APPROACH) return;
    emit(s, ACT_STOP, 0, 0, NULL);
    s->cmd_l = s->cmd_r = 0;
    char t[96];
    snprintf(t, sizeof t, "guard_stop %s %.40s", target_name(s), why ? why : "-");
    emit(s, ACT_EVENT, 0, 0, t);
    /* lidar_guard stops ~0.6 m from anything straight ahead. If that's the target, centred
     * and already big in the frame, we're docked: that's an arrival, not an obstacle. */
    double need = goto_cfg_arrive_h(&s->cfg, target_name(s));
    if (s->have_track && now - s->last_seen < 0.5 && fabs(s->last_bearing) <= s->cfg.realign_deg &&
        s->ema_h >= 0.5 * need) {
        arrived(s);
        return;
    }
    if (++s->replans > (int)s->cfg.nav_max_replans) { goto_sm_fail(s, now, "blocked"); return; }
    s->have_track = 0;
    s->searched_deg = 0;
    start_look(s, now + s->cfg.nav_guard_wait_s);    /* new leg (new lease) only after a fresh look */
}

void goto_sm_scan(goto_sm *s, double now)
{
    s->now = now;
    if (s->st == ST_APPROACH || goto_sm_wants_status(s)) emit(s, ACT_STOP, 0, 0, NULL);
    s->target = -1;
    s->started = now;
    s->searched_deg = 0;
    s->search_dir = 1;
    s->have_track = 0;
    s->last_frame = now;
    s->frames = s->frames_with_target = 0;
    s->mode = MODE_SCAN;
    s->mem_phase = 0;
    s->scan_n = 0;
    s->replans = 0;                           /* guard-hiccup retries of the scan turns */
    snprintf(s->result, sizeof s->result, "scanning");
    emit(s, ACT_LED, 0, 0, "rec");
    emit(s, ACT_BEEP, 1, 0, NULL);
    emit(s, ACT_EVENT, 0, 0, "scanning -");
    start_look(s, now);
}

static void scan_note(goto_sm *s, const goto_frame *f)
{
    for (int i = 0; i < f->n; i++) {
        const goto_det *d = &f->d[i];
        if (d->conf < s->cfg.min_conf || d->box[2] <= d->box[0] || d->box[3] <= d->box[1]) continue;
        char l[32];
        goto_mem_label(s->objs, d->label, l, sizeof l);
        if (!l[0]) continue;
        int k;
        for (k = 0; k < s->scan_n; k++) if (!strcmp(s->scan_seen[k], l)) break;
        if (k == s->scan_n && s->scan_n < GOTO_MEM_MAX) snprintf(s->scan_seen[s->scan_n++], sizeof s->scan_seen[0], "%s", l);
    }
}

static void scan_done(goto_sm *s)
{
    char t[200];
    size_t o = (size_t)snprintf(t, sizeof t, "scanned ");
    for (int k = 0; k < s->scan_n && o < sizeof t; k++)
        o += (size_t)snprintf(t + o, sizeof t - o, "%s%s", k ? "," : "", s->scan_seen[k]);
    if (!s->scan_n) snprintf(t + o, sizeof t - o, "nothing");
    emit(s, ACT_LED, 0, 0, "ok");
    emit(s, ACT_BEEP, 1, 0, NULL);
    emit(s, ACT_EVENT, 0, 0, t);
    char r[64];
    snprintf(r, sizeof r, "scanned:%d", s->scan_n);
    if (s->mem) s->mem->dirty = 1;
    finish(s, r);
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
    return bearing_of(&s->cfg, x);
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
    s->lease_epoch = s->guard_epoch;           /* DL lease for this leg */
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
    /* remember everything seen, whenever the camera is steady and the pose is current */
    if (s->mem && !s->pose_moving && !goto_sm_wants_status(s) && !(s->st == ST_LOOK && now < s->fresh_after))
    {
        sync_nav(s);
        goto_mem_observe(s->mem, &s->cfg, s->objs, f, wall_now(s));
    }
    if (s->st == ST_IDLE) return;
    if (s->mode == MODE_SCAN) {
        s->frames++;
        if (s->st != ST_LOOK || now < s->fresh_after) return;
        scan_note(s, f);
        if (++s->look_seen < s->cfg.look_frames) return;
        if (s->searched_deg + 1e-6 >= 360.0) { scan_done(s); return; }
        double step = s->cfg.search_step_deg > 0 ? s->cfg.search_step_deg : 30.0;
        if (s->searched_deg + step > 360.0) step = 360.0 - s->searched_deg;
        s->searched_deg += step;
        start_turn(s, ST_SEARCH_TURN, step);
        return;
    }
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
                s->last_bearing = s->look_bearing / s->look_hits;
                arrived(s);
                return;
            }
        }
        if (s->look_hits >= s->cfg.confirm_frames) {
            double b = s->look_bearing / s->look_hits;
            s->last_bearing = b;
            if (s->mem_phase && s->mem_phase < 3) s->mem_phase = 4;     /* found where memory said */
            after_sighting(s, b);
        } else if (s->look_seen >= s->cfg.look_frames) {
            look_failed(s, now);
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
            if (++s->arrive_count >= s->cfg.arrive_frames) arrived(s);
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
    if (!goto_sm_wants_status(s) && now - s->last_frame > s->cfg.frame_timeout_s) {
        goto_sm_fail(s, now, "no camera frames");
        return;
    }
    double period = 1.0 / s->cfg.tick_hz;
    int due = now - s->last_tick >= period * 0.95;
    switch (s->st) {
    case ST_SEARCH_TURN:
    case ST_ALIGN_TURN:
    case ST_MEM_TURN:
    case ST_MEM_MOVE:
    case ST_FOLLOW_PATH:
        if (s->st == ST_FOLLOW_PATH && s->leg_settle) {   /* leg done: a post-move pose yet? */
            if (nav_pose_seq(s->nav) - s->settle_seq >= 2 || nav_pose(s, NULL, NULL, NULL) != NAV_POSE_ARKIT ||
                now - s->settle_since >= s->cfg.nav_pose_fresh_s) {
                s->leg_settle = 0;
                leg_done(s);
            }
            return;
        }
        if (now > s->move_deadline) {
            goto_sm_fail(s, now, s->st == ST_MEM_MOVE || (s->st == ST_FOLLOW_PATH && s->leg == 1) ? "move timed out"
                                                                                                   : "turn timed out");
            return;
        }
        /* deadman keepalive: goto.c PINGs at 10 Hz while it waits on a move (a zero D would be
         * refused while the guard denies, and tars_motor's abort is what we want then) */
        if (s->st == ST_FOLLOW_PATH && s->nav && nav_version(s->nav) != s->path_ver && s->leg <= 1) {
            s->path_ver = nav_version(s->nav);
            if (path_blocked(s)) {                    /* the map changed under the route */
                emit(s, ACT_STOP, 0, 0, NULL);
                leg_blocked(s, "map");
            }
        }
        return;
    case ST_PLAN:
        /* CLEAR + a short wait; or, still BLOCKED after 3x the wait, re-plan anyway: the new
         * route starts with a pivot, which tars_motor allows under a fresh forward BLOCKED */
        if (s->plan_wait && (((!s->guard_known || s->guard_ok) && now - s->state_since >= s->cfg.nav_guard_wait_s) ||
                             now - s->state_since >= 3 * s->cfg.nav_guard_wait_s))
            do_plan(s);
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
        char ep[24] = "";
        if (s->guard_known) snprintf(ep, sizeof ep, "%llu", s->lease_epoch);
        emit(s, ACT_DRIVE, l, r, ep);
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
    if (!goto_sm_wants_status(s) || s->move_id < 0 || move_id != s->move_id || s->leg_settle) return;
    if (!strcmp(move, "running")) return;
    if (!strcmp(move, "done")) {
        if (s->st != ST_FOLLOW_PATH) { start_look(s, now + s->cfg.settle_s); return; }
        /* The next leg / re-plan needs the pose AFTER this move. tars_motor's pose (DR) is
         * already current, but the newest ARKit packet may have been measured before the
         * move ended (and nav prefers ARKit while it's < nav_pose_fresh_s old): planning
         * from it turns by a stale heading. Wait for 2 newer poses (the first may have been
         * in flight); goto_sm_tick() finishes the leg. */
        if (nav_pose(s, NULL, NULL, NULL) == NAV_POSE_ARKIT) {
            s->leg_settle = 1;
            s->settle_seq = nav_pose_seq(s->nav);
            s->settle_since = now;
            s->move_deadline = now + s->cfg.nav_pose_fresh_s + 1.0;
            return;
        }
        leg_done(s);
        return;
    }
    if (s->st == ST_FOLLOW_PATH && why && (!strcmp(why, "lidar_guard") || !strcmp(why, "obstacle") ||
                                           !strcmp(why, "watchdog") || !strncmp(why, "guard", 5))) {
        leg_blocked(s, why);
        return;
    }
    if ((s->st == ST_SEARCH_TURN || s->st == ST_ALIGN_TURN || s->st == ST_MEM_TURN) && is_guard_why(why)) {
        turn_guard_retry(s, why);
        return;
    }
    /* aborted / idle (replaced): obstacle, bump, tip, stop from someone else */
    char r[48];
    snprintf(r, sizeof r, "%s %s", s->st == ST_MEM_MOVE ? "move" : "turn", why && *why ? why : move);
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
             "elapsed=%.1f frames=%d hits=%d l=%.2f r=%.2f mode=%s mem=%s map=%s path=%d/%d replans=%d "
             "guard=%s",
             goto_state_name(s->st), tn, s->result,
             s->st == ST_APPROACH && s->now - s->last_seen < 0.5, s->last_bearing,
             s->have_track ? s->ema_h : 0.0, s->searched_deg, s->fps_ema,
             s->st != ST_IDLE ? s->now - s->started : 0.0, s->frames, s->frames_with_target,
             s->cmd_l, s->cmd_r, s->mode == MODE_SCAN ? "scan" : "goto",
             s->mem_phase == 1 ? "turned" : s->mem_phase == 2 ? "drove" : s->mem_phase == 3 ? "not_there"
             : s->mem_phase == 4 ? "hit" : "-",
             !s->nav ? "off" : s->map_mode ? (s->vantage ? "detour" : "route") : "ready",
             s->path_i, s->path_n, s->replans,
             !s->guard_known ? "-" : s->guard_ok ? "clear" : "denied");
}

/* Text -> robot command. C99 port of intent.py (which is the spec: keep the two
 * in step, tests/test_intent.py checks they agree on every test sentence). */
#include "intent.h"
#include "intent_model.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TOKS 64
#define DEFAULT_TURN_DEG 30
#define DEFAULT_MOVE_MM  300
#define STEP_MM          100
#define SMALL_MM         100
#define SMALL_DEG        15
#define MAX_MM           1000
#define MAX_DEG          360

const char *intent_name(int k) { return k >= 0 && k < INTENT_N ? intent_names[k] : "-"; }
int intent_count(void) { return INTENT_N; }
float intent_threshold(void) { return INTENT_THRESHOLD; }

static int in_list(const char *t, const char *const *list)
{
    for (; *list; list++)
        if (!strcmp(t, *list)) return 1;
    return 0;
}

/* ---------------------------------------------------------------- normalize */
int intent_normalize(const char *text, char *buf, int buflen, char **toks, int max_toks)
{
    const unsigned char *b = (const unsigned char *)text;
    int n = (int)strlen(text), o = 0;
    char prev = 0;                 /* 'd', 'a' or 0 */
    for (int i = 0; i < n && o < buflen - 2; i++) {
        if (b[i] == 0xE2 && i + 2 < n && b[i + 1] == 0x80 && (b[i + 2] == 0x98 || b[i + 2] == 0x99)) {
            i += 2;                /* curly apostrophe: dropped like ' */
            continue;
        }
        int c = b[i] < 128 ? b[i] : ' ';
        if (c >= 'A' && c <= 'Z') c += 32;
        if (c == '\'') continue;
        if (c == '.' && prev == 'd' && i + 1 < n && b[i + 1] < 128 && isdigit(b[i + 1])) {
            buf[o++] = '.';
            continue;
        }
        char cls;
        if (c >= '0' && c <= '9') cls = 'd';
        else if (c >= 'a' && c <= 'z') cls = 'a';
        else {
            if (prev) buf[o++] = ' ';
            prev = 0;
            continue;
        }
        if (prev && prev != cls) buf[o++] = ' ';
        if (o < buflen - 1) buf[o++] = (char)c;
        prev = cls;
    }
    buf[o] = '\0';
    int nt = 0;
    char *p = buf;
    while (*p && nt < max_toks) {
        while (*p == ' ') p++;
        if (!*p) break;
        toks[nt++] = p;
        while (*p && *p != ' ') p++;
        if (*p) *p++ = '\0';
    }
    return nt;
}

static int edit_distance(const char *a, const char *b, int cap)
{
    int la = (int)strlen(a), lb = (int)strlen(b);
    if (abs(la - lb) >= cap) return cap;
    if (lb > 62) return cap;
    int prev[64], cur[64];
    for (int j = 0; j <= lb; j++) prev[j] = j;
    for (int i = 1; i <= la; i++) {
        cur[0] = i;
        for (int j = 1; j <= lb; j++) {
            int v = prev[j] + 1, w = cur[j - 1] + 1, s = prev[j - 1] + (a[i - 1] != b[j - 1]);
            cur[j] = v < w ? (v < s ? v : s) : (w < s ? w : s);
        }
        memcpy(prev, cur, sizeof(int) * (size_t)(lb + 1));
    }
    return prev[lb] < cap ? prev[lb] : cap;
}

/* ---------------------------------------------------------------- wake word */
static const char *const WAKE_WORDS[] = {
    "tars", "tarz", "tar", "tares", "tarr", "stars", "star", "taurus", "torres", "tires", "tyres",
    "tours", "towers", "cars", "tarps", "tass", "tas", "task", "tarts", "tarse", "chars", "dars",
    "thars", "tahrs", "tarsh", "tarzan", NULL };
static const char *const WAKE_LEAD[] = { "hey", "ok", "okay", "yo", "hi", "uh", "um", "oh", "so", "a", NULL };

int intent_is_wake(const char *t)
{
    return in_list(t, WAKE_WORDS) || (strlen(t) >= 3 && edit_distance(t, "tars", 2) <= 1);
}

/* returns index of first token after the wake word, or 0 with *wake = 0 */
static int strip_wake(char **toks, int n, int *wake)
{
    for (int i = 0; i < 2 && i < n; i++) {
        if (intent_is_wake(toks[i])) { *wake = 1; return i + 1; }
        if (!in_list(toks[i], WAKE_LEAD)) break;
    }
    *wake = 0;
    return 0;
}

/* ---------------------------------------------------------------- safety pass */
static const char *const STRONG_ROOTS[] = { "stop", "halt", "freeze", "whoa", NULL };
static const char *const STRONG_EXTRA[] = { "stopit", "stopp", "stahp", "stawp", "frees", "freese",
    "freez", "woah", "halted", "stopping", "stopped", "freezing", "froze", NULL };
static const char *const NOT_STOP[] = { "step", "top", "shop", "atop", "half", "hall", "salt", "malt",
    "hat", "alt", "who", "whom", "what", "sop", "spot", "stomp", NULL };
static const char *const WEAK_STOP[] = { "no", "nope", "nah", "wait", "hold", "enough", "cancel",
    "brake", "stay", NULL };

static int strong_stop_word(const char *t)
{
    if (in_list(t, NOT_STOP) || strlen(t) < 3) return 0;
    if (in_list(t, STRONG_EXTRA)) return 1;
    for (const char *const *r = STRONG_ROOTS; *r; r++)
        if (!strcmp(t, *r) || !strncmp(t, *r, strlen(*r)) || edit_distance(t, *r, 2) <= 1) return 1;
    return 0;
}

int intent_safety_stop(char **toks, int n, int wake)
{
    for (int i = 0; i < n; i++)
        if (strong_stop_word(toks[i])) return 1;
    if (wake)
        for (int i = 0; i < n; i++)
            if (in_list(toks[i], WEAK_STOP)) return 1;
    return 0;
}

/* ---------------------------------------------------------------- features + model */
static unsigned fnv1a(const unsigned char *p, int n, int prefix)
{
    unsigned h = 2166136261u;
    if (prefix >= 0) { h ^= (unsigned)prefix; h *= 16777619u; }
    for (int i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

void intent_probs(char **toks, int n, float *probs)
{
    /* s = " " + join(toks, " ") + " " */
    char s[512];
    int L = 0;
    s[L++] = ' ';
    for (int i = 0; i < n; i++) {
        int tl = (int)strlen(toks[i]);
        if (L + tl + 2 >= (int)sizeof s) break;
        if (i) s[L++] = ' ';
        memcpy(s + L, toks[i], (size_t)tl);
        L += tl;
    }
    s[L++] = ' ';

    /* sparse accumulate: idx list + values (collisions merged) */
    enum { MAXF = 2048 };
    int idx[MAXF];
    float val[MAXF];
    int nf = 0;
#define ADD(k) do { int _k = (int)(k); int _j; for (_j = 0; _j < nf; _j++) if (idx[_j] == _k) break; \
                    if (_j < nf) val[_j] += 1.0f; else if (nf < MAXF) { idx[nf] = _k; val[nf++] = 1.0f; } } while (0)
    for (int g = 2; g <= 4; g++)
        for (int i = 0; i + g <= L; i++)
            ADD(fnv1a((const unsigned char *)s + i, g, -1) & (INTENT_N_FEAT - 1));
    for (int i = 0; i < n; i++)
        ADD(fnv1a((const unsigned char *)toks[i], (int)strlen(toks[i]), 1) & (INTENT_N_FEAT - 1));
#undef ADD
    double norm = 0;
    for (int j = 0; j < nf; j++) norm += (double)val[j] * val[j];
    norm = norm > 0 ? sqrt(norm) : 1.0;

    float z[INTENT_N], m = -1e30f;
    for (int k = 0; k < INTENT_N; k++) {
        float a = intent_b[k];
        for (int j = 0; j < nf; j++) a += intent_W[k][idx[j]] * (float)(val[j] / norm);
        z[k] = a;
        if (a > m) m = a;
    }
    float sum = 0;
    for (int k = 0; k < INTENT_N; k++) { probs[k] = expf(z[k] - m); sum += probs[k]; }
    for (int k = 0; k < INTENT_N; k++) probs[k] /= sum;
}

/* ---------------------------------------------------------------- numbers / slots */
typedef struct { const char *w; int v; } wv_t;
static const wv_t ONES[] = { {"zero",0},{"oh",0},{"one",1},{"two",2},{"three",3},{"four",4},{"five",5},
    {"six",6},{"seven",7},{"eight",8},{"nine",9},{"ten",10},{"eleven",11},{"twelve",12},{"thirteen",13},
    {"fourteen",14},{"fifteen",15},{"sixteen",16},{"seventeen",17},{"eighteen",18},{"nineteen",19},{NULL,0} };
static const wv_t TENS[] = { {"twenty",20},{"thirty",30},{"forty",40},{"fourty",40},{"fifty",50},
    {"sixty",60},{"seventy",70},{"eighty",80},{"ninety",90},{NULL,0} };
static const wv_t SOUNDALIKE[] = { {"to",2},{"too",2},{"for",4},{"won",1},{"ate",8},{NULL,0} };
typedef struct { const char *w; const char *u; } wu_t;
static const wu_t UNITS[] = {
    {"degree","deg"},{"degrees","deg"},{"deg","deg"},{"degs","deg"},
    {"metre","m"},{"metres","m"},{"meter","m"},{"meters","m"},{"m","m"},
    {"centimetre","cm"},{"centimetres","cm"},{"centimeter","cm"},{"centimeters","cm"},{"cm","cm"},
    {"millimetre","mm"},{"millimetres","mm"},{"millimeter","mm"},{"millimeters","mm"},{"mm","mm"},
    {"foot","ft"},{"feet","ft"},{"ft","ft"},{"inch","in"},{"inches","in"},
    {"step","step"},{"steps","step"},{NULL,NULL} };

static int lookup(const wv_t *t, const char *w) { for (; t->w; t++) if (!strcmp(t->w, w)) return t->v; return -1; }
static const char *unit_of(const char *w) { for (const wu_t *u = UNITS; u->w; u++) if (!strcmp(u->w, w)) return u->u; return NULL; }

/* digit token "45" / "1.5" (one optional dot, digits on both sides) -> 1 */
static int num_token(const char *t, double *v)
{
    if (!isdigit((unsigned char)t[0])) return 0;
    int dots = 0;
    for (const char *p = t; *p; p++) {
        if (*p == '.') { if (++dots > 1 || !isdigit((unsigned char)p[1])) return 0; }
        else if (!isdigit((unsigned char)*p)) return 0;
    }
    *v = strtod(t, NULL);
    return 1;
}

/* '(one) hundred [and] eighty [five]': add tens/ones after "hundred" (j = index after it) */
static int after_hundred(char **toks, int n, int j, int v, int *jout)
{
    int k = (j < n && !strcmp(toks[j], "and")) ? j + 1 : j;
    int te = k < n ? lookup(TENS, toks[k]) : -1;
    if (te >= 0) {
        v += te; k++;
        int o2 = k < n ? lookup(ONES, toks[k]) : -1;
        if (o2 > 0 && o2 < 10) { v += o2; k++; }
        *jout = k;
        return v;
    }
    int o = k < n ? lookup(ONES, toks[k]) : -1;
    if (o > 0) { *jout = k + 1; return v + o; }
    *jout = j;
    return v;
}

int intent_parse_amount(char **toks, int n, double *out_val, const char **out_unit)
{
    for (int i = 0; i < n; i++) {
        const char *t = toks[i];
        if ((!strcmp(t, "bit") || !strcmp(t, "little") || !strcmp(t, "tad") || !strcmp(t, "smidge"))
            && i > 0 && (!strcmp(toks[i - 1], "a") || !strcmp(toks[i - 1], "little"))) {
            *out_val = 1.0; *out_unit = "small"; return 1;
        }
        int have = 0, j = i;
        double val = 0, d;
        int o = lookup(ONES, t), te = lookup(TENS, t);
        if (num_token(t, &d)) { val = d; j = i + 1; have = 1; }
        else if (o >= 0 || te >= 0) {
            int v;
            if (te >= 0) {
                v = te; j = i + 1;
                int o2 = j < n ? lookup(ONES, toks[j]) : -1;
                if (o2 > 0 && o2 < 10) { v += o2; j++; }
            } else {
                v = o; j = i + 1;
                if (v > 0 && v < 10 && j < n && (lookup(TENS, toks[j]) >= 0 || !strcmp(toks[j], "hundred"))) {
                    if (!strcmp(toks[j], "hundred")) v = after_hundred(toks, n, j + 1, v * 100, &j);
                    else {
                        v = v * 100 + lookup(TENS, toks[j]); j++;
                        int o2 = j < n ? lookup(ONES, toks[j]) : -1;
                        if (o2 > 0 && o2 < 10) { v += o2; j++; }
                    }
                }
            }
            if (j < n && !strcmp(toks[j], "hundred") && v < 10) v = after_hundred(toks, n, j + 1, v * 100, &j);
            val = v; have = 1;
        } else if (!strcmp(t, "hundred")) { val = after_hundred(toks, n, i + 1, 100, &j); have = 1; }
        else if (!strcmp(t, "half") || !strcmp(t, "quarter")) {
            val = t[0] == 'h' ? 0.5 : 0.25; j = i + 1; have = 1;
            if (j < n && (!strcmp(toks[j], "a") || !strcmp(toks[j], "an"))) j++;
        } else if ((!strcmp(t, "a") || !strcmp(t, "an")) && i + 1 < n && unit_of(toks[i + 1])) {
            val = 1; j = i + 1; have = 1;
        } else if (lookup(SOUNDALIKE, t) >= 0 && i + 1 < n && unit_of(toks[i + 1]) && strcmp(toks[i + 1], "m")) {
            val = lookup(SOUNDALIKE, t); j = i + 1; have = 1;
        }
        if (have) {
            if (j + 2 < n && !strcmp(toks[j], "and") && !strcmp(toks[j + 1], "a") && !strcmp(toks[j + 2], "half")) {
                val += 0.5; j += 3;
            } else if (j + 1 < n && !strcmp(toks[j], "and") && !strcmp(toks[j + 1], "half")) {
                val += 0.5; j += 2;
            }
            const char *u = j < n ? unit_of(toks[j]) : NULL;
            if (!u && j + 1 < n && (!strcmp(toks[j], "a") || !strcmp(toks[j], "of")) && unit_of(toks[j + 1]))
                u = unit_of(toks[j + 1]);
            *out_val = val; *out_unit = u;
            return 1;
        }
    }
    *out_unit = NULL;
    return 0;
}

/* Python round() is banker's rounding; match it. */
static long pyround(double x)
{
    double r = floor(x + 0.5);
    if (fabs(x - (floor(x) + 0.5)) < 1e-12 && fmod(r, 2.0) != 0) r -= 1.0;
    return (long)r;
}

static int move_mm(int have, double val, const char *unit)
{
    if (!have) return DEFAULT_MOVE_MM;
    double mm;
    if (unit && !strcmp(unit, "small")) mm = SMALL_MM;
    else if (unit && !strcmp(unit, "m")) mm = val * 1000;
    else if (unit && !strcmp(unit, "cm")) mm = val * 10;
    else if (unit && !strcmp(unit, "mm")) mm = val;
    else if (unit && !strcmp(unit, "ft")) mm = val * 305;
    else if (unit && !strcmp(unit, "in")) mm = val * 25;
    else if (unit && !strcmp(unit, "step")) mm = val * STEP_MM;
    else if (unit && !strcmp(unit, "deg")) return DEFAULT_MOVE_MM;
    else if (val <= 10) mm = val * STEP_MM;
    else if (val <= 200) mm = val * 10;
    else mm = val;
    long r = pyround(mm);
    if (r <= 0) return DEFAULT_MOVE_MM;
    return r > MAX_MM ? MAX_MM : (int)r;
}

static int turn_deg(int have, double val, const char *unit)
{
    if (!have) return DEFAULT_TURN_DEG;
    if (unit && !strcmp(unit, "small")) return SMALL_DEG;
    if (unit && strcmp(unit, "deg")) return DEFAULT_TURN_DEG;
    if (val < 1) val *= 360;
    long d = pyround(val);
    if (d <= 0) return DEFAULT_TURN_DEG;
    return d > MAX_DEG ? MAX_DEG : (int)d;
}

/* ---------------------------------------------------------------- decision */
static void add_line(intent_decision_t *d, const char *fmt, int v)
{
    if (d->n_lines < INTENT_MAX_LINES)
        snprintf(d->lines[d->n_lines++], INTENT_LINE_LEN, fmt, v);
}

void intent_decide(const char *text, int require_wake, intent_decision_t *d)
{
    char buf[512];
    char *all[MAX_TOKS];
    memset(d, 0, sizeof *d);
    d->intent = "-";
    int n = intent_normalize(text, buf, sizeof buf, all, MAX_TOKS);
    int wake = 0;
    int s = strip_wake(all, n, &wake);
    char **toks = all + s;
    int nt = n - s;
    d->wake = wake;
    if (intent_safety_stop(toks, nt, wake)) {
        d->action = INTENT_ACT_STOP; d->intent = "stop"; d->prob = 1.0f; d->why = "safety";
        add_line(d, "S", 0);
        return;
    }
    if (require_wake && !wake) { d->action = INTENT_ACT_IGNORE; d->why = "no_wake"; return; }
    if (nt == 0) { d->action = INTENT_ACT_IGNORE; d->why = "empty"; return; }

    float p[INTENT_N];
    intent_probs(toks, nt, p);
    int k = 0;
    for (int i = 1; i < INTENT_N; i++) if (p[i] > p[k]) k = i;
    const char *name = intent_names[k];
    d->intent = name;
    d->prob = p[k];
    if (!strcmp(name, "stop") && p[k] >= INTENT_STOP_THRESHOLD) {
        d->action = INTENT_ACT_STOP; d->why = "classifier";
        add_line(d, "S", 0);
        return;
    }
    if (!strcmp(name, "unknown") || p[k] < INTENT_THRESHOLD) {
        d->action = INTENT_ACT_REJECT;
        d->why = !strcmp(name, "unknown") ? "unknown" : "low_conf";
        add_line(d, "LED no", 0);
        add_line(d, "BEEP 3", 0);
        return;
    }
    d->action = INTENT_ACT_COMMAND; d->why = "classifier";
    double val = 0;
    const char *unit = NULL;
    int have = intent_parse_amount(toks, nt, &val, &unit);
    if (!strcmp(name, "forward")) add_line(d, "MOVE %d", move_mm(have, val, unit));
    else if (!strcmp(name, "back")) add_line(d, "MOVE -%d", move_mm(have, val, unit));
    else if (!strcmp(name, "left")) add_line(d, "TURN -%d", turn_deg(have, val, unit));
    else if (!strcmp(name, "right")) add_line(d, "TURN %d", turn_deg(have, val, unit));
    else if (!strcmp(name, "turn_around")) add_line(d, "TURN 180", 0);
    else if (!strcmp(name, "spin")) add_line(d, "TURN 360", 0);
    else if (!strcmp(name, "follow")) add_line(d, "MODE follow", 0);
    else if (!strcmp(name, "gestures")) add_line(d, "MODE gesture", 0);
    else if (!strcmp(name, "teach")) add_line(d, "MODE teach", 0);
    else if (!strcmp(name, "status")) { add_line(d, "STATUS", 0); add_line(d, "BEEP 2", 0); return; }
    add_line(d, "BEEP 1", 0);
}

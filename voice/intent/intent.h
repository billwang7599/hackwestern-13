/* Text -> robot command (pure C99, no allocation, no ML runtime).
 * Same steps as intent.py (the spec); tests/test_intent.py checks parity.
 *
 *   normalize -> wake word -> SAFETY STOP -> classifier -> amount slots -> decision
 */
#ifndef TARS_INTENT_H
#define TARS_INTENT_H

#ifdef __cplusplus
extern "C" {
#endif

#define INTENT_MAX_LINES 3
#define INTENT_LINE_LEN  32

typedef enum {
    INTENT_ACT_IGNORE = 0,   /* no wake word / empty: do nothing, no feedback */
    INTENT_ACT_STOP,         /* send S now */
    INTENT_ACT_COMMAND,      /* send lines[] */
    INTENT_ACT_REJECT        /* low confidence / unknown: LED no + BEEP 3 */
} intent_action_t;

typedef struct {
    intent_action_t action;
    const char *intent;      /* "forward" ... "unknown", or "-" */
    float prob;              /* classifier max prob (1.0 for the safety pass) */
    int wake;                /* wake word heard */
    const char *why;         /* safety | classifier | low_conf | unknown | no_wake | empty */
    int n_lines;
    char lines[INTENT_MAX_LINES][INTENT_LINE_LEN];   /* to tars_motor, in order */
} intent_decision_t;

/* Decide what to do with one transcript. Thread-safe (no globals written). */
void intent_decide(const char *text, int require_wake, intent_decision_t *out);

/* Pieces, exposed for tests / tools. */
int  intent_normalize(const char *text, char *buf, int buflen, char **toks, int max_toks);
int  intent_is_wake(const char *tok);
int  intent_safety_stop(char **toks, int n, int wake);
void intent_probs(char **toks, int n, float *probs /* [INTENT_N] */);
int  intent_parse_amount(char **toks, int n, double *val, const char **unit);
const char *intent_name(int k);
int  intent_count(void);
float intent_threshold(void);

#ifdef __cplusplus
}
#endif
#endif

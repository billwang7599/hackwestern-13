/* goto core: config, object names/aliases, detection-line parser and the
 * go-to state machine. Pure C11, no I/O except loading files, so it builds
 * the same with qcc (QNX) and clang (Mac tests). goto.c does the sockets. */
#ifndef GOTO_CORE_H
#define GOTO_CORE_H
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- config */
typedef struct {
    /* camera */
    double hfov_deg;          /* horizontal field of view of the webcam */
    double min_conf;          /* ignore detections below this */
    /* search */
    double search_step_deg;   /* TURN per search step */
    double search_max_deg;    /* give up after turning this far without a sighting */
    double search_speed;      /* TURN speed fraction (0,1] */
    double settle_s;          /* after a TURN finishes, ignore frames this long */
    int    look_frames;       /* fresh frames to look at after each step */
    int    confirm_frames;    /* ... of which this many must contain the target */
    double frame_timeout_s;   /* no frames at all for this long -> fail "no camera" */
    /* align (exact TURN toward the target) */
    double align_deg;         /* |bearing| above this -> exact TURN instead of steering */
    double align_gain;        /* TURN this fraction of the measured bearing */
    double realign_deg;       /* while driving: |bearing| above this -> stop + align */
    /* approach (closed loop D l r) */
    double fwd;               /* forward drive fraction when centred (steppers: 1.0 = ~4 cm/s) */
    double kp_per_deg;        /* steering per degree of bearing */
    double deadband_deg;      /* no steering inside this */
    double turn_max;          /* clamp on the steering term */
    double slow_deg;          /* forward speed falls to 0 at this bearing */
    double ema_alpha;         /* smoothing of box centre/height, 1 = none */
    double tick_hz;           /* D refresh rate (>= 5 for the 500 ms watchdog) */
    /* arrive / lose */
    double arrive_h_frac;     /* box height >= this * frame height -> arrived */
    double arrive_bottom_frac;/* box bottom edge >= this * frame height -> arrived */
    double arrive_bottom_min_h;/* ... only if the box is at least this tall (frac) */
    int    arrive_frames;     /* consecutive frames meeting the arrive test */
    double lost_s;            /* no target for this long while driving -> stop + search */
    double timeout_s;         /* whole GOTO gives up after this */
    double move_timeout_pad_s;/* exact move overdue by est*2 + this -> fail */
    /* per-object arrive override, e.g. person is tall: arrive_h.person = 0.85 */
    int    n_arrive_over;
    char   arrive_over_label[16][32];
    double arrive_over_h[16];
} goto_cfg;

void goto_cfg_defaults(goto_cfg *c);
/* "key = value" lines, # comments. Returns 0, or -1 if the file can't be
 * opened. Unknown keys are reported in err (first one) and skipped. */
int  goto_cfg_load(goto_cfg *c, const char *path, char *err, size_t errlen);
double goto_cfg_arrive_h(const goto_cfg *c, const char *label);

/* ------------------------------------------------------------ objects */
#define GOTO_MAX_OBJ   32
#define GOTO_MAX_ALIAS 256
typedef struct {
    int  n_obj;
    char obj[GOTO_MAX_OBJ][32];              /* canonical names, file order */
    int  n_alias;
    char alias[GOTO_MAX_ALIAS][40];          /* normalized alias text */
    int  alias_obj[GOTO_MAX_ALIAS];          /* -> index into obj */
} goto_objects;

void goto_objects_defaults(goto_objects *o);  /* built-in copy of goto/objects.txt */
int  goto_objects_load(goto_objects *o, const char *path);   /* 0 ok, -1 can't open */
/* phrase ("the big trash can", "Bin.", "dining table") -> canonical index or -1 */
int  goto_objects_resolve(const goto_objects *o, const char *phrase);
/* lowercase, punctuation -> space, collapse spaces, drop articles */
void goto_normalize(const char *in, char *out, size_t outlen);

/* ------------------------------------------------- detection JSON lines */
#define GOTO_MAX_DETS 64
typedef struct {
    char   label[40];
    double conf;
    double box[4];                            /* x1 y1 x2 y2, pixels */
} goto_det;
typedef struct {
    double t;                                 /* sender's timestamp (informational) */
    int    w, h;
    int    n;
    goto_det d[GOTO_MAX_DETS];
} goto_frame;
/* Parse one line {"t":..,"w":640,"h":480,"dets":[{"label":..,"conf":..,"box":[..]}]}.
 * Returns 0 on success, -1 if it isn't a valid frame. Extra keys are ignored. */
int goto_parse_frame(const char *line, goto_frame *f);

/* ------------------------------------------------------ state machine */
typedef enum {
    ST_IDLE = 0,
    ST_LOOK,          /* standing still, looking at fresh frames */
    ST_SEARCH_TURN,   /* exact TURN of one search step running */
    ST_ALIGN_TURN,    /* exact TURN toward the target running */
    ST_APPROACH,      /* closed-loop D l r toward the target */
} goto_state;

typedef enum {
    ACT_NONE = 0,
    ACT_DRIVE,        /* D a b */
    ACT_STOP,         /* S */
    ACT_TURN,         /* TURN a [b = speed] -> reply gives move id; call goto_sm_move_started */
    ACT_BEEP,         /* BEEP a */
    ACT_LED,          /* LED text */
    ACT_EVENT,        /* text for control-port watchers: "arrived trash can" */
} goto_act_kind;

typedef struct {
    goto_act_kind kind;
    double a, b;
    char text[64];
} goto_act;

#define GOTO_MAX_ACTS 16
typedef struct {
    goto_cfg cfg;
    const goto_objects *objs;
    goto_state st;
    int target;                       /* canonical index, -1 = none */
    char result[64];                  /* last outcome: arrived / not found / ... */
    double now;
    double started, state_since;
    /* look window */
    double fresh_after;               /* frames received after this count as fresh */
    int look_seen, look_hits;
    double look_bearing;              /* averaged bearing of the hits */
    double last_frame;                /* time of the last frame of any kind */
    /* search */
    double searched_deg;
    int search_dir;                   /* +1 right, -1 left */
    /* move tracking */
    int move_id;                      /* -1 = waiting for the TURN reply */
    double move_deadline;
    /* target track */
    int have_track;
    double ema_x, ema_h;              /* x: -1..1 left..right, h: box height / frame height */
    double last_seen;
    double last_bearing;
    int arrive_count;
    double last_tick;
    double cmd_l, cmd_r;              /* last D sent */
    /* stats for STATUS */
    double fps_ema, last_frame_for_fps;
    int frames, frames_with_target;
    /* output queue */
    int n_act;
    goto_act act[GOTO_MAX_ACTS];
} goto_sm;

const char *goto_state_name(goto_state s);
void goto_sm_init(goto_sm *s, const goto_cfg *cfg, const goto_objects *objs);
/* All take the current monotonic time in seconds. After each call, the caller
 * sends s->act[0..n_act) to tars_motor in order and then clears n_act. */
void goto_sm_goto(goto_sm *s, double now, int target);
void goto_sm_cancel(goto_sm *s, double now, const char *why);      /* sends S */
void goto_sm_fail(goto_sm *s, double now, const char *why);        /* S + BEEP 3 + LED no */
void goto_sm_frame(goto_sm *s, double now, const goto_frame *f);
void goto_sm_tick(goto_sm *s, double now);
void goto_sm_move_started(goto_sm *s, double now, int move_id, double est_s);
/* from STATUS: move_id, and move= running|done|aborted|idle, move_why= */
void goto_sm_move_status(goto_sm *s, double now, int move_id, const char *move, const char *why);
int  goto_sm_busy(const goto_sm *s);
int  goto_sm_wants_status(const goto_sm *s);   /* an exact move is being waited on */
/* "state=approach target=trash can ..." (no newline) */
void goto_sm_describe(const goto_sm *s, char *out, size_t len);

#ifdef __cplusplus
}
#endif
#endif

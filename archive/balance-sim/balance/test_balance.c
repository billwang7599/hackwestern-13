/* Unit tests for balance.c. Build & run: make test */
#include "balance.h"

#include <math.h>
#include <stdio.h>

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define DEG (3.14159265f / 180.0f)
#define DT  (1.0f / 400.0f)

/* IMU sample for a robot sitting still at `pitch` */
static balance_imu_t still(float pitch)
{
    balance_imu_t m = { 0.0f, -9.81f * sinf(pitch), 9.81f * cosf(pitch), pitch };
    return m;
}

/* run n ticks at constant imu; returns the last command */
static balance_cmd_t run(balance_state_t *s, const balance_cfg_t *c, balance_imu_t m,
                         float ws, const balance_setpoint_t *sp, int n)
{
    balance_cmd_t o = {0};
    for (int i = 0; i < n; i++) o = balance_step(s, c, &m, ws, sp, DT);
    return o;
}

static void arm(balance_state_t *s, const balance_cfg_t *c)
{
    balance_setpoint_t sp = { 0, 0, 1 };
    run(s, c, still(0), NAN, &sp, (int)(c->rearm_time / DT) + 2);
}

static void test_filter_converges(void)
{
    balance_cfg_t c; balance_default_cfg(&c);
    balance_state_t s; balance_init(&s);
    balance_setpoint_t off = { 0, 0, 0 };
    balance_cmd_t o = run(&s, &c, still(10 * DEG), NAN, &off, 10);
    CHECK(fabsf(o.pitch - 10 * DEG) < 0.2f * DEG, "first sample pitch %f", o.pitch / DEG);
    /* gyro says 0 but accel says 20 deg: should converge with time constant comp_tau */
    o = run(&s, &c, still(20 * DEG), NAN, &off, (int)(5 * c.comp_tau / DT));
    CHECK(fabsf(o.pitch - 20 * DEG) < 0.3f * DEG, "filter converged to %f", o.pitch / DEG);
    CHECK(o.motors_on == 0 && o.state == BAL_DISABLED, "disabled -> motors off");
}

static void test_gyro_integration(void)
{
    balance_cfg_t c; balance_default_cfg(&c); c.comp_tau = 1e6f; /* pure gyro */
    c.gyro_bias_rate = 0;
    balance_state_t s; balance_init(&s);
    balance_setpoint_t off = { 0, 0, 0 };
    balance_imu_t m = still(0);
    balance_step(&s, &c, &m, NAN, &off, DT);
    m.gyro_pitch = 1.0f;              /* 1 rad/s for 0.1 s */
    balance_cmd_t o = run(&s, &c, m, NAN, &off, 40);
    CHECK(fabsf(o.pitch - 0.1f) < 2e-3f, "gyro integral %f", o.pitch);
}

static void test_bias_learning(void)
{
    balance_cfg_t c; balance_default_cfg(&c);
    balance_state_t s; balance_init(&s);
    balance_setpoint_t off = { 0, 0, 0 };
    balance_imu_t m = still(0); m.gyro_pitch = 0.02f;
    run(&s, &c, m, NAN, &off, (int)(10.0f / DT));
    CHECK(fabsf(s.gyro_bias - 0.02f) < 1e-3f, "bias learned %f", s.gyro_bias);
}

static void test_arming_and_cutoff(void)
{
    balance_cfg_t c; balance_default_cfg(&c);
    balance_state_t s; balance_init(&s);
    balance_setpoint_t on = { 0, 0, 1 };
    /* tilted 20 deg: must not arm */
    balance_cmd_t o = run(&s, &c, still(20 * DEG), NAN, &on, 800);
    CHECK(o.state == BAL_WAIT_UPRIGHT && !o.motors_on, "no arm when tilted (state %d)", o.state);
    /* hold upright: arms after rearm_time, not before */
    balance_init(&s);
    o = run(&s, &c, still(0), NAN, &on, (int)(c.rearm_time / DT) - 5);
    CHECK(o.state == BAL_WAIT_UPRIGHT, "not armed yet");
    o = run(&s, &c, still(0), NAN, &on, 10);
    CHECK(o.state == BAL_RUNNING && o.motors_on, "armed");
    /* fused-angle mode so the tilt is seen instantly */
    c.use_fused_pitch = 1;
    o = run(&s, &c, still(45 * DEG), NAN, &on, 2);
    CHECK(o.state == BAL_FALLEN && !o.motors_on && o.left == 0, "fell (state %s)",
          balance_state_name(o.state));
    o = run(&s, &c, still(0), NAN, &on, (int)(c.rearm_time / DT) + 2);
    CHECK(o.state == BAL_RUNNING, "re-armed after being stood up");
}

static void test_signs(void)
{
    balance_cfg_t c; balance_default_cfg(&c); c.use_fused_pitch = 1;
    balance_state_t s; balance_init(&s); arm(&s, &c);
    balance_setpoint_t on = { 0, 0, 1 };
    balance_cmd_t o = run(&s, &c, still(5 * DEG), NAN, &on, 1);
    CHECK(o.accel > 0 && o.left > 0 && o.right > 0, "lean fwd -> drive fwd (a=%f)", o.accel);
    balance_init(&s); arm(&s, &c);
    o = run(&s, &c, still(-5 * DEG), NAN, &on, 1);
    CHECK(o.accel < 0 && o.left < 0, "lean back -> drive back");
    /* D term on measurement: forward pitch rate -> forward accel */
    balance_init(&s); arm(&s, &c);
    float w = 0.5f * c.max_accel / c.kd;            /* stay below the accel clamp */
    balance_imu_t m = still(0); m.gyro_pitch = w;
    o = balance_step(&s, &c, &m, NAN, &on, DT);
    CHECK(fabsf(o.accel - c.kd * w) < 1e-3f, "D term %f", o.accel);
    /* moving forward with zero setpoint -> outer loop asks to lean back */
    balance_init(&s); arm(&s, &c);
    o = run(&s, &c, still(0), 0.5f, &on, 100);
    CHECK(o.pitch_target < 0, "drifting fwd -> lean back target (%f)", o.pitch_target);
    /* forward speed setpoint -> lean forward target */
    balance_init(&s); arm(&s, &c);
    balance_setpoint_t fwd = { 0.5f, 0, 1 };
    o = run(&s, &c, still(0), 0.0f, &fwd, 100);
    CHECK(o.pitch_target > 0, "speed sp -> lean fwd target");
    /* turn left -> right wheel faster */
    balance_init(&s); arm(&s, &c);
    balance_setpoint_t left = { 0, 1.0f, 1 };
    o = run(&s, &c, still(0), NAN, &left, 1);
    CHECK(o.right > o.left, "turn left -> right wheel faster");
}

static void test_clamps_and_windup(void)
{
    balance_cfg_t c; balance_default_cfg(&c); c.use_fused_pitch = 1; c.ki = 50.0f;
    balance_state_t s; balance_init(&s); arm(&s, &c);
    balance_setpoint_t on = { 0, 0, 1 };
    balance_cmd_t o = run(&s, &c, still(30 * DEG), 0.0f, &on, 2000);
    CHECK(fabsf(o.accel) <= c.max_accel + 1e-4f, "accel clamped");
    CHECK(fabsf(o.left) <= c.max_speed + 1e-4f, "speed clamped");
    CHECK(fabsf(s.i_inner) <= c.i_limit + 1e-4f, "inner integral clamped");
    CHECK(fabsf(s.i_outer) <= c.speed_i_limit + 1e-4f, "outer integral clamped");
    CHECK(fabsf(o.pitch_target) <= c.max_lean + 1e-4f, "lean clamped");
    /* integrator should not have wound up while saturated: within 0.1 s of returning
     * upright the command must have reversed sign */
    o = run(&s, &c, still(-2 * DEG), 0.0f, &on, 40);
    CHECK(o.accel < 0, "no windup: accel reversed quickly (%f)", o.accel);
}

static void test_bad_dt(void)
{
    balance_cfg_t c; balance_default_cfg(&c);
    balance_state_t s; balance_init(&s); arm(&s, &c);
    balance_setpoint_t on = { 0, 0, 1 };
    balance_imu_t m = still(1 * DEG);
    balance_cmd_t o = balance_step(&s, &c, &m, NAN, &on, NAN);
    CHECK(!isnan(o.left) && !isnan(o.pitch), "NaN dt handled");
    o = balance_step(&s, &c, &m, NAN, &on, 5.0f);
    CHECK(!isnan(o.left) && fabsf(o.left) <= c.max_speed, "huge dt clamped");
}

int main(void)
{
    test_filter_converges();
    test_gyro_integration();
    test_bias_learning();
    test_arming_and_cutoff();
    test_signs();
    test_clamps_and_windup();
    test_bad_dt();
    if (fails) { printf("%d check(s) FAILED\n", fails); return 1; }
    printf("all balance tests passed\n");
    return 0;
}

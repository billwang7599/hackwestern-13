/* Mac test harness for the nav parity test (tests/nav_parity.py).
 * stdin, one case per line:  <40000 chars of 0/1/2> <radius> <sx> <sz> <tx> <tz>
 * (floats in %a hex so nothing is lost in transit)
 * stdout, one line per case: "ERR <code>" or "OK <n> x z x z ..." with %a floats. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/nav.h"

int main(void)
{
    static char grid[NAV_SIZE * NAV_SIZE + 2];
    nav_t *n = nav_new();
    if (!n) return 1;
    for (;;) {
        float r, sx, sz, tx, tz;
        if (scanf("%40001s %a %a %a %a %a", grid, &r, &sx, &sz, &tx, &tz) != 6) break;
        if (strlen(grid) != NAV_SIZE * NAV_SIZE) { fprintf(stderr, "bad grid\n"); return 1; }
        nav_reset(n);
        nav_set_robot_radius(n, r);
        for (int i = 0; i < NAV_SIZE * NAV_SIZE; i++) nav_set_cell(n, i % NAV_SIZE, i / NAV_SIZE, grid[i] - '0');
        nav_pt_t out[NAV_MAX_PATH];
        int k = nav_plan(n, sx, sz, tx, tz, out, NAV_MAX_PATH);
        if (k < 0) { printf("ERR %d\n", k); continue; }
        printf("OK %d", k);
        for (int i = 0; i < k; i++) printf(" %a %a", (double)out[i].x, (double)out[i].z);
        printf("\n");
        fflush(stdout);
    }
    nav_free(n);
    return 0;
}

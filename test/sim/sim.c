/* SIL (REQ-H05): real conveyor/ctrl code + plant model, CSV in the host.py telemetry format.
   Usage: sim step-vel <mm/s> | move <mm>   [K_mms_per_permille tau_s dz_permille] */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "conveyor.h"
#include "plant.h"

static conveyor_t c;
static plant_t pl;
static uint32_t t_ms;

static void apply(cmd_t k)
{
    conveyor_apply(&c, &k);
}

static bool step(void)
{
    conv_in_t in = {.enc_cnt = plant_cnt(&pl)};
    conv_out_t out;
    conveyor_step(&c, &in, &out);
    plant_period(&pl, out.duty_permille);

    proto_telem_t t;
    conveyor_telem(&c, t_ms, &t);
    printf("%u,%u,%u,%.3f,%.1f,%.1f,%.3f,%d,%u\n", (unsigned)t.t_ms, t.state, t.fault_bits,
           t.pos_um / 1000.0, t.vel_dmms / 10.0, t.vel_ref_dmms / 10.0, t.pos_ref_um / 1000.0,
           t.duty_permille, t.missed_cycles);
    t_ms += CTRL_PERIOD_MS;
    return out.move_done;
}

static void run_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms / CTRL_PERIOD_MS; i++)
        step();
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: sim step-vel <mm/s> | move <mm> [K tau dz]\n");
        return 2;
    }
    pl.k_mms_per_permille = argc > 3 ? atof(argv[3]) : 0.24;
    pl.tau_s = argc > 4 ? atof(argv[4]) : 0.05;
    pl.dz_permille = argc > 5 ? atof(argv[5]) : 60.0;
    double arg = atof(argv[2]);

    conveyor_init(&c, 0);
    printf("t_ms,state,fault_bits,pos_mm,vel_mms,vel_ref_mms,pos_ref_mm,duty,missed_cycles\n");

    apply((cmd_t){.cmd = PROTO_CMD_ENABLE, .arg.enable = 1});
    run_ms(500);

    if (strcmp(argv[1], "step-vel") == 0) {
        apply((cmd_t){.cmd = PROTO_CMD_SET_LIMITS,
                      .arg.limits = {V_MAX_MMS, A_MAX_MAX_MMS2}}); /* close to a step */
        apply((cmd_t){.cmd = PROTO_CMD_SET_VEL, .arg.vel_mms = (int16_t)arg});
        run_ms(3000);
        apply((cmd_t){.cmd = PROTO_CMD_STOP});
        run_ms(1000);
    } else if (strcmp(argv[1], "move") == 0) {
        apply((cmd_t){.cmd = PROTO_CMD_SET_LIMITS, .arg.limits = {60, 200}}); /* PERF-02 */
        apply((cmd_t){.cmd = PROTO_CMD_MOVE_REL, .arg.dist_um = (int32_t)(arg * 1000.0)});
        for (int i = 0; i < 6000 && !step(); i++)
            ;
        run_ms(500);
    } else {
        fprintf(stderr, "unknown scenario %s\n", argv[1]);
        return 2;
    }
    return 0;
}

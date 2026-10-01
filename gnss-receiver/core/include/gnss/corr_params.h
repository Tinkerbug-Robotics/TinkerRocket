/*
 * The correlator's parameters: the one header both sides of the FPGA <-> P4
 * contract read, and the bit-exact model (fpga/model/corr_model.c) implements.
 * PROPOSED in milestone 4 (2026-09-30), for the owner and the hardware design
 * to confirm. Measured on the SignalSim static file against a float correlator:
 * - carrier table: 4 sectors -0.55 dB, 8 sectors (below) -0.10 dB, 16 sectors
 *   -0.07 dB, 32 sectors -0.015 dB;
 * - accumulators: the largest 1 ms value seen is 4,058 against 2^23.
 */
#ifndef GNSS_CORR_PARAMS_H
#define GNSS_CORR_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Channels in the bank. */
#define CORR_MAX_CH 32

/*
 * Carrier NCO: phase in cycles * 2^32; signed increment per sample. At
 * 6.75 MS/s one step is 1.57 mHz.
 */
#define CORR_CARR_BITS 32

/*
 * Code NCO: phase = (chip index << CORR_CODE_FRAC_BITS) | fraction of a chip;
 * unsigned increment per sample in the same units. 40 fraction bits give a
 * chip-rate step of 6 uHz at 6.75 MS/s (2 mm/s), so carrier aiding leaves no
 * code-rate bias the DLL has to absorb.
 */
#define CORR_CODE_FRAC_BITS 40

/* Pending NCO commands per channel (corr_if.h): commands tagged two periods ahead need two. */
#define CORR_CMD_QUEUE 2

/* Correlator taps per channel: early, prompt, late, at +-tap offset from prompt. */
#define CORR_NTAPS 3

/*
 * Carrier mixer (fpga/model/corr_model.c): the top CORR_CARR_LUT_BITS of the
 * carrier phase pick one of 2^bits sectors, and each sector holds integer cos
 * and sin levels for its centre angle. Default: 8 sectors with levels 1 and 2,
 * the GP2021's mixer. Sector k covers phases [k, k+1) / 8 of a cycle.
 */
#define CORR_CARR_LUT_BITS 3
#define CORR_CARR_COS_LUT {2, 1, -1, -2, -2, -1, 1, 2}
#define CORR_CARR_SIN_LUT {1, 2, 2, 1, -1, -2, -2, -1}

/*
 * Accumulators: signed two's complement. A sample's mixed product is at most
 * 3 * 2 + 3 * 2 = 12 in magnitude, so a 1 ms C/A period at 6.75 MS/s stays
 * below 81,000 (18 bits) and a 10 ms B1C period below 810,000 (21 bits). The
 * model flags any value that would not fit.
 */
#define CORR_ACC_BITS 24

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CORR_PARAMS_H */

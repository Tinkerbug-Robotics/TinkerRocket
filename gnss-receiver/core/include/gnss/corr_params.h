/*
 * The correlator's parameters: the one header both sides of the FPGA <-> P4
 * contract read, and the bit-exact model (fpga/model/corr_model.c) implements.
 * PROPOSED in milestone 4 (2026-09-30), for the owner and the hardware design
 * to confirm; the hardware session reports that they all fit the ECP5 plan
 * (time-multiplexed engines, channel state in EBR) and set the counter widths.
 * Measured on the SignalSim static file against a float correlator:
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

/*
 * Counters, set by the hardware session (2026-09-30). The sample counter
 * (t_samp, t_start) is 48 bits: it wraps after 1.3 years at 6.75 MS/s. A
 * channel's period count (dump seq, command apply_seq) is 16 bits: it wraps
 * every 65.5 s of 1 ms periods. The P4 extends both (core/rx.c), and every
 * comparison of two counts is a signed difference modulo the width
 * (corr_if.h).
 */
#define CORR_TSAMP_BITS 48
#define CORR_SEQ_BITS   16

/* Pending NCO commands per channel (corr_if.h): commands tagged two periods ahead need two. */
#define CORR_CMD_QUEUE 2

/*
 * Correlator taps per channel (milestone 6): very early, early, prompt, late and very late
 * on the tracked code (+-tap_offset2, +-tap_offset from prompt), and a prompt on the data
 * code where the signal has one (Galileo E1-B beside E1-C, B1C data beside pilot). Each tap is
 * one I/Q accumulator pair; the hardware session confirmed 6 pairs fit the LFE5U-25F.
 */
#define CORR_NTAPS 6

/* Longest primary code, chips: B1C's 10230 (10 ms). E1 is 4092 (4 ms), L1 C/A 1023 (1 ms). */
#define CORR_MAX_CODE_LEN 10230

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

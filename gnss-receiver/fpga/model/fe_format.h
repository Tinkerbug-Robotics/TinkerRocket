/*
 * The sample stream the FPGA receives from the MAX2769B and hands to its
 * correlators. Every number here is a hardware-session decision; the values are
 * the stage-0 defaults until that session settles them (the correlator's own
 * widths go in corr_params.h, milestone 4).
 */
#ifndef GNSS_FPGA_FE_FORMAT_H
#define GNSS_FPGA_FE_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

#define FE_FS_ADC_HZ   27000000.0                 /* MAX2769B sample clock = the 27 MHz reference */
#define FE_DECIM       4                          /* FPGA L1 decimation */
#define FE_FS_CORR_HZ  (FE_FS_ADC_HZ / FE_DECIM)  /* 6.75 MS/s into the correlators */

/*
 * Frequency plan (hardware session, 2026-09-30; PROVISIONAL until the owner
 * confirms it against the MAX2769B Rev 2 filter-centre settings). Low-IF complex
 * I/Q with the LO below L1, so L1 sits at +IF in the ADC stream: the MAX2769B's
 * default 4.092 MHz centre, from a fractional-N synthesizer on the 27 MHz
 * reference (N = 58, F = 206921 / 2^20): LO 1571.328052 MHz, IF 4.091948 MHz.
 * Keeping every 4th sample folds it to 4.091948 - 6.75 = -2.658052 MHz in the
 * correlators' stream, 42 kHz from 2 fs / 5 and clear of the other simple ratios
 * of 6.75 MS/s. The sign depends on the I/Q assignment and the chip's mixer
 * convention, and first light settles it: every tool takes the IF as a parameter.
 */
#define FE_LO_HZ       (FE_FS_ADC_HZ * (58.0 + 206921.0 / 1048576.0))
#define FE_IF_ADC_HZ   (1575.42e6 - FE_LO_HZ)          /* +4.091948 MHz at 27 MS/s */
#define FE_IF_HZ       (FE_IF_ADC_HZ - FE_FS_CORR_HZ)  /* -2.658052 MHz at 6.75 MS/s */

/*
 * One complex sample = one nibble: 2-bit sign/magnitude on each of I and Q, as
 * the MAX2769B drives its I1 I0 Q1 Q0 pins. Sign 1 = negative; magnitude 1 =
 * the large level. Packed files carry two samples per byte, the earlier sample
 * in the high nibble.
 */
#define FE_CODE_I_SIGN 0x8u
#define FE_CODE_I_MAG  0x4u
#define FE_CODE_Q_SIGN 0x2u
#define FE_CODE_Q_MAG  0x1u

/* Correlator weights of the two magnitude levels (small, large). */
#define FE_WEIGHT_SMALL 1
#define FE_WEIGHT_LARGE 3

#ifdef __cplusplus
}
#endif

#endif /* GNSS_FPGA_FE_FORMAT_H */

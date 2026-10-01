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

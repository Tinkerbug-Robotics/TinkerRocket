/*
 * The correlator's parameters: the one header both sides of the FPGA <-> P4
 * contract read. PROVISIONAL until milestone 4 settles them with the hardware
 * design; the float correlator (host/corr_float.c) already honours every
 * width here, so the loops see the FPGA's NCO resolution from the start.
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

/* Correlator taps per channel: early, prompt, late, at +-tap offset from prompt. */
#define CORR_NTAPS 3

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CORR_PARAMS_H */

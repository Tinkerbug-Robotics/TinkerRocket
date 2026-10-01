/* Per-file metadata for the rig's IQ files, from data/iq_files.ini. Host only. */
#ifndef GNSS_HOST_MANIFEST_H
#define GNSS_HOST_MANIFEST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int found;               /* the file has a manifest entry */
    double fs, fc;           /* Hz; 0 = unknown */
    int has_noise;           /* the file already carries receiver noise */
    int dc_auto;             /* measure the DC offset from the data */
    double dc_i, dc_q;       /* offset to remove, LSB (when not auto) */
    double carrier_fix_hz;   /* carrier-only shift that undoes a generator offset */
    double sig_power;        /* per-satellite C, LSB^2, complex baseband; 0 = unknown */
    double noise_density;    /* N0 already in the file, LSB^2/Hz */
    int have_iono;           /* iono_params: Klobuchar alpha0..3, beta0..3 the generator applied */
    double iono[8];
    int tropo_none;          /* tropo = none: the generator applied no troposphere */
    char generator[32];
    char start_gpst[64];
    char truth[256];
} iq_meta_t;

/* Fills m for the entry named basename (the file name without directories). Returns 0 if found. */
int manifest_lookup(const char *manifest_path, const char *basename, iq_meta_t *m);

/* The default manifest: $GNSS_MANIFEST, else the source tree's data/iq_files.ini. */
const char *manifest_default_path(void);

/*
 * Resolves an IQ file argument: an existing path is used as is; otherwise the
 * name is looked up in $GNSS_IQ_DIR. Writes the path into out. Returns 0 if the file exists.
 */
int manifest_resolve_iq(const char *arg, char *out, unsigned long out_size);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_MANIFEST_H */

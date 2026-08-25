/*
 * lfs_host_stub.c — Stand-in for littlefs in the host build of
 *                   TR_LogToFlash.
 *
 * WHY NOT COMPILE THE REAL lfs.c.  littlefs is portable and would build
 * fine here, but it is useless without a block device, and the block device
 * is TR_LogToFlash's own NAND driver talking to a chip that does not exist
 * on the host.  Standing that up means modelling an SPI NAND — program /
 * erase / page cache / status polling — which is a much larger fake than
 * anything the sink-mode accounting under test needs.
 *
 * WHAT THIS COVERS.  Exactly the calls begin() makes on the way to a
 * mounted filesystem, answered so that begin() reaches runStartupRecovery()
 * and returns true:
 *
 *     lfs_mount        succeeds — begin() skips its format-and-retry path
 *     lfs_file_open    succeeds — the /.health_check probe reads as healthy
 *     lfs_file_close   succeeds
 *     lfs_remove       succeeds — the health-check file is cleaned up
 *     lfs_stat         LFS_ERR_NOENT — no dirty marker, so a clean startup
 *
 * WHAT THIS DELIBERATELY DOES NOT COVER.  LFS-mode LOGGING.  lfs_file_write
 * returns LFS_ERR_IO, so the legacy (cfg.write_sink == NULL) flush path
 * fails visibly on the host rather than passing against a filesystem that
 * is not there.  Sink mode — issue #50 Stage 2c-3c, what every shipping
 * board runs — never calls it: openLogSession, flushRingToNand and
 * closeLogSession all branch on cfg.write_sink first.  A future test that
 * wants the LFS path wants a real block-device fake, not looser stubs here.
 */

#include "lfs.h"

int lfs_format(lfs_t *lfs, const struct lfs_config *config)
{
    (void)lfs; (void)config;
    return 0;
}

int lfs_mount(lfs_t *lfs, const struct lfs_config *config)
{
    (void)lfs; (void)config;
    return 0;
}

int lfs_unmount(lfs_t *lfs)
{
    (void)lfs;
    return 0;
}

int lfs_remove(lfs_t *lfs, const char *path)
{
    (void)lfs; (void)path;
    return 0;
}

int lfs_rename(lfs_t *lfs, const char *oldpath, const char *newpath)
{
    (void)lfs; (void)oldpath; (void)newpath;
    return 0;
}

/* Nothing exists.  This is what makes checkDirtyOnStartup() report a clean
 * previous session and openLogSession()'s filename search settle on the
 * first candidate. */
int lfs_stat(lfs_t *lfs, const char *path, struct lfs_info *info)
{
    (void)lfs; (void)path; (void)info;
    return LFS_ERR_NOENT;
}

lfs_ssize_t lfs_getattr(lfs_t *lfs, const char *path,
        uint8_t type, void *buffer, lfs_size_t size)
{
    (void)lfs; (void)path; (void)type; (void)buffer; (void)size;
    return LFS_ERR_NOATTR;
}

int lfs_setattr(lfs_t *lfs, const char *path,
        uint8_t type, const void *buffer, lfs_size_t size)
{
    (void)lfs; (void)path; (void)type; (void)buffer; (void)size;
    return 0;
}

int lfs_file_open(lfs_t *lfs, lfs_file_t *file, const char *path, int flags)
{
    (void)lfs; (void)file; (void)path; (void)flags;
    return 0;
}

int lfs_file_close(lfs_t *lfs, lfs_file_t *file)
{
    (void)lfs; (void)file;
    return 0;
}

int lfs_file_sync(lfs_t *lfs, lfs_file_t *file)
{
    (void)lfs; (void)file;
    return 0;
}

lfs_ssize_t lfs_file_read(lfs_t *lfs, lfs_file_t *file,
        void *buffer, lfs_size_t size)
{
    (void)lfs; (void)file; (void)buffer; (void)size;
    return LFS_ERR_IO;
}

/* See the header comment: LFS-mode logging is out of scope for this harness
 * and fails loudly rather than silently succeeding. */
lfs_ssize_t lfs_file_write(lfs_t *lfs, lfs_file_t *file,
        const void *buffer, lfs_size_t size)
{
    (void)lfs; (void)file; (void)buffer; (void)size;
    return LFS_ERR_IO;
}

lfs_soff_t lfs_file_seek(lfs_t *lfs, lfs_file_t *file,
        lfs_soff_t off, int whence)
{
    (void)lfs; (void)file; (void)off; (void)whence;
    return LFS_ERR_IO;
}

lfs_soff_t lfs_file_size(lfs_t *lfs, lfs_file_t *file)
{
    (void)lfs; (void)file;
    return 0;
}

int lfs_dir_open(lfs_t *lfs, lfs_dir_t *dir, const char *path)
{
    (void)lfs; (void)dir; (void)path;
    return LFS_ERR_NOENT;
}

int lfs_dir_close(lfs_t *lfs, lfs_dir_t *dir)
{
    (void)lfs; (void)dir;
    return 0;
}

int lfs_dir_read(lfs_t *lfs, lfs_dir_t *dir, struct lfs_info *info)
{
    (void)lfs; (void)dir; (void)info;
    return 0;   /* 0 = end of directory */
}

int lfs_fs_traverse(lfs_t *lfs, int (*cb)(void*, lfs_block_t), void *data)
{
    (void)lfs; (void)cb; (void)data;
    return 0;
}

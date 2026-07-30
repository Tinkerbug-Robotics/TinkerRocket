# Vendored 3D models

STEP models for the parts on these boards, referenced by the custom footprints in
[`../footprints/`](../footprints/) and by the board files.

They live here because the alternative was worse. Every one of these was previously
referenced by an absolute path on one developer's machine, which meant the 3D view was
broken for everyone else who cloned the repo, and the repository published a local
directory layout it had no reason to. References now resolve through `${KIPRJMOD}`, so
they work anywhere.

Models that ship with KiCad itself are **not** duplicated here — those are referenced
through `${KICAD10_3DMODEL_DIR}` and resolve from your own KiCad install.

## Provenance and licensing

**These files are supplied by the component manufacturers**, not authored by this
project. They are redistributed here for the practical purpose of viewing and
mechanically checking the boards.

They are third-party material in the sense of section 6 of the hardware licence, and are
**not** covered by [`../LICENSE`](../LICENSE) (CERN-OHL-S v2). Each remains subject to
whatever terms its manufacturer applies. The originating vendor is identifiable from the
file name — for example `ESP32-S3.step` is Espressif's, `TPS62913RPUR.step` is Texas
Instruments'.

If you are a manufacturer and would prefer your model not be redistributed here, open an
issue and it will be removed.

## Known missing

Two references do not resolve, and neither did before these were vendored:

| Reference | Why |
|---|---|
| `WE-CBA_0805_CBA_rev1.stp` | Würth ferrite bead. The file was already absent from the machine the boards were drawn on — re-downloadable from Würth if the 3D view matters. |
| `TE_826576-6_1x06_P3.96mm_Vertical.step` | Shipped with KiCad 9 and dropped in KiCad 10. Still referenced as `${KICAD9_3DMODEL_DIR}/...`, deliberately — repointing it at the KiCad 10 path would name a file that is not there either. |

Neither affects schematics, layout, ERC/DRC, or fabrication output. Only the 3D viewer.

## Contents

55 models, 29.6 MB on disk (STEP is verbose ASCII and compresses to
roughly a fifth of that in git).

| File | Size |
|---|---|
| `1043P.step` | 3658 KB |
| `21-0137I_T1433-2_MXM.step` | 130 KB |
| `479480001.stp` | 160 KB |
| `6130XX21021_61300621021.step` | 217 KB |
| `878321620.stp` | 556 KB |
| `8L_WSON_8x6x3p4x4p3_MAC.step` | 83 KB |
| `ASPI-0530HI-2R2M-T2.step` | 1057 KB |
| `B2B-PH-SM4-TB.step` | 476 KB |
| `B4B-PH-SM4-TB.step` | 598 KB |
| `BLM18PG471SN1D.step` | 92 KB |
| `BM05B-SRSS-TB.STEP` | 754 KB |
| `BQ21040DBVR.step` | 299 KB |
| `CON-SMA-EDGE-S.step` | 350 KB |
| `DFE201610E-1R0M_P2.step` | 92 KB |
| `DFN_XGIQTR_WIN.step` | 83 KB |
| `DLW21SN670HQ2L.step` | 668 KB |
| `DRL0006A.stp` | 132 KB |
| `DTC123JEBTL.step` | 114 KB |
| `E07-900MM10S.step` | 158 KB |
| `ECS-.327-6-16R-TR.step` | 109 KB |
| `ECS-400-10-37B2-CKY-TR.step` | 275 KB |
| `EKYC160ELL103MM25S.STEP` | 257 KB |
| `ESP32-S3.step` | 1859 KB |
| `FSUSB63UMX.step` | 281 KB |
| `INA230AIRGTR.step` | 666 KB |
| `ISM6HG256XTR.stp` | 447 KB |
| `JS202011JCQN.step` | 472 KB |
| `KMR221GLFS.step` | 618 KB |
| `LGA-12_2X2X0P7_STM.step` | 80 KB |
| `LGA9_BMP585_BOS.step` | 64 KB |
| `MLT-8530.step` | 1428 KB |
| `PB400EEQR1BLK.step` | 252 KB |
| `PMPB14XNX.step` | 451 KB |
| `QFN-104_10x10mm_ESP32P4.step` | 1658 KB |
| `RTE0016K.stp` | 645 KB |
| `S25FL256LAGNFI010_CYP.step` | 64 KB |
| `SAM-M10Q-00B.step` | 618 KB |
| `SF-DFN8_EVS.step` | 83 KB |
| `SMT-0540-T-9-R.step` | 80 KB |
| `SOIC8_4X5_ONS.step` | 149 KB |
| `SP0503BAHTG.step` | 152 KB |
| `SRP4020TA-1R5M.step` | 112 KB |
| `SS24FL.step` | 86 KB |
| `TAJC337M006RNJ.step` | 136 KB |
| `TBLH11-350-05-BK.step` | 1291 KB |
| `TPS2121RUXR.step` | 304 KB |
| `TPS259824ONRGER.step` | 836 KB |
| `TPS62152RGTR.step` | 666 KB |
| `TPS62913RPUR.step` | 245 KB |
| `TPS63021DSJR.step` | 498 KB |
| `TSON_Advance_z90.stp` | 199 KB |
| `USB4110GFA.step` | 3002 KB |
| `USC-ZKN_r.stp` | 59 KB |
| `VLS3012CX-2R2M-1.step` | 782 KB |
| `WE-MAPI_4020.step` | 326 KB |

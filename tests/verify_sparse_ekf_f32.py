#!/usr/bin/env python3
"""Verify sparse EKF timeUpdate + accelMeasUpdate in FLOAT32 (matching ESP32-P4).

This re-runs the same comparison as verify_sparse_ekf.py but forces all
arithmetic through numpy float32 to expose single-precision divergence.
"""

import numpy as np
np.set_printoptions(precision=8, linewidth=120, suppress=True)

F32 = np.float32

G = F32(9.807)
EARTH_RADIUS = F32(6371000.0)
aMarkovTau_s = F32(100.0)
wMarkovTau_s = F32(100.0)
aNoiseSigma = F32(0.05)
wNoiseSigma = F32(0.002)
aMarkovSigma = F32(0.01)
wMarkovSigma = F32(0.001)
R_accel = F32(0.25)

def build_Rw():
    Rw = np.zeros((12,12), dtype=F32)
    for i in range(3): Rw[i,i] = aNoiseSigma**2
    for i in range(3,6): Rw[i,i] = wNoiseSigma**2
    for i in range(6,9): Rw[i,i] = F32(2)*(aMarkovSigma**2)/aMarkovTau_s
    for i in range(9,12): Rw[i,i] = F32(2)*(wMarkovSigma**2)/wMarkovTau_s
    return Rw

def build_static_Fs():
    Fs = np.zeros((15,15), dtype=F32)
    Fs[0,3]=1; Fs[1,4]=1; Fs[2,5]=1
    Fs[5,2]=F32(-2.0)*G/EARTH_RADIUS
    Fs[6,12]=F32(-0.5); Fs[7,13]=F32(-0.5); Fs[8,14]=F32(-0.5)
    for i in range(9,12): Fs[i,i] = F32(-1)/aMarkovTau_s
    for i in range(12,15): Fs[i,i] = F32(-1)/wMarkovTau_s
    return Fs

def build_static_Gs():
    Gs = np.zeros((15,12), dtype=F32)
    Gs[6,3]=F32(-0.5); Gs[7,4]=F32(-0.5); Gs[8,5]=F32(-0.5)
    Gs[9,6]=1; Gs[10,7]=1; Gs[11,8]=1
    Gs[12,9]=1; Gs[13,10]=1; Gs[14,11]=1
    return Gs

def set_dynamic(Fs, Gs, T_B2NED, aEst_B, wEst_B):
    skew_a = np.array([[0, -aEst_B[2], aEst_B[1]],
                       [aEst_B[2], 0, -aEst_B[0]],
                       [-aEst_B[1], aEst_B[0], 0]], dtype=F32)
    Fs_sub = T_B2NED @ skew_a
    Fs[3:6, 6:9] = F32(-2) * Fs_sub
    Fs[3:6, 9:12] = -T_B2NED

    skew_w = np.array([[0, -wEst_B[2], wEst_B[1]],
                       [wEst_B[2], 0, -wEst_B[0]],
                       [-wEst_B[1], wEst_B[0], 0]], dtype=F32)
    Fs[6:9, 6:9] = -skew_w
    Gs[3:6, 0:3] = -T_B2NED

# ===== DENSE REFERENCE (float32) =====
def dense_timeUpdate(P, Fs, Gs, Rw, dt):
    dt = F32(dt)
    PHI = np.eye(15, dtype=F32) + Fs * dt
    Q_c = Gs @ Rw @ Gs.T
    Q_d = dt * Q_c + (dt*dt / F32(2)) * (Fs @ Q_c + Q_c @ Fs.T)
    P_new = PHI @ P @ PHI.T + Q_d
    P_new = F32(0.5) * (P_new + P_new.T)
    return P_new

# ===== SPARSE (matching C++ code, element-by-element in float32) =====
def sparse_timeUpdate(P, Fs, Gs, Rw, dt):
    dt = F32(dt)
    dt2 = F32(dt * dt)
    rw_d = np.diag(Rw).copy()

    # Step 1: B = phi_dt * Gs * diag(Rw)
    B = np.zeros((15, 12), dtype=F32)

    # Cols 0-2: Gs rows 3-5 (3x3 block)
    for i in range(15):
        for c in range(3):
            s = F32(0.0)
            for k in range(3):
                pdt = F32(dt2 * Fs[i, k+3])
                if i == k+3: pdt = F32(pdt + dt)
                s = F32(s + F32(pdt * Gs[k+3, c]))
            B[i, c] = F32(s * rw_d[c])

    # Cols 3-11: single non-zero per column
    src_row = [6, 7, 8, 9, 10, 11, 12, 13, 14]
    src_val = [F32(-0.5), F32(-0.5), F32(-0.5),
               F32(1.0), F32(1.0), F32(1.0),
               F32(1.0), F32(1.0), F32(1.0)]
    for i in range(15):
        for cc in range(9):
            c = cc + 3
            k = src_row[cc]
            pdt = F32(dt2 * Fs[i, k])
            if i == k: pdt = F32(pdt + dt)
            B[i, c] = F32(F32(pdt * src_val[cc]) * rw_d[c])

    # Step 2: Q = B * Gs^T
    Q = np.zeros((15, 15), dtype=F32)

    for i in range(15):
        for j in range(3, 6):
            s = F32(0.0)
            for c in range(3):
                s = F32(s + F32(B[i, c] * Gs[j, c]))
            Q[i, j] = s

    for i in range(15):
        Q[i, 6] = F32(B[i, 3] * F32(-0.5))
        Q[i, 7] = F32(B[i, 4] * F32(-0.5))
        Q[i, 8] = F32(B[i, 5] * F32(-0.5))

    for i in range(15):
        for j in range(9, 15):
            Q[i, j] = B[i, j-3]

    # Symmetrize Q
    for i in range(15):
        for j in range(i+1):
            s = F32(F32(0.5) * F32(Q[i,j] + Q[j,i]))
            Q[i,j] = s; Q[j,i] = s

    # P propagation
    PHI_P = P.copy()

    for j in range(15): PHI_P[0, j] = F32(PHI_P[0, j] + F32(dt * P[3, j]))
    for j in range(15): PHI_P[1, j] = F32(PHI_P[1, j] + F32(dt * P[4, j]))
    for j in range(15): PHI_P[2, j] = F32(PHI_P[2, j] + F32(dt * P[5, j]))

    for i in range(3):
        for j in range(15):
            s = F32(0.0)
            for k in range(3):
                s = F32(s + F32(Fs[i+3, k+6] * P[k+6, j]))
                s = F32(s + F32(Fs[i+3, k+9] * P[k+9, j]))
            PHI_P[i+3, j] = F32(PHI_P[i+3, j] + F32(dt * s))

    f52_dt = F32(dt * Fs[5, 2])
    for j in range(15): PHI_P[5, j] = F32(PHI_P[5, j] + F32(f52_dt * P[2, j]))

    for i in range(3):
        for j in range(15):
            s = F32(0.0)
            for k in range(3):
                s = F32(s + F32(Fs[i+6, k+6] * P[k+6, j]))
            s = F32(s + F32(Fs[i+6, i+12] * P[i+12, j]))
            PHI_P[i+6, j] = F32(PHI_P[i+6, j] + F32(dt * s))

    for i in range(9, 12):
        f_dt = F32(dt * Fs[i, i])
        for j in range(15): PHI_P[i, j] = F32(PHI_P[i, j] + F32(f_dt * P[i, j]))

    for i in range(12, 15):
        f_dt = F32(dt * Fs[i, i])
        for j in range(15): PHI_P[i, j] = F32(PHI_P[i, j] + F32(f_dt * P[i, j]))

    # Step 2: P = PHI_P + Q, then column contributions
    P_new = np.zeros((15, 15), dtype=F32)
    for i in range(15):
        for j in range(15):
            P_new[i, j] = F32(PHI_P[i, j] + Q[i, j])

    for i in range(15): P_new[i, 0] = F32(P_new[i, 0] + F32(dt * PHI_P[i, 3]))
    for i in range(15): P_new[i, 1] = F32(P_new[i, 1] + F32(dt * PHI_P[i, 4]))
    for i in range(15): P_new[i, 2] = F32(P_new[i, 2] + F32(dt * PHI_P[i, 5]))

    for j in range(3):
        for i in range(15):
            s = F32(0.0)
            for k in range(3):
                s = F32(s + F32(PHI_P[i, k+6] * Fs[j+3, k+6]))
                s = F32(s + F32(PHI_P[i, k+9] * Fs[j+3, k+9]))
            P_new[i, j+3] = F32(P_new[i, j+3] + F32(dt * s))

    f52_dt = F32(dt * Fs[5, 2])
    for i in range(15): P_new[i, 5] = F32(P_new[i, 5] + F32(f52_dt * PHI_P[i, 2]))

    for j in range(3):
        for i in range(15):
            s = F32(0.0)
            for k in range(3):
                s = F32(s + F32(PHI_P[i, k+6] * Fs[j+6, k+6]))
            s = F32(s + F32(PHI_P[i, j+12] * Fs[j+6, j+12]))
            P_new[i, j+6] = F32(P_new[i, j+6] + F32(dt * s))

    for j in range(9, 12):
        f_dt = F32(dt * Fs[j, j])
        for i in range(15): P_new[i, j] = F32(P_new[i, j] + F32(f_dt * PHI_P[i, j]))

    for j in range(12, 15):
        f_dt = F32(dt * Fs[j, j])
        for i in range(15): P_new[i, j] = F32(P_new[i, j] + F32(f_dt * PHI_P[i, j]))

    # Symmetrize
    for i in range(15):
        for j in range(i+1):
            s = F32(F32(0.5) * F32(P_new[i,j] + P_new[j,i]))
            P_new[i,j] = s; P_new[j,i] = s
    return P_new

# ===== DENSE accelMeasUpdate (float32) =====
def dense_accelMeasUpdate(P, H, R_accel, y):
    HP = H @ P
    S = HP @ H.T + R_accel * np.eye(3, dtype=F32)
    S_inv = np.linalg.inv(S).astype(F32)
    K = (P @ H.T @ S_inv).astype(F32)

    I_KH = np.eye(15, dtype=F32) - K @ H
    P_new = (I_KH @ P @ I_KH.T + K @ (R_accel * np.eye(3, dtype=F32)) @ K.T).astype(F32)
    P_new = F32(0.5) * (P_new + P_new.T)
    return P_new, K

# ===== SPARSE accelMeasUpdate (float32, matching C++) =====
def sparse_accelMeasUpdate(P, H, R_accel, y):
    HP = np.zeros((3, 15), dtype=F32)
    for i in range(3):
        for j in range(15):
            for k in range(6, 12):
                HP[i, j] = F32(HP[i, j] + F32(H[i, k] * P[k, j]))

    S = np.zeros((3, 3), dtype=F32)
    for i in range(3):
        for j in range(3):
            for k in range(6, 12):
                S[i, j] = F32(S[i, j] + F32(HP[i, k] * H[j, k]))
            if i == j: S[i, j] = F32(S[i, j] + R_accel)

    S_inv = np.linalg.inv(S).astype(F32)

    PHt = np.zeros((15, 3), dtype=F32)
    for i in range(15):
        for j in range(3):
            for k in range(6, 12):
                PHt[i, j] = F32(PHt[i, j] + F32(P[i, k] * H[j, k]))

    K = (PHt @ S_inv).astype(F32)

    I_KH_P = np.zeros((15, 15), dtype=F32)
    for i in range(15):
        for j in range(15):
            I_KH_P[i, j] = P[i, j]
            for m in range(3):
                I_KH_P[i, j] = F32(I_KH_P[i, j] - F32(K[i, m] * HP[m, j]))

    AHt = np.zeros((15, 3), dtype=F32)
    for i in range(15):
        for m in range(3):
            for k in range(6, 12):
                AHt[i, m] = F32(AHt[i, m] + F32(I_KH_P[i, k] * H[m, k]))

    P_new = np.zeros((15, 15), dtype=F32)
    for i in range(15):
        for j in range(15):
            val = I_KH_P[i, j]
            for m in range(3):
                val = F32(val - F32(AHt[i, m] * K[j, m]))
            for m in range(3):
                val = F32(val + F32(F32(K[i, m] * R_accel) * K[j, m]))
            P_new[i, j] = val

    for i in range(15):
        for j in range(i+1):
            s = F32(F32(0.5) * F32(P_new[i,j] + P_new[j,i]))
            P_new[i,j] = s; P_new[j,i] = s
    return P_new, K


# ===== RUN TEST =====
if __name__ == '__main__':
    np.random.seed(42)

    Fs = build_static_Fs()
    Gs = build_static_Gs()
    Rw = build_Rw()

    cp, sp = F32(np.cos(np.radians(80))), F32(np.sin(np.radians(80)))
    T_B2NED = np.array([[sp, 0, -cp],
                         [0, 1, 0],
                         [cp, 0, sp]], dtype=F32)

    aEst_B = np.array([0.1, -0.05, -9.8], dtype=F32)
    wEst_B = np.array([0.01, -0.005, 0.002], dtype=F32)

    set_dynamic(Fs, Gs, T_B2NED, aEst_B, wEst_B)

    P = np.diag(np.array([10.0, 10.0, 10.0,
                           1.0, 1.0, 1.0,
                           0.01, 0.01, 0.01,
                           0.001, 0.001, 0.001,
                           0.001, 0.001, 0.001], dtype=F32))

    noise = (np.random.randn(15,15) * 0.001).astype(F32)
    P = P + noise
    P = F32(0.5) * (P + P.T)
    P = P + F32(0.1) * np.eye(15, dtype=F32)

    dt = F32(0.001)

    print("=== Single-step timeUpdate comparison (float32) ===")
    P_dense = dense_timeUpdate(P, Fs, Gs, Rw, dt)
    P_sparse = sparse_timeUpdate(P, Fs, Gs, Rw, dt)

    diff = np.abs(P_dense - P_sparse)
    max_diff = np.max(diff)
    print(f"Max element-wise difference: {max_diff:.2e}")

    if max_diff > 1e-6:
        worst = np.unravel_index(np.argmax(diff), diff.shape)
        print(f"Worst element at [{worst[0]}][{worst[1]}]: dense={P_dense[worst]:.10e} sparse={P_sparse[worst]:.10e}")
        print("\nAll diffs > 1e-7:")
        for i in range(15):
            for j in range(15):
                if diff[i,j] > 1e-7:
                    print(f"  [{i:2d}][{j:2d}]: dense={P_dense[i,j]:+.8e} sparse={P_sparse[i,j]:+.8e} diff={diff[i,j]:.2e}")
    else:
        print("PASS: timeUpdate matches in float32!")

    # accelMeasUpdate
    print("\n=== Single-step accelMeasUpdate comparison (float32) ===")
    aGrav_B = np.array([T_B2NED[0,2]*G, T_B2NED[1,2]*G, T_B2NED[2,2]*G], dtype=F32)

    H = np.zeros((3, 15), dtype=F32)
    H[0,6] = 0.0;           H[0,7] = 2*aGrav_B[2];  H[0,8] = -2*aGrav_B[1]
    H[1,6] = -2*aGrav_B[2]; H[1,7] = 0.0;           H[1,8] = 2*aGrav_B[0]
    H[2,6] = 2*aGrav_B[1];  H[2,7] = -2*aGrav_B[0]; H[2,8] = 0.0
    H[0,9] = 1.0; H[1,10] = 1.0; H[2,11] = 1.0

    y = np.array([0.05, -0.02, 0.01], dtype=F32)

    P_d, K_d = dense_accelMeasUpdate(P_dense, H, R_accel, y)
    P_s, K_s = sparse_accelMeasUpdate(P_sparse, H, R_accel, y)

    K_diff = np.max(np.abs(K_d - K_s))
    P_diff = np.max(np.abs(P_d - P_s))
    print(f"K max diff: {K_diff:.2e}")
    print(f"P max diff: {P_diff:.2e}")

    if P_diff > 1e-5:
        worst = np.unravel_index(np.argmax(np.abs(P_d - P_s)), P_d.shape)
        print(f"Worst P at [{worst[0]}][{worst[1]}]: dense={P_d[worst]:.10e} sparse={P_s[worst]:.10e}")

    # ===== LONG-DURATION STABILITY TEST =====
    print("\n=== Multi-iteration stability test (1000 steps, float32) ===")
    P_d = P.copy()
    P_s = P.copy()

    max_diff_seen = F32(0.0)
    for step in range(1000):
        aEst_B_t = aEst_B + (np.random.randn(3) * 0.001).astype(F32)
        wEst_B_t = wEst_B + (np.random.randn(3) * 0.0001).astype(F32)
        set_dynamic(Fs, Gs, T_B2NED, aEst_B_t, wEst_B_t)

        P_d = dense_timeUpdate(P_d, Fs, Gs, Rw, dt)
        P_s = sparse_timeUpdate(P_s, Fs, Gs, Rw, dt)

        if step % 10 == 0:
            y_t = (0.01 * np.random.randn(3)).astype(F32)
            P_d, _ = dense_accelMeasUpdate(P_d, H, R_accel, y_t)
            P_s, _ = sparse_accelMeasUpdate(P_s, H, R_accel, y_t)

        diff = np.max(np.abs(P_d - P_s))
        if diff > max_diff_seen:
            max_diff_seen = diff

        if step % 100 == 0:
            # Check P positive definiteness
            eig_d = np.min(np.linalg.eigvalsh(P_d))
            eig_s = np.min(np.linalg.eigvalsh(P_s))
            print(f"Step {step:4d}: diff={diff:.2e}, max_diff_ever={max_diff_seen:.2e}, "
                  f"dense_min_eig={eig_d:.4e}, sparse_min_eig={eig_s:.4e}, "
                  f"dense_diag_max={np.max(np.diag(P_d)):.4e}, sparse_diag_max={np.max(np.diag(P_s)):.4e}")

        if diff > 1.0:
            print(f"\n!!! CATASTROPHIC DIVERGENCE at step {step} !!!")
            print(f"Dense P diag: {np.diag(P_d)}")
            print(f"Sparse P diag: {np.diag(P_s)}")
            break

    final_diff = np.max(np.abs(P_d - P_s))
    print(f"\nFinal max difference after iterations: {final_diff:.2e}")
    print(f"Max difference ever seen: {max_diff_seen:.2e}")

    if max_diff_seen > 0.01:
        print("WARNING: Significant float32 divergence between dense and sparse!")
    elif max_diff_seen > 1e-4:
        print("CAUTION: Measurable float32 drift between dense and sparse")
    else:
        print("PASS: Dense and sparse agree well in float32!")

    # Also check: does the dense float32 itself diverge vs float64 dense?
    print("\n=== Dense float32 vs float64 reference (1000 steps) ===")
    P_f64 = P.astype(np.float64).copy()
    P_f32 = P.copy()
    Fs_f64 = Fs.astype(np.float64)
    Gs_f64 = Gs.astype(np.float64)
    Rw_f64 = Rw.astype(np.float64)

    for step in range(1000):
        aEst_B_t = aEst_B + (np.random.randn(3) * 0.001).astype(F32)
        wEst_B_t = wEst_B + (np.random.randn(3) * 0.0001).astype(F32)

        set_dynamic(Fs, Gs, T_B2NED, aEst_B_t, wEst_B_t)
        Fs_f64 = Fs.astype(np.float64)
        Gs_f64 = Gs.astype(np.float64)

        # Dense f32
        P_f32 = dense_timeUpdate(P_f32, Fs, Gs, Rw, dt)
        # Dense f64
        PHI64 = np.eye(15) + Fs_f64 * np.float64(dt)
        Q_c64 = Gs_f64 @ Rw_f64 @ Gs_f64.T
        Q_d64 = np.float64(dt) * Q_c64 + (np.float64(dt)**2 / 2) * (Fs_f64 @ Q_c64 + Q_c64 @ Fs_f64.T)
        P_f64 = PHI64 @ P_f64 @ PHI64.T + Q_d64
        P_f64 = 0.5 * (P_f64 + P_f64.T)

        if step % 100 == 0:
            diff = np.max(np.abs(P_f64.astype(F32) - P_f32))
            print(f"Step {step:4d}: f32-vs-f64 max diff = {diff:.2e}")

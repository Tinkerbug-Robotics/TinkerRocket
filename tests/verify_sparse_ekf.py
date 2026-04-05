#!/usr/bin/env python3
"""Verify sparse EKF timeUpdate + accelMeasUpdate against dense reference."""

import numpy as np
np.set_printoptions(precision=8, linewidth=120, suppress=True)

G = 9.807
EARTH_RADIUS = 6371000.0
aMarkovTau_s = 100.0
wMarkovTau_s = 100.0
aNoiseSigma = 0.05
wNoiseSigma = 0.002
aMarkovSigma = 0.01
wMarkovSigma = 0.001
R_accel = 0.25

def build_Rw():
    Rw = np.zeros((12,12))
    for i in range(3): Rw[i,i] = aNoiseSigma**2
    for i in range(3,6): Rw[i,i] = wNoiseSigma**2
    for i in range(6,9): Rw[i,i] = 2*(aMarkovSigma**2)/aMarkovTau_s
    for i in range(9,12): Rw[i,i] = 2*(wMarkovSigma**2)/wMarkovTau_s
    return Rw

def build_static_Fs():
    Fs = np.zeros((15,15))
    Fs[0,3]=1; Fs[1,4]=1; Fs[2,5]=1
    Fs[5,2]=-2*G/EARTH_RADIUS
    Fs[6,12]=-0.5; Fs[7,13]=-0.5; Fs[8,14]=-0.5
    for i in range(9,12): Fs[i,i] = -1/aMarkovTau_s
    for i in range(12,15): Fs[i,i] = -1/wMarkovTau_s
    return Fs

def build_static_Gs():
    Gs = np.zeros((15,12))
    Gs[6,3]=-0.5; Gs[7,4]=-0.5; Gs[8,5]=-0.5
    Gs[9,6]=1; Gs[10,7]=1; Gs[11,8]=1
    Gs[12,9]=1; Gs[13,10]=1; Gs[14,11]=1
    return Gs

def set_dynamic(Fs, Gs, T_B2NED, aEst_B, wEst_B):
    """Set dynamic Fs and Gs blocks."""
    skew_a = np.array([[0, -aEst_B[2], aEst_B[1]],
                       [aEst_B[2], 0, -aEst_B[0]],
                       [-aEst_B[1], aEst_B[0], 0]])
    Fs_sub = T_B2NED @ skew_a
    Fs[3:6, 6:9] = -2 * Fs_sub
    Fs[3:6, 9:12] = -T_B2NED

    skew_w = np.array([[0, -wEst_B[2], wEst_B[1]],
                       [wEst_B[2], 0, -wEst_B[0]],
                       [-wEst_B[1], wEst_B[0], 0]])
    Fs[6:9, 6:9] = -skew_w

    Gs[3:6, 0:3] = -T_B2NED

# ===== DENSE REFERENCE =====
def dense_timeUpdate(P, Fs, Gs, Rw, dt):
    PHI = np.eye(15) + Fs * dt
    Q_c = Gs @ Rw @ Gs.T
    # First-order Q_d approximation (Van Loan)
    Q_d = dt * Q_c + (dt**2 / 2) * (Fs @ Q_c + Q_c @ Fs.T)
    P_new = PHI @ P @ PHI.T + Q_d
    P_new = 0.5 * (P_new + P_new.T)
    return P_new

# ===== SPARSE (matching C++ code) =====
def sparse_timeUpdate(P, Fs, Gs, Rw, dt):
    dt2 = dt * dt
    rw_d = np.diag(Rw).copy()

    # Step 1: B = phi_dt * Gs * diag(Rw)
    B = np.zeros((15, 12))

    # Cols 0-2: Gs rows 3-5 (3x3 block)
    for i in range(15):
        for c in range(3):
            s = 0.0
            for k in range(3):
                pdt = dt2 * Fs[i, k+3]
                if i == k+3: pdt += dt
                s += pdt * Gs[k+3, c]
            B[i, c] = s * rw_d[c]

    # Cols 3-11: single non-zero per column
    src_row = [6, 7, 8, 9, 10, 11, 12, 13, 14]
    src_val = [-0.5, -0.5, -0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    for i in range(15):
        for cc in range(9):
            c = cc + 3
            k = src_row[cc]
            pdt = dt2 * Fs[i, k]
            if i == k: pdt += dt
            B[i, c] = pdt * src_val[cc] * rw_d[c]

    # Step 2: Q = B * Gs^T
    Q = np.zeros((15, 15))

    # j = 3..5
    for i in range(15):
        for j in range(3, 6):
            s = 0.0
            for c in range(3):
                s += B[i, c] * Gs[j, c]
            Q[i, j] = s

    # j = 6..8
    for i in range(15):
        Q[i, 6] = B[i, 3] * (-0.5)
        Q[i, 7] = B[i, 4] * (-0.5)
        Q[i, 8] = B[i, 5] * (-0.5)

    # j = 9..14
    for i in range(15):
        for j in range(9, 15):
            Q[i, j] = B[i, j-3]

    # Symmetrize Q
    Q = 0.5 * (Q + Q.T)

    # P propagation
    PHI_P = P.copy()

    # Row 0-2
    for j in range(15): PHI_P[0, j] += dt * P[3, j]
    for j in range(15): PHI_P[1, j] += dt * P[4, j]
    for j in range(15): PHI_P[2, j] += dt * P[5, j]

    # Rows 3-5
    for i in range(3):
        for j in range(15):
            s = 0.0
            for k in range(3):
                s += Fs[i+3, k+6] * P[k+6, j]
                s += Fs[i+3, k+9] * P[k+9, j]
            PHI_P[i+3, j] += dt * s

    # Row 5 extra: Fs[5][2]
    f52_dt = dt * Fs[5, 2]
    for j in range(15): PHI_P[5, j] += f52_dt * P[2, j]

    # Rows 6-8
    for i in range(3):
        for j in range(15):
            s = 0.0
            for k in range(3):
                s += Fs[i+6, k+6] * P[k+6, j]
            s += Fs[i+6, i+12] * P[i+12, j]
            PHI_P[i+6, j] += dt * s

    # Rows 9-11
    for i in range(9, 12):
        f_dt = dt * Fs[i, i]
        for j in range(15): PHI_P[i, j] += f_dt * P[i, j]

    # Rows 12-14
    for i in range(12, 15):
        f_dt = dt * Fs[i, i]
        for j in range(15): PHI_P[i, j] += f_dt * P[i, j]

    # Step 2: P = PHI_P + Q, then add column contributions
    P_new = PHI_P + Q

    # Col 0-2
    for i in range(15): P_new[i, 0] += dt * PHI_P[i, 3]
    for i in range(15): P_new[i, 1] += dt * PHI_P[i, 4]
    for i in range(15): P_new[i, 2] += dt * PHI_P[i, 5]

    # Cols 3-5
    for j in range(3):
        for i in range(15):
            s = 0.0
            for k in range(3):
                s += PHI_P[i, k+6] * Fs[j+3, k+6]
                s += PHI_P[i, k+9] * Fs[j+3, k+9]
            P_new[i, j+3] += dt * s

    # Col 5 extra
    f52_dt = dt * Fs[5, 2]
    for i in range(15): P_new[i, 5] += f52_dt * PHI_P[i, 2]

    # Cols 6-8
    for j in range(3):
        for i in range(15):
            s = 0.0
            for k in range(3):
                s += PHI_P[i, k+6] * Fs[j+6, k+6]
            s += PHI_P[i, j+12] * Fs[j+6, j+12]
            P_new[i, j+6] += dt * s

    # Cols 9-11
    for j in range(9, 12):
        f_dt = dt * Fs[j, j]
        for i in range(15): P_new[i, j] += f_dt * PHI_P[i, j]

    # Cols 12-14
    for j in range(12, 15):
        f_dt = dt * Fs[j, j]
        for i in range(15): P_new[i, j] += f_dt * PHI_P[i, j]

    # Symmetrize
    P_new = 0.5 * (P_new + P_new.T)
    return P_new

# ===== DENSE accelMeasUpdate reference =====
def dense_accelMeasUpdate(P, H, R_accel, y):
    HP = H @ P
    S = HP @ H.T + R_accel * np.eye(3)
    S_inv = np.linalg.inv(S)
    K = P @ H.T @ S_inv

    I_KH = np.eye(15) - K @ H
    P_new = I_KH @ P @ I_KH.T + K @ (R_accel * np.eye(3)) @ K.T
    P_new = 0.5 * (P_new + P_new.T)
    return P_new, K

# ===== SPARSE accelMeasUpdate (matching C++) =====
def sparse_accelMeasUpdate(P, H, R_accel, y):
    # HP (using H sparsity: cols 6-11)
    HP = np.zeros((3, 15))
    for i in range(3):
        for j in range(15):
            for k in range(6, 12):
                HP[i, j] += H[i, k] * P[k, j]

    # S
    S = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            for k in range(6, 12):
                S[i, j] += HP[i, k] * H[j, k]
            if i == j: S[i, j] += R_accel

    S_inv = np.linalg.inv(S)

    # PHt
    PHt = np.zeros((15, 3))
    for i in range(15):
        for j in range(3):
            for k in range(6, 12):
                PHt[i, j] += P[i, k] * H[j, k]

    K = PHt @ S_inv

    # I_KH_P
    I_KH_P = np.zeros((15, 15))
    for i in range(15):
        for j in range(15):
            I_KH_P[i, j] = P[i, j]
            for m in range(3):
                I_KH_P[i, j] -= K[i, m] * HP[m, j]

    # AHt
    AHt = np.zeros((15, 3))
    for i in range(15):
        for m in range(3):
            for k in range(6, 12):
                AHt[i, m] += I_KH_P[i, k] * H[m, k]

    # P_new
    P_new = np.zeros((15, 15))
    for i in range(15):
        for j in range(15):
            val = I_KH_P[i, j]
            for m in range(3):
                val -= AHt[i, m] * K[j, m]
            for m in range(3):
                val += K[i, m] * R_accel * K[j, m]
            P_new[i, j] = val

    P_new = 0.5 * (P_new + P_new.T)
    return P_new, K


# ===== RUN TEST =====
if __name__ == '__main__':
    np.random.seed(42)

    Fs = build_static_Fs()
    Gs = build_static_Gs()
    Rw = build_Rw()

    # Realistic T_B2NED for nose-up rocket (45 deg pitch)
    cp, sp = np.cos(np.radians(80)), np.sin(np.radians(80))
    T_B2NED = np.array([[sp, 0, -cp],
                         [0, 1, 0],
                         [cp, 0, sp]])

    aEst_B = np.array([0.1, -0.05, -9.8])  # ~1g gravity in body frame
    wEst_B = np.array([0.01, -0.005, 0.002])  # small rotation

    set_dynamic(Fs, Gs, T_B2NED, aEst_B, wEst_B)

    # Initial P (diagonal, typical values)
    P = np.diag([10.0, 10.0, 10.0,   # position
                 1.0, 1.0, 1.0,       # velocity
                 0.01, 0.01, 0.01,    # attitude
                 0.001, 0.001, 0.001, # accel bias
                 0.001, 0.001, 0.001])# gyro bias

    # Add some off-diagonal terms to make it realistic
    P += np.random.randn(15,15) * 0.001
    P = 0.5 * (P + P.T)  # ensure symmetric
    P += 0.1 * np.eye(15)  # ensure positive definite

    dt = 0.001  # 1 ms

    print("=== timeUpdate comparison ===")
    P_dense = dense_timeUpdate(P, Fs, Gs, Rw, dt)
    P_sparse = sparse_timeUpdate(P, Fs, Gs, Rw, dt)

    diff = np.abs(P_dense - P_sparse)
    max_diff = np.max(diff)
    print(f"Max element-wise difference: {max_diff:.2e}")

    if max_diff > 1e-10:
        # Find the worst elements
        worst = np.unravel_index(np.argmax(diff), diff.shape)
        print(f"Worst element at [{worst[0]}][{worst[1]}]: dense={P_dense[worst]:.10e} sparse={P_sparse[worst]:.10e}")

        print("\nDense P diagonal:", np.diag(P_dense))
        print("Sparse P diagonal:", np.diag(P_sparse))

        print("\nDifference matrix (showing abs > 1e-12):")
        for i in range(15):
            for j in range(15):
                if diff[i,j] > 1e-12:
                    print(f"  [{i:2d}][{j:2d}]: dense={P_dense[i,j]:+.8e} sparse={P_sparse[i,j]:+.8e} diff={diff[i,j]:.2e}")
    else:
        print("PASS: timeUpdate matches!")

    # Now test accelMeasUpdate
    print("\n=== accelMeasUpdate comparison ===")
    aGrav_B = np.array([T_B2NED[0,2]*G, T_B2NED[1,2]*G, T_B2NED[2,2]*G])

    H = np.zeros((3, 15))
    H[0,6] = 0.0;           H[0,7] = 2*aGrav_B[2];  H[0,8] = -2*aGrav_B[1]
    H[1,6] = -2*aGrav_B[2]; H[1,7] = 0.0;           H[1,8] = 2*aGrav_B[0]
    H[2,6] = 2*aGrav_B[1];  H[2,7] = -2*aGrav_B[0]; H[2,8] = 0.0
    H[0,9] = 1.0; H[1,10] = 1.0; H[2,11] = 1.0

    y = np.array([0.05, -0.02, 0.01])  # small innovation

    P_d, K_d = dense_accelMeasUpdate(P_dense, H, R_accel, y)
    P_s, K_s = sparse_accelMeasUpdate(P_sparse, H, R_accel, y)

    K_diff = np.max(np.abs(K_d - K_s))
    P_diff = np.max(np.abs(P_d - P_s))
    print(f"K max diff: {K_diff:.2e}")
    print(f"P max diff: {P_diff:.2e}")

    if P_diff > 1e-8:
        worst = np.unravel_index(np.argmax(np.abs(P_d - P_s)), P_d.shape)
        print(f"Worst P element at [{worst[0]}][{worst[1]}]: dense={P_d[worst]:.10e} sparse={P_s[worst]:.10e}")
    else:
        print("PASS: accelMeasUpdate matches!")

    # Run multiple iterations to check for instability
    print("\n=== Multi-iteration stability test (100 iterations) ===")
    P_d = P.copy()
    P_s = P.copy()

    for step in range(100):
        # Small dynamic changes each step
        aEst_B_t = aEst_B + 0.001 * np.random.randn(3)
        wEst_B_t = wEst_B + 0.0001 * np.random.randn(3)
        set_dynamic(Fs, Gs, T_B2NED, aEst_B_t, wEst_B_t)

        P_d = dense_timeUpdate(P_d, Fs, Gs, Rw, dt)
        P_s = sparse_timeUpdate(P_s, Fs, Gs, Rw, dt)

        if step % 10 == 0:
            # Occasionally do accel update
            y_t = 0.01 * np.random.randn(3)
            P_d, _ = dense_accelMeasUpdate(P_d, H, R_accel, y_t)
            P_s, _ = sparse_accelMeasUpdate(P_s, H, R_accel, y_t)

        diff = np.max(np.abs(P_d - P_s))
        if step % 20 == 0:
            print(f"Step {step:3d}: max diff={diff:.2e}, dense diag max={np.max(np.diag(P_d)):.4e}, sparse diag max={np.max(np.diag(P_s)):.4e}")

    final_diff = np.max(np.abs(P_d - P_s))
    print(f"\nFinal max difference after 100 steps: {final_diff:.2e}")
    if final_diff > 1e-6:
        print("WARNING: Significant divergence between dense and sparse!")
        worst = np.unravel_index(np.argmax(np.abs(P_d - P_s)), P_d.shape)
        print(f"Worst at [{worst[0]}][{worst[1]}]: dense={P_d[worst]:.10e} sparse={P_s[worst]:.10e}")
    else:
        print("PASS: Dense and sparse agree after 100 iterations!")

#%%
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

#%% STARTUP
print("Program Started")

client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

# === GET HANDLES ===
wR_Handle   = sim.getObject("/rightMotor")
wL_Handle   = sim.getObject("/leftMotor")
s3_Handle   = sim.getObject("/ultrasonicSensor[3]")
p3dx_Handle = sim.getObject("/PioneerP3DX")
disc_Handle = sim.getObject("/Disc")

# === TRANSFORMATION MATRIX ===
def transformMat(alpha, beta, gamma, tx, ty, tz):
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
    ])

    roty = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
    ])

    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0, 0, 1]
    ])

    rot_total = rotx @ roty @ rotz

    trans_vector = np.array([[tx],
                             [ty],
                             [tz]])

    R_t_3x4 = np.hstack((rot_total, trans_vector))
    homogeneous_row = np.array([[0, 0, 0, 1]])
    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4


# === DATA CONTAINER ===
d_xyyaw     = []
d_t         = []
dat_disc2rob = np.zeros((4, 1))
dat_errors   = np.zeros((3, 1))

t_prv = 0.0

sim.addLog(1, "get vel start")
time.sleep(2)
start_time = time.time()

# === PARAMETER KONTROL ===
rw    = 0.195 / 2.0   # wheel radius (m)
rb    = 0.381 / 2.0   # body radius (half track) (m)
d_sw  = 0.05           # switching distance (m), silakan disetel
kp_lin = 0.6          # gain kecepatan linear
k_h    = 0.8          # base gain heading
k_o    = 8.0          # base gain orientasi

kin_mat = np.array([
    [1.0,  rb],
    [1.0, -rb]
])

# === MAIN LOOP ===
while True:
    t_now = time.time() - start_time
    if t_now > 30.0:
        break

    # --- pose robot & disc di world frame ---
    bod_pos_xyz = sim.getObjectPosition(p3dx_Handle)      # [x_r, y_r, z_r]
    bod_pos_abg = sim.getObjectOrientation(p3dx_Handle)   # [α_r, β_r, θ_r]

    disc_pos_xyz = sim.getObjectPosition(disc_Handle)     # [x_t, y_t, z_t]
    disc_pos_abg = sim.getObjectOrientation(disc_Handle)  # [α_t, β_t, θ_t]

    # --- posisi disc dalam homogeneous koordinat world ---
    disc_pos_xyz_hom = np.array([
        [disc_pos_xyz[0]],
        [disc_pos_xyz[1]],
        [disc_pos_xyz[2]],
        [1.0]
    ])

    # --- transformasi disc ke body frame robot ---
    T_wB = transformMat(0, 0, bod_pos_abg[2],
                        bod_pos_xyz[0], bod_pos_xyz[1], 0.0)
    T_Bw = np.linalg.inv(T_wB)
    disc2body_pos = T_Bw @ disc_pos_xyz_hom  # 4x1

    # Ambil hanya komponen x,y (body frame)
    x_b = float(disc2body_pos[0, 0])
    y_b = float(disc2body_pos[1, 0])

    # === ERROR CALCULATION ===
    # distance error di sumbu depan robot
    ed = x_b

    # heading error (sudut target relatif terhadap heading robot)
    eh = math.atan2(y_b, x_b)

    # orientation error (beda orientasi robot & disc), dinormalisasi
    eo_raw = bod_pos_abg[2] - disc_pos_abg[2]
    eo = math.atan2(math.sin(eo_raw), math.cos(eo_raw))

    # Euclidean distance di world frame
    dx = disc_pos_xyz[0] - bod_pos_xyz[0]
    dy = disc_pos_xyz[1] - bod_pos_xyz[1]
    abs_d = math.hypot(dx, dy)  # d_eu ≥ 0

    errors = np.vstack(([[ed]], [[eh]], [[eo]]))

    # === SWITCHING MODE (POSISI vs ORIENTASI) ===
    # mode ∈ (0,1], makin dekat target → mode ~ 1 (orientasi dominan)
    mode = math.exp(-abs_d / d_sw)

    kp_ang = k_h * (1.0 - mode)   # heading control aktif saat jauh
    kp_ori = k_o * mode           # orientasi control aktif saat dekat

    # === KONTROL v & ω ===
    v_cmd = kp_lin * ed
    w_cmd = kp_ang * eh + kp_ori * eo

    vel_vec = np.vstack(([[v_cmd]], [[w_cmd]]))  # [v; ω]

    # === WHEEL VELOCITY (linear v_R dan v_L) ===
    v_rl = kin_mat @ vel_vec   # [v_R; v_L]

    # Konversi ke kecepatan sudut roda (rad/s)
    phi_R = v_rl[0, 0] / rw
    phi_L = v_rl[1, 0] / rw

    # === ACTUATION ===
    sim.setJointTargetVelocity(wR_Handle, phi_R)
    sim.setJointTargetVelocity(wL_Handle, phi_L)

    # === SAVE DATA ===
    dat_disc2rob = np.hstack((dat_disc2rob, disc2body_pos))
    dat_errors   = np.hstack((dat_errors, errors))

    d_xyyaw.append([
        bod_pos_xyz[0],
        bod_pos_xyz[1],
        bod_pos_abg[2]
    ])
    d_t.append(t_now)

    # === LOGGING ===
    sim.addLog(
        1,
        f"t={t_now:.2f}s | ed={ed:.2f} m | eh={math.degrees(eh):.2f} deg | "
        f"eo={math.degrees(eo):.2f} deg | d={abs_d:.2f} m | mode={mode:.2f}"
    )

    t_prv = t_now

# === STOP ROBOT ===
sim.setJointTargetVelocity(wR_Handle, 0.0)
sim.setJointTargetVelocity(wL_Handle, 0.0)
sim.addLog(1, "sim com ended")

# === CONVERT TO NP ARRAY ===
dat_xyyaw   = np.array(d_xyyaw)
dat_t       = np.array(d_t)
dat_disc2rob = dat_disc2rob[:, 1:]  # remove first zero column
dat_errors   = dat_errors[:, 1:]

# normalisasi yaw
dat_xyyaw[:, 2] = np.arctan2(np.sin(dat_xyyaw[:, 2]),
                             np.cos(dat_xyyaw[:, 2]))

# %% PLOT XY DOMAIN
plt.figure(figsize=(8, 6))
plt.plot(dat_xyyaw[:, 0], dat_xyyaw[:, 1],
         linewidth=2, label='$^wB$')
plt.scatter(dat_xyyaw[0, 0], dat_xyyaw[0, 1],
            marker='o', s=100, color='red', label='Start')
plt.scatter(dat_xyyaw[-1, 0], dat_xyyaw[-1, 1],
            marker='x', s=100, color='green', label='End')

plt.xlabel('$x_w$ (m)', fontsize=12)
plt.ylabel('$y_w$ (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_xy_track") + ".svg"
print(f"Plot saved successfully as '{filename}'")

# %% PLOT e_d(t)
plt.figure(figsize=(8, 6))
plt.plot(dat_t, dat_errors[0, :],
         linewidth=2, label='$e_d$')
plt.xlabel('$t$ (s)', fontsize=12)
plt.ylabel('$e_d$ (m)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_ed_track") + ".svg"
print(f"Plot saved successfully as '{filename}'")

# %% PLOT e_h(t)
plt.figure(figsize=(8, 6))
plt.plot(dat_t, np.rad2deg(dat_errors[1, :]),
         linewidth=2, label='$e_h$')
plt.xlabel('$t$ (s)', fontsize=12)
plt.ylabel('$e_h$ (deg)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_eh_track") + ".svg"
print(f"Plot saved successfully as '{filename}'")

# (opsional) PLOT e_o(t)
plt.figure(figsize=(8, 6))
plt.plot(dat_t, np.rad2deg(dat_errors[2, :]),
         linewidth=2, label='$e_o$')
plt.xlabel('$t$ (s)', fontsize=12)
plt.ylabel('$e_o$ (deg)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_eo_track") + ".svg"
print(f"Plot saved successfully as '{filename}'")

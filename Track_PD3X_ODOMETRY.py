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
wR_Handle = sim.getObject("/rightMotor")
wL_Handle = sim.getObject("/leftMotor")
p3dx_Handle = sim.getObject("/PioneerP3DX")
disc_Handle = sim.getObject("/Disc")


# === TRANSFORM FUNCTION ===
def transformMat(alpha, beta, gamma, tx, ty, tz):
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
    ])
    roty = np.array([
        [math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
    ])
    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0, 0, 1]
    ])
    rot_total = rotx @ roty @ rotz
    trans_vector = np.array([[tx], [ty], [tz]])
    R_t_3x4 = np.hstack((rot_total, trans_vector))
    homogeneous_row = np.array([[0, 0, 0, 1]])
    return np.vstack((R_t_3x4, homogeneous_row))


# === BUFFER ===
d_xyyaw = []
d_t = []
dat_disc2rob = np.zeros((4, 1))
dat_errors = np.zeros((3, 1))

# === CONTROL CONSTANTS ===
rw = 0.195 / 2     # wheel radius
rb = 0.381 / 2     # body radius
d  = 0.04          # switching distance for exponential mode
d_th = 0.10        # orientation mute threshold (10 cm)

# toleransi "udah sampai"
tol_d  = 0.08                     # 8 cm
tol_eh = math.radians(8)          # 8 deg tolerance
dead_ed = 0.01                    # deadzone X (1 cm)
dead_eh = math.radians(3)         # deadzone heading (3 deg)

# STOP MODE flag
stop_mode = False

start_time = time.time()
sim.addLog(1, "Starting control loop...")
time.sleep(1)

# ===========================================================
# MAIN LOOP
# ===========================================================
while True:
    t_now = time.time() - start_time
    if t_now > 30:
        break

    # ===== POSE ROBOT & DISC =====
    bod_pos_xyz = sim.getObjectPosition(p3dx_Handle)
    bod_pos_abg = sim.getObjectOrientation(p3dx_Handle)

    disc_pos_xyz = sim.getObjectPosition(disc_Handle)
    disc_pos_abg = sim.getObjectOrientation(disc_Handle)

    disc_pos_xyz_hom = np.array([
        [disc_pos_xyz[0]],
        [disc_pos_xyz[1]],
        [disc_pos_xyz[2]],
        [1]
    ])

    # === Transform disc → robot frame ===
    disc2body_mat = transformMat(0, 0, bod_pos_abg[2],
                                 bod_pos_xyz[0], bod_pos_xyz[1], 0)
    disc2body_pos = np.linalg.inv(disc2body_mat) @ disc_pos_xyz_hom

    # === ERROR CALCULATION ===
    ed = disc2body_pos[0][0]
    eh = math.atan2(disc2body_pos[1][0], disc2body_pos[0][0])

    # Orientation error (raw)
    eo_raw = bod_pos_abg[2] - disc_pos_abg[2]
    eo_raw = math.atan2(math.sin(eo_raw), math.cos(eo_raw))

    # Euclidean distance (world frame)
    abs_d = math.sqrt((disc_pos_xyz[0] - bod_pos_xyz[0])**2 +
                      (disc_pos_xyz[1] - bod_pos_xyz[1])**2)

    # === STOP MODE LOGIC: target still inside tolerance ===
    if stop_mode:
        # keluar dari stop mode kalau target keluar toleransi
        if (abs_d > tol_d) or (abs(eh) > tol_eh):
            stop_mode = False
            sim.addLog(1, "TARGET MOVED — EXIT STOP MODE")
        else:
            # Tetap berhenti
            sim.setJointTargetVelocity(wR_Handle, 0.0)
            sim.setJointTargetVelocity(wL_Handle, 0.0)

            # Log data
            d_xyyaw.append([bod_pos_xyz[0], bod_pos_xyz[1], bod_pos_abg[2]])
            d_t.append(t_now)
            dat_disc2rob = np.hstack((dat_disc2rob, disc2body_pos))
            dat_errors = np.hstack((dat_errors, np.array([[0.0], [0.0], [0.0]])))

            continue  # skip kontrol

    # === DEADZONE ===
    if abs(ed) < dead_ed:
        ed = 0.0
    if abs(eh) < dead_eh:
        eh = 0.0

    # === GOAL CONDITION: masuk STOP MODE ===
    if (abs_d < tol_d) and (abs(eh) < tol_eh):
        stop_mode = True

        # Stop motors
        sim.setJointTargetVelocity(wR_Handle, 0.0)
        sim.setJointTargetVelocity(wL_Handle, 0.0)

        # Log data
        d_xyyaw.append([bod_pos_xyz[0], bod_pos_xyz[1], bod_pos_abg[2]])
        d_t.append(t_now)
        dat_disc2rob = np.hstack((dat_disc2rob, disc2body_pos))
        dat_errors = np.hstack((dat_errors, np.array([[0.0], [0.0], [0.0]])))

        sim.addLog(1, f"STOP MODE ACTIVE | dist={abs_d:.3f}")
        continue  # tetap loop agar bisa detect target dipindah

    # === MUTE ORIENTATION ERROR SAAT DEKAT ===
    if abs_d < d_th:
        eo = 0.0
    else:
        eo = eo_raw

    # === SWITCHING MODE ===
    mode = math.exp(-abs_d / d)

    # === GAINS ===
    kp_lin = 0.6
    kp_ang = 0.8 * (1 - mode)
    kp_ori = 8 * mode

    # === CONTROL LAW ===
    v_x = kp_lin * ed
    omega = kp_ang * eh + kp_ori * eo

    # === DIFFERENTIAL DRIVE ===
    kin_mat = np.array([
        [1,  rb],
        [1, -rb]
    ])
    vel_vec = np.array([[v_x], [omega]])

    v_rl = kin_mat @ vel_vec
    vR = v_rl[0][0]
    vL = v_rl[1][0]

    sim.setJointTargetVelocity(wR_Handle, vR / rw)
    sim.setJointTargetVelocity(wL_Handle, vL / rw)

    # === LOGGING ===
    d_xyyaw.append([
        bod_pos_xyz[0],
        bod_pos_xyz[1],
        bod_pos_abg[2]
    ])
    d_t.append(t_now)

    dat_disc2rob = np.hstack((dat_disc2rob, disc2body_pos))
    dat_errors = np.hstack((dat_errors, np.array([[ed], [eh], [eo]])))

# === SAFETY STOP ===
sim.setJointTargetVelocity(wR_Handle, 0)
sim.setJointTargetVelocity(wL_Handle, 0)
sim.addLog(1, "Simulation ended.")


# ===========================================================
#                   PLOTTING
# ===========================================================

d_xyyaw = np.array(d_xyyaw)
d_t = np.array(d_t)
dat_disc2rob = dat_disc2rob[:, 1:]
dat_errors = dat_errors[:, 1:]

# XY Trajectory
plt.figure(figsize=(8,6))
plt.plot(d_xyyaw[:,0], d_xyyaw[:,1], 'b', linewidth=2, label='Robot Path')
plt.scatter(d_xyyaw[0,0], d_xyyaw[0,1], s=100, c='red', label='Start')
plt.scatter(d_xyyaw[-1,0], d_xyyaw[-1,1], s=100, c='green', label='End')
plt.grid(); plt.legend()
plt.xlabel("x (m)"); plt.ylabel("y (m)")
plt.title("XY Trajectory")
plt.show()

# Error distance
plt.figure(figsize=(8,6))
plt.plot(d_t, dat_errors[0], linewidth=2)
plt.grid(); plt.xlabel("t (s)"); plt.ylabel("e_d (m)")
plt.title("Distance Error")
plt.show()

# Error heading
plt.figure(figsize=(8,6))
plt.plot(d_t, np.rad2deg(dat_errors[1]), linewidth=2)
plt.grid(); plt.xlabel("t (s)"); plt.ylabel("e_h (deg)")
plt.title("Heading Error")
plt.show()

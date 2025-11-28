#%%
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

#%%
print("Program Started")

# inisialisasi koneksi ke CoppeliaSim
client = RemoteAPIClient()
sim = client.require('sim')

# simulasi dibuat free-running (tidak step-by-step)
sim.setStepping(False)
sim.startSimulation()

# ambil handle objek yang diperlukan
wR_Handle   = sim.getObject("/rightMotor")
wL_Handle   = sim.getObject("/leftMotor")
s3_Handle   = sim.getObject("/ultrasonicSensor[3]")
p3dx_Handle = sim.getObject("/PioneerP3DX")
disc_Handle = sim.getObject("/Disc")
path_Handle = sim.getObject("/Path")
LH_Handle   = sim.getObject("/LH")
perp_Handle = sim.getObject("/Perp")

# --------------------------------------------------------
# Fungsi pembentuk matriks transformasi homogen 4x4
# --------------------------------------------------------
def transformMat(alpha, beta, gamma, tx, ty, tz):
    rot_x = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
    ])
    rot_y = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
    ])
    rot_z = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0, 0, 1]
    ])

    rot_total = rot_x @ rot_y
    rot_total = rot_total @ rot_z

    trans_vec = np.array([[tx], [ty], [tz]])

    R_t_3x4 = np.hstack((rot_total, trans_vec))
    last_row = np.array([[0, 0, 0, 1]])
    T_4x4 = np.vstack((R_t_3x4, last_row))
    return T_4x4

# --------------------------------------------------------
# Membaca data path dari custom data 'PATH' pada objek /Path
# --------------------------------------------------------
pathData_raw = sim.readCustomDataBlock(path_Handle, 'PATH')
if pathData_raw is None:
    raise Exception("Custom data 'PATH' tidak ditemukan di objek /Path. Periksa scene di CoppeliaSim.")

pathData = sim.unpackDoubleTable(pathData_raw)
np_path = np.array(pathData).reshape(-1, 7)  # data = [x,y,z,q1,q2,q3,q4], yang dipakai x,y (kolom 0,1)

# --------------------------------------------------------
# Variabel untuk logging
# --------------------------------------------------------
d_xyyaw      = []
d_t          = []
dat_disc2rob = np.zeros((4, 1))
dat_errors   = np.zeros((3, 1))

sim.addLog(1, "get vel start")
time.sleep(2)  # jeda sebentar sebelum kontrol dijalankan

# Parameter fisik dan kontrol
rw  = 0.195 / 2.0  # jari-jari roda (m)
rb  = 0.381 / 2.0  # half wheelbase (jarak sumbu roda ke pusat) (m)
d   = 0.05         # konstanta untuk fungsi mode
L_H = 0.5          # jarak look-ahead (m) -> dapat dituning

kp_lin = 1.2

# --------------------------------------------------------
# LOOP UTAMA – gunakan waktu simulasi internal, berhenti saat t_sim > 40 s
# --------------------------------------------------------
while True:
    # keluar bila simulasi dihentikan dari GUI
    sim_state = sim.getSimulationState()
    if sim_state == sim.simulation_stopped:
        print("Simulasi dihentikan dari Coppelia, keluar loop.")
        break

    # baca waktu simulasi dari CoppeliaSim (bukan time.time())
    t_now = sim.getSimulationTime()
    if t_now > 40.0:
        print("Batas waktu simulasi 40 detik tercapai, keluar loop.")
        break

    # posisi dan orientasi body robot pada world frame
    bod_pos_xyz = sim.getObjectPosition(p3dx_Handle)     # [x, y, z]
    bod_pos_abg = sim.getObjectOrientation(p3dx_Handle)  # [alpha, beta, gamma]
    yaw = bod_pos_abg[2]

    # ------------------------------------------------
    # Titik look-ahead pada world frame
    # ------------------------------------------------
    look_ahead_pt = np.array([
        bod_pos_xyz[0] + L_H * math.cos(yaw),
        bod_pos_xyz[1] + L_H * math.sin(yaw)
    ])

    sim.setObjectPosition(
        LH_Handle,
        [look_ahead_pt[0], look_ahead_pt[1], 0.14]
    )

    # ------------------------------------------------
    # Cari titik path terdekat terhadap look-ahead point
    # ------------------------------------------------
    np_path_xy = np_path[:, 0:2] - look_ahead_pt   # (N,2)
    path_dist = np.linalg.norm(np_path_xy, axis=1) # jarak euclidean
    pendic_idx = int(np.argmin(path_dist))

    if pendic_idx > len(path_dist) - 1:
        pendic_idx = 0

    # titik target pada path (world frame)
    target_x = np_path[pendic_idx, 0]
    target_y = np_path[pendic_idx, 1]

    sim.setObjectPosition(
        perp_Handle,
        [target_x, target_y, 0.14]
    )

    # ------------------------------------------------
    # Proyeksi titik path ke body frame robot
    # ------------------------------------------------
    path_pos_xyz_hom = np.array([
        [target_x],
        [target_y],
        [0.0],
        [1.0]
    ])

    T_body_world = transformMat(
        0, 0,
        bod_pos_abg[2],
        bod_pos_xyz[0],
        bod_pos_xyz[1],
        0.0
    )

    path2body_pos = np.linalg.inv(T_body_world) @ path_pos_xyz_hom
    # path2body_pos = [x_b; y_b; z_b; 1]

    # ------------------------------------------------
    # Perhitungan error
    # x_b = error jarak sepanjang sumbu-x body,
    # y_b dipakai untuk error heading
    # ------------------------------------------------
    ed = path2body_pos[0]   # error jarak maju (body x)
    eh = math.atan2(path2body_pos[1], path2body_pos[0])  # error heading

    disc_pos_abg = sim.getObjectOrientation(disc_Handle)
    eo = disc_pos_abg[2] - bod_pos_abg[2]                # selisih orientasi disc terhadap body

    abs_d = math.sqrt(path2body_pos[0]**2 + path2body_pos[1]**2)

    errors = np.vstack((ed, eh, eo))

    # ------------------------------------------------
    # Gain switching berbasis mode
    # ------------------------------------------------
    mode = math.exp(-float(abs_d) / d)  # bernilai 0..1

    kp_ang = 0.8 * (1.0 - mode)
    kp_ori = 8.0 * mode

    # mapping kinematik (v, w) ke (v_R, v_L)
    kin_mat = np.array([
        [1.0,  rb],
        [1.0, -rb]
    ])

    # vektor kecepatan body [v_lin; w]
    v_lin = float(ed) * kp_lin
    w     = float(eh) * kp_ang + float(eo) * kp_ori

    vel_vec = np.vstack((v_lin, w))

    v_rl = kin_mat @ vel_vec  # [v_R; v_L]

    # ------------------------------------------------
    # Konversi ke kecepatan sudut roda dan kirim ke aktuator
    # ------------------------------------------------
    phi_R = v_rl[0][0] / rw
    phi_L = v_rl[1][0] / rw

    sim.setJointTargetVelocity(wR_Handle, phi_R)
    sim.setJointTargetVelocity(wL_Handle, phi_L)

    # ------------------------------------------------
    # Logging data untuk analisis
    # ------------------------------------------------
    dat_disc2rob = np.hstack((dat_disc2rob, path2body_pos))
    dat_errors   = np.hstack((dat_errors, errors))

    d_xyyaw.append([
        bod_pos_xyz[0],
        bod_pos_xyz[1],
        bod_pos_abg[2]
    ])
    d_t.append(t_now)

    sim.addLog(
        1,
        f"ed, eh, abs_d, t_sim="
        f"{float(ed):.2f}m, "
        f"{math.degrees(float(eh)):.2f}deg, "
        f"{float(abs_d):.2f}m, "
        f"{t_now:.2f}s"
    )

    # sedikit delay agar tidak membebani CPU
    time.sleep(0.01)

# --------------------------------------------------------
# Hentikan gerakan robot & stop simulasi
# --------------------------------------------------------
sim.setJointTargetVelocity(wR_Handle, 0)
sim.setJointTargetVelocity(wL_Handle, 0)
time.sleep(0.1)
sim.addLog(1, "sim com ended")
sim.stopSimulation()

# konversi list ke array numpy
dat_xyyaw    = np.array(d_xyyaw)
dat_t        = np.array(d_t)
dat_disc2rob = dat_disc2rob[:, 1:]  # buang kolom nol awal
dat_errors   = dat_errors[:, 1:]

# normalisasi yaw ke rentang [-pi, pi]
dat_xyyaw[:, 2] = np.atan2(np.sin(dat_xyyaw[:, 2]), np.cos(dat_xyyaw[:, 2]))

# %%
# Plot lintasan robot pada bidang X-Y
plt.figure(figsize=(8, 6))
plt.plot(dat_xyyaw[:, 0], dat_xyyaw[:, 1],
         linewidth=2, label='$^wB$')
plt.scatter(dat_xyyaw[0, 0], dat_xyyaw[0, 1],
            marker='o', s=100, label='Start')
plt.scatter(dat_xyyaw[-1, 0], dat_xyyaw[-1, 1],
            marker='x', s=100, label='End')

plt.xlabel('$x_w$ (m)', fontsize=12)
plt.ylabel('$y_w$ (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

now = datetime.now()
filename = now.strftime("%y%m%d%H%M_xy_track") + ".svg"
plt.savefig(filename, format='svg')
print(f"Plot saved successfully as '{filename}'")

# Plot error jarak e_d(t)
plt.figure(figsize=(8, 6))
plt.plot(dat_t, dat_errors[0].T,
         linewidth=2, label='$e_d$')
plt.xlabel('$t$ (sec)', fontsize=12)
plt.ylabel('$e_d$ (m)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()

now = datetime.now()
filename = now.strftime("%y%m%d%H%M_ed_track") + ".svg"
print(f"Plot saved successfully as '{filename}'")

# Plot error heading e_h(t)
plt.figure(figsize=(8, 6))
plt.plot(dat_t, np.rad2deg(dat_errors[1]).T,
         linewidth=2, label='$e_h$')
plt.xlabel('$t$ (sec)', fontsize=12)
plt.ylabel('$e_h$ (deg)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()

now = datetime.now()
filename = now.strftime("%y%m%d%H%M_eh_track") + ".svg"
print(f"Plot saved successfully as '{filename}'")
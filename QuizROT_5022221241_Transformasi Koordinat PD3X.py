from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time, math, csv, os
import numpy as np
import matplotlib.pyplot as plt

# ====== PARAMETER ROBOT (SESUAIKAN) ======
wheel_radius = 0.05   # meter (rw)
L = 0.20              # meter (half-track; jarak pusat ke roda)

# ====== PARAMETER LOGGING ======
T_total = 30.0        # detik perekaman
dt = 0.05             # detik (~20 Hz)
csv_path = "p3dx_odometry_log.csv"

print("Program Started")

# --- koneksi & mulai simulasi ---
client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(False)       # biar real-time terus
sim.startSimulation()

# --- handle joint roda (sesuaikan path di scene) ---
w_right = sim.getObject("/rightMotor")
w_left  = sim.getObject("/leftMotor")

# --- (opsional) baca pose awal robot ---
theta0 = 0.0
x0, y0 = 0.0, 0.0
try:
    body = sim.getObject("/Pioneer_p3dx")      # ganti kalau beda
    p = sim.getObjectPosition(body, sim.handle_world)
    o = sim.getObjectOrientation(body, sim.handle_world) # [roll, pitch, yaw]
    x0, y0 = p[0], p[1]
    theta0 = o[2]
except Exception:
    pass  # jika gagal, mulai dari (0,0,0)

# --- buffer data ---
t_log, x_log, y_log, yaw_log = [], [], [], []
vr_log, vl_log, vx_log, w_log = [], [], [], []

# --- state awal ---
t = 0.0
x, y, theta = x0, y0, theta0

sim.addLog(sim.verbosity_scriptinfos, "Odometry logging start")
time.sleep(0.2)  # beri waktu sim mulai

t_start = time.time()
while t < T_total:
    loop_start = time.time()

    # 1) baca kecepatan sudut joint (rad/s)
    wr = sim.getJointVelocity(w_right)
    wl = sim.getJointVelocity(w_left)

    # 2) konversi ke kecepatan linier roda (m/s)
    vr = wr * wheel_radius
    vl = wl * wheel_radius

    # 3) kecepatan badan (body frame)
    vx    = 0.5 * (vr + vl)      # m/s
    omega = (vr - vl) / L        # rad/s  (ingat: L = half-track)

    # 4) update pose di world frame (Euler integrator)
    x     += vx * math.cos(theta) * dt
    y     += vx * math.sin(theta) * dt
    theta += omega * dt

    # 5) simpan log
    t_log.append(t)
    x_log.append(x)
    y_log.append(y)
    yaw_log.append(math.degrees(theta))
    vr_log.append(vr)
    vl_log.append(vl)
    vx_log.append(vx)
    w_log.append(omega)

    # 6) status ringkas tiap ~0.5 s
    if int(t*2) != int((t-dt)*2):
        sim.addLog(sim.verbosity_scriptinfos,
                   f"t={t:4.1f}s  vx={vx:5.2f} m/s  w={omega:5.2f} rad/s  pose=({x:5.2f},{y:5.2f},{math.degrees(theta):6.1f} deg)")

    # loop-rate control
    t = time.time() - t_start
    to_sleep = dt - (time.time() - loop_start)
    if to_sleep > 0:
        time.sleep(to_sleep)

sim.addLog(sim.verbosity_scriptinfos, "Odometry logging end")

# ====== simpan CSV ======
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_s","x_m","y_m","yaw_deg","vr_mps","vl_mps","vx_mps","omega_rps"])
    for i in range(len(t_log)):
        w.writerow([t_log[i], x_log[i], y_log[i], yaw_log[i],
                    vr_log[i], vl_log[i], vx_log[i], w_log[i]])

print(f"CSV saved: {os.path.abspath(csv_path)}")

# ====== EKSPLORASI: ROTASI 90° (sesuai assignment) ======
X = np.array(x_log)
Y = np.array(y_log)

# matriks rotasi 90 derajat CCW
Rz90 = np.array([[0.0, -1.0],
                [1.0,  0.0]])

# rotasi sekitar ORIGIN:
tx, ty = 2.0, 3.0
Xt_temp = X + tx
Yt_temp = Y + ty

XY   = np.vstack((X, Y))              # shape (2, N)
XY_temp = np.vstack((Xt_temp, Yt_temp))

XY_r = Rz90 @ XY
Xr, Yr = XY_r[0, :], XY_r[1, :]
Xt = Xr + tx
Yt = Yr + ty
XY_final = Rz90 @ XY_temp
X_final, Y_final = XY_final[0, :], XY_final[1, :]




# --- jika ingin rotasi sekitar titik awal/centroid, aktifkan blok ini ---
# c = np.array([[X[0]], [Y[0]]])      # sekitar titik awal
# XY_r = Rz90 @ (XY - c) + c
# Xr, Yr = XY_r[0, :], XY_r[1, :]

#eks 1
plt.figure()
plt.plot(x_log, y_log, label="asli")
plt.plot(Xr, Yr, '--', label="rotasi 90° (CCW)")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("P3DX: Trajectory (asli vs rotasi 90°)")
plt.axis("equal")
plt.grid(True)
plt.legend()
#eks 2
plt.figure()
plt.plot(x_log, y_log, label="asli")
plt.plot(Xr, Yr, '--', label="rotasi 90° (CCW)")
plt.plot(Xt, Yt, ':', label="rotasi + translasi (tx=2, ty=3)")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("P3DX: Trajectory (asli, rotasi, translasi)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()

#eks 3
plt.figure()
plt.plot(x_log, y_log, label="asli")
plt.plot(Xr, Yr, '--', label="rotasi 90° (CCW)")
plt.plot(Xt, Yt, ':', label="rotasi lalu translasi (Exp 2)")
plt.plot(X_final, Y_final, '-.', label="translasi lalu rotasi (Exp 3)")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("P3DX: Trajectory (asli, exp2, exp3)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()




# (opsional) hentikan simulasi
# sim.stopSimulation()

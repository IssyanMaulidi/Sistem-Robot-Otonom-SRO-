from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time, math, csv, os
import matplotlib.pyplot as plt

# ====== PARAMETER ROBOT (SESUAIKAN) ======
wheel_radius = 0.05   # meter (rw)
L = 0.20              # meter (jarak roda ke pusat; half-track)

# ====== PARAMETER LOGGING ======
T_total = 20.0        # detik perekaman
dt = 0.05             # detik (~20 Hz)
csv_path = "p3dx_odometry_log.csv"

print("Program Started")

# --- koneksi & mulai simulasi ---
client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(False)
sim.startSimulation()

# --- handle joint roda (pastikan path sesuai di scene) ---
w_right = sim.getObject("/rightMotor")
w_left  = sim.getObject("/leftMotor")

# --- (opsional) handle body untuk baca pose awal; kalau tak ada, mulai dari (0,0,theta0) ---
theta0 = 0.0
x0, y0 = 0.0, 0.0
try:
    body = sim.getObject("/Pioneer_p3dx")  # ganti kalau nama beda
    p = sim.getObjectPosition(body, sim.handle_world)
    o = sim.getObjectOrientation(body, sim.handle_world) # [roll, pitch, yaw]
    x0, y0 = p[0], p[1]
    theta0 = o[2]
except Exception:
    pass  # tidak fatal; integrasi tetap jalan dari (0,0,0)

# --- buffer data ---
t_log, x_log, y_log, yaw_log = [], [], [], []
vr_log, vl_log, vx_log, w_log = [], [], [], []

# --- state awal ---
t = 0.0
x, y, theta = x0, y0, theta0

sim.addLog(sim.verbosity_scriptinfos, "Odometry logging start")
time.sleep(0.2)  # beri waktu sim jalan

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
    vx = 0.5 * (vr + vl)       # m/s
    omega = (vr - vl) / L      # rad/s

    # 4) kinematic update di world frame
    x  += vx * math.cos(theta) * dt
    y  += vx * math.sin(theta) * dt
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

    # 6) tampilkan ringkas di status bar tiap ~0.5 s
    if int(t*2) != int((t-dt)*2):
        sim.addLog(sim.verbosity_scriptinfos,
                   f"t={t:4.1f}s  vx={vx:5.2f} m/s  w={omega:5.2f} rad/s  pose=({x:5.2f},{y:5.2f},{math.degrees(theta):6.1f} deg)")

    # kontrol loop-rate
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

# ====== plotting ======
# 1) x vs t
plt.figure()
plt.plot(t_log, x_log)
plt.xlabel("t (s)")
plt.ylabel("x (m)")
plt.title("P3DX: x(t)")
plt.grid(True)

# 2) y vs t
plt.figure()
plt.plot(t_log, y_log)
plt.xlabel("t (s)")
plt.ylabel("y (m)")
plt.title("P3DX: y(t)")
plt.grid(True)

# 3) yaw vs t (deg)
plt.figure()
plt.plot(t_log, yaw_log)
plt.xlabel("t (s)")
plt.ylabel("yaw (deg)")
plt.title("P3DX: yaw(t)")
plt.grid(True)

# 4) lintasan x-y
plt.figure()
plt.plot(x_log, y_log)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("P3DX: Trajectory x–y")
plt.axis("equal")
plt.grid(True)

plt.show()

# Tidak otomatis menghentikan simulasi agar bisa lanjut inspeksi.
# sim.stopSimulation()
# billiards_terminal.py
# CoppeliaSim + ZMQ Remote API (Python)
# Fitur:
# - Input gaya & torsi (global) dari terminal
# - Pilih bola mana yang dipukul
# - Durasi dorongan (impulse) dapat diatur
# - Cue otomatis diposisikan di belakang bola sesuai arah gaya
# - Reset posisi awal
# - Pesan statusbar agar mudah dilihat di CoppeliaSim

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time, math, sys

# -------------------- Util --------------------
def norm3(v, eps=1e-9):
    n = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    return [v[0]/n, v[1]/n, v[2]/n] if n > eps else [1.0, 0.0, 0.0]

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def place_cue_near_ball(sim, cue_h, ball_h, F_dir, back_dist=0.20, lift=0.00):
    """
    Tempatkan cue di belakang bola (di bidang XY) mengikuti arah F_dir.
    back_dist: jarak mundur dari pusat bola (meter)
    lift: offset ketinggian cue (meter)
    """
    p = sim.getObjectPosition(ball_h, sim.handle_world)   # posisi bola
    d = norm3(F_dir)                                      # arah unit
    cue_z = sim.getObjectPosition(cue_h, sim.handle_world)[2] + lift
    cue_pos = [p[0] - d[0]*back_dist, p[1] - d[1]*back_dist, cue_z]
    yaw = math.atan2(d[1], d[0])                          # orientasi yaw
    sim.setObjectPosition(cue_h, sim.handle_world, cue_pos)
    sim.setObjectOrientation(cue_h, sim.handle_world, [0.0, 0.0, yaw])

def apply_force_for_duration(sim, shape_h, F, T, dur, hz=100):
    """
    Beri gaya & torsi kontinu selama dur detik (global frame).
    Per 1/hz detik gaya dipanggil ulang agar efek impulse terasa mulus.
    """
    dur = max(0.0, float(dur))
    if dur == 0.0:
        return
    dt = 1.0/float(hz)
    t0 = time.time()
    while time.time() - t0 < dur:
        sim.addForceAndTorque(shape_h, F, T)   # global
        time.sleep(dt)

# -------------------- Koneksi & Scene Handles --------------------
print("Program Started")

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(False)   # biarkan CoppeliaSim berjalan bebas
sim.startSimulation()

# Ambil handle: cue ball, target balls, cue stick
# Ubah indeks sesuai scene-mu bila berbeda.
b_handles = []
for i in range(0, 7):  # Sphere[0]..Sphere[6]
    name = f"/Sphere[{i}]"
    try:
        h = sim.getObject(name)
        b_handles.append(h)
    except Exception:
        break

if len(b_handles) == 0:
    print("Tidak menemukan bola /Sphere[i]. Pastikan nama objek sesuai di scene.")
    sys.exit(1)

cue = sim.getObject("/Cylinder")   # tongkat 

# Simpan posisi awal (untuk reset)
initial_positions = [sim.getObjectPosition(h, sim.handle_world) for h in b_handles]
initial_orientations = [sim.getObjectOrientation(h, sim.handle_world) for h in b_handles]
cue_initial_pos = sim.getObjectPosition(cue, sim.handle_world)
cue_initial_ori = sim.getObjectOrientation(cue, sim.handle_world)

print(f"Terhubung. Bola yang ditemukan: {len(b_handles)} (Sphere[0..{len(b_handles)-1}])")
print("Perintah: ketik 'help' untuk melihat daftar perintah.\n")

selected_idx = 0  # default: Sphere[0] (cue ball putih)

# -------------------- Bantuan --------------------
HELP = """
Daftar perintah (ketik tanpa tanda kutip):

help
    Tampilkan bantuan ini.

select <i>
    Pilih bola yang akan dipukul, contoh: select 0  (untuk /Sphere[0]).

pose
    Tampilkan posisi (x,y,z) bola terpilih & cue (world).

aim <fx> <fy> <fz> [back] [lift]
    Atur arah cue berdasarkan vektor gaya yang akan kamu pakai.
    back (m, default 0.20), lift (m, default 0.00)
    Contoh: aim 5 0 0  0.25  0.00

hit <fx> <fy> <fz> <dur> [tx] [ty] [tz]
    Beri gaya (N) & durasi (s) ke bola terpilih.
    Torsi opsional (N·m). Semua di global frame.
    Contoh translasi murni: hit 5 0 0 0.2
    Contoh dengan spin:     hit 5 0 0 0.2  0 0 1

park [back] [lift]
    Parkirkan cue di belakang bola terpilih mengikuti arah gaya TERAKHIR yang kamu 'aim' atau 'hit'.
    Default back=0.25 lift=0.0 jika tidak disebut.

reset
    Kembalikan semua bola & cue ke pose awal.

list
    Tampilkan indeks semua bola yang tersedia.

quit
    Keluar dari program (simulasi tetap jalan; stop manual bila diinginkan).
"""

# state kecil untuk menyimpan arah gaya terakhir (buat park/aim)
last_Fdir = [1.0, 0.0, 0.0]

# -------------------- Loop Terminal --------------------
try:
    while True:
        try:
            cmd = input(">> ").strip()
        except EOFError:
            break
        if not cmd:
            continue
        parts = cmd.split()
        op = parts[0].lower()

        # --- HELP
        if op == "help":
            print(HELP)

        # --- SELECT
        elif op == "select":
            if len(parts) < 2:
                print("format: select <i>")
                continue
            i = int(parts[1])
            if i < 0 or i >= len(b_handles):
                print(f"Indeks out of range. 0..{len(b_handles)-1}")
                continue
            selected_idx = i
            print(f"Bola terpilih: Sphere[{selected_idx}]")

        # --- POSE
        elif op == "pose":
            bh = b_handles[selected_idx]
            p_b = sim.getObjectPosition(bh, sim.handle_world)
            p_c = sim.getObjectPosition(cue, sim.handle_world)
            print(f"Sphere[{selected_idx}] (x,y,z) = ({p_b[0]:.3f}, {p_b[1]:.3f}, {p_b[2]:.3f})")
            print(f"Cue              (x,y,z) = ({p_c[0]:.3f}, {p_c[1]:.3f}, {p_c[2]:.3f})")

        # --- LIST
        elif op == "list":
            for i in range(len(b_handles)):
                p = sim.getObjectPosition(b_handles[i], sim.handle_world)
                tag = "<--" if i == selected_idx else ""
                print(f"[{i:02d}] Sphere[{i}] at ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) {tag}")

        # --- AIM
        elif op == "aim":
            if len(parts) < 4:
                print("format: aim <fx> <fy> <fz> [back] [lift]")
                continue
            fx, fy, fz = float(parts[1]), float(parts[2]), float(parts[3])
            back = float(parts[4]) if len(parts) >= 5 else 0.20
            lift = float(parts[5]) if len(parts) >= 6 else 0.00
            Fdir = [fx, fy, fz]
            last_Fdir = Fdir[:]
            place_cue_near_ball(sim, cue, b_handles[selected_idx], Fdir, back_dist=back, lift=lift)
            print(f"Cue diarahkan. back={back:.3f} m, lift={lift:.3f} m.")

        # --- HIT
        elif op == "hit":
            if len(parts) < 5:
                print("format: hit <fx> <fy> <fz> <dur> [tx] [ty] [tz]")
                continue
            fx, fy, fz = float(parts[1]), float(parts[2]), float(parts[3])
            dur = float(parts[4])
            if len(parts) >= 8:
                tx, ty, tz = float(parts[5]), float(parts[6]), float(parts[7])
            else:
                tx, ty, tz = 0.0, 0.0, 0.0

            F = [fx, fy, fz]
            T = [tx, ty, tz]
            last_Fdir = F[:]

            # posisikan cue sebelum memukul (opsional)
            place_cue_near_ball(sim, cue, b_handles[selected_idx], F, back_dist=0.20, lift=0.00)

            # apply
            apply_force_for_duration(sim, b_handles[selected_idx], F, T, dur, hz=100)
            sim.addStatusbarMessage(f"Hit Sphere[{selected_idx}] with F={F}, T={T}, dur={dur}s")
            print(f"OK: Hit Sphere[{selected_idx}]  F={F}, T={T}, dur={dur}s")

        # --- PARK
        elif op == "park":
            back = float(parts[1]) if len(parts) >= 2 else 0.25
            lift = float(parts[2]) if len(parts) >= 3 else 0.00
            place_cue_near_ball(sim, cue, b_handles[selected_idx], last_Fdir, back_dist=back, lift=lift)
            print(f"Cue diparkir di belakang Sphere[{selected_idx}] (back={back}, lift={lift}).")

        # --- RESET
        elif op == "reset":
            # kembalikan bola-bola
            for i,h in enumerate(b_handles):
                sim.setObjectPosition(h, sim.handle_world, initial_positions[i])
                sim.setObjectOrientation(h, sim.handle_world, initial_orientations[i])
                # bersihkan kecepatan (opsional)
                try:
                    sim.setObjectVelocity(h, [0,0,0], [0,0,0])
                except Exception:
                    pass
            # cue
            sim.setObjectPosition(cue, sim.handle_world, cue_initial_pos)
            sim.setObjectOrientation(cue, sim.handle_world, cue_initial_ori)
            try:
                sim.setObjectVelocity(cue, [0,0,0], [0,0,0])
            except Exception:
                pass
            print("Reset selesai.")

        # --- QUIT
        elif op in ("quit","exit","q"):
            print("Keluar. (Stop simulasi dari CoppeliaSim bila diperlukan.)")
            break

        else:
            print("Perintah tidak dikenali. Ketik 'help'.")

except KeyboardInterrupt:
    pass

# Tidak otomatis stopSimulation agar kamu bisa inspeksi hasilnya.
# sim.stopSimulation()

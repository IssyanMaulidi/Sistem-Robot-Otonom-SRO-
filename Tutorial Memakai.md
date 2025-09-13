# Sistem-Robot-Otonom-SRO-
Repository ini dibuat untuk menyimpan dan mendokumentasikan kode Python yang dikembangkan selama proses pembelajaran bersama Bapak M.Q. Zaman, meliputi tugas, dan eksperimen

Tugas 3[][]|
           |(<0_0<): Tutorial memainkan Billiard paad Tugas 3_Issyan Maulidi_Billiard:
-Sebelum bermain Billiard di Copelia.sim via Python, pastikan anda telah melakukan: Open file location COPELIA ->  Programming -> Legacy remote API -> legacyRemote API -> remoteAPIBindings -> python -> Copy Patse ALL ke file yang diinginkan.
- Klik strat pada Copelia.sim yang telah berisi: Meja billiard, 6 buah bola (5 bola warna warni dan 1 bola putih), dan 1 buah tongkat pemukul bola putih.
- Jalankan prgoram Python yang bernama "Tugas 3_Issyan Maulidi_Billiard" di perangkat anda: dapat melalui VSCode, pycharm, dan lainnya sesuai preferensi masing-masing.
- Anda sudah masuk ke permainan "Billiard" via Copelia.sim, pastikan anda siap untuk bermain dengan teliti karena: posisi dan gaya pukul berbasis vektor. 
-ketik "HELP" jika anda butuh bantuan untuk memahami sintaks yang dapat dipakai pada permainan billiard kita.
- ketik "list" untuk mengetahui posisi secara jelas  [x y z] setiap bola yang akan dipakai (bola putih adalah bola nomor 6).
- Pilihlah bola yang ingin anda gunakan untuk bermain (permainan billiard umumnya menggunakan bola putih, jika disini kita pakai bola 6) dengan mengetik "select i (i = nomor bola yang anda arahkan untuk dipukul)", kemudian "Run" program.
- Arahkan tongkat pukul (cue stick) terhadap bola yang anda pilih tadi dengan menggunakan sintaks "aim x y z Jaark_mundur_tongkat lift(seberapa tongkat terangkat terhadap bola), kemudian jalankan; Misal: aim -1 1 1 0.2 0"
- Setelah tongkat terfokus kepada bola yang tertuju, berikan gaya untuk menghantam bola tujuan dengan mengetik "hit x y z Durasi_pukul", kemudian jalankan.
; Misal: 10 0 0 0.2
- Setelahnya parkirkan tongkat pukul anda dengan mengetikkan "park Jarak_tongkat_pukul_ke_bola_target"; misal: park 0.3
- Lanjutkan permainan seperti langkah awal tadi:
  1. select i (i = 0.1,2,3,4,5,6)
  2. aim x y z distance lift
  3. hit x y z t
  4. park m 
- Apabila anda sudah menang/kalah dan ingin bermain lagi, anda bisa saja mengulangi permainan dengan mengetik "reset", maka permainan akan bermulai dari awal.
- Selamat mencobaa!😊

Tugas 3[][]|
           |(<0_0<): :

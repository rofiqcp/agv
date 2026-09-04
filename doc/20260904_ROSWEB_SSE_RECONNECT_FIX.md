# ROS Web SSE Reconnect Fix — 2026-09-04

## Masalah
GUI ADV Operator Console dapat dibuka melalui LAN/Tailscale, tetapi indikator realtime tetap atau kembali ke `Reconnecting`.
HTTP `/api/health` sehat dan server sudah bind ke `0.0.0.0:5000`.

## Diagnosis
Frontend memakai `EventSource('/api/events')` untuk realtime telemetry.
Endpoint SSE diuji dari loopback, LAN, dan Tailscale; semuanya HTTP 200 dengan `Content-Type: text/event-stream`, `Connection: keep-alive`, dan data kontinu.
Frontend lama hanya mengganti label menjadi `Reconnecting` pada `EventSource.onerror` dan tidak memiliki fallback jika long-lived SSE terganggu oleh browser/jaringan.

## Perbaikan
- SSE tetap menjadi transport realtime utama.
- Ditambahkan watchdog 1 Hz untuk mendeteksi SSE tidak OPEN atau data SSE stale >2.5 s.
- Saat SSE terganggu, GUI polling `/api/state` sebagai fallback sehingga telemetry tetap hidup.
- Status fallback ditampilkan sebagai `Polling live`, bukan macet di `Reconnecting`.
- Saat SSE kembali OPEN, UI otomatis kembali menjadi `Live stream` / `CONNECTED`.
- Asset token `app.js` dinaikkan ke `20260904-sse-fallback-v1` agar browser memuat kode baru.

## Verifikasi
- `node --check app.js`: PASS.
- SSE curl loopback/LAN/Tailscale: stream kontinu PASS.
- Chrome headless via LAN `10.241.5.107:5000`: `Live stream`, `CONNECTED`, EventSource OPEN.
- Chrome headless via Tailscale `100.81.176.67:5000`: `Live stream`, `CONNECTED`, EventSource OPEN.

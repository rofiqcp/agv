# AGV GUI

Dokumentasi penggunaan GUI terdapat di `navigation/gui/README.md` pada source workspace.

## GNSS Driver V2 Part 1 telemetry

The GNSS page also consumes the following driver-level topics:

- `/gnss/vel` — receiver Doppler velocity in ENU (`velE`, `velN`, `-velD`).
- `/gnss/velocity_position_fit` — weighted multi-point position-derived velocity validator; diagnostic only in Part 1.
- `/gnss/state` — receiver protocol/rate/iTOW/timestamp/covariance counters and quality reason.
- `/gnss/motion_diagnostics` — position-fit baseline/RMSE and fit-vs-Doppler/course residuals.

`/gnss/quality` remains append-only. Fields 0..8 retain the previous contract used by `LocalizationCore`; Driver V2 fields are appended so older consumers remain compatible.

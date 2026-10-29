# flexpi ABC real-robot eval — 2026-10-29

YAM bimanual, `rd infer` + `YamFlexPiRaidenBridgeWS` (`use_depth=false`, ee_pose / eef_ik, 30 Hz, chunk 32).
Head-camera recordings only (1280x720 @ 30 fps). Harness configs are in [`configs/`](configs/).

Sub-second clips from runs that crashed at startup (right-wrist camera dropout) were removed.

| Task | Prompt | Episode | Frames | Video length (s) |
|---|---|---|---:|---:|
| pen | put pen into the bowl | [`20261029T134223_ep0000/head.mp4`](flexpi-rgb_pen/20261029T134223_ep0000/head.mp4) | 499 | 16.6 |
| pen | put pen into the bowl | [`20261029T135524_ep0000/head.mp4`](flexpi-rgb_pen/20261029T135524_ep0000/head.mp4) | 201 | 6.7 |
| pen | put pen into the bowl | [`20261029T135609_ep0000/head.mp4`](flexpi-rgb_pen/20261029T135609_ep0000/head.mp4) | 464 | 15.5 |
| pen | put pen into the bowl | [`20261029T135709_ep0000/head.mp4`](flexpi-rgb_pen/20261029T135709_ep0000/head.mp4) | 413 | 13.8 |
| pen | put pen into the bowl | [`20261029T135807_ep0000/head.mp4`](flexpi-rgb_pen/20261029T135807_ep0000/head.mp4) | 181 | 6.0 |
| plates | load the plates into the dish rack | [`20261029T124002_ep0000/head_h264.mp4`](flexpi-rgb_plates/20261029T124002_ep0000/head_h264.mp4) | 685 | 22.8 |
| plates | load the plates into the dish rack | [`20261029T124002_ep0000/head.mp4`](flexpi-rgb_plates/20261029T124002_ep0000/head.mp4) | 685 | 22.8 |
| plates | load the plates into the dish rack | [`20261029T124929_ep0000/head.mp4`](flexpi-rgb_plates/20261029T124929_ep0000/head.mp4) | 967 | 32.2 |
| plates | load the plates into the dish rack | [`20261029T125100_ep0000/head.mp4`](flexpi-rgb_plates/20261029T125100_ep0000/head.mp4) | 1051 | 35.0 |
| plates | load the plates into the dish rack | [`20261029T125223_ep0000/head.mp4`](flexpi-rgb_plates/20261029T125223_ep0000/head.mp4) | 2918 | 97.3 |
| plates | load the plates into the dish rack | [`20261029T130136_ep0000/head.mp4`](flexpi-rgb_plates/20261029T130136_ep0000/head.mp4) | 1241 | 41.4 |
| plates | load the plates into the dish rack | [`20261029T130334_ep0000/head.mp4`](flexpi-rgb_plates/20261029T130334_ep0000/head.mp4) | 1308 | 43.6 |
| plates | load the plates into the dish rack | [`20261029T132155_ep0000/head.mp4`](flexpi-rgb_plates/20261029T132155_ep0000/head.mp4) | 1217 | 40.6 |
| plates | load the plates into the dish rack | [`20261029T132428_ep0000/head.mp4`](flexpi-rgb_plates/20261029T132428_ep0000/head.mp4) | 512 | 17.1 |
| schoolbag | pack the student bag | [`20261029T133633_ep0000/head.mp4`](flexpi-rgb_schoolbag/20261029T133633_ep0000/head.mp4) | 606 | 20.2 |
| schoolbag | pack the student bag | [`20261029T133758_ep0000/head.mp4`](flexpi-rgb_schoolbag/20261029T133758_ep0000/head.mp4) | 489 | 16.3 |

Notes:

- `flexpi-rgb_plates/20261029T124002_ep0000` was recorded before the first batch; `head.mp4` is the original MPEG-4 recording and `head_h264.mp4` an H.264 re-encode of it.
- The right-wrist camera disconnected from USB partway through `flexpi-rgb_plates/20261029T130334_ep0000` and `flexpi-rgb_plates/20261029T132428_ep0000`, so the policy lost that view before those episodes ended.

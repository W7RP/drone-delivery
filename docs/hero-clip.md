# Phase 8: hero clip

One simulated VTOL delivery visit over real Pittsburgh terrain and buildings, rendered in
Gazebo and cut into a 60 to 90 s clip with a HUD built from the flight logs. Every vehicle, pad
and network number in it is a placeholder (spec A01); the clip says "simulated".

## 1. Render check

`px4/tools/render_check.sh <out> 60` starts a headless Gazebo server on a generated block city
(1500 boxes, shadows on), with three 1280x720 cameras at 30 Hz and the 320x240 10 Hz marker
camera, and streams the three large cameras into H.264 through `px4/tools/frame_recorder.py`.
Frames go from the camera topic straight into ffmpeg's stdin; nothing uncompressed is written.
ffmpeg is the static binary from the `imageio-ffmpeg` wheel in the px4 venv (none on the host).

| Renderer | Frames per wall second, each 720p camera | Real-time factor | Missing sim-time frames |
|---|---|---|---|
| GPU: Mesa 23.2.1 D3D12 driver on an RTX 4070 SUPER (WSL2 GPU passthrough, EGL headless) | 19.2 | 0.63 | 0 of 1146 |
| Software (`LIBGL_ALWAYS_SOFTWARE=1`, llvmpipe) | 9.8 | 0.32 | 0 of 585 |

60 wall seconds each, ogre2. The sensors system holds the sim step until a due frame is
rendered, so the real-time factor drops instead of frames being skipped: frame stamps step by
32 or 36 ms (the 4 ms physics step), 30.3 Hz on average. The edit samples an exact 30 Hz
sim-time grid by taking the latest frame at or before each tick.

With the GPU a 250 s visit takes about 400 s of wall time with three cameras. Rendering is not a
blocker, and the GPU path is used.

## 2. Lane

`pipeline/run.sh hero lane` applies the rule and writes `pipeline/hero/out/lane.json`.

- **Candidates:** lanes of the VTOL demo network (`parts_logistics_pittsburgh_vtol`) with at
  least one order delivered by drone in `parts_logistics_pittsburgh_vtol_demo` at seed 42. Each
  must be no longer than the network's 5 kg range (14 689.8 m) and have a straight line that
  intersects a water polygon from the cached OSM extract.
- **Order:** highest road/straight-line ratio, then the shortest lane, then the lowest lane ID.
  The ratio is truck road metres over great-circle metres, as in `ddpipe.truck.detour_ratios`.

| Lane | From | To | Drone-delivered | Length (m) | Crosses water | Road / straight |
|---|---|---|---|---|---|---|
| 1 | Consumer_Fresh_Produce | UPMC_Oakland | 137 | 2408.4 | no | 1.761 |
| 2 | Consumer_Fresh_Produce | UPMC_Magee_Womens_Hospital | 132 | 2719.5 | no | 1.902 |
| 3 | Consumer_Fresh_Produce | VA_Pittsburgh_Healthcare_System | 131 | 2093.4 | no | 2.132 |
| 4 | Consumer_Fresh_Produce | Mon_Valley_Works_Edgar_Thomson_P | 4 | 12895.6 | yes | 1.344 |
| **5** | **Consumer_Fresh_Produce** | **Josh_Steel** | **21** | **11138.8** | **yes** | **1.429** |
| 7 | Achieva_Mailing_Service | UPMC_Oakland | 133 | 2141.5 | no | 1.567 |
| 8 | Achieva_Mailing_Service | UPMC_Magee_Womens_Hospital | 128 | 2512.8 | no | 1.570 |
| 9 | Achieva_Mailing_Service | VA_Pittsburgh_Healthcare_System | 153 | 1715.3 | no | 2.087 |
| 10 | Achieva_Mailing_Service | Mon_Valley_Works_Edgar_Thomson_P | 5 | 12545.8 | yes | 1.292 |
| 11 | Achieva_Mailing_Service | Josh_Steel | 16 | 10794.8 | yes | 1.370 |

Every candidate is within range. Four of them cross water, all of them crossing the Monongahela
into the Mon Valley.

The chosen lane is **lane 5**, from the Strip District hub (Consumer Fresh Produce) to Josh Steel:
11.1 km with a road/straight ratio of 1.43. That is about 740 s of cruise at 15 m/s, three times
the 2 km lanes of Phases 4 and 7. The clip compresses that cruise.

Lane IDs are those of the VTOL network, which keeps every pad, so they match the placeholder
catalog that `px4/tools/select_lane.py` numbers from.

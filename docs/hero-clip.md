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

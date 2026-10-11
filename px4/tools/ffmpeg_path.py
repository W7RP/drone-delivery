"""Print the path of the ffmpeg binary that the imageio-ffmpeg wheel ships (no system ffmpeg)."""

import imageio_ffmpeg

print(imageio_ffmpeg.get_ffmpeg_exe())

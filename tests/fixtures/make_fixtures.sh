#!/bin/sh
# Regenerates the MP3 test files (FFmpeg with libmp3lame). The files are
# committed, so this is only needed to change them.
set -e
cd "$(dirname "$0")"
Q="-hide_banner -loglevel error -y -fflags +bitexact"

# Constant bitrate without a Xing header: the length is first estimated, then
# measured by the background scan
ffmpeg $Q -f lavfi -i "sine=frequency=1000:sample_rate=44100:duration=3" -ac 2 \
  -c:a libmp3lame -b:a 64k -write_xing 0 -id3v2_version 0 -map_metadata -1 cbr_64k_stereo_3s.mp3

# Variable bitrate, mono, with a Xing header (exact length)
ffmpeg $Q -f lavfi -i "sine=frequency=440:sample_rate=44100:duration=2" -ac 1 \
  -c:a libmp3lame -q:a 6 -id3v2_version 0 -map_metadata -1 vbr_mono_2s.mp3

# MPEG-2 Layer III (22.05 kHz), mono
ffmpeg $Q -f lavfi -i "sine=frequency=500:sample_rate=22050:duration=2" -ac 1 \
  -c:a libmp3lame -b:a 32k -id3v2_version 0 -map_metadata -1 mpeg2_22k_mono_2s.mp3

# ID3v2.3 title, artist and an embedded baseline JPEG cover (48x32)
ffmpeg $Q -f lavfi -i "color=c=0x3366cc:s=48x32" -frames:v 1 cover.jpg
ffmpeg $Q -f lavfi -i "sine=frequency=1000:sample_rate=44100:duration=1" -i cover.jpg \
  -map 0:a -map 1:v -ac 2 -c:a libmp3lame -b:a 64k -c:v copy -id3v2_version 3 \
  -metadata title="Test Title" -metadata artist="Test Artist" \
  -metadata:s:v comment="Cover (front)" -disposition:v attached_pic tagged_cover.mp3
rm cover.jpg

# AVI: MJPEG video (96x64) with 16-bit PCM audio, with MP3 audio, and without audio
ffmpeg $Q -f lavfi -i "testsrc=size=96x64:rate=10:duration=2" -f lavfi -i "sine=frequency=440:sample_rate=8000:duration=2" \
  -map 0:v -map 1:a -c:v mjpeg -q:v 8 -pix_fmt yuvj420p -c:a pcm_s16le -ac 1 mjpeg_pcm_96x64.avi
ffmpeg $Q -f lavfi -i "testsrc=size=96x64:rate=12:duration=2" -f lavfi -i "sine=frequency=440:sample_rate=22050:duration=2" \
  -map 0:v -map 1:a -c:v mjpeg -q:v 8 -pix_fmt yuvj420p -c:a libmp3lame -b:a 32k -ac 1 mjpeg_mp3_96x64.avi
ffmpeg $Q -f lavfi -i "testsrc=size=96x64:rate=10:duration=1" -c:v mjpeg -q:v 8 -pix_fmt yuvj420p mjpeg_silent_96x64.avi

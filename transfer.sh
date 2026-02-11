#!/usr/bin/env bash
set -e

# Usage: ./transfer.sh input_audio.wav [ip] [port]

if [ $# -lt 1 ]; then
  echo "Usage: $0 <input_audio> [ip] [port]"
  exit 1
fi

INPUT="$1"
IP="${2:-192.168.0.204}"
PORT="${3:-5000}"

if [ ! -f "$INPUT" ]; then
  echo "Error: file not found: $INPUT"
  exit 1
fi

echo "Sending '$INPUT' to $IP:$PORT as 44.1kHz mono PCM16 WAV..."

EXT="${INPUT##*.}"
BASE="${INPUT%.*}"
OUT="${BASE}_44100mono.wav"
if [ "$OUT" = "$INPUT" ]; then
  OUT="${BASE}_converted.wav"
fi

# Pre-convert to an on-disk WAV so we send a stable, known-good file
ffmpeg -hide_banner -loglevel error -y \
  -i "$INPUT" \
  -ac 1 -ar 44100 \
  -c:a pcm_s16le \
  "$OUT"

# Throttle moderately to match receiver; adjust -L if needed
pv -L 200k -B 64k "$OUT" | nc -N -w 5 "$IP" "$PORT"

echo "Done."

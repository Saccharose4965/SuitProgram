#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
fft_test_dir=$(mktemp -d)
trap 'rm -rf "$fft_test_dir"' EXIT
flags=(-std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -fsanitize=address,undefined -I components/fft/include)
"${CC:-gcc}" "${flags[@]}" components/fft/tests/test_detector.c components/fft/fft_detector.c -lm -o "$fft_test_dir/detector"
"$fft_test_dir/detector"
"${CC:-gcc}" "${flags[@]}" components/fft/tests/test_phase.c components/fft/fft_phase.c -lm -o "$fft_test_dir/phase"
"$fft_test_dir/phase"
"${CC:-gcc}" "${flags[@]}" -I components/fft/tests/host_stubs components/fft/tests/test_display.c components/fft/fft_display.c -lm -o "$fft_test_dir/display"
"$fft_test_dir/display"
if [[ -f ../localFFT/LatestFFT.c ]] && pkg-config --exists alsa sdl2; then
    read -ra pc_cflags <<< "$(pkg-config --cflags alsa sdl2)"
    read -ra pc_libs <<< "$(pkg-config --libs alsa sdl2)"
    "${CC:-gcc}" "${flags[@]}" -D_DEFAULT_SOURCE "${pc_cflags[@]}" components/fft/tests/test_localfft_reference.c components/fft/fft_detector.c components/fft/fft_phase.c "${pc_libs[@]}" -lm -o "$fft_test_dir/reference"
    "$fft_test_dir/reference"
else
    echo 'SKIP actual desktop parity: requires ../localFFT/LatestFFT.c and ALSA/SDL2 development packages.'
fi

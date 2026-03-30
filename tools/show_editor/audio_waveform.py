from __future__ import annotations

from array import array
from dataclasses import dataclass
import math
from pathlib import Path
import wave

DEFAULT_WAVEFORM_PEAK_COUNT = 65536


@dataclass(frozen=True)
class AudioWaveform:
    path: Path
    sample_rate: int
    channel_count: int
    sample_width: int
    frame_count: int
    duration_ms: int
    peaks: tuple[float, ...]
    mono_samples: array


def _sample_max_abs(sample_width: int) -> float:
    if sample_width == 1:
        return 128.0
    if sample_width == 2:
        return 32768.0
    if sample_width == 3:
        return 8388608.0
    if sample_width == 4:
        return 2147483648.0
    raise ValueError(f"unsupported WAV sample width: {sample_width}")


def _decode_sample(raw: bytes, offset: int, sample_width: int) -> int:
    if sample_width == 1:
        return raw[offset] - 128
    if sample_width == 2 or sample_width == 4:
        return int.from_bytes(raw[offset:offset + sample_width], "little", signed=True)
    if sample_width == 3:
        value = raw[offset] | (raw[offset + 1] << 8) | (raw[offset + 2] << 16)
        if value & 0x800000:
            value -= 1 << 24
        return value
    raise ValueError(f"unsupported WAV sample width: {sample_width}")


def load_audio_waveform(path: str | Path, peak_count: int = DEFAULT_WAVEFORM_PEAK_COUNT) -> AudioWaveform:
    audio_path = Path(path)
    with wave.open(str(audio_path), "rb") as wav:
        channel_count = wav.getnchannels()
        sample_rate = wav.getframerate()
        sample_width = wav.getsampwidth()
        frame_count = wav.getnframes()

        if channel_count < 1:
            raise ValueError("WAV file must contain at least one channel")
        if sample_rate < 1:
            raise ValueError("WAV file must have a positive sample rate")
        if frame_count < 0:
            raise ValueError("WAV frame count must be non-negative")

        duration_ms = int(round(frame_count * 1000.0 / sample_rate))
        if frame_count == 0:
            return AudioWaveform(
                path=audio_path,
                sample_rate=sample_rate,
                channel_count=channel_count,
                sample_width=sample_width,
                frame_count=frame_count,
                duration_ms=duration_ms,
                peaks=(0.0,),
                mono_samples=array("f"),
            )

        bucket_count = max(1, min(int(peak_count), frame_count))
        bucket_size = max(1, math.ceil(frame_count / bucket_count))
        frame_stride = channel_count * sample_width
        max_abs = _sample_max_abs(sample_width)
        peaks = [0.0] * bucket_count
        mono_samples = array("f")
        frame_index = 0

        while frame_index < frame_count:
            chunk_frames = min(4096, frame_count - frame_index)
            raw = wav.readframes(chunk_frames)
            available_frames = len(raw) // frame_stride
            for local_index in range(available_frames):
                frame_offset = local_index * frame_stride
                amplitude_sum = 0.0
                sample_sum = 0.0
                for channel_index in range(channel_count):
                    sample_offset = frame_offset + channel_index * sample_width
                    sample_value = _decode_sample(raw, sample_offset, sample_width)
                    normalized = sample_value / max_abs
                    amplitude_sum += abs(normalized)
                    sample_sum += normalized
                amplitude = min(1.0, amplitude_sum / channel_count)
                mono_samples.append(max(-1.0, min(1.0, sample_sum / channel_count)))
                bucket_index = min(bucket_count - 1, (frame_index + local_index) // bucket_size)
                if amplitude > peaks[bucket_index]:
                    peaks[bucket_index] = amplitude
            frame_index += available_frames

    return AudioWaveform(
        path=audio_path,
        sample_rate=sample_rate,
        channel_count=channel_count,
        sample_width=sample_width,
        frame_count=frame_count,
        duration_ms=duration_ms,
        peaks=tuple(peaks),
        mono_samples=mono_samples,
    )

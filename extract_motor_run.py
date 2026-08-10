#!/usr/bin/env python3
"""
Extract an AmbatuDrone Teensy stereo logger run.

Input:
  RUN0001.WAV
  RUN0001.JSON (automatically loaded from the same folder)

Output:
  RUN0001_audio.wav
  RUN0001_current.csv

The Teensy stores microphone samples on WAV channel 1 and ACS72981 samples on
channel 2. This script separates the audio and converts current ADC counts to
amps using the zero offset and sensitivity saved in the JSON metadata.
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import wave
from pathlib import Path


ADC_BITS = 12
ADC_MIDPOINT = 1 << (ADC_BITS - 1)
PCM_SHIFT = 16 - ADC_BITS


def pcm_to_adc(sample: int) -> int:
    """Reconstruct the exact unsigned 12-bit ADC count stored by the logger."""
    return (sample >> PCM_SHIFT) + ADC_MIDPOINT


def load_metadata(wav_path: Path, metadata_path: Path | None) -> tuple[Path, dict]:
    resolved = metadata_path or wav_path.with_suffix(".JSON")
    if not resolved.exists():
        raise FileNotFoundError(
            f"Metadata file not found: {resolved}\n"
            "Copy the matching RUNxxxx.JSON file from the microSD card."
        )
    with resolved.open("r", encoding="utf-8") as handle:
        metadata = json.load(handle)
    return resolved, metadata


def validate_input(wav_file: wave.Wave_read, metadata: dict) -> None:
    if wav_file.getnchannels() != 2:
        raise ValueError("Expected a two-channel Teensy logger WAV file.")
    if wav_file.getsampwidth() != 2:
        raise ValueError("Expected 16-bit PCM samples.")
    if wav_file.getcomptype() != "NONE":
        raise ValueError("Expected uncompressed PCM WAV data.")

    metadata_rate = int(metadata.get("sample_rate_hz", wav_file.getframerate()))
    if wav_file.getframerate() != metadata_rate:
        raise ValueError(
            "WAV and JSON sample rates disagree: "
            f"{wav_file.getframerate()} vs {metadata_rate}"
        )


def extract_run(
    wav_path: Path,
    metadata_path: Path | None = None,
    chunk_frames: int = 8192,
) -> tuple[Path, Path]:
    _, metadata = load_metadata(wav_path, metadata_path)
    audio_path = wav_path.with_name(f"{wav_path.stem}_audio.wav")
    current_path = wav_path.with_name(f"{wav_path.stem}_current.csv")

    current_zero_adc = float(metadata["current_zero_adc"])
    counts_per_amp = float(metadata["current_counts_per_amp"])
    if counts_per_amp <= 0:
        raise ValueError("current_counts_per_amp must be greater than zero.")

    with wave.open(str(wav_path), "rb") as source:
        validate_input(source, metadata)
        sample_rate = source.getframerate()

        with wave.open(str(audio_path), "wb") as audio_output, current_path.open(
            "w", newline="", encoding="utf-8"
        ) as csv_handle:
            audio_output.setnchannels(1)
            audio_output.setsampwidth(2)
            audio_output.setframerate(sample_rate)

            writer = csv.writer(csv_handle)
            writer.writerow(
                [
                    "sample_index",
                    "time_s",
                    "current_A",
                    "current_adc",
                    "microphone_adc",
                ]
            )

            sample_index = 0
            while True:
                raw = source.readframes(chunk_frames)
                if not raw:
                    break
                if len(raw) % 4:
                    raise ValueError("WAV data ended in a partial stereo frame.")

                audio_bytes = bytearray(len(raw) // 2)
                audio_offset = 0

                for microphone_pcm, current_pcm in struct.iter_unpack("<hh", raw):
                    struct.pack_into("<h", audio_bytes, audio_offset, microphone_pcm)
                    audio_offset += 2

                    microphone_adc = pcm_to_adc(microphone_pcm)
                    current_adc = pcm_to_adc(current_pcm)
                    current_amps = (
                        current_adc - current_zero_adc
                    ) / counts_per_amp

                    writer.writerow(
                        [
                            sample_index,
                            f"{sample_index / sample_rate:.9f}",
                            f"{current_amps:.6f}",
                            current_adc,
                            microphone_adc,
                        ]
                    )
                    sample_index += 1

                audio_output.writeframesraw(audio_bytes)

    return audio_path, current_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Split an AmbatuDrone Teensy logger WAV into microphone audio "
            "and calibrated current CSV data."
        )
    )
    parser.add_argument("wav_file", type=Path, help="RUNxxxx.WAV file")
    parser.add_argument(
        "--metadata",
        type=Path,
        help="Matching JSON file; defaults to RUNxxxx.JSON",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    wav_path = args.wav_file.resolve()
    if not wav_path.exists():
        raise FileNotFoundError(f"WAV file not found: {wav_path}")

    audio_path, current_path = extract_run(wav_path, args.metadata)
    print(f"Created: {audio_path}")
    print(f"Created: {current_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

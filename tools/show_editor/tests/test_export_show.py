from __future__ import annotations

import struct
import tempfile
import unittest
from pathlib import Path
import zlib

from tools.show_editor.export_show import (
    CLIP_STRUCT,
    COLOR_ANIM_STRUCT,
    COLOR_STOP_STRUCT,
    HEADER_STRUCT,
    ROLE_STRUCT,
    SOLID_PARAM_STRUCT,
    TIMING_STRUCT,
    export_show,
)
from tools.show_editor.project_model import ShowClip, ShowProject


class ExportShowTests(unittest.TestCase):
    def project(self, color_to: int = 255) -> ShowProject:
        return ShowProject(
            slug="test-show",
            title="Test Show",
            duration_ms=4000,
            tempo_bpm=123.5,
            beat_offset_ms=-42,
            global_clips=[
                ShowClip(
                    start_ms=0,
                    end_ms=4000,
                    effect="solid",
                    target_kind="all",
                    target="all",
                    params={
                        "color": [255, 0, 0, 255],
                        "color_from": [255, 0, 0, 255],
                        "color_to": [0, 0, color_to, 255],
                        "color_mode": "smooth",
                    },
                )
            ],
        )

    def export_bytes(self, project: ShowProject) -> bytes:
        with tempfile.TemporaryDirectory() as directory:
            result = export_show(project, directory)
            return result.show_bin_path.read_bytes()

    def test_v1_1_offsets_crc_timing_and_color_suffix(self) -> None:
        payload = bytearray(self.export_bytes(self.project()))
        header = HEADER_STRUCT.unpack_from(payload)
        self.assertEqual(header[0], 0x31535753)
        self.assertEqual(header[1:3], (1, 1))
        self.assertEqual(header[3], HEADER_STRUCT.size + TIMING_STRUCT.size)
        self.assertEqual(header[4], len(payload))

        stored_crc = header[5]
        struct.pack_into("<I", payload, 16, 0)
        self.assertEqual(zlib.crc32(payload) & 0xFFFFFFFF, stored_crc)
        self.assertEqual(TIMING_STRUCT.unpack_from(payload, HEADER_STRUCT.size),
                         (123500, -42))

        role_offset = header[14]
        roles = [
            ROLE_STRUCT.unpack_from(payload, role_offset + i * ROLE_STRUCT.size)
            for i in range(header[8])
        ]
        self.assertEqual([role[0] for role in roles], [0, 1, 2])

        clip = CLIP_STRUCT.unpack_from(payload, header[18])
        param_offset = clip[12]
        param_bytes = clip[13]
        base_bytes = SOLID_PARAM_STRUCT.size
        mode, _flags, stop_count, _reserved, _rate = COLOR_ANIM_STRUCT.unpack_from(
            payload, param_offset + base_bytes
        )
        self.assertEqual(mode, 2)
        self.assertEqual(stop_count, 2)
        self.assertEqual(
            param_bytes,
            base_bytes + COLOR_ANIM_STRUCT.size + stop_count * COLOR_STOP_STRUCT.size,
        )

    def test_show_uid_changes_with_choreography(self) -> None:
        first = HEADER_STRUCT.unpack_from(self.export_bytes(self.project(255)))[6]
        second = HEADER_STRUCT.unpack_from(self.export_bytes(self.project(128)))[6]
        self.assertNotEqual(first, second)


if __name__ == "__main__":
    unittest.main()

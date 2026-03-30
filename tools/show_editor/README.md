# Show Editor Scaffold

This is the initial scaffold for the dedicated music show editor/export path.

Current scope:

- JSON source project model
- canonical section/group IDs
- compiled `show.bin` exporter
- optional audio copy to `track.wav`
- Qt-based editor window scaffold

Current limitations:

- GUI requires `PySide6`
- waveform loader currently supports WAV input only
- audio playback depends on Qt multimedia working on the local machine
- no firmware renderer yet
- exporter only packs a minimal runtime structure
- effect authoring still relies on the clip inspector and raw params JSON rather than dedicated per-effect widgets

## Example

```sh
python3 -m tools.show_editor.export_cli \
  tools/show_editor/example_project.json \
  /tmp/demo_show
```

This writes:

- `/tmp/demo_show/show.bin`

And, if `--copy-audio` is used and the source audio exists:

- `/tmp/demo_show/track.wav`

## Launch UI

```sh
python3 -m pip install -r tools/show_editor/requirements.txt
./show-editor
```

By default this resumes the last autosaved editor session. If no prior
session exists, it falls back to the bundled example project and loads
`[music.wav](/home/anotherone/Documents/suit/SuitProgram/tools/show_editor/music.wav)` automatically.

To override session resume and open a specific project:

```sh
./show-editor --project path/to/show.json
```

The editor now includes:

- integrated preview viewport plus timeline in one window
- `2D` / `3D` preview mode switching
- mode and camera selection from compact combo boxes
- `Preview` selection for `A`, `B`, `C`, or `All`
- audio transport buttons for local playback when Qt multimedia is available
- left-side effect library plus saved presets
- double-click on a timeline lane to add the selected effect at that time/layer
- inspector panel for editing selected clips, with changes committing on normal field validation instead of an explicit apply button
- section/group targeting from a checklist instead of raw target text
- quick effect controls for axis, width, softness, frequency, phase, repeats, duty cycle, reverse, and pulse intensity range
- BPM-sync toggle that locks free-rate controls when enabled
- timeline clip selection, drag-to-move, and edge-drag resize
- beat-based snap with editable `BPM` and `Beat Offset`
- beat/subdivision guide lines aligned to the defined beat
- timeline zoom with mouse wheel, `Shift` + wheel pan, and `Zoom - / + / Fit` buttons
- smart cursor with raw time, snapped time, beat readout, lane/layer info, and ghost clip placement
- clip copy/paste/duplicate via menu and shortcuts
- clip timeline lanes for `global`, `A`, `B`, and `C`, each with layered clips
- a playhead tied to the main time scrubber and transport
- click/drag seeking in the timeline
- WAV waveform loading when `audio.source` resolves to a local file
- LED-precise preview tied to the same timeline position

Useful shortcuts:

- `Space`: toggle play/pause from the current cursor position
- `Ctrl+Return`: add the selected effect at the playhead
- `Ctrl+C`: copy selected clip
- `Ctrl+V`: paste clip at the playhead
- `Ctrl+D`: duplicate selected clip after itself
- `Ctrl+Z`: undo the last edit, or cancel a pending inspector edit
- `Ctrl+Y`: redo the last undone edit
- `Left` / `Right`: move the cursor by the current snap step
- `Shift+Left` / `Shift+Right`: move the cursor by a larger step
- `Up` / `Down`: select the previous or next clip
- `Delete`: delete selected clip
- `Ctrl++`: zoom timeline in
- `Ctrl+-`: zoom timeline out

Session behavior:
- project edits and editor state are autosaved to a local session file
- reopening `./show-editor` resumes the last autosaved project, cursor, layout, and view mode
- opening a project with `--project` or from the File menu replaces the resumed session

Timing notes:
- `Frequency Hz` is the free-running rate for effects like blink, pulse, and strobe.
- With `Use Project BPM` enabled, that same frequency field becomes `Cycles / Beat`, and new edits are stored that way.

## Render One Preview Frame

```sh
python3 -m tools.show_editor.render_frame_cli \
  tools/show_editor/example_project.json \
  /tmp/demo_show/frame_flat.svg \
  --layout ../jonas/led_layout.txt \
  --role B \
  --time-ms 5000
```

For an isometric preview:

```sh
python3 -m tools.show_editor.render_frame_cli \
  tools/show_editor/example_project.json \
  /tmp/demo_show/frame_iso.svg \
  --layout ../jonas/led_layout.txt \
  --role B \
  --time-ms 5000 \
  --projection iso
```

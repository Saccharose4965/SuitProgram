Pending review

# LED layout editor

## Purpose

This screen edits the wiring properties that can be checked directly on the
costume: strip assignment, section length, physical direction and ring start
angle. It also saves, reloads and restores the layout through the LED layout
service. Geometry coordinates remain file-authored because the 64-pixel screen
cannot represent precise three-dimensional measurements.

## Selection and editing

The editor presents one strip and one section within that strip. It resolves
that local selection back to the service's canonical section index before each
edit, so changing a section's strip cannot leave later edits pointing at an
unrelated section. Selection is clamped after reloads and resets.

Up/down select a field. Left/right change its value. Coarse input changes LED
length by ten and ring rotation by fifteen degrees; regular input changes them
by one. Save commits the in-memory layout to the SD card. Reload discards
unsaved edits. Reset restores the compiled default in memory and remains dirty
until Save is selected.

## Physical preview

The preview writes a temporary logical RGB frame through the normal LED output
path. Section mode lights the selected section. Runner mode advances one LED
every 70 ms and exposes direction and wiring continuity. The two strips use
different colours. Closing the screen clears the preview and restores the LED
mode that was active when the editor opened.

The preview frame is allocated statically in external RAM. Drawing the OLED
only reads layout snapshots; SD access occurs on explicit save or reload.

## State and errors

Edits update the service immediately and mark the screen dirty. A short status
line reports persistence errors. The editor keeps the modified in-memory
layout after a failed save, allowing another save attempt without repeating
the edits.

Hardware review should verify both strips, every section boundary, reversal,
the closed ring's start LED, save/reboot persistence and restoration of the
previous animation on exit.

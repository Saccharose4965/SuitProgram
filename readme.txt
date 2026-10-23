SuitProgram — ESP32 suit firmware
=================================
Interactive LED suits: synchronized shows, live beat tracking, audio,
Bluetooth, motion tools, messaging and games.

Start here
----------
docs/architecture-proposal.md — apps / services / drivers.
services/paint/README.md — example design explanation.
docs/README.md — feedback and design validation.
TODO.md — work queue and preserved wishlist.
PROGRESS.md — completed batches and remaining checks.

The migration is in progress. Source still mostly lives in main/ and components/.
Services include background work AND shared logic.

Build and flash (from this folder)
---------------------------------
bash -c 'source ../esp-idf/export.sh && idf.py build'
bash -c 'source ../esp-idf/export.sh && idf.py -p /dev/ttyUSB0 -b 115200 flash monitor'

The bash wrapper also works from fish. Exit monitor with Ctrl+].
Use the working sibling ESP-IDF checkout; do not upgrade it as cleanup.
Do not commit local credentials or SDK settings.

Desktop editor
--------------
PC-only editor: ../ShowEditor/show-editor
It is not compiled into firmware. Consult its instructions to export SD packages.

Documentation
-------------
One main design explanation per app, service and driver, reviewed with the user.
Earlier technical notes: docs/reference/technical/
Previous long README, preserved intact: docs/reference/previous-readme.txt
That archive is historical reference, not verified current state.

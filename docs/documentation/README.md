# Source inspection checklist

[Design documentation and user validation](../README.md) use the first-line
Pending review marker in each module README.

This folder retains the file inventory and current source-inspection receipts.
Its checkmarks mean an explicit review of a particular source hash. They are
separate from user approval of a design explanation.

Run from the firmware root:

    python3 docs/documentation/update_checklist.py --write
    python3 docs/documentation/update_checklist.py --check

Only record a source review after reading the complete file and checking its
description:

    python3 docs/documentation/update_checklist.py --record-review PATH --doc DOC

Changed files and missing descriptions invalidate receipts. Moving a file requires
a new review receipt. A design overview does not replace reading the complete
source file named by a receipt.

The inventory includes tracked and non-ignored untracked firmware files. Sibling
PC projects, SD contents and vendor/toolchain code require separate inventories.

# Firmware and suit documentation programme

## Revised direction

Start with the [architecture proposal](../architecture-proposal.md) and the
[light-painting design explanation](../design/light-painting.md). The agreed
architecture is **apps, services (background + shared logic), drivers**.

The earlier documents below are implementation reference notes, not the target
explanation style. New explanations should teach the full decision flow: what
problem is solved, what information exists, why the model was chosen, how state
changes, and what happens when assumptions fail. Keep register lists, function
inventories and incidental constants out of that primary narrative. Do not invent
historical rationale where the code only establishes behaviour.

The checklist currently records source inspection under the earlier review
rules. It does **not** mean every checked file has a satisfactory design
explanation under this revised standard. Preserve those receipts as reference;
do not automatically award new coverage for a dependency map or proposed move.
Agree on the sample explanation before rewriting the rest or restructuring code.

The goal is complete **English-language** documentation of what the ESP32 and
suit do, including implementation details and the reasons for logic choices.
This is an ongoing audit, not a claim that existing documentation is complete.

Start with [the file checklist](file-checklist.md). Every existing, non-ignored
file in this firmware working tree has an entry, including headers, tests,
configuration, build definitions, assets, scripts and documentation. Untracked
files are included so unfinished work cannot disappear from the audit.

## When a file may be checked off

Read the whole current file. Write or update a linked English document covering
the applicable items below, then record the review. A filename mentioned in an
architecture overview is **not** sufficient.

1. Purpose, build inclusion and whether the code is active, optional or legacy.
2. Public API, units, coordinate systems, data formats and relevant constants.
3. Functions and important execution paths, state transitions and invariants.
4. Callers, dependencies, task/core context, callbacks and interrupt boundaries.
5. Resource ownership and lifecycle: memory, I2S, SPI, SD, radio and LED output.
6. Timing, buffering, latency, rate limits and performance compromises.
7. Failure paths, recovery, cancellation, concurrency and persistence.
8. User-visible controls and hardware assumptions, where applicable.
9. Why the logic is this way, alternatives rejected and their tradeoffs.
   Separate reasons established by code/history from inferred intent. If the
   original reason is unknown, say so; do not invent a justification.
10. Tests, what they prove, what they do not prove and remaining hardware checks.

Headers need API contracts, not duplicated source commentary. Tests need their
scenarios, fixtures and blind spots. Assets need their role, format, provenance
where known and replacement process. Configuration needs units/defaults and
behavioural consequences, **never copied passwords, tokens or private keys**.

Several files can share a subsystem document, but each checked file needs a
clearly identified section. Shared dependencies outside that review remain
unchecked. Known limitations belong in the document, not concealed by a tick.

## Keeping the inventory honest

From the firmware root:

```sh
python3 docs/documentation/update_checklist.py --write
python3 docs/documentation/update_checklist.py --check
```

After a human/agent has completed the review described above:

```sh
python3 docs/documentation/update_checklist.py --record-review main/example.c --doc docs/example.md
```

That command records the file's SHA-256 and generates the checklist. It does not
judge documentation quality. Editing reviewed source automatically makes the
next generated checklist mark it **needs re-review**. A missing documentation
file also invalidates the tick. Updating a document still requires editorial
review: the script cannot detect incorrect explanations or changed dependencies.
`--check` fails if the checked-in checklist no longer matches the working tree.

`reviews.json` stores review receipts; do not manually tick generated Markdown.
Removed files disappear from the live checklist, but their receipts remain for
history. Rename/replacement is a new review. None of these operations deletes
source files or rewrites existing technical documentation.

## Scope and exclusions

Ignored build output, `.git`, caches, toolchains and vendor dependencies are not
owned firmware source and are excluded by Git's ignore rules. Inventory their
versions and any local patches in the build/environment documentation rather
than pretending to audit all upstream ESP-IDF code here. Tracked files are
included even if a later ignore rule matches them. Files deleted in the current
working tree are not counted as live code.

The sibling ShowEditor, localFFT PC reference/benchmark, relay/server utilities,
SD-card contents, physical wiring and local ESP-IDF patches need **separate
inventories and English documentation**. This firmware checklist does not claim
to cover those folders. Create those inventories before declaring the whole
suit system documented. Record the actual root/revision and exclusions for each.

## Recommended review order

- Boot/build, shell lifecycle, task scheduling and hardware/pin map.
- Shared buses and resource ownership; audio, microphone, Bluetooth and FFT.
- LEDs, layout/legs, animation timing, shows and multi-suit synchronization.
- Link/telemetry, messages, calls, identities and delivery semantics.
- IMU fusion, coordinate conventions, arm pose, painting and synth.
- Menus, applications, games, tests, assets and unused/legacy modules.
- Cross-system end-to-end flows, recovery matrix, hardware procedures and
  rationale review. Reconcile old documents against current implementation.

Progress is measured by reviewed current files, **not by word count**. Good
documentation should explain the system without duplicating every line of C.

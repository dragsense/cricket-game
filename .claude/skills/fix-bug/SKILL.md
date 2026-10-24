---
name: fix-bug
description: Use when something is broken (crash, wrong behaviour, build failure) to reproduce it, fix the root cause minimally and guard it with a test.
---

# Fix a bug

1. **Reproduce it.** Get exact steps or input. Run the game or tests and capture the
   log, assert or crash output. If it can't be reproduced, say so and ask for details.
2. **Find the root cause.** Trace from the symptom to the actual faulty assumption
   (ordering, units, conventions such as GLM's (w, x, y, z) quats, Y-up/-Z camera,
   lifetime). Don't patch the symptom.
3. **Fix with the smallest change** that addresses the cause. Don't refactor unrelated
   code.
4. **Add a test** (doctest) when the logic is testable without a window or GPU,
   especially for math, physics or IK.
5. Build debug and release, run the tests, and confirm the original repro is gone.
6. If the bug isn't fully fixed, or a workaround remains, record it under
   **Known issues** in PROGRESS.md.
7. Commit with the message "Fix: <symptom> (<root cause>)".
8. Summary: cause, fix, how it was verified.

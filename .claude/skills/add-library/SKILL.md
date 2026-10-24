---
name: add-library
description: Use whenever a library outside the CLAUDE.md stack might be needed; evaluate it, get approval, then pin and record it.
---

# Add a library

1. **Check the need.** Confirm nothing in the current stack already covers it
   (Filament's own libs, SDL3, GLM, EnTT…).
2. **Evaluate the candidate:**
   - popular and widely used (stars, notable users);
   - actively maintained (recent releases and commits);
   - permissive licence (MIT, BSD, Apache 2.0, zlib);
   - supports Windows, Android and iOS;
   - builds with CMake (FetchContent-friendly), or ships prebuilt;
   - size and runtime cost, with mobile in mind.
3. **Show alternatives:** compare 1–3 others in a short table, plus "write it
   ourselves" if relevant, and recommend one.
4. **Wait for explicit approval.** Do not download anything before it.
5. After approval:
   - pin an exact version (a tag at least two weeks old) in `cmake/dependencies.cmake`;
   - add the FetchContent declaration there;
   - link it only to the layer that needs it.
6. **Record it:**
   - DECISIONS.md entry: decision, reason, alternatives rejected;
   - CLAUDE.md stack list;
   - the DESIGN.md library table.
7. Build both configs and run the tests, then commit.

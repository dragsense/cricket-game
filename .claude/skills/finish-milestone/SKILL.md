---
name: finish-milestone
description: Use when a milestone's checklist is complete, to verify, clean up, document, commit and tag it, then hand over for visual review.
---

# Finish a milestone

1. **Build and test both configs** and fix every error and warning in our code (/W4):
   - `scripts\build.ps1 -NoRun`
   - `scripts\build.ps1 -Config release -NoRun`
2. **Run the game** briefly, and check the log for errors and startup failures.
3. **Duplication:** search for duplicated logic, helpers that reimplement library
   features, and dead or unused files, functions and config keys. Remove or merge them.
4. **Layer rules:**
   - grep includes, so each layer only includes layers below it
     (core → math → rig/ik → motion → agents → ai → match);
   - no `filament/`, `SDL` or `glm/` includes outside `src/render/filament`,
     `src/camera`, `src/platform` and `src/math`;
   - no heap allocation in sim tick paths.
5. **Data-driven:** no new magic numbers that belong in config/*.json.
6. **Docs:**
   - DESIGN.md: fill in each built system's section.
   - PROGRESS.md: mark the milestone done (with the date) and set the next one.
   - DECISIONS.md: log any decisions made during the milestone.
7. **Commit and tag:** commit, then `git tag -a mN -m "Milestone N: <name>"`.
8. **Summary:** what was built, how to run it, known issues, and a short
   **"please check visually"** list. Name specific things to look at (motion, camera
   feel, scale, shadows) and how to trigger them.

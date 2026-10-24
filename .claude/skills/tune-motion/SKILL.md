---
name: tune-motion
description: Use for visual feedback on procedural motion ("the knee bends too early", "dive looks stiff") to find the responsible system and tune config before code.
---

# Tune procedural motion

1. **Restate the feedback precisely:**
   - which character and action;
   - which joint or phase;
   - what is wrong: timing, angle, speed, balance or foot sliding.
2. **Find the responsible system** (rig, ik, motion, agents) and the code path that
   drives that joint or phase. Use debug draw (bones, IK targets, velocities) to see it.
3. **Config first:**
   - find the tunables in config/*.json (joint limits, timings, gains, speeds) and
     change those;
   - if the needed knob doesn't exist, expose it in config rather than hardcoding a
     new constant.
4. **Change code only if** the behaviour can't be expressed by tuning (a wrong
   algorithm, a missing constraint). Keep the change minimal and in the right layer.
5. Rebuild and run. If possible, add a debug toggle or overlay value that shows the
   change.
6. **Explain:**
   - what was changed (file, key, old → new value);
   - why it addresses the feedback;
   - exactly what to look at to judge it.
   Motion quality is judged by the user's eye, so ask for a re-check.
7. Commit with the message "Tune: <action> <joint> (<keys changed>)".

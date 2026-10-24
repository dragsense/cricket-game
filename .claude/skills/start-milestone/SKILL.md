---
name: start-milestone
description: Use when beginning a new milestone (e.g. "start M1") to plan scope, steps and libraries, and get approval before writing any code.
---

# Start a milestone

1. Read PROGRESS.md (roadmap, the milestone's scope, known issues), the DESIGN.md
   sections for the affected systems, DECISIONS.md and CLAUDE.md.
2. State the milestone's exact scope. Also list what is explicitly out of scope.
3. List any open decisions that block it (e.g. the player body mesh before M1), and ask
   about them.
4. Break the scope into small steps, each one buildable and testable on its own, with a
   visible result where possible (debug draw or overlay).
5. For each step, name the library that covers it, or why it is our own code
   (cricket-specific logic or glue). Any new library goes through the `add-library`
   skill.
6. Search the codebase for existing code each step can reuse or extend.
7. Note the config/*.json tunables each step will add.
8. Present the plan as a short table (step | library or own code | files | how to
   verify), then **wait for approval** before writing code.
9. After approval: mark the milestone "in progress" in PROGRESS.md, with the steps as a
   checklist, and commit.

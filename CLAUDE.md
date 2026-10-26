# px4_practice_cpp — Claude Code Rules

## Project Overview
A standalone C++17 practice flight stack modelled on PX4 Autopilot (uORB
pub/sub, work-queue modules, estimator, cascaded attitude/rate control,
control allocation, commander) flying a simulated quadrotor in lockstep SITL.
It exists so the owner can learn PX4 concepts and flight-software C++, both
locally and from remote Claude Code sessions.

## Tech Stack
C++17, CMake (3.20+), GoogleTest (fetched via FetchContent if not installed).
No external runtime dependencies.

## Build and Test
```bash
cmake -S . -B build -G Ninja -DPX4P_WERROR=ON
cmake --build build
ctest --test-dir build --output-on-failure
./build/px4p_sitl flight.csv
```

## Learning Workflow (important)
- The owner uses this repo to learn. When explaining a concept, use `/teach`;
  to check retention, use `/drill-me`. Both read and update
  `docs/learning/PROGRESS.md` — read it at the start of a teaching session.
- Lessons live in `docs/learning/lessons/`, drills in `docs/learning/drills/`.
- Practice tasks are in `docs/EXERCISES.md`. When the owner works on an
  exercise, coach rather than solve: give hints and review their code first,
  and only write the solution if asked.

## Skill Usage Policy
Standards and analysis skills are OPT-IN. Do NOT apply python-standards,
c-cpp-standards or complexity-reduction unless explicitly invoked.

## Conventions
- Frames: world NED, body FRD. Quaternions Hamilton (w, x, y, z), body -> world.
- Motor order: PX4 Quadrotor X (1 front-right CCW, 2 rear-left CCW,
  3 front-left CW, 4 rear-right CW).
- Floats only in flight code (`0.5f`, not `0.5`); the build uses `-Wdouble-promotion`.
- One folder per module under `src/modules/`, each a `ScheduledWorkItem`.
  Modules talk only through uORB topics, never by calling each other.
- Keep control laws as free functions (e.g. `mix()`, `attitude_control_update()`)
  so they can be unit-tested without the module.
- Tests: GoogleTest, one `tests/test_<area>.cpp` per area; fixtures call
  `uorb::reset_all()` in `SetUp()`.

## Hard Rules
- No placeholders or TODOs in committed code
- Every public function needs a comment
- Tests required for new logic; closed-loop behaviour gets a test in
  `tests/test_closed_loop.cpp`
- Build must be warning-free with `-DPX4P_WERROR=ON`

## Workflow
- For new features: start with /grill-me to align before coding
- Use plan mode for any change touching 3+ files
- For unfamiliar code: use /zoom-out for context
- For bugs: use /diagnose for structured debugging (no guessing fixes)
- Commit format: type(scope): description
- Push to feature branches, never directly to main
- At session end: /recap for learning, /handoff if continuing later

## Communication Style
- Be direct and concise
- Surface trade-offs explicitly
- Ask questions when requirements are unclear

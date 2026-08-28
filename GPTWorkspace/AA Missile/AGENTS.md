# AA Missile project instructions

## Scope

These instructions apply to every file under `AA Missile/`.

This is a Stormworks simulation project named 「防空ミサイル性能向上プロジェクト」.
Keep all analysis, terminology, implementation, and verification specific to the game and the files in this directory.

## Project objective

Improve the air-defence missile's ability to keep a beam-manoeuvring target inside the terminal-guidance radar field of view (FOV), while preserving stable tracking and interception performance.

The current primary hypothesis is:

- During proportional-navigation guidance, the target can leave the terminal radar FOV.
- Once radar observations are lost, terminal tracking becomes weak or fails.
- The first goal is to reproduce and measure this sequence before choosing a guidance change.

Do not treat this hypothesis as proven beyond what the user has stated. Separate confirmed code behaviour, observed in-game behaviour, and proposed explanations.

## Start every task here

Read these files in order before making a material change:

1. `README.md`
2. `docs/PROJECT_PLAN.md`
3. `docs/INTERFACES.md`
4. `docs/TOOLING.md`
5. `docs/TEST_PLAN.md`
6. The Lua file(s) relevant to the task
7. The latest applicable record under `experiments/`

Use `Stormworks 概要 (v7).docx` as the game-specification reference. Do not edit or replace it unless the user explicitly asks.

## Current code ownership

- `ADSV3 MissileKF.lua`: terminal-radar processing, target association, EKF state estimation, target selection, and radar beam pointing outputs.
- `ADSV3 Missile.lua`: launch sequence, mid-course guidance, pursuit/PN switching, gravity compensation, terminal-radar handoff, and detonation outputs.
- `ADSV3 MissileFOVDebug.lua`: optional, output-free DebugView instrumentation for terminal-radar FOV margin and raw-observation continuity.

Before moving responsibilities between these files, explain why and update `docs/INTERFACES.md`.

## Stormworks invariants

- Simulation rate is 60 ticks per second; the normal time step is `1/60` seconds.
- Radar angles from composite channels are in turns and are converted to radians where required.
- The Physics Sensor world frame used here is `X = east`, `Y = up`, `Z = north` and is left-handed.
- The missile local frame used by the guidance code is `X = right`, `Y = up`, `Z = forward`.
- Account explicitly for configured logic delay when comparing observations, commands, and outcomes.
- The readable Lua source is the development authority. Do not sacrifice readability merely to meet the in-game character limit unless producing a clearly identified deployment build.

## Development rules

- Preserve existing files and user changes. Never discard or rewrite unrelated work.
- Prefer one testable hypothesis per change.
- Establish a baseline before tuning gains or changing guidance laws.
- Do not silently change composite channel numbers, property names, coordinate conventions, units, or state-vector ordering.
- When an interface changes, update `docs/INTERFACES.md` in the same task.
- When a design choice is accepted, append it to `docs/DECISIONS.md`.
- When an in-game trial is run, copy the experiment template and record inputs, build/version, observations, and result.
- Avoid adding defensive checks solely for style when they materially increase the Stormworks character count. Add checks when they prevent a demonstrated failure.
- Keep temporary instrumentation clearly labelled and easy to remove.
- Do not claim an in-game result from static inspection or an offline calculation.

## Guidance terminology used in this project

- `isPPN == true` in the current code selects direct angle/pursuit steering (the comments call it 「単追尾」).
- `isPPN == false` selects the current LOS-rate proportional-navigation branch.
- Use the explicit terms `pursuit` and `PN` in new documentation; do not rely on the variable name alone.
- `missileRadarIO` is the terminal-radar handoff/enable state.
- `radarManualSweepX/Y` are the terminal radar's commanded azimuth/elevation pointing values.

## Verification expectations

For documentation-only changes:

- Check links, filenames, channel mappings, units, and consistency with the current Lua.

For Lua changes:

- Run available syntax/lint checks.
- Inspect every changed input/output and property mapping.
- Check angle units, coordinate frames, signs, and tick/second conversions.
- Compare against the baseline beam-manoeuvre scenario in `docs/TEST_PLAN.md`.
- Report what was verified offline and what still requires Stormworks testing.

## Completion report

At the end of a task, report:

- Files changed and the behavioural intent
- Hypothesis tested or documentation improved
- Checks performed and their results
- Remaining unknowns or in-game verification required
- Recommended next experiment, if applicable

# HGV Missile project instructions

## Scope

These instructions apply to every file under `HGV Missile/`.

This is a Stormworks simulation project named「HGV機動ミサイル開発プロジェクト」. Its purpose is to create a reproducible manoeuvring target missile for evaluating air-defence systems in the game.

Do not edit files under `AA Missile/` as part of this project unless the user explicitly requests a cross-project change. Record interoperability requirements here first.

## Current state

- The project environment exists, but no HGV guidance Lua or vehicle interface has been selected yet.
- “HGV機動” is currently a project label, not a finalized trajectory or control law.
- Do not invent channel mappings, property values, vehicle performance, or maneuver profiles. Mark them `TBD` until confirmed.

## Start every task here

Read these files in order before making a material change:

1. `README.md`
2. `docs/PROJECT_PLAN.md`
3. `docs/INTERFACES.md`
4. `docs/TOOLING.md`
5. `docs/TEST_PLAN.md`
6. The Lua file(s) relevant to the task, once they exist
7. The latest applicable record under `experiments/`

Use the workspace-root `Stormworks 仕様書 (v7).docx` as the game-specification reference. Do not edit or replace it unless the user explicitly asks.

## Project boundary

In scope:

- Launch and flight-phase management for the HGV target missile
- Reproducible horizontal and vertical maneuver profiles
- Guidance, attitude/fin commands, and vehicle-limit handling
- Output-free DebugView telemetry needed to validate the target trajectory
- Test scenarios that can later be flown against `AA Missile`

Out of scope unless explicitly requested:

- Changes to the air-defence missile
- Changes to the air-defence system or datalink protocol
- Real-world weapon design or performance claims
- Assuming realistic aerodynamics where Stormworks behavior has not been measured

## Stormworks invariants

- Simulation rate is 60 ticks per second; the normal time step is `1/60` seconds.
- Record whether angles are turns or radians at every interface.
- For Physics Sensor data, use the world frame `X = east`, `Y = up`, `Z = north`, left-handed, unless a test proves the vehicle uses a different mapping.
- Define the missile-local frame before implementing guidance. The expected default is `X = right`, `Y = up`, `Z = forward`.
- Treat configured logic and wiring delay explicitly.
- The readable Lua source is the development authority.

## Development rules

- Preserve existing files and user changes.
- Define the maneuver requirement and vehicle interface before implementing a controller.
- Prefer one testable hypothesis per change.
- Do not silently assign or change composite channels, property names, coordinate conventions, units, or flight-phase meanings.
- Update `docs/INTERFACES.md` whenever an interface is accepted or changed.
- Append accepted design choices to `docs/DECISIONS.md`.
- Create an experiment record for every in-game trial.
- Keep each deployable Lua block within Stormworks' 8192-character limit after compression.
- Use `debug.log` and string concatenation with `..` for diagnostic telemetry.
- Do not add composite output channels solely for debugging. Operational outputs must have a flight-control purpose.

## Verification expectations

For documentation-only changes, check filenames, links, units, scope boundaries, and `TBD` status.

For Lua changes:

- Run available Lua syntax and major-diagnostic checks.
- Measure LifeBoat-compressed character count.
- Inspect every changed input, output, property, unit, coordinate sign, and tick/second conversion.
- Test the maneuver without an interceptor before using it as an air-defence target.
- Separate offline verification from confirmed Stormworks results.

## Completion report

At the end of a task, report:

- Files changed and intended behavior
- Assumptions and remaining `TBD` items
- Offline checks performed
- Stormworks tests still required
- Recommended next experiment

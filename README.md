## Project Autodrive

This repository contains a first implementation of the fixed-path
reciprocal-caution branching planner described in
`prompts/implementation_plan.md`.

The code is organized as a Python package under `src/rcbranch`:

- `commonroad_adapter`: lazy CommonRoad loading, route, state, and trajectory adapters.
- `geometry`: reference paths, projection, conflict regions, and active-set selection.
- `belief`: pairwise crossing-order features and temporal belief filtering.
- `mpc`: CasADi NLP builder, reciprocal-caution MPC, branch scoring, and branch MPC.
- `planners`: proposed dual-priced branching planner and baseline wrappers.
- `evaluation`: simple closed-loop runner, metrics, validation hooks, and plotting helpers.
- `scripts`: command-line entry points and root-level wrappers.

### Environment

The project targets Python `>=3.11,<3.14`, matching the CommonRoad package
ecosystem described in the implementation plan.

```bash
uv sync --python /opt/homebrew/bin/python3.11 --extra dev
```

CommonRoad-specific adapters use lazy imports. Install the optional CommonRoad
suite when loading/exporting real CommonRoad scenarios:

```bash
uv sync --python /opt/homebrew/bin/python3.11 --extra dev --extra commonroad
```

### Tests

```bash
uv run --python /opt/homebrew/bin/python3.11 pytest
```

The current tests cover projection round trips, conflict-region extraction,
the CasADi toy multiplier sanity check, belief updates, and branch-trigger
scoring.

### Walkthrough

For a detailed tutorial and codebase map, read
[`docs/framework_walkthrough.md`](docs/framework_walkthrough.md).

### Visualization Demo

Launch an interactive matplotlib crossroad demo with a time seekbar, play/pause,
and step controls:

```bash
uv run --python /opt/homebrew/bin/python3.11 rcbranch-demo-traffic
```

To save a static frame without opening a GUI:

```bash
uv run --python /opt/homebrew/bin/python3.11 rcbranch-demo-traffic --no-show --save-frame traffic_demo.png
```

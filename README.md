# webots-projects

Worlds, controllers, and other resources I build for my robotics university
course, using [Webots](https://cyberbotics.com/). Everything is kept
in one repository and the Python environment is managed with
[uv](https://docs.astral.sh/uv/).

## Prerequisites

- [**Webots**](https://cyberbotics.com/) (installed at `/Applications/Webots.app` on macOS).
- [**uv**](https://docs.astral.sh/uv/) (`brew install uv`).

### Pointing Webots at the uv environment

The `controller` module is provided by Webots itself — it is **not** a pip
package. For controllers to also see the packages from this project (`numpy`,
the shared `epucklib` library, …), Webots must run them with **this
project's** Python interpreter.

Set the Python command in Webots to the project venv:

**Webots → Settings → General → Python command:**

```bash
/Users/asg/workspaces/webots-projects/.venv/bin/python
```

Alternatively, set it per-controller with a `runtime.ini` next to the
controller file:

```ini
[python]
COMMAND = /Users/asg/workspaces/webots-projects/.venv/bin/python
```

## Adding a new exercise

1. Create the world: `worlds/<exercise>.wbt` (usually done from the Webots GUI).
2. Create the controller folder and file with **matching names**:

   ```text
   controllers/<exercise>/<exercise>.py
   ```

3. In the world, set the robot's `controller` field to `<exercise>`.
4. A minimal controller using the shared helpers:

   ```python
   import sys
   from pathlib import Path

   REPO_ROOT = Path(__file__).resolve().parents[2]
   sys.path.insert(0, str(REPO_ROOT))

   from controller import Robot

   from epucklib import devices

   robot = Robot()
   dev = devices.setup(robot)
   while robot.step(dev.timestep) != -1:
       pass  # read sensors, run logic, drive motors each time step
   ```

## Exercises

- **`worlds/epuck.wbt` + `controllers/epuck_roomba`** — systematic floor
  coverage in the Tutorial 1 arena (three wooden boxes): a basic Roomba. The
  robot is told the size of its arena and nothing else; the walls and boxes are
  discovered by infra-red.

  It sweeps the floor in boustrophedon lanes 5 cm apart while building a 2 cm
  occupancy and coverage grid, then switches to breadth-first search toward the
  nearest cell that would still sweep new ground, until nothing reachable is
  left. Layers: wheel-encoder dead reckoning re-anchored to ground truth every
  5 s through the supervisor API (`MOCAP_INTERVAL_S = 0` to watch pure dead
  reckoning smear the map); IR readings converted to metric distances by
  inverting the sensor's own `lookupTable`; a repulsive potential field with a
  tangential term blended into the waypoint bearing; and a stall detector that
  backs out of traps. Coverage progress goes to the console.

  On its last measured run it covered 91.5% of the arena floor (99.4% of what
  the coverage metric can represent — a one-cell ring around the border can
  never be credited as swept, see below) and stopped at t≈339 s of simulated
  time because nothing reachable remained.

  Headless run:

  ```bash
  /Applications/Webots.app/Contents/MacOS/webots --batch --minimize \
      --no-rendering --mode=fast --stdout --stderr worlds/epuck.wbt
  ```

  Then render the figure:

  ```bash
  uv run python analysis/plot_coverage.py
  ```

  ![coverage map](docs/figures/roomba_coverage.png)

  The light grey border in the figure is not missed floor: the robot's centre
  can never get within a body radius of the wall, so that outer ring is
  structurally out of the coverage metric's reach rather than actually
  unswept.

  The white specks *inside* the black boxes are not unswept floor either —
  they are box interior that only ever got seen once, so it never reached the
  two-hit threshold that confirms an obstacle. Of the 207 uncovered cells, 193
  are that out-of-reach ring and 12 of the remaining 14 are these interior
  specks, which puts the robot's true miss count at 2 cells.

## Tests

The logic in `epucklib/` never imports the Webots `controller` module, so it
runs under plain pytest:

```bash
uv run pytest
```

## Dependencies

- Runtime: `numpy`, `matplotlib` (see `pyproject.toml`).
- Dev group: `pytest` — installed by `uv sync`, skipped with `uv sync --no-dev`.

Add more with `uv add <package>` (or `uv add --dev <package>`).

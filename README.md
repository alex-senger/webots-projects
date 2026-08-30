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
<your-path-to-webots-projects>/.venv/bin/python
```

## Exercises

- **`worlds/industrial_example.wbt` + `controllers/scara_food_industry`** — an
  Epson SCARA T6 sorting fruit, adapted from the Cyberbotics sample of the same
  name so that it sorts the fruit *into the wrong crates*: the robot alternates
  orange, apple, orange, … and places each one in the crate originally intended
  for the other. Its LED starts off and toggles on every fifth orange that has
  been both picked and placed, so it changes state after oranges 5, 10, 15, …

  Each placement is printed, which is the quickest way to see the LED rule
  holding:

  ```text
  cycle   9  placed orange in the apple  bin  (orange #5)  -- LED ON
  cycle  19  placed orange in the apple  bin  (orange #10)  -- LED OFF
  ```
  
  As in the original sample the grasp is faked: with supervisor access the
  fruit node is teleported just under the suction tool on every step it is
  held, and released by no longer doing so.

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

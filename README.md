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
the shared `webots_projects` library, …), Webots must run them with **this
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
   from controller import Robot
   from webots_projects import run

   robot = Robot()
   for step in run(robot):
       pass  # read sensors, run logic, drive motors each time step
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

  Headless run:

  ```bash
  /Applications/Webots.app/Contents/MacOS/webots --batch --minimize \
      --no-rendering --mode=fast --stdout --stderr worlds/industrial_example.wbt
  ```

## Dependencies

- Runtime: `numpy` (see `pyproject.toml`).
- Dev group: `matplotlib` for plotting/analysis — installed by `uv sync`,
  skipped with `uv sync --no-dev`.

Add more with `uv add <package>` (or `uv add --dev <package>`).

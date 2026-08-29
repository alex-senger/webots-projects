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

## Dependencies

- Runtime: `numpy` (see `pyproject.toml`).
- Dev group: `matplotlib` for plotting/analysis — installed by `uv sync`,
  skipped with `uv sync --no-dev`.

Add more with `uv add <package>` (or `uv add --dev <package>`).

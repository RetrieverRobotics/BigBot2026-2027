# BigBot2026-2027

VEX U (VURC) — **Override**, 2026-27 season. Retriever Robotics.

## Layout

| Path | What |
|---|---|
| `2026BigRobot/` | PROS project for the 24" robot. C++, builds via the PROS VS Code extension. |
| `VisionSystem/` | Coprocessor perception. Python, managed with `uv`. Runs on the Linux coprocessor, talks to the V5 brain over serial. |

## Robot code (`2026BigRobot`)

Toolchain is bundled by the `sigbots.pros` VS Code extension — no separate PROS CLI install needed.
The extension also generates `compile_commands.json`, which is what clangd reads. It is
gitignored because the paths are machine-specific; regenerate it from the PROS extension
if IntelliSense goes cold.

## Vision (`VisionSystem`)

```
uv sync          # create .venv and install from uv.lock
uv run pytest    # tests
uv run visionsystem --help
```

`uv.lock` is committed — it is what makes the environment reproducible on the ARM coprocessor.

## Shared config

`.clang-format`, `.clang-tidy`, and `.editorconfig` live at the repo root and apply to both
projects. Vendored trees (`include/liblvgl`, `include/pros`) carry their own `.clang-tidy`
that disables linting.

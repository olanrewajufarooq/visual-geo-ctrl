# Coding Conventions

## Namespacing

MATLAB package names follow the `fth.<package>.<symbol>` pattern.

Examples:

- `fth.sim.Config`
- `fth.sim.SimRunner`
- `fth.se3.expSE3`
- `fth.ctrl.ControllerWrench`

Package folders use MATLAB `+package` directory naming under `src/+fth/`.

## File Organization

- Keep domain logic inside the relevant package rather than in run scripts.
- Use `run/` for reproducible scenarios and demos, not for core library behavior.
- Prefer extending factories and dedicated subpackages over adding large conditional blocks in unrelated files.

## Configuration Style

- Use `fth.sim.Config` as the single configuration entry point.
- Call `cfg.done()` before constructing `SimRunner`.
- Prefer explicit run names and script names for batch scenarios so output folders stay readable.

## Commits

Use Conventional Commits when possible.

Examples:

- `feat: add helix trajectory sweep`
- `fix: correct adaptive gain batch expansion`
- `refactor: simplify results naming`
- `test: cover bregman adaptation edge cases`
- `docs: split readme into docs hub`

## Documentation

- Keep the root `README.md` concise and link out to `docs/`.
- Put durable how-to and architecture details in topic-specific markdown files.
- Update documentation when changing public configuration or run-script behavior.

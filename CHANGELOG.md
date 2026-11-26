# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-11-26

First tagged release. The project is a packing cell with two Epson SCARA T6-602S robots in
Webots R2025a: a conveyor brings parts to one pick point, the two robots alternate and each fills
a 6-pocket tray, and the full tray leaves through a fence on the north side. It has a PLC supervisor,
an interlocked door, e-stop and area-scanner safety model, and an HTML5 animation published on
GitHub Pages.

### Added
- MIT `LICENSE` and a `VERSION` file (0.1.0).
- CI runs the offline cell tests with pytest, a compile check of every Python file, ruff (syntax
  errors and undefined names), and a check that the committed world matches `tools/gen_world.py`.
- A `security audit` job: `pip-audit` on the hash-pinned CI toolchain and `zizmor` on the workflows.
- CodeQL (Python and Actions), OpenSSF Scorecard and a tag-triggered release workflow with build
  provenance; Dependabot for actions.
- `SECURITY.md`, `CONTRIBUTING.md` and this changelog.

[Unreleased]: https://github.com/sergiudanstan/dual-scara-cell-webots/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/sergiudanstan/dual-scara-cell-webots/releases/tag/v0.1.0

# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Entries for releases before this file existed were generated from commit subjects.

## [2.0.4] - 2026-09-10

- Add initial_mode option to move to a mode on startup

## [2.0.3] - 2026-09-10

- initial mode from PARKED to IDLE

## [2.0.2] - 2026-09-10

- Pass Zaber driver options via required zaber block

## [2.0.1] - 2026-09-03

- Add INSMODE FITS header (#872)

## [2.0.0] - 2026-08-26

- Require stable pyobs-core>=2.0.0
- Gate auto-merge on the PR author, not the event actor
- Enable Dependabot auto-merge for patch/minor updates
- Add baseline test suite and CI (pytest, pyrefly), grouped Dependabot
- Upgrade uv.lock to clear open Dependabot alerts
- Require pyobs-core>=2.0.0.dev48
- Add dependabot.yml, targeting develop for PRs
- Use zaber_motion's async settings.set_async() instead of the sync variant
- Update set_mode signature for name-based IMode group parameter
- Add Sphinx documentation
- Expand README with uv install and configuration docs
- add --no-sync flag to ruff check command in workflow
- fixed path
- publish sdist only
- enhance zabermodeselector: integrate ModeChangedEvent, implement mode capabilities/state management, and streamline mode handling
- add newline in pyobs_zaber __init__.py to improve readability
- new pyobs-core
- Add Ruff CI workflows for pyobs-qhyccd and pyobs-zaber
- refactor zabermodeselector: replace state initialization with updated MotionState and ReadyState enums
- update type hints in zaberdriver: use AsyncGenerator for asynccontextmanager functions
- update uv.lock and pyproject.toml: bump pyobs-core to 2.0.0.dev3 and add pyrefly>=1.1.1
- delete DEVELOPMENT.md as it is no longer needed for migration
- clarify import alias in pyobs_zaber module
- remove unused import in zaberdriver.py
- replace flake8 with ruff in pre-commit configuration
- update uv.lock: new dependency versions and Python requirement 3.11+
- add DEVELOPMENT.md for pyobs 2.0 migration instructions

## [1.1.1] - 2026-03-31

- new python
- new lock file

## [1.1.0] - 2025-07-07

- migrated to uv

## [1.0.8] - 2025-03-01

- defaults

## [1.0.7] - 2023-12-11

- disable LED in open()

## [1.0.6] - 2023-12-09

- Fixed get_mode()

## [1.0.5] - 2023-12-06

- to async

## [1.0.4] - 2023-12-06

- addes stop()
- extracted ZaberDriver

## [1.0.3] - 2023-12-04

- fixed bugs

## [1.0.2] - 2023-12-04

- use asyncio

## [1.0.1] - 2023-12-02

- added kwargs
- cleaning up for PyPi publishing
- Update mode_selector.yaml
- added examplary configs
- removed profile option again, acceleration is sufficient
- use built-in acceleration
- actually use profile, not the string
- added option to use a velocity profile
- bugfix
- no async for enable_led
- actually turn LED on or off
- added warning if unknown modes are tried to be set
- add option of turning off the status LED
- replaced move_rel by move_abs and use async version now
- debugging
- first working version
- insert 'await' before async functions
- removed @abstractmethod again
- removed @abstractmethod
- corrected name of module
- aligned child modules with parents
- renaming
- added is_ready method to
- integrated Motor module into the ModeSelector
- Add files into pyobs-zaber, that previously have been removed from pyobs-core
- first commit

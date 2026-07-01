Zaber module for *pyobs*
========================

This is a [pyobs](https://www.pyobs.org) module for selecting between named positions (e.g. instrument ports)
on a [Zaber](https://www.zaber.com/) linear motor.


Install *pyobs-zaber*
-----------------------
Clone the repository:

    git clone https://github.com/pyobs/pyobs-zaber.git
    cd pyobs-zaber

Install it with [uv](https://docs.astral.sh/uv/):

    uv sync

Alternatively, with plain `venv`/`pip`:

    python3 -m venv .venv
    source .venv/bin/activate
    pip install .


Configuration
-------------
The *ZaberModeSelector* class takes a dictionary of named modes mapped to motor positions, plus some parameters
for the underlying Zaber driver:

    modes:
        Dictionary of available modes in the form {name: position}.
    port:
        USB port of the motor (default: /dev/ttyUSB1).
    speed:
        Velocity of the selector movement (default: 10000).
    acceleration:
        Acceleration of the selector movement (default: 800).

A basic module configuration would look like this:

    class: pyobs_zaber.ZaberModeSelector
    name: Mode selector
    port: /dev/ttyUSB0
    modes:
      Photometry: 0
      Spectroscopy: 50000


Dependencies
------------
* [pyobs-core](https://github.com/pyobs/pyobs-core) for the core functionality.
* [zaber-motion](https://pypi.org/project/zaber-motion/) for communicating with the motor.

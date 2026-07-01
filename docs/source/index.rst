pyobs-zaber
###########

This is a `pyobs <https://www.pyobs.org>`_ (`documentation <https://docs.pyobs.org>`_) module for selecting
between named positions (e.g. instrument ports) on a `Zaber <https://www.zaber.com/>`_ linear motor.


Example configuration
*********************

This is an example configuration::

    class: pyobs_zaber.ZaberModeSelector
    port: /dev/ttyUSB0
    modes:
      Photometry: 0
      Spectroscopy: 50000

    # communication
    comm:
      jid: test@example.com
      password: ***


Available classes
*****************

There is one single class for Zaber motors.

ZaberModeSelector
=================
.. autoclass:: pyobs_zaber.ZaberModeSelector
   :members:
   :show-inheritance:

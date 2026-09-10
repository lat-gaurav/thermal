"""Operational scaffolding for running the pipeline as a flight service.

The detector, filters and tracker live in detector/, filters/ and experiment/.
This package is everything AROUND them that a flight needs and a bench does not:

    flight.rawrec     write .rawrec captures, byte-compatible with this repo's readers
    flight.rc_arm     run only while an RC switch is held, and fail safe when it isn't

Both replicate the operational envelope this rig already flew under, so a sortie
behaves the same way it always did -- same switches, same file names, same
failure behaviour -- with this repo's detection and tracking inside it.
"""

"""Everything that touches Gazebo.

Modules that tracking/ is allowed to import: ``frame_source`` and ``gimbal_io`` (and ``telemetry``).
Everything else here is simulator plumbing and must stay invisible to tracking/ and sandbox/.

The pure models (wind jet, OU processes, gimbal dynamics, image post-processing)
do not import gz.*, so they can be unit-tested without a simulator.
"""

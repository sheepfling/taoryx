"""Bundled sensor-family plug-ins registered through the public sensor API."""

from __future__ import annotations

from taoryx.sensor_api import PayloadCodecRegistry, SensorPluginRegistry


def register_builtin_sensor_plugins(
    plugins: SensorPluginRegistry,
    codecs: PayloadCodecRegistry,
) -> None:
    """Register bundled descriptors and payload codecs exactly once."""

    from .gnss import GNSS_PAYLOAD_CODECS, GNSS_SENSOR_PLUGINS
    from .imu import IMU_PAYLOAD_CODECS, IMU_SENSOR_PLUGINS
    from .infrared import INFRARED_PAYLOAD_CODECS, INFRARED_SENSOR_PLUGINS

    for descriptor in (*IMU_SENSOR_PLUGINS, *INFRARED_SENSOR_PLUGINS, *GNSS_SENSOR_PLUGINS):
        plugins.register(descriptor)
    for codec in (*IMU_PAYLOAD_CODECS, *INFRARED_PAYLOAD_CODECS, *GNSS_PAYLOAD_CODECS):
        codecs.register(codec)
    ####


__all__ = ["register_builtin_sensor_plugins"]
####

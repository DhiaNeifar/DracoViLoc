"""Loads odas_ros.lib_odas_server_node for unit tests without a ROS 2 environment.

If rclpy (and the message packages) are importable they are used as-is;
otherwise minimal stubs are injected into sys.modules so the ROS-free classes
(PcmFrameExtractor, convert_sst_snapshot, PairCoordinator) can be tested
without spinning rclpy.
"""

import os
import sys
import types


def _stub_module(name):
    module = types.ModuleType(name)
    sys.modules[name] = module
    return module


def _ensure_ros_importable():
    try:
        import rclpy  # noqa: F401
        import rclpy.node  # noqa: F401
        import rclpy.time  # noqa: F401
        import libconf  # noqa: F401
        import odas_ros_msgs.msg  # noqa: F401
        import audio_utils_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass

    rclpy = _stub_module('rclpy')
    rclpy.ok = lambda: True

    node_mod = _stub_module('rclpy.node')

    class _Node:
        def get_logger(self):
            raise NotImplementedError

        def create_publisher(self, *args, **kwargs):
            raise NotImplementedError

        def create_subscription(self, *args, **kwargs):
            raise NotImplementedError

        def get_clock(self):
            raise NotImplementedError

    node_mod.Node = _Node
    rclpy.node = node_mod

    time_mod = _stub_module('rclpy.time')

    class _Time:
        def __init__(self, seconds=0, nanoseconds=0, clock_type=None):
            self._seconds = seconds

        def to_msg(self):
            return self._seconds

    time_mod.Time = _Time
    rclpy.time = time_mod

    _stub_module('libconf')

    odas_ros_msgs = _stub_module('odas_ros_msgs')
    odas_msgs = _stub_module('odas_ros_msgs.msg')

    class _Msg:
        pass

    for class_name in ('OdasSst', 'OdasSstArrayStamped', 'OdasSsl', 'OdasSslArrayStamped'):
        setattr(odas_msgs, class_name, type(class_name, (_Msg,), {}))
    odas_ros_msgs.msg = odas_msgs

    audio_utils_msgs = _stub_module('audio_utils_msgs')
    audio_msgs = _stub_module('audio_utils_msgs.msg')
    audio_msgs.AudioFrame = type('AudioFrame', (_Msg,), {})
    audio_utils_msgs.msg = audio_msgs


_ensure_ros_importable()

# Make the odas_ros package importable when running pytest from src/odas_ros.
_PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

import odas_ros.lib_odas_server_node as lib_odas_server_node  # noqa: E402

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# USB vendor/product IDs for o3p cameras (mirrors UsbCommon.hpp).
_O3P_USB_VID = '1c28'
_O3P_USB_PID = 'f003'


def _discover_serials():
    """Return sorted list of o3p camera serial numbers via sysfs (Linux only).

    Iterate /sys/bus/usb/devices,
    match idVendor/idProduct, read the serial attribute.
    """
    serials = set()
    base = '/sys/bus/usb/devices'
    try:
        for name in os.listdir(base):
            dev = os.path.join(base, name)
            try:
                vid = open(os.path.join(dev, 'idVendor')).read().strip().lower()
                pid = open(os.path.join(dev, 'idProduct')).read().strip().lower()
                if vid == _O3P_USB_VID and pid == _O3P_USB_PID:
                    serial_path = os.path.join(dev, 'serial')
                    if os.path.exists(serial_path):
                        serials.add(open(serial_path).read().strip())
            except OSError:
                continue
    except OSError as exc:
        print('[multi_camera.launch.py] Could not read {}: {}'.format(base, exc),
              file=sys.stderr)
    return sorted(serials)


def launch_setup(context, *args, **kwargs):
    # Comma separated list of serial numbers, one per camera to open.
    serials = [s.strip() for s in LaunchConfiguration('serial_nos').perform(context).split(',') if s.strip()]
    # Optional comma separated list of names. Falls back to camera1, camera2, ...
    names = [n.strip() for n in LaunchConfiguration('camera_names').perform(context).split(',') if n.strip()]

    if not serials:
        serials = _discover_serials()
        if not serials:
            print('[multi_camera.launch.py] No o3p cameras found; no nodes will be launched.',
                  file=sys.stderr)

    nodes = []
    for index, serial in enumerate(serials):
        camera_name = names[index] if index < len(names) else 'camera{}'.format(index + 1)
        nodes.append(
            Node(
                package='o3p_node',
                executable='camera_node',
                name='o3p_camera_{}'.format(index + 1),
                output='screen',
                parameters=[{
                    'serial_no': serial,
                    'camera_name': camera_name,
                }],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_nos',
            default_value='',
            description='Comma separated list of camera serial numbers to open, one node per serial. '
                        'When empty, all connected cameras are opened automatically.',
        ),
        DeclareLaunchArgument(
            'camera_names',
            default_value='',
            description='Optional comma separated list of names used as topic/TF prefixes. '
                        'Defaults to camera1, camera2, ... when omitted.',
        ),
        OpaqueFunction(function=launch_setup),
    ])

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/psd/SmartDrive_project/psd_ws/install/ros2_can_interface'

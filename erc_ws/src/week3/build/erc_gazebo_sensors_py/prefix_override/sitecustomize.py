import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arnavb/erc_ws/src/week3/install/erc_gazebo_sensors_py'

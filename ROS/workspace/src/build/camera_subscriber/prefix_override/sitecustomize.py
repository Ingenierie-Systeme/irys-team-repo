import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/noeca/ROS/workspace/src/install/camera_subscriber'

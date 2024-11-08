import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gerardo/turtlebot3_group6v2/src/install/joint_state_publisher_custom'

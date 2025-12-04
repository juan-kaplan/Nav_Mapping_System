import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juan/Documents/fundamentos_robotica/destroyer_ws/install/state_machine'

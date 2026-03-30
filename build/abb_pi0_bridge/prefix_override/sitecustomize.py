import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/heng/workspace/ws_RAMS/install/abb_pi0_bridge'

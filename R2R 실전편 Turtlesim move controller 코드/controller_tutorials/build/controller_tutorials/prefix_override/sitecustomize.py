import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/woolim/Documents/for_ROS2_study/R2R 실전편 Turtlesim move controller 코드/controller_tutorials/install/controller_tutorials'

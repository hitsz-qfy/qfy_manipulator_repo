launch:
    move_total.launch can drive dynameixel to desired positions, waitting for trajectory.
    PS: ax for /dev/ttyUSB0 and mx for /dev/ttyUSB1

scripts:
    Then pub_joint.py to publish desired positions.
    trajectory_sub_client.py conduct trajectory planning and drive dynameixel to move.
   
Test:
    tf_test.py can sub tf_broadcaster and conduct inversekinematic computation.
    tf_test_pub.py can pub grasping pose(position and orientation)

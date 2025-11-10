from ModelServer import go2_foxy, go2_noetic
import time


# go2_foxy.set_gripper(1.0)
# time.sleep(1)
# go2_foxy.set_gripper(0.0)
go2_noetic.set_arm([0,-1.5,2,0,0,0], wait=True)
go2_noetic.set_arm_default()
# go2_foxy.stand()
# time.sleep(1)
# go2_foxy.sit()

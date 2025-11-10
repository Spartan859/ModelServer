import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient


ChannelFactoryInitialize(0, 'eth0')

sport_client = SportClient()
sport_client.SetTimeout(5.0) # timeout
sport_client.Init()

msc = MotionSwitcherClient()
msc.SetTimeout(5.0)
msc.Init()

_, result = msc.CheckMode()
while result['name']:
    sport_client.StandDown()
    msc.ReleaseMode()
    _, result = msc.CheckMode()
    time.sleep(1)
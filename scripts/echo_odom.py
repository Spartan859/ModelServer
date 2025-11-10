from ModelServer import go2_foxy
import time

while True:
    try:
        odom = go2_foxy.get_odom()
        print(odom)
    except Exception as e:
        print(e)
    time.sleep(0.4)
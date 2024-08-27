import time

class Pump():
    
    def pump_on():
        mc.set_basic_output(5, 0)
        time.sleep(0.05)

    def pump_off():
        mc.set_basic_output(5, 1)
        time.sleep(0.05)
        mc.set_basic_output(2, 0)
        time.sleep(1)
        mc.set_basic_output(2, 1)
        time.sleep(0.05)
from sensors.vl53l4cd import VL53L4CD
from machine import I2C, Pin

i2c = I2C(0, sda=Pin(4), scl=Pin(5))

print(i2c.scan())

vl53 = VL53L4CD(i2c)

#vl53.inter_measurement = 0
vl53.timing_budget = 1000 // 10

print("VL53L4CD Simple Test.")
print("--------------------")
model_id, module_type = vl53.model_info
print("Model ID: 0x{:0X}".format(model_id))
print("Module Type: 0x{:0X}".format(module_type))
print("Timing Budget: {}".format(vl53.timing_budget))
print("Inter-Measurement: {}".format(vl53.inter_measurement))
print("--------------------")

vl53.start_ranging()

import time

while True:
    start_time = time.ticks_ms()
    while not vl53.data_ready:
        pass
    end_time = time.ticks_ms()
    vl53.clear_interrupt()
    status = vl53.range_status()
    if status != 4:
        print("Distance: {} cm, Status {}, Taken {} ms".format(vl53.distance, status, time.ticks_diff(end_time, start_time)))
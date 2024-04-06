from sensors.vl53l4cd import VL53L4CD
from machine import I2C, Pin

i2c = I2C(0, sda=Pin(4), scl=Pin(5))

RIGHT_TOF = 0
FRONT_TOF = 1
LEFT_TOF = 2

xshut_pins = [Pin(0, Pin.OUT), Pin(1, Pin.OUT), Pin(2, Pin.OUT)]

# Shut all the ToF sensors down
for xshut in xshut_pins:
    xshut.low()
    
# Confirm there are no I2C devices on the bus
print(i2c.scan())


tof_sensors = []

for i, xshut in enumerate(xshut_pins):
    xshut.high()
    vl53 = VL53L4CD(i2c)
    vl53.set_address(0x41 + 1 + i)

    #vl53.inter_measurement = 0
    vl53.timing_budget = 1000 // 10
    tof_sensors.append(vl53)

for i, vl53 in enumerate(tof_sensors):
    print(f"VL53L4CD[{i}] Simple Test.")
    print("--------------------")
    model_id, module_type = vl53.model_info
    print("Model ID: 0x{:0X}".format(model_id))
    print("Module Type: 0x{:0X}".format(module_type))
    print("Timing Budget: {}".format(vl53.timing_budget))
    print("Inter-Measurement: {}".format(vl53.inter_measurement))
    print("--------------------")

    vl53.start_ranging()

while True:
    for i, vl53 in enumerate(tof_sensors):
        if vl53.data_ready:
            vl53.clear_interrupt()
            status = vl53.range_status()
            if status != 4:
                if i == 0:
                    print()
                print(f"[{i}] Distance: {vl53.distance} cm, Status {status}", end=",")

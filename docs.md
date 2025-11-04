./waf configure --debug

./waf configure --board sitl
./waf copter


./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map --speedup 1 -w --out udp:127.0.0.1:14550


./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad \
  --console --map --out udp:127.0.0.1:14550 \
  -C "mode GUIDED; arm throttle; takeoff 5"


# from ardupilot/ root
./waf configure --board fmuv2
./waf copter      # or: plane, rover, sub, etc.


./waf configure --board Pixhawk1     # or: --board fmuv3
./waf copter


modules/mavlink/message_definitions/v1.0/ardupilotmega.xml


      <entry value="31" name="COPTER_MODE_SEEK">
        <description>SEEK</description>
      </entry>




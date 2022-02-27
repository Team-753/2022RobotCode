from networktables import NetworkTables
import threading
import time
cond = threading.Condition()
notified = [False]
connected = False
def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        connected = True
        sd = NetworkTables.getTable('SmartDashboard')
        while True:
            print(sd.getNumber("Test", 70))
            time.sleep(1)
        
NetworkTables.initialize(server='192.168.1.20') # roborio-753-frc.local
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print('Waiting')
    if not notified[0]:
        cond.wait()

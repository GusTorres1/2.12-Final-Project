from odrive_ros import odrive_interface
od = odrive_interface.ODriveInterfaceAPI()

from time import sleep

def runTwoMotors():
    od.connect(serial_number = od.usb_serials[0])
    od.calibrate()
    
    od.full_init()
    od.connect(serial_number = od.usb_serials[0])
    
    od.drivePos(400000, 400000)
    
    sleep(10)

def runAllMotors():
    '''
    TODO: Test this, dawg.
    '''
    od.connect_all()
    od.calibrate()
    
    #od.full_init()
    #od.connect_all()
    
    od.trajMoveCnt((400000, 400000, 400000))
    sleep(10)
    
    od.trajMoveCnt((0,0,0))
    sleep(10)
    
runAllMotors()

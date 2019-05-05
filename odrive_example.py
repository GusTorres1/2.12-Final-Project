from odrive_ros import odrive_interface
od = odrive_interface.ODriveInterfaceAPI()

od.connect(serial_number = '2086378C3548')
od.calibrate()

# Once the above works:
# uncomment the below:

od.full_init()
od.connect(serial_number = '2086378C3548')

od.drivePos(400000, 400000)

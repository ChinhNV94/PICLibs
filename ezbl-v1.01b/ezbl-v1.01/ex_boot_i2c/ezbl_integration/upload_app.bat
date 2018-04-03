java -jar "%~dp0ezbl_tools.jar" --communicator --communicator -com=I2C -slave_address=0x60 -baud=400000 -timeout=1000 -artifact="%1"
@pause
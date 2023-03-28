
device = serialport('COM13', 9600, 'Timeout',5);
pause(1);
write(device, "M123123123123", "string");
device = [];
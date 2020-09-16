# Communication of Serial port and CAN 

- Test versions, please make changes as needed.

## Device Settings

- The device works in passthrough mode.
- If you want to change the working mode, You need to short-circuit resistance and use 9600 baud rate. Then, you can make configuration by instruction.

## Communication

- Test process

  Send the test signal of the detection module
  ```bash
  rosrun port_serial test_det2con.py
  ```
  Send and receive data

  ```bash
  rosrun port_serial serial_communication.py
  ```
  
  The test control msg msg type is string, the specific type need to be changed later as needed.

- Select the serial port as required

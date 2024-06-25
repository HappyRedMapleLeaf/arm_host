## Setting Up USB Connection for WSL2
https://learn.microsoft.com/en-us/windows/wsl/connect-usb
0. connect stm32
1. open powershell with admin
2. `usbipd list` and find busid (format %d-%d)
3. `usbipd bind --busid <busid>`
4. `usbipd attach --busid <busid>` (on the second, etc. time around, if plugged into the same port, you just have to do this)
5. verify in wsl that it's there by running `lsusb`
6. detatch with `usbipd detach --busid <busid>` in powershell

Sometimes the device randomly detaches itself but it still shows up with `lsusb`, and shows up twice if re-attached. Just `wsl --shutdown` if this happens. Not sure why it does.

## Using the `serial_comms` ROS2 package
`serial_setup` node runs once and sets up usb device for proper communication
`read` node prints any data received from the microcontroller. It uses printf because it works for now. This might break things later.
`write` sends data to update the desired position based on requests sent to its server
`key_teleop` sends requests to update the desired position based on keystrokes
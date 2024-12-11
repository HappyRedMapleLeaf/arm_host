## Setting Up USB Connection for WSL2
https://learn.microsoft.com/en-us/windows/wsl/connect-usb
0. connect stm32
1. open powershell with admin
2. `usbipd list` and find busid (format %d-%d)
3. `usbipd bind --busid <busid>`
4. `usbipd attach --wsl --busid <busid>` (on the second, etc. time around, if plugged into the same port, you just have to do this)
5. verify in wsl that it's there by running `lsusb`
6. remember to  `ros2 run arm serial_setup`
7. detatch with `usbipd detach --busid <busid>` in powershell

Sometimes the device randomly detaches itself but it still shows up with `lsusb`, and shows up twice if re-attached. Just `wsl --shutdown` if this happens. Not sure why it does.

## Flashing code
I like keeping my project on the windows side so that I can use the CubeIDE if I really want to, and so I build on Windows as well with `cd Release; make -j16 all`
However I want to keep the usb connection on the WSL side when pushing code so I do that with OpenOCD. See run.bat file in arm_controller repo.

## Using the `arm` ROS2 package
`serial_setup` node runs once and sets up usb device for proper communication
`read` node prints any data received from the microcontroller. It uses printf because it works for now. This might break things later.
`write` sends data to update the desired position based on requests sent to its server
`key_teleop` sends requests to update the desired position based on keystrokes

## Setting up ROS
`colcon build` creates build, install, and log directories
`source install/setup.sh` to be able to run nodes
Added .vscode directory since intellisense was giving some problems and the current configuration fixes it

`ros2 run arm read` to print output from stm
`ros2 run arm write` to write whenever requested
`ros2 run arm servo_set` to set servo position with keyboard

## Setting up RViz
`ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map world`

## Dependencies
sudo apt-get install libncurses5-dev libncursesw5-dev
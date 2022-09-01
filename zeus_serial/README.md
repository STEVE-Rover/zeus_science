# zeus_serial

## Launch Serial Arduino
1. Find USB port on Computer
`lsusb` or `ls /dev`

You should see alot of devices, one must be ACMX or USBX.

2. Change it inside the launch file.

3. If the device cant be opened (permission denied) you need to set permissions to read and write for your device. To do that you need to create a rule. (This also sets permissions for world to read and write, which you may not want to do - (stack_overflow)[https://stackoverflow.com/questions/27858041/oserror-errno-13-permission-denied-dev-ttyacm0-using-pyserial-from-pyth])

```bash
# navigate to rules.d directory
cd /etc/udev/rules.d
# create a new rule file
sudo touch my-newrule.rules
# open the file
sudo nano my-newrule.rules
# add the following
KERNEL=="ttyACM0", MODE="0666"
```


## Generate message file header for arduino library
The arduino library might need custom messages to use publishers and subscribers. To generate a header msg file, you need to run a node call make_library.py from rosserial_arduino.
```bash
rosrun rosserial_arduino make_libraries.py <path_to_folder_where_libraries_headers_are_created> <path_to_msg_folder>
```

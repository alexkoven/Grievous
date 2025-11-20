# Creating Persistent Udev Names for Motor Controllers

This guide shows how to give each USB motor controller a stable device
name on Ubuntu, so it no longer changes between `/dev/ttyACM*` numbers.

------------------------------------------------------------------------

## 1. Identify Each Controller's Serial Number

For each motor controller, run:

    lerobot-find-ports

Then identify which /dev/ttyACM* port the motor controller is running on and run:

    udevadm info -a -n /dev/ttyACM0 | grep serial

Record each device's serial number.

------------------------------------------------------------------------

## 2. Create the Udev Rules File

Create a new rules file:

    sudo nano /etc/udev/rules.d/99-motor-controllers.rules

Add one line per controller, using only the **serial** field:

    SUBSYSTEM=="tty", ATTRS{serial}=="###", SYMLINK+="follower_left"
    SUBSYSTEM=="tty", ATTRS{serial}=="###", SYMLINK+="leader_left"
    SUBSYSTEM=="tty", ATTRS{serial}=="###", SYMLINK+="follower_right"
    SUBSYSTEM=="tty", ATTRS{serial}=="###", SYMLINK+="leader_right"

Save and exit.

------------------------------------------------------------------------

## 3. Reload Udev Rules

Apply your changes:

    sudo udevadm control --reload-rules
    sudo udevadm trigger

You may need to unplug and replug the controllers.

------------------------------------------------------------------------

## 4. Verify the Symlinks

List the created symlinks:

    ls -l /dev/follower_left /dev/follower_right /dev/leader_left /dev/leader_right

You should see output similar to:

    /dev/follower_left  -> ttyACM0
    /dev/follower_right -> ttyACM1
    /dev/leader_left    -> ttyACM3
    /dev/leader_right   -> ttyACM2

------------------------------------------------------------------------

## 5. Use the Stable Device Names

You can now always refer to the controllers using:

    /dev/follower_left
    /dev/follower_right
    /dev/leader_left
    /dev/leader_right

These will not change, regardless of USB port order or enumeration.

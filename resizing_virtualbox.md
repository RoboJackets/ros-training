# How to increase the size of the virtualbox disk
Due to a mishap by me, the IGVC hard drive sizes are only 10GB large (which is a bit too small).

To increase the size of the hard drive, we can make use of utilities that Virtualbox provides.

## Finding where the hard drive is located
1. Turn off the VM (if it's on).
2. Click on Settings -> Storage -> Storage Devices -> `igvc-XXXXXX.vdi` (the name might be different)
3. Take a note of the "Location" field

- [Windows](#windows)
- [Mac](#mac)

## Windows
1. Open the file explorer to the location of the "Location" field from just now
2. Open a command prompt
3. Paste in `"C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" modifyhd ` (Don't hit enter yet)
4. Drag the `igvc-XXXXXXX.vdi` file from the file explorer into the command prompt
5. Type in ` --resize 25000`
6. The entire command should be
    `"C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" modifyhd "C:\Users\[username]\VirtualBox VMs\igvc\igvcXXXX.vdi" --resize 25000`
7. Hit enter. The command should show something like `0%...10%...20%...30%...40%...50%...60%...70%...80%...90%...100%`
8. Continue by following [these instructions](#after-resizing-the-hard-drive)

## Mac
1. Open finder to the location of the "Location" field from just now
2. Open terminal
3. Paste in `VBoxManage modifyhd --resize 25000 ` (Don't hit enter yet)
4. Drag the `igvc-XXXXXXX.vdi` file from finder into the command prompt
6. The entire command should be
    `VBoxManage modifyhd --resize 25000 "/Users/[username]/VirtualBox VMs/igvc/igvcXXX.vdi"`
7. Hit enter. The command should show something like `0%...10%...20%...30%...40%...50%...60%...70%...80%...90%...100%`
8. Continue by following [these instructions](#after-resizing-the-hard-drive)

## After resizing the hard drive
1. Start the Virtualbox VM
2. Open terminal
3. Type in `sudo apt install gparted`
    - It'll show something like `[sudo] password for robojackets`. Type in `robojackets`. It's normal that nothing shows
    when you type, so just type out the password normally and hit enter even though you don't see anything
4. When it finishes installing, type in `gparted` in terminal. It (may) ask you for your password. Just type in `robojackets`.
    You should see a GUI screen pop up
    ![](https://www.dedoimedo.com/images/computers/2009/gparted-start-with-extended.jpg)
5. Click the partition on the left, and then hit `Resize/Move`
6. Type in `0` for the `Free space following (MiB)`, and click the `Resize` button
7. Hit the green checkmark at the top. Hopefully a loading bar will show, and you should see the partition take up all
    the space.

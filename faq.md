# FAQ
This document contains a list of common problems and solutions encountered during ROS training:

- [Ubuntu is complaining that my hard drive is running out of space](#ubuntu-is-complaining-that-my-hard-drive-is-running-out-of-space)
- [The compiler is complaining that "virtual memory exhausted"](#the-compiler-is-complaining-that-virtual-memory-exhausted)
- [CLion isn't working with ROS (If you're dual booting)](#clion-isnt-working-with-ros-if-youre-dual-booting)

## Ubuntu is complaining that my hard drive is running out of space
I screwed up the virtualbox image, and it's only 10GB (which isn't big enough).
See [How to increase the size of the virtualbox disk](resizing_virtualbox.md) on how to resize the virtualbox
image.

## The compiler is complaining that `virtual memory exhausted`
- The VM image by default only allocate 4GB, which is not enough
    - If your computer has 8GB, give the VM 6 GB (6000 MB)
    - If your computer has 16 GB, give the VM 12 GB (12000 MB)
    - You can change the amount of memory allocated by going to Settings -> System and adjusting the memory
- This may also be caused by the hard drive running out of space. See
[Ubuntu is complaining that my hard drive is running out of space](#ubuntu-is-complaining-that-my-hard-drive-is-running-out-of-space)
- If this still doesn't work, try doing `catkin_make -j2`, which will reduce the amount of memory used to compile.

## CLion isn't working with ROS (If you're dual booting)
- Install the `Hatchery` and `ROS-Robot Operating System` plugins
    - File -> Settings -> Plugins -> Search ROS, and click install for the two above plugins
- Restart Clion
- When Clion restarts, hit File -> Close Project
- Hit the 'X' to the right of the current project to remove it
- Click the "Import ROS Workspace" option, navigate to the `catkin_ws` folder, and click open
    

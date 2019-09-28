---
title: Week 2.5
---
# Week 2.5

---

## Outline
- Package Structure
- Catkin_make
- Dual-boot
- CLion

---
## ROS top-level Structure
- catkin_ws/ must be in home folder (~/)
- Consists of build/, devel/, and src/ folders
- build/ and devel/ folders created after running `catkin_make`

```
~/catkin_ws/              --WORKSPACE
    build/                --BUILD SPACE(cmake files)
    devel/                --DEVEL SPACE (binaries and runtime libraries)
    src/                  --SOURCE SPACE
        CMakeLists.txt    --Top-level CMakeLists file
        package_1/        --Packages
        ...
        package_n/
```

---
## Packages

- Examples include: `rr_common`, `rr_iarrc`, `igvc_training_exercises`
- Used in `rosrun <Package> <Executable>`
- Each package must have its own folder
- The package must contain a package.xml and CMakeLists.txt file
- Both files mainly consist of build configuration information 

```
my_package/
      CMakeLists.txt        --Used by `catkin_make` to compile
      package.xml           --Provides meta information
      other_folders/
```
---

## Packages in catkin Workspace
- Inside src/, we have the two cloned github repositories (roboracing-software, ros-training)
- Inside these repositories are multiple packages

```
catkin_ws/                    -- WORKSPACE
   src/                       -- SOURCE SPACE
     CMakeLists.txt           -- Top-level CMake file, provided by catkin
     roboracing-software/
         package_1/
           CMakeLists.txt     -- CMakeLists.txt file for package_1
           package.xml        -- Package manifest for package_1
         ...
         package_n/
           CMakeLists.txt     -- CMakeLists.txt file for package_n
           package.xml        -- Package manifest for package_n
      ros-training/
```
---
## catkin_make
- **Important Note**: `catkin_make` must be ran within catkin_ws/, (**not in a subfolder**)
- Need to compile every time you change a file and want to run something
- If you delete the build/ and devel/ folders it will do a full build
- If you edit a dependency file (ex. CMakeLists.txt) or change git branches it also do a full build
- If you only change some files, those files will be colored green in the terminal when linked
- Compilation errors will be in red
- If you run it and only see white 'Built target', then nothing has changed

---
## What Returning Members Use

How do we run multiple OS (Operating Systems)?
- Mostly everyone on the subteam uses dual-boots
    - Faster and more reliable than VMs
    - Partitioning resources isn't great for either OS
    - Better for low-end laptops (little RAM and few CPU cores)
    - More long-term
    - Requires some trouble shooting
- Most commonly, people start with VMs then transition to dual-boot

What IDE do we use?
- We normally use CLion because it integrates pretty well with ROS
- Use free educational pack for JetBrains
    - Applying is very fast and easy

---
## CLION
Why you should probably use it:
- Auto-complete (**EXTREMELY** useful)
- Error Highlighting 
    - Better than normal editors because it knows about ROS libraries
- Code style recommendations 
- `catkin_make` with keyboard shortcut
- Automatic Formatting
- Connects with git 
- Multiple terminal access
- Allows you to use other JetBrain products (IntelliJ, Pycharm, etc.)
- Most of the team uses it 
- Pretty great overall

---


Questions!

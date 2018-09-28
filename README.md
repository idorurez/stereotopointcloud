# Usage

Syntax is: 
    ./jpointcloud input_stereo.exr input_depthmap.exr output_left_eye.ply/combined.ply output_right.ply <options>

Where options are:
- -ascii (default: binary)
- -use_camera 1/0 (default: false)
- -separate = write out as separate or combined ply (default: combined)

# Build 

jPointcloud is built statically into linux/bin.

Due to the sheer massive size when built statically, these libraries are not included. 
Please be aware you'll need to statically build or source these libraries if you intend to build jpointcloud
on your own.

- **boost** [Getting Started with Unix Variants](http://www.boost.org/doc/libs/1_61_0/more/getting_started/unix-variants.html)
-- Be sure to use the correct gcc version that is supported by the verison of boost you are running
-- mpicc is also required, and the resultant install path should be added to PATH before building boost libraries.
- **flann** (Most package distros include a static version of flann)
- **pcl** (Please note: http://www.pcl-users.org/How-to-build-PCL-statically-td4027660.html)

## Building PCL **
1. clone pcl from the [PCL Github Repo](https://github.com/PointCloudLibrary/pcl)

2. edit CMakeFiles.txt and insert this at the top:
```
set(PCL_SHARED_LIBS OFF) 
set(Boost_USE_STATIC ON) 
set(BUILD_SHARED_LIBS OFF)
```

3. Edit CMakeCache.txt and make sure:
```
PCL_SHARED_LIBS:BOOL=OFF
```
4. make && sudo make install

# Acknowledgements and Contributions

Thanks to Chris Redmann, Philip Lee, and Matt Hu 

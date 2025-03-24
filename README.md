## Compile
1. Make sure you have installed K4A SDK  
2. put rgbd_azure_kinect_dk.cc and KinectDKRectifiedRealtime.yaml into ORBSLAM3/Example/RGB-D/
   put mono_azure_kinect_dk.cc and KinectDKRectifiedRealtime.yaml into ORBSLAM3/Example/Monocular/  
2. Add to your CMakeLists in ORB_SLAM3:
```cmake
find_package(k4a)
find_package(k4arecord)
set(K4A_LIBS k4a::k4a;k4a::k4arecord)
include_directories(
    ${PROJECT_NAME}
    ${K4A_INCLUDE_DIR}
    ${K4ARECORD_INCLUDE_DIR}
)
target_link_libraries(
    ${PROJECT_NAME}
    ${K4A_LIBS}
)

add_executable(mono_azure_kinect_dk
            Examples/Monocular/mono_azure_kinect_dk.cc)
    target_link_libraries(mono_azure_kinect_dk ${PROJECT_NAME})

add_executable(rgbd_azure_kinect_dk
            Examples/RGB-D/rgbd_azure_kinect_dk.cc)
target_link_libraries(rgbd_azure_kinect_dk ${PROJECT_NAME})
```
3. run build.sh in ORBSLAM3  

## Usage
```bash
./rgbd_azure_kinect_dk path_to_vocabulary path_to_settings (trajectory_file_name)
```
For example, use the config file KinectDKRectifiedRealtime.yaml
```bash
Examples/RGB-D/rgbd_azure_kinect_dk Vocabulary/ORBvoc.txt KinectDKRectifiedRealtime.yaml
```
before running the command, ensure the Kinect camera is properly connected and drivers are installed.  

## Notice
Three new parameters used for Kinect devices are added into the config file:
```yaml
Kinect.depthmode: "nfov"
Kinect.resolution: 720
Adjusted: 0
```
Open KinectDKRectifiedRealtime.yaml to see valid values.  
Each time you change Kinect.depthmod or Kinect.resolution, you should set Adjusted to 0.  
Run the program once, it will print new Camera width, height, fx, cx, fy, cy.  
Change Camera1.fx Camera1.cx Camera1.fy Camera1.cy Camera.width Camera.height to proper value and set Adjusted to 1.  
Then you can run this program.  

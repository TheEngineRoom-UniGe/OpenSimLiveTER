# OpenSimLive

- [Dependencies](#dependencies)
  * [Installing](#installing)
- [Similar projects](#similar-projects)
<!-- toc -->


OpenSimLive is a C++ package that streams orientation data from inertial measurement units and calculates inverse kinematics based on that data. It relies on OpenSim for biomechanical analyses and related tools. The current version uses [OpenSim 4.1 API](https://simtk.org/api_docs/opensim/api_docs/index.html). Two types of IMUs are currently supported: Xsens MTw Awinda and Delsys Trigno Avanti. Xsens IMUs use XDA 4.6 and Delsys IMUs use Delsys Trigno Control Utility.

Some of OpenSimLive's features can be tested without actual IMUs by using simulated IMU data that that OpenSimLive generates as random unit quaternions.

## dependencies
To install this package you need:
 XDA 4.6 from [the Xsens website](https://www.xsens.com/software-downloads) by downloading MT Software Suite under MTw Awinda
 Visual studio core build tools from 2017
 Opensim-Core version 4.1

./TER-osim folder contains the model for a hand with 4 DoF in each finger, 5 for the thumb and 3 for the wrist and the relative constraints.


### Installing

Follow the guide here to install Opensim-core 4.1:	https://github.com/mitkof6/opensim-core/tree/bindings_timestepper#on-ubuntu-using-unix-makefiles
Step by step instructions on how to install this project.

#### Windows (64-bit)

1. Download and unzip the package to a directory on your hard drive.
2. Open CMake and select the directory from the previous step as the source code directory.
- Put YourFilePath/YourSourceCodeFolder-build or whatever else you want as the build folder and allow CMake to create a new folder when prompted.
- Select **x64** as the generator when prompted.
- Select **Configure**. CMake variables and their values should now be displayed. If any of them are not found, you can manually type in folders. An example file path of each is as follows:
   - CMAKE_CONFIGURATION_TYPES  Debug;Release;MinSizeRel;RelWithDebInfo
   - CONFIG_PATH:               C:/Users/YourUserHere/Documents/OpenSimLive/Config
   - INCLUDE_CLASSES_PATH:      C:/Users/YourUserHere/Documents/OpenSimLive/Classes
   - INCLUDE_FUNCTIONS_PATH:    C:/Users/YourUserHere/Documents/OpenSimLive/Functions
   - MTSDK_PATH:                C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/include
   - MT_LIB_PATH:               C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/lib
   - OPENSIM_INCLUDE_PATH:      C:/OpenSim 4.1/sdk/include/OpenSim
   - OPENSIM_INCLUDE_PATH_TWO:  C:/OpenSim 4.1/sdk/include
   - OPENSIM_LIB_PATH:          C:/OpenSim 4.1/sdk/lib
   - SIMBODY_INCLUDE_PATH:      C:/OpenSim 4.1/sdk/Simbody/include
   - SIMBODY_LIB_PATH:          C:/OpenSim 4.1/sdk/Simbody/lib
   - PYTHON_LIB:				  C:/Users/YourUserHere/AppData/Local/Programs/Python38/libs
   - PYTHON_PATH:				  C:/Users/YourUserHere/AppData/Local/Programs/Python38
- The two last entries are not required unless you wish to plot EMG data with Delsys IMUs.
- Finally, select **Generate**.
3. Open Visual Studio. Open the solution you just generated in the build directory. Make sure that the solution configuration in the top bar is set to RelWithDebInfo or Release instead of Debug, as Debug builds will not work without including the external .pdb debug information databases. Build **ALL_BUILD**. Visual Studio should now create the required executable(s) in a subdirectory in the build directory. It will probably be **.../BuildFolderName/MirrorTherapy/RelWithDebInfo/** for mirror therapy applications and **.../BuildFolderName/Tests/RelWithDebInfo/** for tests.
4. Copy **xsensdeviceapi64.dll** and **xstypes64.dll** from .../Xsens/MT Software Suite 4.6/MT SDK/x64/lib to the directories where the executables are or add their locations to the *PATH* environmental variable. **This is required to run Xsens-related scripts, but is otherwise optional.**
5. Make sure **...\OpenSim 4.1\bin** is present in your *PATH* environmental variable. Otherwise you will receive an error message when trying to run any of the executables.
6. Go to **.../OpenSimLive/Config** and make sure the .xml files have the right values for your directory paths.
- You need to download an .osim model file to use with the program. You can find several models here: https://simtk-confluence.stanford.edu/display/OpenSim/Musculoskeletal+Models
- For testing, the gait2392 model and the Hamner full body model are recommended for lower-body and full-body kinematics, respectively.
7. Installation complete. You are ready to run OpenSimLive.

The original implementation of this code can be found at this link [OpenSimLive](https://github.com/jerela/OpenSimLive).
## Similar projects

For marker-based real-time inverse kinematics using OpenSim 3.3 API, see C. Pizzolato's [RTOSIM](https://github.com/RealTimeBiomechanics/rtosim/).
For marker and IMU-based real time kinematical and dynamical analysis using OpenSim 4.1 API, see D. Stanev's [OpenSimRT](https://github.com/mitkof6/OpenSimRT).

# Introduction to the O3P Python3 wrapper (O3PY)

## How to use

To use the O3P Python samples you need to install NumPy, Matplotlib (for the IMU visualization example) and OpenCV (for the OpenCV example).
This version of the Python wrapper was built against Python 3.10.12. To use it you will need to install Python with the same major and minor version. 
If you want to build your own wrapper with a different Python version, please refer to "How to build".

To use o3py in your applications you need to include these lines:  

```py
O3P_DIR = "PATH/TO/YOUR/O3P/INSTALLATION/python"
sys.path.append(O3P_DIR)

import o3py
```

We have a few samples that show how to use the wrapper.  

- **helloPython_o3p.py**: prints some information about the connected O3P device
- **opencvPython_o3p.py**: displays the different live frames collected with the device
- **imuVisPython_o3p.py**: shows a 2D visualization of the IMU data next to the rgb image
- **exportPlyPython_o3p.py**: exports a .ply file that holds the pointcloud of the current scene
- **multiDevicePython_o3p.py**: shows how to use multiple devices
- **deviceInfoPython_o3p.py**: shows how retrieve device specific information

There are some new functions available in Python that allow working with the collected data:
    - VideoStream.fromStream(Stream s) creates a VideoStream from a given Stream
    - VideoFrame.getData() get a python list containing the values in m_data
    - DepthFrame.getData() get the depth data as array
    - PointcloudFrame.getData() get the pointcloud as array
    - PluginFrame.getData() same as above
    - ImuFrame.getAcceleration() returns a list containing the current acceleration in x, y and z direction
    - ImuFrame.getAngularVelocity() returns a list containing the current angular velocities in x, y and z direction

## How to build

To build the Python wrapper you need to install the Python libraries and SWIG (Version >= 4.2.0, install manually) and make sure that they are found in CMake.
If no debug Python library is found, the debug wrapper will link against the release library.  

Proceed as follows:

- start CMake GUI
- select the python/swig subfolder as source and choose a build folder
- hit 'Configure' and select your compiler
- hit 'Generate'
- use the generated solution file to build the wrapper  

## Environment

You can use `o3py_requirements.txt` to setup a virtual python environment with all required packages.

- `python -m venv "O3PY_Env"`
- `pip install -r o3py_requirements.txt`
- activate the virtual environment afterwards

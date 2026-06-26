#!/usr/bin/python3

# Copyright (C) 2026 pmdtechnologies gmbh
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import time 
import cv2 as cv 
import numpy as np

import o3py

# Map the depth values to [0, 255]
def adjustZValues(zImage):
    adjustedZImage = np.zeros((zImage.shape))
    max = 8000
    for i in range(zImage.shape[0]):
        for j in range(zImage.shape[1]):
            adjustedZImage[i][j] = np.log(zImage[i][j] + 1) * 255 / np.log(max + 1) # log for nicer visualization
    return adjustedZImage

# create, initialize and start the pipeline
pipe = o3py.Pipeline()
pipe.init()
pipe.start()

start = time.time()
loop_time = 10

# for 10 seconds 
while time.time() - start < loop_time:
    try:
        frames = pipe.waitForFrames() # get frames

        # DEPTH
        depth = frames.getDepthFrame()
        w = depth.getWidth()
        h = depth.getHeight()
        data = depth.getData() # get data as python list
        if data.size == 0:
            continue
        else:
            zImage = adjustZValues(data.reshape(h, w))
            zImage8 = np.uint8(zImage)
            cv.convertScaleAbs(zImage, zImage8, 1.0)
            zImageRGB = cv.applyColorMap(zImage8, cv.COLORMAP_JET)
            cv.imshow("Depth Image", zImageRGB)

        # Infrared
        ir = frames.getInfraredFrame()
        w = ir.getWidth()
        h = ir.getHeight()
        data = ir.getData()
        if data.size == 0:
            continue
        else:
            irimage = cv.cvtColor(data.reshape(h, w), cv.COLOR_GRAY2BGR)
            cv.imshow("ir image", irimage)

        # RGB
        rgb = frames.getColorFrame()
        w = rgb.getWidth()
        h = rgb.getHeight()
        data = rgb.getData() 
        if data.size == 0:
            continue
        else:
            rgbImage = np.array(data, dtype=np.uint8).reshape((h*3//2, w))
            rgbImage = cv.cvtColor(rgbImage, cv.COLOR_YUV2BGRA_NV12)
            cv.namedWindow("RGB Image", 0)
            cv.resizeWindow("RGB Image", 640, 480)
            cv.imshow("RGB Image", rgbImage)

        # IMU
        imu = frames.getImuFrame()
        angVel = imu.getAngularVelocity() # get current angular velocities in x, y and z direction
        accel = imu.getAcceleration() # get current acceleration in x, y and z direction
        print("Angular Velocity: x:", angVel[0], " y:", angVel[1], " z:", angVel[2])
        print("Angular Velocity Timestamp: ", imu.angularTimestamp)
        print("Acceleration: x:", accel[0], " y:", accel[1], " z:", accel[2])
        print("Acceleration Timestamp: ", imu.accelerationTimestamp)

        # Plugin
        plugin = frames.getPluginFrame()
        data = plugin.getData()
        print("Plugin Frame Size (Bytes): ", plugin.getSize())

        print("=======================================================================================")
        cv.waitKey(1)
    except Exception as e:
        print(f"Error grabbing frame: {e}")

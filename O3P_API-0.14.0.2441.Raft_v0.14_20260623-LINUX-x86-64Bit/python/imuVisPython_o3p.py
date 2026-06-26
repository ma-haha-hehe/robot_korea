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

import sys
import time 
import matplotlib.pyplot as plt 
import numpy as np

import o3py

# convert the yuv data to rgb (without using opencv)
def convertToRGB(imgdata, w, h):
    y_size = w * h

    y = np.array(imgdata[:y_size], dtype=np.uint8).reshape((h, w))
    uv = np.array(imgdata[y_size:], dtype=np.uint8).reshape((h // 2, w))

    u = uv[:, 0::2].repeat(2, axis=0).repeat(2, axis=1)
    v = uv[:, 1::2].repeat(2, axis=0).repeat(2, axis=1)

    y = y.astype(np.float32)
    u = u.astype(np.float32) - 128
    v = v.astype(np.float32) - 128

    r = np.clip(y + 1.402 * v, 0, 255).astype(np.uint8)
    g = np.clip(y - 0.344136 * u - 0.714136 * v, 0, 255).astype(np.uint8)
    b = np.clip(y + 1.772 * u, 0, 255).astype(np.uint8)

    return np.stack([r, g, b], axis=-1)

# create, initialize and start pipeline
pipe = o3py.Pipeline()
pipe.init()
pipe.start()

# use plt's interactive mode
plt.ion()

# init figure
fig = plt.figure(figsize=(12, 4)) 

# axis that will hold the rgb image
ax_img = plt.subplot(1, 3, 1)
displayedImage = ax_img.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
ax_img.axis('off')
ax_img.set_title("RGB Image")

# axis that will hold the angular velocity plot
ax_plot = plt.subplot(1, 3, 2)
ax_plot.set_title("IMU Angular Velocity")
avX_line = ax_plot.plot([], [], c="red", label="Angular Velocity X")
avY_line = ax_plot.plot([], [], c="green", label="Angular Velocity Y")
avZ_line = ax_plot.plot([], [], c="blue", label="Angular Velocity Z")
ax_plot.legend()

# axis that will hold the acceleration plot
ax_plot2 = plt.subplot(1, 3, 3)
ax_plot2.set_title("IMU Acceleration")
acX_line = ax_plot2.plot([], [], c="red", label="Acceleration X")
acY_line = ax_plot2.plot([], [], c="green", label="Acceleration Y")
acZ_line = ax_plot2.plot([], [], c="blue", label="Acceleration Z")
ax_plot2.legend()

# show the plots
plt.tight_layout()
plt.show()

# set up values for the loop
start = time.time()
loop_time = 50
max_len = 20 

# init lists for the collected data
angVelX = []
angVelY = []
angVelZ = []
accelX = []
accelY = []
accelZ = []
angTime = []
accelTime = []

while time.time() - start < loop_time:
    try: 
        frames = pipe.waitForFrames() # wait for the frames

        # RGB
        rgb = frames.getColorFrame() # get the color frame
        w = rgb.getWidth()
        h = rgb.getHeight()
        data = rgb.getData()
        rgbImage = convertToRGB(data, w, h)
        displayedImage.set_data(rgbImage) # update the displayed image with the current frame

        # IMU
        imu = frames.getImuFrame() # get the imu frame

        # if max_len datapoints are collected, remove the oldest one
        if(len(angTime) >= max_len):
            angVelX.pop(0)
            angVelY.pop(0)
            angVelZ.pop(0)
            accelX.pop(0)
            accelY.pop(0)
            accelZ.pop(0)
            angTime.pop(0)
            accelTime.pop(0)

        # append the new datapoints to the lists
        angVelX.append(imu.getAngularVelocity()[0])
        angVelY.append(imu.getAngularVelocity()[1])
        angVelZ.append(imu.getAngularVelocity()[2])
        accelX.append(imu.getAcceleration()[0])
        accelY.append(imu.getAcceleration()[1])
        accelZ.append(imu.getAcceleration()[2])
        angTime.append(imu.angularTimestamp)
        accelTime.append(imu.accelerationTimestamp)

        # clear the plots before redrawing 
        ax_plot.clear()
        ax_plot2.clear()

        # plot the collected data
        avX_line = ax_plot.plot(angTime, angVelX, c="red", label="Angular Velocity X")
        avY_line = ax_plot.plot(angTime, angVelY, c="green", label="Angular Velocity Y")
        avZ_line = ax_plot.plot(angTime, angVelZ, c="blue", label="Angular Velocity Z")

        acX_line = ax_plot2.plot(accelTime, accelX, c="red", label="Acceleration X")
        acY_line = ax_plot2.plot(accelTime, accelY, c="green", label="Acceleration Y")
        acZ_line = ax_plot2.plot(accelTime, accelZ, c="blue", label="Acceleration Z")

        # redraw the figure
        fig.canvas.draw()
        fig.canvas.flush_events()
    except Exception as e:
        print(f"Error grabbing frame: {e}")

# turn interactive mode off 
plt.ioff()
    
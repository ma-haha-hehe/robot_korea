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
from o3py import Context, Pipeline, PipelineConfig

# Get available cameras
cameras = Context.getAvailableCameras()

if not cameras:
    print("No cameras found")
    exit(1)

print("Available cameras:")
for i, cam in enumerate(cameras):
    print(f"Camera {i} : {cam}")

# Create pipelines
pipelines = []

for cam in cameras:
    pipeline = Pipeline()
    config = PipelineConfig()
    config.enableDevice(cam)
    try:
        pipeline.init(config)
    except Exception as e:
        print(f"Error initializing camera: {e}")
        exit(1)

    pipeline.start()
    pipelines.append(pipeline)

# Run for 5 seconds
start_time = time.time()
duration = 5  # seconds

while time.time() - start_time < duration:
    for i, pipeline in enumerate(pipelines):
        try:
            frames = pipeline.waitForFrames()
            depth = frames.getDepthFrame()

            width = depth.getWidth()
            height = depth.getHeight()

            dist_to_center = depth.getDistance(width // 2, height // 2)

            print(f"Camera {cameras[i]} distance: {dist_to_center} millimeters away")
        except Exception as e:
            print(f"Error grabbing frame: {e}")

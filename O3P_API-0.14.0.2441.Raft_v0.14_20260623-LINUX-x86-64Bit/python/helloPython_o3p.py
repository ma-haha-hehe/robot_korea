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

import o3py

print("Hello from O3PY!")

# initialize a pipeline
pipe = o3py.Pipeline()

if len(sys.argv) > 1:
    # if a filename is given as argument, play back the file instead of connecting to a camera
    filename = sys.argv[1]
    config = o3py.PipelineConfig()
    config.setPlaybackFile(filename)
    # disable timestamps playback to get the frames as fast as possible, otherwise the frames will be played back with the same timing as they were recorded
    config.setUseTimestampsPlayback(False)
    profile = pipe.init(config)
else:
    # initialize the pipeline with default configuration, which will connect to the first available camera
    profile = pipe.init()

# retrieve the serial number
print("Serial number: ", profile.getSerialNumber())

# get the streams
streams = profile.getStreams()
print("Number of Streams: ", len(streams))

# print information about the streams
for i in range(0, len(streams)):
    if((streams[i].streamType() == o3py.O3P_STREAM_DEPTH) or (streams[i].streamType() == o3py.O3P_STREAM_COLOR)):
        vstream = o3py.VideoStream.fromStream(streams[i])
        lensParams = vstream.getIntrinsics()

        print("Stream: ", i)
        print("Type: ", o3py.streamType2String(streams[i].streamType()))
        print("Width: ", vstream.width(), " Height: ", vstream.height())
        print("Lens parameters: ")
        print("Principal point cx: ", lensParams.ppx, " cy: ", lensParams.ppy)
        print("Focal length fx: ", lensParams.fx, " fy: ", lensParams.fy)

        if lensParams.model in (o3py.PIN_HOLE, o3py.PIN_HOLE_REAL, o3py.PIN_HOLE_VIRTUAL):
            print("Distortion coefficients (Brown-Conrady: k1, k2, p1, p2, k3): ")
        elif lensParams.model == o3py.FISH_EYE:
            print("Distortion coefficients (F-Theta Fish-eye: k1, k2, k3, k4): ")
        else:
            print("Distortion coefficients (unknown model): ")
        coeffs = lensParams.getCoeffs()
        for j in range(5):
            print("coeffs[", j, "]: ", coeffs[j])

        extrinsics = vstream.getExtrinsics()
        print("Extrinsics:")
        rotation = extrinsics.getRotation()
        print("  Rotation matrix:")
        print(rotation)
        translation = extrinsics.getTranslation()
        print("  Translation [m]: ", translation[0], translation[1], translation[2])

# start the pipeline
pipe.start()

uc = pipe.getAvailableUseCases()
print("Available Use Cases:")
for i in range(len(uc)):
    print(i+1, ": ", uc[i])

print("Current Use Case is:", pipe.getUseCase())

start = time.time()
loop_time = 5

# for 5 seconds
while time.time() - start < loop_time:
    try:
        frames = pipe.waitForFrames() # get frames
        depth = frames.getDepthFrame() # get depth frame
        w = depth.getWidth() 
        h = depth.getHeight()

        dist_to_center = depth.getDistance(w//2, h//2) # get depth value of the middle pixel

        if depth.supportsMetadata(o3py.O3P_METADATA_TIMESTAMP):
            timestamp = depth.getMetadataUint64(o3py.O3P_METADATA_TIMESTAMP)
            print("Timestamp: ", timestamp)

        if depth.supportsMetadata(o3py.O3P_METADATA_FRAME_NUMBER):
            frame_number = depth.getMetadataUint32(o3py.O3P_METADATA_FRAME_NUMBER)
            print("Frame number: ", frame_number)

        if depth.supportsMetadata(o3py.O3P_METADATA_EXPOSURE_TIME_1):
            exposure_time_1 = depth.getMetadataUint32(o3py.O3P_METADATA_EXPOSURE_TIME_1)
            print("Exposure time 1: ", exposure_time_1, " microseconds")

        if depth.supportsMetadata(o3py.O3P_METADATA_TEMPERATURE):
            temperature = depth.getMetadataFloat(o3py.O3P_METADATA_TEMPERATURE)
            print("Temperature: ", temperature, " degrees Celsius")

        print("The camera is facing an object ", dist_to_center, " millimeters away.")
    except Exception as e:
        print(f"Error grabbing frame: {e}")

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

import cv2 as cv 
import numpy as np

import o3py

print("Do you want to record or play the last recording? (r/p)")

maxDepth = 8000 # maximum depth value in mm, used for visualization of the depth image

# Mode for record and play
mode = input("Choose Mode: [r] Record | [p] Play: ").lower()

if mode not in ["r", "p"]:
    print("Invalid Input!")
    exit()

# Map the depth values to [0, 255]
def adjustZValues(zImage):
    adjustedZImage = np.zeros((zImage.shape))
    for i in range(zImage.shape[0]):
        for j in range(zImage.shape[1]):
            adjustedZImage[i][j] = np.log(zImage[i][j] + 1) * 255 / np.log(maxDepth + 1) # log for nicer visualization
    return adjustedZImage
                
# create, initialize and start the pipeline
pipe = o3py.Pipeline()
config = o3py.PipelineConfig()

if mode == "p": 
    config.setPlaybackFile("recorded_file.bag")

    loop = input("Do you want to loop the Recording? (yes/no) ").lower()
    if loop == "yes":
        config.setLoopPlayback(True)
    if loop == "no":
        config.setLoopPlayback(False)

profile = pipe.init(config)
pipe.start()

if mode == "r":
    pipe.startRecording("recorded_file.bag")
elif mode == "p":
    numFrames = pipe.getNumberOfFramesInRecording()
    print("Number of frames in Recording: ", numFrames)

print("Press: [space] pause replay | [ESC/q] Quit")

# Create window early to ensure it has focus
cv.namedWindow("Depth Image", cv.WINDOW_AUTOSIZE)

while True:
    # Check for key input with 30ms timeout
    key = cv.waitKey(30) & 0xFF
    
    if key == ord(' '):  # Space key
        print("PAUSED")
        cv.waitKey(0)  # Wait indefinitely until any key is pressed
        print("RESUMED")
    
    if key == 27 or key == ord('q') or key == ord('Q'):  # ESC or Q key
        print("EXIT PROGRAM")
        break        
        
    try: 
        frames = pipe.waitForFrames()

        #DEPTH      
        depth = frames.getDepthFrame()
        w = depth.getWidth()
        h = depth.getHeight()
        data = depth.getData() # get data as python list
        if data.size == 0:
            continue
        else:
            zImage = np.array(data, dtype=np.uint16)
            zImage = zImage.reshape((h, w))

            # find the nearest point in the depth image, i.e. the point with the smallest non-zero depth value
            minDepth = maxDepth # set min to max depth value
            for i in range(0,h):
                for j in range(0,w):
                    if (zImage[i][j] > 0) and zImage[i][j] < minDepth:
                        minDepth = zImage[i][j]
                        idx = (i,j) 
            
            zImage = adjustZValues(zImage)
            zImage8 = np.uint8(zImage)
            cv.convertScaleAbs(zImage, zImage8, 1.0)
            zImageRGB = cv.applyColorMap(zImage8, cv.COLORMAP_JET)

            # draw a green circle around the nearest point in the colored depth image
            color = (0, 255, 0)
            zImageRGB[idx[0]][idx[1]] = color
            cv.circle(zImageRGB, (idx[1], idx[0]), radius = 10, color = color)

            cv.imshow("Depth Image", zImageRGB)

    except EOFError as e:
        print(f"Reached end of recording. Exiting.")
        break
    except Exception as e:
        print(f"Error grabbing frame: {e}")

if mode == "r":
    pipe.stopRecording()
pipe.stop()

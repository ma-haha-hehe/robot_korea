/****************************************************************************\
* Copyright (C) 2026 pmdtechnologies gmbh
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
\****************************************************************************/

#include "realsense-example-helper/rs-example.hpp"
#include <iostream>
#include <mutex>
#include <o3p/o3p.hpp>

// Simple cube model data: 8 vertices
static const float cubeVertices[] = {
    -2.5f, -2.5f, -2.5f, // 0
    2.5f, -2.5f, -2.5f,  // 1
    2.5f, 2.5f, -2.5f,   // 2
    -2.5f, 2.5f, -2.5f,  // 3
    -2.5f, -2.5f, 2.5f,  // 4
    2.5f, -2.5f, 2.5f,   // 5
    2.5f, 2.5f, 2.5f,    // 6
    -2.5f, 2.5f, 2.5f    // 7
};

// Each side is made up of two triangles
static const unsigned short cubeIndices[] = {
    0, 1, 2, 2, 3, 0, // back
    4, 5, 6, 6, 7, 4, // front
    0, 4, 7, 7, 3, 0, // left
    1, 5, 6, 6, 2, 1, // right
    3, 2, 6, 6, 7, 3, // top
    0, 1, 5, 5, 4, 0  // bottom
};

void drawAxes() {
    glLineWidth(2);
    glBegin(GL_LINES);
    // Draw x, y, z axes
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(-1, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, -1, 0);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);
    glEnd();

    glLineWidth(1);
}

void drawFloor() {
    glBegin(GL_LINES);
    glColor4f(0.4f, 0.4f, 0.4f, 1.f);
    // Render "floor" grid
    for (int i = 0; i <= 8; i++) {
        glVertex3i(i - 4, 1, 0);
        glVertex3i(i - 4, 1, 8);
        glVertex3i(-4, 1, i);
        glVertex3i(4, 1, i);
    }
    glEnd();
}

void renderScene(const glfw_state &app_state) {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glColor3f(1.0, 1.0, 1.0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 4.0 / 3.0, 1, 40);

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(1, 0, 5, 1, 0, 0, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
    glRotated(app_state.pitch, -1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    drawFloor();
}

class cameraRenderer {
  public:
    // Takes the calculated angle as input and rotates the 3D camera model accordingly
    void renderCamera(float3 theta) {

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);

        glPushMatrix();
        // Set the rotation, converting theta to degrees
        glRotatef(theta.x * 180 / PI_FL, 0, 0, -1);
        glRotatef(theta.y * 180 / PI_FL, 0, -1, 0);
        glRotatef((theta.z - PI_FL / 2) * 180 / PI_FL, -1, 0, 0);

        drawAxes();

        // Scale camera drawing
        glScalef(0.1f, 0.1f, 0.1f);
        glColor4f(0.4f, 0.2f, 0.8f, 0.5f);

        glBegin(GL_TRIANGLES);
        for (auto i = 0u; i < sizeof(cubeIndices) / sizeof(unsigned short); i += 3) {
            glVertex3f(cubeVertices[3 * cubeIndices[i]],
                       cubeVertices[3 * cubeIndices[i] + 1],
                       cubeVertices[3 * cubeIndices[i] + 2]);
            glVertex3f(cubeVertices[3 * cubeIndices[i + 1]],
                       cubeVertices[3 * cubeIndices[i + 1] + 1],
                       cubeVertices[3 * cubeIndices[i + 1] + 2]);
            glVertex3f(cubeVertices[3 * cubeIndices[i + 2]],
                       cubeVertices[3 * cubeIndices[i + 2] + 1],
                       cubeVertices[3 * cubeIndices[i + 2] + 2]);
        }
        glEnd();

        glPopMatrix();

        glDisable(GL_BLEND);
        glFlush();
    }
};

class rotationEstimator {
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98f;
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    uint64_t last_ts_gyro = 0;

  public:
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(o3p_vector gyro_data, uint64_t ts) {
        if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            firstGyro = false;
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = static_cast<double>(ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    }

    void processAccel(o3p_vector accel_data) {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (firstAccel) {
            firstAccel = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convention for the initial pose
            theta.y = PI_FL;
        } else {
            /*
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }

    // Returns the current rotation angle
    float3 getTheta() {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
};

int main(int argc, char *argv[]) {
    const auto cameras = o3p::Context::getAvailableCameras();
    if (cameras.size() > 0) {
        try {
            // Create a Pipeline - this serves as a top-level API for streaming and processing frames
            o3p::Pipeline pipe;
            o3p::PipelineProfile profile;

            o3p::PipelineConfig config;

            try {
                profile = pipe.init(config);
            } catch (std::exception &e) {
                std::cerr << "Error initializing camera : " << e.what() << std::endl;
                return EXIT_FAILURE;
            }

            auto streams = profile.getStreams();

            std::cout << streams.size() << " streams" << std::endl;

            // Initialize window for rendering
            window app(1280, 720, "O3P Motion Example");

            // Construct an object to manage view state
            glfw_state app_state(0.0, 0.0);

            // Register callbacks to allow manipulation of the view state
            register_glfw_callbacks(app, app_state);

            // Declare object for rendering camera motion
            cameraRenderer camera;
            // Declare object that handles camera pose calculations
            rotationEstimator algo;

            pipe.start();

            // Example : plugin data to be sent
            // std::vector<uint8_t> pluginMessage(1024, 1);

            while (app) {
                // Block program until frames arrive
                o3p::FrameSet frames = pipe.waitForFrames();

                // Try to get a frame of a IMU Frame
                auto imu = frames.getImuFrame();

                // Configure scene, draw floor, handle manipultation by the user etc.
                renderScene(app_state);

                // Draw the camera according to the computed theta
                // Handle accelerometer data
                o3p_vector accel_data;
                accel_data.x = imu->acceleration[0];
                accel_data.y = imu->acceleration[1];
                accel_data.z = imu->acceleration[2];
                algo.processAccel(accel_data);

                // Handle gyroscopes data
                o3p_vector gyro_data;
                uint64_t ts = imu->getTimestamp();
                gyro_data.x = imu->angularVelocity[0];
                gyro_data.y = imu->angularVelocity[1];
                gyro_data.z = imu->angularVelocity[2];
                algo.process_gyro(gyro_data, ts);

                // Draw
                camera.renderCamera(algo.getTheta());
            }
            // Stop the pipeline
            pipe.stop();

        } catch (std::exception &e) {
            std::cerr << "Error : " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    } else {
        std::cerr << "Error finding a camera" << std::endl;
        return EXIT_FAILURE;
    }
}
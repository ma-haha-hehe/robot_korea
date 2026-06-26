#------------------------------------------------------------------------------
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
#
#------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.24)

project(examples-o3p)

set (O3P_EXAMPLE_FOLDER "examples")

add_subdirectory (deviceInfo-o3p)
add_subdirectory (fwLogger-o3p)
add_subdirectory (fwUpdate-o3p)
add_subdirectory (hello-o3p)
add_subdirectory (multiDevice-o3p)
add_subdirectory (opencv-o3p)
add_subdirectory (options-o3p)

if (UNIX)
    add_subdirectory (ros-o3p)
endif ()    

option(O3P_BUILD_IMGUI_EXAMPLES "Build the examples using IMGUI." ON)

set(REALSENSE_EXAMPLE_HELPER_DIR ${CMAKE_CURRENT_LIST_DIR}/contrib/realsense-example-helper CACHE INTERNAL "")
if(O3P_BUILD_IMGUI_EXAMPLES)
    find_package(glfw3 3.3 QUIET)
    if(glfw3_FOUND)
        set(useGlfw3 ON CACHE INTERNAL "")
    else()
        if(WIN32)
            message(STATUS "GLFW3 not found; using internal version")
            set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
            add_subdirectory(contrib/glfw)
            include_directories(contrib/glfw/include)
            set(useGlfw3 ON CACHE INTERNAL "")
        else()
            message(STATUS "GLFW3 not found; please install libglfw3-dev")
        endif()
    endif()

    if(useGlfw3)
        add_subdirectory(contrib/imgui)

        add_subdirectory(motion-o3p)
        add_subdirectory(pointcloud-o3p)
        add_subdirectory(measure-o3p)		
    endif()
endif()

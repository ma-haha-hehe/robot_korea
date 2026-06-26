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

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/cmake")

project (pointcloud-o3p)

find_package (o3p REQUIRED)
find_package (OpenGL REQUIRED)

add_executable(pointcloud-o3p pointcloud-o3p.cpp)
link_directories(${GLFW_LIBRARY_DIRS})

target_include_directories (pointcloud-o3p PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/../../include
)

target_include_directories (pointcloud-o3p
    SYSTEM PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../contrib
)

target_link_directories(pointcloud-o3p PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../lib)

if(WIN32)
    target_link_libraries(pointcloud-o3p PRIVATE o3p glfw imgui OpenGL::GL OpenGL::GLU)
elseif(UNIX)
    target_link_libraries(pointcloud-o3p PRIVATE o3p glfw imgui OpenGL::GL OpenGL::GLU dl pthread)
endif()

copy_o3p_libs (pointcloud-o3p)

set_target_properties (pointcloud-o3p PROPERTIES FOLDER examples)

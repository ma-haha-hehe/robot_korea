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

project (recording-o3p)

find_package (o3p REQUIRED)
find_package( OpenCV QUIET )

if(NOT OpenCV_FOUND)
  message("Recording-O3P sample will not be build as no OpenCV was found!")
  return()
endif()

add_executable (recording-o3p recording-o3p.cpp)

target_include_directories (recording-o3p 
                            PRIVATE 
                            ${CMAKE_CURRENT_LIST_DIR}/../../include
                            ${OpenCV_INCLUDE_DIRS})

copy_o3p_libs (recording-o3p)

target_link_libraries (recording-o3p
                       o3p
                       ${OpenCV_LIBS})
target_link_directories(recording-o3p PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../lib)

set_target_properties (recording-o3p PROPERTIES FOLDER examples)
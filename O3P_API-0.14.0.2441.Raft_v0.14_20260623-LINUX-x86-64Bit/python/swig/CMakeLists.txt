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

project(O3Py)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/cmake")

if(POLICY CMP0086)
    # UseSWIG honors SWIG_MODULE_NAME via -module flag
    cmake_policy(SET CMP0086 NEW)
endif()

find_package(o3p REQUIRED)

find_package(SWIG REQUIRED)

find_package(Python3 ${PYTHON_VERSION_STRING} REQUIRED
    COMPONENTS Interpreter Development
    OPTIONAL_COMPONENTS NumPy)

if (NOT SWIG_FOUND OR
    NOT Python3_Development_FOUND)
    message("SWIG not found. Python wrapper will not be built!")
    return()
endif()

find_package (Numpy)
if (NOT Numpy_FOUND)
    message (
        WARNING "Numpy not found. Python wrapper will not be build."
    )
    return()
endif ()

include(${SWIG_USE_FILE})

set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin" CACHE STRING "" FORCE )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin" CACHE STRING "" FORCE )

set_source_files_properties(o3py.i PROPERTIES CPLUSPLUS ON)

swig_add_library(o3py LANGUAGE python SOURCES o3py.i)

if(DEFINED Numpy_INCLUDE_DIR)
    message(STATUS "Adding NumPy include to SWIG and target: ${Numpy_INCLUDE_DIR}")
    set_property(SOURCE o3py.i PROPERTY SWIG_FLAGS "-I${Numpy_INCLUDE_DIR}")
    target_include_directories(o3py PRIVATE ${Numpy_INCLUDE_DIR})
endif()

target_include_directories( o3py PRIVATE
    ./
    ../../include
)
# If we don't have the debug library link against the release
if(NOT PYTHON_DEBUG_LIBRARIES AND MSVC)
    target_compile_definitions(o3py
        PRIVATE
        $<$<CONFIG:Debug>:SWIG_PYTHON_INTERPRETER_NO_DEBUG>)
endif()

target_link_libraries(o3py PRIVATE o3p::o3p Python3::Python)

set_target_properties(o3py
    PROPERTIES
    FOLDER python
    DEBUG_POSTFIX ""
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
)

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set_property(TARGET o3py PROPERTY SWIG_GENERATED_COMPILE_OPTIONS -Wno-deprecated-declarations)
endif()

COPY_O3P_LIBS(o3py)

#****************************************************************************
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
#****************************************************************************


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was o3pConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

get_filename_component(O3P_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(O3P_INSTALL_PREFIX "${O3P_CMAKE_DIR}/../.." ABSOLUTE)

set_and_check(O3P_INCLUDE_DIR "${O3P_INSTALL_PREFIX}/include")
set_and_check(O3P_LIBRARY_DIR "${O3P_INSTALL_PREFIX}/lib")

include("${CMAKE_CURRENT_LIST_DIR}/o3pTargets.cmake")

macro (copy_o3p_libs copytarget)
    # Compute the installation prefix
    get_target_property(O3P_LOCATION_DIR o3p::o3p LOCATION)
    get_filename_component(O3P_LOCATION_DIR "${O3P_LOCATION_DIR}" PATH)
    if(O3P_LOCATION_DIR STREQUAL "/")
        set(O3P_LOCATION_DIR "")
    endif()

    if (WIN32)
        add_custom_command (
            TARGET ${copytarget} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${O3P_LOCATION_DIR}/o3p.dll"
                    $<TARGET_FILE_DIR:${copytarget}>)
    endif (WIN32)

    # Cleanup variables
    set(O3P_LOCATION_DIR)
endmacro ()

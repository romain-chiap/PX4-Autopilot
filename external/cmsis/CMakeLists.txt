############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_git_submodule(TARGET git_cmsis_v5 PATH "${CMAKE_CURRENT_LIST_DIR}/CMSIS_5")

# Needed to include the configBoot module
# Define the path to CMSIS-DSP (ROOT is defined on command line when using cmake)
set(ROOT ${CMAKE_CURRENT_LIST_DIR}/CMSIS_5)
set(DSP ${ROOT}/CMSIS/DSP)

set(CMSIS_ROOT ${ROOT} PARENT_SCOPE)
set(CMSIS_DSP ${DSP} PARENT_SCOPE)

# Add DSP folder to module path
list(APPEND CMAKE_MODULE_PATH ${DSP})

set(GCC ON CACHE BOOL "")
set(PLATFORM "FVP" CACHE STRING "")
set(ARM_CPU ${CMAKE_SYSTEM_PROCESSOR} CACHE STRING "Set ARM CPU. Default : cortex-m4")

add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-nested-externs>)
add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-float-equal>)
add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-cast-align>)
add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-double-promotion>)

#add_compile_options(${MAX_CUSTOM_OPT_LEVEL})

add_subdirectory(${ROOT}/CMSIS/DSP/Source)

include(config)
set_platform_core()
SET(PLATFORMID ${PLATFORMID} PARENT_SCOPE)
set_platform_core()
SET(PLATFORMID ${PLATFORMID} PARENT_SCOPE)
SET(COREID ${COREID} PARENT_SCOPE)

add_dependencies(CMSISDSPBasicMath prebuild_targets)
add_dependencies(CMSISDSPFastMath prebuild_targets)
add_dependencies(CMSISDSPBayes prebuild_targets)
add_dependencies(CMSISDSPCommon prebuild_targets)
add_dependencies(CMSISDSPComplexMath prebuild_targets)
add_dependencies(CMSISDSPController prebuild_targets)
add_dependencies(CMSISDSPDistance prebuild_targets)
add_dependencies(CMSISDSPFastMath prebuild_targets)
add_dependencies(CMSISDSPFiltering prebuild_targets)
add_dependencies(CMSISDSPMatrix prebuild_targets)
add_dependencies(CMSISDSPStatistics prebuild_targets)
add_dependencies(CMSISDSPSupport prebuild_targets)
add_dependencies(CMSISDSPSVM prebuild_targets)
add_dependencies(CMSISDSPTransform prebuild_targets)

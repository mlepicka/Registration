SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
FIND_PACKAGE(Boost 1.41.0 REQUIRED COMPONENTS system thread filesystem date_time)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Find another necessary libraries
FIND_PACKAGE(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find another DCLs our one depends on
DISCODE_FIND_DCL(PCLCoreTypes)
DISCODE_FIND_DCL(CvCoreTypes)

# Add standard libraries for this DCL: Boost & PCL
MESSAGE(STATUS "${PCL_LIBRARIES}")
SET(DisCODe_LIBRARIES ${DisCODe_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} )

# Validate whether components are properly linked to utilized libraries.
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs") 

# ##############################################################################
# Add source directories
# ##############################################################################

# Registration components
ADD_SUBDIRECTORY(Components)

# Registration types
ADD_SUBDIRECTORY(Types)

# Prepare config file to use from another DCLs
CONFIGURE_FILE(RegistrationConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/RegistrationConfig.cmake @ONLY)

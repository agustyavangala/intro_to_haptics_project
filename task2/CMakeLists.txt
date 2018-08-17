# create an executable
ADD_EXECUTABLE (task2 main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (task2 ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/task2)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY ../resources/world.urdf ../resources/RR.urdf  DESTINATION ${APP_RESOURCE_DIR})
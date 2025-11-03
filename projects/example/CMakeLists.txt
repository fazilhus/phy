#--------------------------------------------------------------------------
# example project
#--------------------------------------------------------------------------

PROJECT(example)
FILE(GLOB example_headers code/*.h)
FILE(GLOB example_sources code/*.cc)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${GSCEPT_LAB_ENV_OUTPUT_ROOT}/${PROJECT_NAME})

SET(files_example ${example_headers} ${example_sources})
SOURCE_GROUP("example" FILES ${files_example})

ADD_EXECUTABLE(example ${files_example})
TARGET_LINK_LIBRARIES(example core render)
ADD_DEPENDENCIES(example core render)
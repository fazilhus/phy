#--------------------------------------------------------------------------
# assignment-5 project
#--------------------------------------------------------------------------

PROJECT(assignment-5)
FILE(GLOB project_headers code/*.h)
FILE(GLOB project_sources code/*.cc)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${GSCEPT_LAB_ENV_OUTPUT_ROOT}/${PROJECT_NAME})

SET(files_project ${project_headers} ${project_sources})
SOURCE_GROUP("assignment-5" FILES ${files_project})

ADD_EXECUTABLE(assignment-5 ${files_project})
TARGET_LINK_LIBRARIES(assignment-5 core render)
ADD_DEPENDENCIES(assignment-5 core render)

IF (MSVC)
    set_property(TARGET assignment-5 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
ENDIF ()
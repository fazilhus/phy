#--------------------------------------------------------------------------
# physics
#--------------------------------------------------------------------------
SET(files_physics
)
SOURCE_GROUP("core" FILES ${files_physics})

SET(files_pch ../config.h ../config.cc)
SOURCE_GROUP("pch" FILES ${files_pch})
ADD_LIBRARY(physics STATIC ${files_physics} ${files_pch})
TARGET_PCH(physics ../)
ADD_DEPENDENCIES(physics glm)
TARGET_LINK_LIBRARIES(physics PUBLIC engine exts glm)
#--------------------------------------------------------------------------
# exts
#--------------------------------------------------------------------------

ADD_LIBRARY(exts INTERFACE)

ADD_SUBDIRECTORY(glew)
SET_TARGET_PROPERTIES(glew PROPERTIES FOLDER "exts/glew")
TARGET_INCLUDE_DIRECTORIES(exts INTERFACE glew/include)

SET(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build the GLFW example programs")
SET(GLFW_BUILD_TESTS OFF CACHE BOOL "Build the GLFW test programs")
SET(GLFW_BUILD_DOCS OFF CACHE BOOL "Build the GLFW documentation")
SET(GLFW_INSTALL OFF CACHE BOOL "Generate installation target")		  

ADD_SUBDIRECTORY(glfw)
SET_TARGET_PROPERTIES(glfw PROPERTIES FOLDER "exts/glfw")
TARGET_INCLUDE_DIRECTORIES(exts INTERFACE glfw/include)

ADD_LIBRARY(imgui STATIC imgui/imgui.cpp imgui/imgui_demo.cpp imgui/imgui_draw.cpp 
			imgui/imgui.h imgui/imgui_internal.h imgui/imgui_tables.cpp imgui/imgui_widgets.cpp
			imgui/imstb_rectpack.h imgui/imstb_textedit.h imgui/imstb_truetype.h
			imgui/backends/imgui_impl_glfw.h imgui/backends/imgui_impl_glfw.cpp
            imgui/backends/imgui_impl_opengl3.h imgui/backends/imgui_impl_opengl3.cpp)

TARGET_LINK_LIBRARIES(imgui PUBLIC exts)
TARGET_INCLUDE_DIRECTORIES(imgui PUBLIC imgui imgui/backends)
SET_TARGET_PROPERTIES(imgui PROPERTIES FOLDER "exts/imgui")

TARGET_LINK_LIBRARIES(exts INTERFACE imgui)

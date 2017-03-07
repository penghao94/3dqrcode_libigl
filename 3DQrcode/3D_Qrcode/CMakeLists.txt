cmake_minimum_required(VERSION 2.8.12)
project(3D_Qrcode)
#Add qrlib
include_directories("${PROJECT_SOURCE_DIR}/qrgen")
add_subdirectory (qrgen) 
set (QR_LIBS ${QR_LIBS} qrgen)
# Add your project files
FILE(GLOB SRCFILES *.cpp)
FILE(GLOB HEADFILES *.h)
add_executable(${PROJECT_NAME}_bin ${SRCFILES} ${HEADFILES} ${LIBIGL_EXTRA_SOURCES})
target_include_directories(${PROJECT_NAME}_bin PRIVATE ${LIBIGL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME}_bin PRIVATE ${LIBIGL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}_bin ${LIBIGL_LIBRARIES} ${LIBIGL_VIEWER_EXTRA_LIBRARIES} ${LIBIGL_OPENGL_EXTRA_LIBRARIES} ${LIBIGL_OPENGL_GLFW_EXTRA_LIBRARIES} ${LIBIGL_TRIANGLE_EXTRA_LIBRARIES} ${LIBIGL_PNG_EXTRA_LIBRARIES} ${QR_LIBS})

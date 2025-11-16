cmake_minimum_required(VERSION 3.5)
project(challenge_c06)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(FLAG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/flag")

if(EXISTS "${FLAG_FILE}")
    file(READ "${FLAG_FILE}" FLAG_CONTENT)
    string(STRIP "${FLAG_CONTENT}" FLAG_CONTENT)
    message(STATUS "Flag loaded from: ${FLAG_FILE}")
else()
    message(WARNING "Flag file not found! Using default flag.")
    set(FLAG_CONTENT "CTF{flag_not_found}")
endif()

ament_auto_add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE 
    FLAG_VALUE="${FLAG_CONTENT}"
)

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
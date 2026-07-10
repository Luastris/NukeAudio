cmake_minimum_required(VERSION 3.20)
project(NukeAudio CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Match the rest of the toolchain: dynamic debug CRT (/MDd in Debug).
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

set(NUKE_ROOT       "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(VCPKG_INSTALLED "$ENV{VCPKG_ROOT}/installed/x64-windows")

add_library(NukeAudio SHARED
    src/NukeAudio.cpp
)

# SERVICE plugin implementing iAudio behind the POD seam. miniaudio + stb_vorbis are
# vendored single-file libraries compiled INTO the DLL — no extra runtime files.
target_compile_definitions(NukeAudio PRIVATE
    WIN32 _WINDOWS NOMINMAX _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS
    GLM_ENABLE_EXPERIMENTAL
    BOOST_ALL_DYN_LINK
)

target_include_directories(NukeAudio PRIVATE
    ${NUKE_ROOT}/NukeEngine
    ${NUKE_ROOT}/NukeEngine/include
    ${NUKE_ROOT}/NukeEngine/deps
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/miniaudio
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/stb
    ${VCPKG_INSTALLED}/include                    # boost (NUKEModule header chain)
)

target_link_directories(NukeAudio PRIVATE
    ${VCPKG_INSTALLED}/$<$<CONFIG:Debug>:debug/>lib   # boost auto-link (header chain)
    ${NUKE_ROOT}/NukeEngine/x64/$<CONFIG>             # NukeEngine.dll import lib
)

target_link_libraries(NukeAudio PRIVATE
    NukeEngine
)

# Deploy the plugin DLL into the editor's modules/ folder.
set(MODULES_DIR "${NUKE_ROOT}/NukeEngine/x64/$<CONFIG>/modules")
add_custom_command(TARGET NukeAudio POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${MODULES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:NukeAudio>" "${MODULES_DIR}/"
    COMMENT "Deploying NukeAudio.dll to modules/"
)

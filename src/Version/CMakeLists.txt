set(private_files
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Version.cpp"
)

set(public_files
    "${CMAKE_CURRENT_SOURCE_DIR}/include/OpenLoco/Version.hpp"
)

loco_add_library(Version STATIC
    PRIVATE_FILES
        ${private_files}
    PUBLIC_FILES
        ${public_files}
)

set(version_header "${CMAKE_CURRENT_BINARY_DIR}/generated/OpenLocoVersion.h")

add_custom_target(VersionMetadata
    COMMAND
        "${CMAKE_COMMAND}"
        "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
        "-DOPENLOCO_SOURCE_DIR=${CMAKE_SOURCE_DIR}"
        "-DOPENLOCO_PROJECT_VERSION=${PROJECT_VERSION}"
        "-DOPENLOCO_VERSION_OUTPUT=${version_header}"
        -P "${CMAKE_SOURCE_DIR}/cmake/GenerateOpenLocoVersion.cmake"
    BYPRODUCTS
        "${version_header}"
    COMMENT "Refreshing OpenLoco version metadata"
    VERBATIM
)

target_include_directories(Version
    PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

add_dependencies(Version VersionMetadata)

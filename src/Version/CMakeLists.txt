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

target_compile_definitions(Version
    PRIVATE
        OPENLOCO_VERSION_TAG="${OPENLOCO_VERSION_TAG}"
        OPENLOCO_BRANCH="${OPENLOCO_BRANCH}"
        OPENLOCO_COMMIT_SHA1_SHORT="${OPENLOCO_COMMIT_SHA1_SHORT}"
)

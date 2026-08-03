if (NOT DEFINED OPENLOCO_VERSION_OUTPUT)
    message(FATAL_ERROR "OPENLOCO_VERSION_OUTPUT is required")
endif()

if (NOT DEFINED OPENLOCO_SOURCE_DIR)
    message(FATAL_ERROR "OPENLOCO_SOURCE_DIR is required")
endif()

if (NOT DEFINED OPENLOCO_PROJECT_VERSION)
    set(OPENLOCO_PROJECT_VERSION "unknown")
endif()

set(OPENLOCO_VERSION_TAG "${OPENLOCO_PROJECT_VERSION}")
set(OPENLOCO_BRANCH "unknown")
set(OPENLOCO_COMMIT_SHA1_SHORT "unknown")

if (GIT_EXECUTABLE)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" describe HEAD --always
        WORKING_DIRECTORY "${OPENLOCO_SOURCE_DIR}"
        RESULT_VARIABLE describe_result
        OUTPUT_VARIABLE OPENLOCO_VERSION_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    if (describe_result EQUAL 0 AND OPENLOCO_VERSION_TAG)
        string(REGEX REPLACE "-g[0-9A-Fa-f]+$" "" OPENLOCO_VERSION_TAG "${OPENLOCO_VERSION_TAG}")
    else()
        set(OPENLOCO_VERSION_TAG "${OPENLOCO_PROJECT_VERSION}")
    endif()

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY "${OPENLOCO_SOURCE_DIR}"
        RESULT_VARIABLE branch_result
        OUTPUT_VARIABLE OPENLOCO_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if (NOT branch_result EQUAL 0 OR NOT OPENLOCO_BRANCH)
        set(OPENLOCO_BRANCH "unknown")
    endif()

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
        WORKING_DIRECTORY "${OPENLOCO_SOURCE_DIR}"
        RESULT_VARIABLE commit_result
        OUTPUT_VARIABLE OPENLOCO_COMMIT_SHA1_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if (NOT commit_result EQUAL 0 OR NOT OPENLOCO_COMMIT_SHA1_SHORT)
        set(OPENLOCO_COMMIT_SHA1_SHORT "unknown")
    endif()
endif()

function(escape_cpp_string variable)
    string(REPLACE "\\" "\\\\" value "${${variable}}")
    string(REPLACE "\"" "\\\"" value "${value}")
    string(REPLACE "\n" "\\n" value "${value}")
    string(REPLACE "\r" "\\r" value "${value}")
    set(${variable} "${value}" PARENT_SCOPE)
endfunction()

escape_cpp_string(OPENLOCO_VERSION_TAG)
escape_cpp_string(OPENLOCO_BRANCH)
escape_cpp_string(OPENLOCO_COMMIT_SHA1_SHORT)

configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/OpenLocoVersion.h.in"
    "${OPENLOCO_VERSION_OUTPUT}"
    @ONLY
)

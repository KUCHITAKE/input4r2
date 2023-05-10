FILE(GLOB HDRS "*.h")
FILE(GLOB SRCS "*.cc")

add_library(proto SHARED STATIC)

set_property(TARGET proto PROPERTY POSITION_INDEPENDENT_CODE ON)

target_sources(proto
    PRIVATE
    ${SRCS}
)
set_property(TARGET proto
    PROPERTY PUBLIC_HEADER
    ${HDRS}
)
target_include_directories(proto
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(proto ${PROTOBUF_LIBRARY})
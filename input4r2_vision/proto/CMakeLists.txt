cmake_minimum_required(VERSION 3.2)
project(proto C CXX)

find_package(Protobuf REQUIRED)

FILE(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR} PROTOMODEL_PATH)
FILE(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/generated PROTOBINDING_PATH)

FILE(GLOB DATAMODEL_PROTOS "*.proto")

FOREACH(protos ${DATAMODEL_PROTOS})
   FILE(TO_NATIVE_PATH ${protos} proto_native)
   EXECUTE_PROCESS(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} -I=${PROTOMODEL_PATH} --cpp_out=${PROTOBINDING_PATH} ${proto_native}
   RESULT_VARIABLE rv)
   # Optional, but that can show the user if something have gone wrong with the proto generation 
   IF(${rv})
        MESSAGE("Generation of data model returned ${rv} for proto ${proto_native}")
   ENDIF()
ENDFOREACH(protos)

add_subdirectory(generated)

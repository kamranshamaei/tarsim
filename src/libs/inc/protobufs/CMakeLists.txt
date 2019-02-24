project (EitProtoProj)

include_directories(
    ${PROTOBUF_INCLUDE_DIRS}
    )

set(FILE_EIT_PROTO_HDRS 
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}/common.pb.h
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}/rbs.pb.h
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}/win.pb.h
    )
    
set(FILE_EIT_PROTO_SRCS 
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}/common.pb.cc
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}/rbs.pb.cc
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}/win.pb.cc
    )
    
add_library(simProto ${FILE_EIT_PROTO_SRCS} ${FILE_EIT_PROTO_HDRS})
target_link_libraries(simProto ${PROTOBUF_LIBRARIES})

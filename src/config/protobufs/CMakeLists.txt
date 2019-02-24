file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
message(STATUS "Protobuf Source Files: ${files}")
message(STATUS "Protobuf Output Directory: ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}")

file(REMOVE_RECURSE ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY})
file(MAKE_DIRECTORY ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY})

execute_process(COMMAND protoc --proto_path=${CMAKE_CURRENT_SOURCE_DIR} ${files}
--cpp_out=${CMAKE_PROTOBUF_OUTPUT_DIRECTORY})


INSTALL(FILES ${CMAKE_SOURCE_DIR}/config/protobufs/common.proto DESTINATION ./user/client/samples/proto)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/config/protobufs/rbs.proto DESTINATION ./user/client/samples/proto)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/config/protobufs/win.proto DESTINATION ./user/client/samples/proto)

INSTALL(FILES ${CMAKE_SOURCE_DIR}/config/protobufs/common.proto DESTINATION ./user/server/samples/proto)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/config/protobufs/rbs.proto DESTINATION ./user/server/samples/proto)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/config/protobufs/win.proto DESTINATION ./user/server/samples/proto)
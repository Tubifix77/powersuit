# ps_proto — vendors the protocol contract library (common/c) as an IDF
# component. The same sources build on the host via common/c/CMakeLists.txt;
# both are locked together by the shared test vectors (docs/network-map.md).
idf_component_register(
    SRCS
        "${CMAKE_CURRENT_LIST_DIR}/../../../common/c/src/crc16.c"
        "${CMAKE_CURRENT_LIST_DIR}/../../../common/c/src/spi_frame.c"
        "${CMAKE_CURRENT_LIST_DIR}/../../../common/c/src/adpcm_ima.c"
    INCLUDE_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/../../../common/c/include"
)

# ps_audio — AUDIO-plane packetizer + VOX energy gate. Both sources are pure C99
# (no IDF/FreeRTOS includes) and are compiled a second time on the host by
# firmware/tests/host/CMakeLists.txt with -Wall -Wextra -Werror.
idf_component_register(
    SRCS "src/audio_pkt.c"
         "src/energy_gate.c"
    INCLUDE_DIRS "include"
    REQUIRES ps_proto
)

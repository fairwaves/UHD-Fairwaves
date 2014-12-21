########################################################################
# Build and Install the UmTRX utils
########################################################################
INSTALL(PROGRAMS
    umtrx_net_burner
    DESTINATION bin
)

add_executable(umtrx_test_chains umtrx_test_chains.cpp)
target_link_libraries(umtrx_test_chains ${UMTRX_LIBRARIES})
install(TARGETS umtrx_test_chains DESTINATION bin)

add_executable(umtrx_cal_tx_dc_offset umtrx_cal_tx_dc_offset.cpp)
target_link_libraries(umtrx_cal_tx_dc_offset ${UMTRX_LIBRARIES})
install(TARGETS umtrx_cal_tx_dc_offset DESTINATION bin)

add_executable(umtrx_pa_ctrl umtrx_pa_ctrl.cpp)
target_link_libraries(umtrx_pa_ctrl ${UMTRX_LIBRARIES})
install(TARGETS umtrx_pa_ctrl DESTINATION bin)


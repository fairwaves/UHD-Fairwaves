########################################################################
# Build and Install the UmTRX utils
########################################################################
INSTALL(PROGRAMS
    umtrx_nmea
    umtrx_gps_coord
    umtrx_auto_calibration
    umtrx_firmware
    umtrx_net_burner
    umtrx_query_sensors
    umtrx_query_versions
    DESTINATION bin
)

add_executable(umtrx_test_chains umtrx_test_chains.cpp)
target_link_libraries(umtrx_test_chains ${UMTRX_LIBRARIES})
install(TARGETS umtrx_test_chains DESTINATION bin)

add_executable(umtrx_test_gains umtrx_test_gains.cpp)
target_link_libraries(umtrx_test_gains ${UMTRX_LIBRARIES})
install(TARGETS umtrx_test_gains DESTINATION bin)

add_executable(umtrx_cal_tx_dc_offset umtrx_cal_tx_dc_offset.cpp)
target_link_libraries(umtrx_cal_tx_dc_offset ${UMTRX_LIBRARIES})
install(TARGETS umtrx_cal_tx_dc_offset DESTINATION bin)

add_executable(umtrx_cal_tx_iq_balance umtrx_cal_tx_iq_balance.cpp)
target_link_libraries(umtrx_cal_tx_iq_balance ${UMTRX_LIBRARIES})
install(TARGETS umtrx_cal_tx_iq_balance DESTINATION bin)

add_executable(umtrx_pa_ctrl umtrx_pa_ctrl.cpp)
target_link_libraries(umtrx_pa_ctrl ${UMTRX_LIBRARIES})
install(TARGETS umtrx_pa_ctrl DESTINATION bin)


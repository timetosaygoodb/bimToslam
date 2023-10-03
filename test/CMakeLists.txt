SET(TEST_SOURCES TestORBextractor Test3Dto3Dmatch TestISSextractor)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp)
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} bimToslam)
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)





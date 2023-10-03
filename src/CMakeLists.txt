add_library(bimToslam SHARED
        frame.cpp
        camera.cpp
        feature.cpp
        mappoint.cpp
        frontend.cpp
        ORBextractor.cpp
        ISSextractor.cpp
        ICPsolver.cpp
)
target_link_libraries(bimToslam
        ${THIRD_PARTY_LIBS})

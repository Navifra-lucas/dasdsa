# ------------------------
# Blaze: header-only
# ------------------------
add_library(blaze INTERFACE)
target_include_directories(blaze INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/blaze/include>
    $<INSTALL_INTERFACE:include/blaze>
)
# cmake와 header를 install 경로에 복사
install(DIRECTORY blaze/include/ DESTINATION include/blaze)
install(DIRECTORY blaze/share/ DESTINATION share/blaze)

# ------------------------
# OSQP: built library
# ------------------------
add_library(osqp INTERFACE)
target_include_directories(osqp INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/osqp/include/osqp>
    $<INSTALL_INTERFACE:include/osqp>
)
target_link_libraries(osqp INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/osqp/lib/libosqpstatic.a
)
# cmake와 header, library를 install 경로에 복사
install(DIRECTORY osqp/include/osqp/ DESTINATION include/osqp)
install(DIRECTORY osqp/lib/ DESTINATION lib)


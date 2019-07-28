add_executable(main main.cpp discriminant_img.cpp transform_position.cpp
        Aubo_driver.cpp MainWindows.cpp)

target_link_libraries(main ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES}
        ${Qt5Network_LIBRARIES} ${OpenCV_LIBRARIES}
        ${Aubo_LIBRARIES_DIRS} ${TORCH_LIBRARIES}
        libfreenect2.so)

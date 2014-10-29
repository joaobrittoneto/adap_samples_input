rock_library(adap_samples_input
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(adap_samples_input_bin Main.cpp
    DEPS adap_samples_input)


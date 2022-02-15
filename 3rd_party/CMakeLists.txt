cmake_minimum_required(VERSION 3.1)

add_subdirectory("cminpack-1.3.6")
set_target_properties(cminpack PROPERTIES FOLDER "3rd_party")

add_subdirectory("liblbfgs")
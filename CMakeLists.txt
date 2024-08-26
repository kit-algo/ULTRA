cmake_minimum_required(VERSION 3.22)
project(ULTRA)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#Compiler settings
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_FLAGS "-pipe -march=native -Wfatal-errors")
set(CMAKE_CXX_FLAGS_DEBUG "-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-ffast-math -DNDEBUG -O3")

# Add an option for enabling prefetch
option(USE_PREFETCH "Enable prefetch optimization" OFF)

if(USE_PREFETCH)
    add_compile_definitions(ENABLE_PREFETCH)
endif()

#Libraries
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

function(add_runnable name)
    add_executable(${ARGV})
    target_link_libraries(${name} PUBLIC OpenMP::OpenMP_CXX)
endfunction()

set(RUNNABLES DelayExperiments Network ULTRA ULTRAPHAST)
foreach(f ${RUNNABLES})
    add_runnable(${f} Runnables/${f}.cpp)
endforeach()

add_custom_target(All)
add_dependencies(All ${RUNNABLES})
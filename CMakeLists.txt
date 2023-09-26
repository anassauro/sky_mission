# Especifica versao minima de CMake necessaria.
# Aqui requer a versao 3.0.2 ou superior do CMake.
cmake_minimum_required(VERSION 3.0.2)

# Define o nome do projeto como "skyrats_cpp"
project(sky_mission)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    roscpp
)

# Define os diretorios de inclusao, bibliotecas
# e dependencias para o pacote.
catkin_package (
    INCLUDE_DIRS include
    LIBRARIES sky_mission
    CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories (
    include
    ${catkin_INCLUDE_DIRS}
)

# Declara um C++ executavel, no caso, example.cpp
add_executable(sky_mission src/stage_3/sky_mission.cpp)

# Adiciona dependencias para o "target" skyrats_cpp. Isso garante que
# o executavel seja construido somente apos a construcao dos "targets"

add_dependencies(sky_mission 
    ${skyrats_cpp_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Vincula o "target" skyrats_cpp com as bibliotecas necessarias.
## Aqui, ele vincula as bibliotecas do catkin (por exemplo, roscpp).
target_link_libraries(skyrats_cpp
    ${catkin_LIBRARIES}
)
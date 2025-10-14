cmake_minimum_required(VERSION 3.8)
project(rb_theron_description_fortress)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Paquete de construcción mínimo para description
find_package(ament_cmake REQUIRED)

# Instalar solo los directorios que existan (evita fallos)
foreach(dir launch urdf meshes robots worlds config)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
      USE_SOURCE_PERMISSIONS
    )
  endif()
endforeach()

# (Opcional) Si tienes scripts Python ejecutables en, p.ej., scripts/
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src)
  install(PROGRAMS src/generate_paths.py
          DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
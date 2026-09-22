cmake_minimum_required(VERSION 3.8)
project(dual_crx_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)
find_package(ruckig REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

pybind11_add_module(_ruckig src/ruckig_binding.cpp)
target_link_libraries(_ruckig PRIVATE ruckig::ruckig)
install(TARGETS _ruckig LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

install(PROGRAMS
  scripts/keyboard_control.py
  scripts/preplanned_trajectory/simple_motion.py
  scripts/preplanned_trajectory/joint_sine.py
  scripts/preplanned_trajectory/cartesian_sine.py
  scripts/preplanned_trajectory/cartesian_circle.py
  scripts/preplanned_trajectory/facing_circle.py
  scripts/util/interpolation_node
  scripts/analysis/record_joint_streams.py
  scripts/analysis/analyze_latency.py
  DESTINATION lib/${PROJECT_NAME}
)

# Tests are kept locally and may be absent from a fresh checkout.
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cartesian.py")
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(cartesian tests/test_cartesian.py
    ENV PYTHONDONTWRITEBYTECODE=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    TIMEOUT 60
  )
  ament_add_pytest_test(interpolation tests/test_interpolation.py TIMEOUT 30)
  ament_add_pytest_test(ruckig tests/test_ruckig.py TIMEOUT 30)
  ament_add_pytest_test(interpolation_node tests/test_interpolation_node.py TIMEOUT 120
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(dual_test4_transport tests/test_dual_test4_transport.py TIMEOUT 30
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(motion_routes tests/test_motion_routes.py TIMEOUT 60
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(circle tests/test_circle.py
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST TIMEOUT 60)
  ament_add_pytest_test(facing_circle tests/test_facing_circle.py TIMEOUT 60)
  ament_add_pytest_test(latency tests/test_latency.py
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OPENBLAS_NUM_THREADS=1 TIMEOUT 60)
  ament_add_pytest_test(simple_motion tests/test_simple_motion.py
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OPENBLAS_NUM_THREADS=1 TIMEOUT 90)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ruckig_timing tools/test_ruckig_timing.py TIMEOUT 60)
  ament_add_pytest_test(ruckig_mock_launch tools/test_ruckig_mock_launch.py TIMEOUT 90
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  ament_add_pytest_test(keyboard tools/test_keyboard.py TIMEOUT 60
    ENV PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(motion_refactor tools/test_motion_refactor.py TIMEOUT 60
    ENV PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/dual_crx_matplotlib)
endif()

install(
  DIRECTORY
    launch
    urdf
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()

include(FetchContent)

# Suppress Eigen testing and documentation
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen testing" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen docs" FORCE)
set(BUILD_TESTING OFF)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)

FetchContent_MakeAvailable(Eigen)

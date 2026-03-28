include(FetchContent)

# Suppress Eigen testing and documentation
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen testing" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen docs" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable testing" FORCE)

FetchContent_Declare(
  Eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
)

FetchContent_MakeAvailable(Eigen)

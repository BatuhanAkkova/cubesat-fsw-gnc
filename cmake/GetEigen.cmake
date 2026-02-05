include(FetchContent)

set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen testing" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen docs" FORCE)
set(BUILD_TESTING OFF) # Temporarily disable global testing for Eigen
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)

FetchContent_MakeAvailable(Eigen)
set(BUILD_TESTING ON) # Re-enable for our project

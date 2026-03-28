include(FetchContent)

FetchContent_Declare(
  spdlog
  URL https://github.com/gabime/spdlog/archive/refs/tags/v1.12.0.zip
)

FetchContent_MakeAvailable(spdlog)

# Format the codebase using clang-format
$extensions = "*.cpp", "*.hpp", "*.h", "*.cc", "*.c"
$files = Get-ChildItem -Recurse -Include $extensions | Where-Object { $_.FullName -notmatch "build" -and $_.FullName -notmatch ".git" }

foreach ($file in $files) {
    Write-Host "Formatting $($file.FullName)..."
    clang-format -i $file.FullName
}

Write-Host "Formatting complete."


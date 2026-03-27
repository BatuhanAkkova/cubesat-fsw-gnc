# Find all .cpp and .hpp files in src, tests, and python (if any C++)
$files = Get-ChildItem -Path "src","tests" -Include *.cpp,*.hpp -Recurse

if ($null -eq $files) {
    Write-Host "No C++ files found to format."
    exit 0
}

Write-Host "Formatting $($files.Count) files..."

foreach ($file in $files) {
    Write-Host "Formatting $($file.FullName)..."
    clang-format -i $file.FullName
}

Write-Host "Formatting complete!"

#!/bin/bash
# Format the codebase using clang-format

# Find all relevant files and format them
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cc" -o -name "*.c" \) \
       -not -path "*/build/*" \
       -not -path "*/.git/*" \
       -exec clang-format -i {} +

echo "Formatting complete."

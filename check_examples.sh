#!/bin/bash
# Example Verification Script

cd "$(dirname "$0")/examples"

echo "🔍 Checking Examples..."
echo

ERRORS=0

# Check each example
for example in actions arm auth fleet metrics mqtt_iot quickstart; do
    echo "Checking: $example"
    
    # Check README.md
    if [ ! -f "$example/README.md" ]; then
        echo "  ❌ Missing README.md"
        ((ERRORS++))
    else
        echo "  ✅ README.md"
    fi
    
    # Check Dockerfile.ros2
    if [ ! -f "$example/Dockerfile.ros2" ]; then
        echo "  ❌ Missing Dockerfile.ros2"
        ((ERRORS++))
    else
        echo "  ✅ Dockerfile.ros2"
    fi
    
    # Check docker-compose.yml
    if [ ! -f "$example/docker-compose.yml" ]; then
        echo "  ❌ Missing docker-compose.yml"
        ((ERRORS++))
    else
        echo "  ✅ docker-compose.yml"
    fi
    
    # Check index.html (web dashboard)
    if [ ! -f "$example/index.html" ]; then
        echo "  ❌ Missing index.html"
        ((ERRORS++))
    else
        echo "  ✅ index.html"
    fi
    
    # Check Python file
    PY_COUNT=$(find "$example" -maxdepth 1 -name "*.py" -type f | wc -l)
    if [ "$PY_COUNT" -eq 0 ]; then
        echo "  ❌ Missing Python file"
        ((ERRORS++))
    else
        echo "  ✅ Python file(s): $PY_COUNT"
    fi
    
    echo
done

echo "Checking Playground Examples..."
echo

for example in playground/*; do
    name=$(basename "$example")
    echo "Checking: $name"
    
    # Check README.md
    if [ ! -f "$example/README.md" ]; then
        echo "  ❌ Missing README.md"
        ((ERRORS++))
    else
        echo "  ✅ README.md"
    fi
    
    # Check Dockerfile.ros2
    if [ ! -f "$example/Dockerfile.ros2" ]; then
        echo "  ❌ Missing Dockerfile.ros2"
        ((ERRORS++))
    else
        echo "  ✅ Dockerfile.ros2"
    fi
    
    # Check docker-compose.ros2.yml
    if [ ! -f "$example/docker-compose.ros2.yml" ]; then
        echo "  ❌ Missing docker-compose.ros2.yml"
        ((ERRORS++))
    else
        echo "  ✅ docker-compose.ros2.yml"
    fi
    
    # Check HTML dashboard
    HTML_COUNT=$(find "$example" -maxdepth 1 -name "*.html" -type f | wc -l)
    if [ "$HTML_COUNT" -eq 0 ]; then
        echo "  ❌ Missing HTML dashboard"
        ((ERRORS++))
    else
        echo "  ✅ HTML dashboard: $HTML_COUNT"
    fi
    
    # Check ros2 directory
    if [ ! -d "$example/ros2" ]; then
        echo "  ❌ Missing ros2/ directory"
        ((ERRORS++))
    else
        echo "  ✅ ros2/ directory"
    fi
    
    echo
done

echo "================================"
if [ $ERRORS -eq 0 ]; then
    echo "✅ All examples are complete!"
    exit 0
else
    echo "❌ Found $ERRORS issues"
    exit 1
fi

#!/bin/bash
# Verification script for 2D Slalom Simulator project

echo "======================================"
echo "2D Slalom Simulator - Project Verification"
echo "======================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 (MISSING)"
        return 1
    fi
}

check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✓${NC} $1/"
        return 0
    else
        echo -e "${RED}✗${NC} $1/ (MISSING)"
        return 1
    fi
}

cd "$(dirname "$0")"

echo "Checking directory structure..."
check_dir "config"
check_dir "launch"
check_dir "resource"
check_dir "slalom_simulator/slalom_simulator"
check_dir "test"

echo ""
echo "Checking core files..."
check_file "package.xml"
check_file "setup.py"
check_file "setup.cfg"
check_file "README.md"

echo ""
echo "Checking configuration files..."
check_file "config/params.yaml"

echo ""
echo "Checking launch files..."
check_file "launch/slalom.launch.py"

echo ""
echo "Checking ROS2 nodes..."
check_file "slalom_simulator/slalom_simulator/__init__.py"
check_file "slalom_simulator/slalom_simulator/simulator_node.py"
check_file "slalom_simulator/slalom_simulator/imu1_localization_node.py"
check_file "slalom_simulator/slalom_simulator/imu2_localization_node.py"
check_file "slalom_simulator/slalom_simulator/kalman_localization_node.py"
check_file "slalom_simulator/slalom_simulator/controller_node.py"
check_file "slalom_simulator/slalom_simulator/utils.py"

echo ""
echo "Checking resource marker..."
check_file "resource/slalom_simulator"

echo ""
echo "======================================"
echo "Checking Python syntax..."
echo "======================================"

SYNTAX_OK=true
for file in slalom_simulator/slalom_simulator/*.py launch/*.py; do
    if [ -f "$file" ]; then
        if python3 -m py_compile "$file" 2>/dev/null; then
            echo -e "${GREEN}✓${NC} $file (syntax OK)"
        else
            echo -e "${RED}✗${NC} $file (syntax error)"
            SYNTAX_OK=false
        fi
    fi
done

echo ""
echo "======================================"
echo "Summary"
echo "======================================"

if [ "$SYNTAX_OK" = true ]; then
    echo -e "${GREEN}All checks passed!${NC}"
    echo ""
    echo "Project is ready to build. Run:"
    echo "  cd ~/ros2_ws"
    echo "  colcon build --packages-select slalom_simulator"
    echo "  source install/setup.bash"
    echo "  ros2 launch slalom_simulator slalom.launch.py"
else
    echo -e "${RED}Some syntax errors detected. Please fix them before building.${NC}"
fi

echo ""

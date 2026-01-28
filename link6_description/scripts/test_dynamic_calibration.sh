#!/bin/bash
# Test script for dynamic calibration system
# This script verifies that the dynamic calibration URDF loads correctly

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Dynamic Calibration Test Script${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Get package share directories
PKG_SHARE=$(ros2 pkg prefix link6_description)/share/link6_description
KORTEX_HW_SHARE=$(ros2 pkg prefix kortex3_hardware)/share/kortex3_hardware

echo -e "${YELLOW}Test 1: Verify files exist${NC}"
echo "----------------------------"

FILES=(
    "$PKG_SHARE/config/default_calibration.yaml"
    "$PKG_SHARE/urdf/link6_macro_dynamic_calib.xacro"
    "$PKG_SHARE/urdf/link6_dynamic_calib.urdf.xacro"
    "$KORTEX_HW_SHARE/scripts/calibration_yaml_generator.py"
)

ALL_EXIST=true
for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} Found: $(basename $file)"
    else
        echo -e "${RED}✗${NC} Missing: $file"
        ALL_EXIST=false
    fi
done

if [ "$ALL_EXIST" = false ]; then
    echo -e "${RED}Some files are missing. Please build the package.${NC}"
    exit 1
fi
echo ""

echo -e "${YELLOW}Test 2: Process URDF with default calibration${NC}"
echo "-----------------------------------------------"
URDF_OUTPUT=$(ros2 run xacro xacro \
    "$PKG_SHARE/urdf/link6_dynamic_calib.urdf.xacro" \
    gripper:="" \
    2>&1)

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} URDF processing successful"

    # Check if joint_1 is in the output
    if echo "$URDF_OUTPUT" | grep -q "joint_1"; then
        echo -e "${GREEN}✓${NC} joint_1 found in URDF"
    else
        echo -e "${RED}✗${NC} joint_1 not found in URDF"
        exit 1
    fi
else
    echo -e "${RED}✗${NC} URDF processing failed"
    echo "$URDF_OUTPUT"
    exit 1
fi
echo ""

echo -e "${YELLOW}Test 3: Verify joint_1 origin values${NC}"
echo "--------------------------------------"
JOINT1_ORIGIN=$(echo "$URDF_OUTPUT" | grep -A 1 'joint name="joint_1"' | grep "origin")

if [ -n "$JOINT1_ORIGIN" ]; then
    echo -e "${GREEN}✓${NC} Found joint_1 origin:"
    echo "  $JOINT1_ORIGIN"

    # Check if it contains the nominal xyz values
    if echo "$JOINT1_ORIGIN" | grep -q "0 0 0.053"; then
        echo -e "${GREEN}✓${NC} XYZ values match nominal calibration"
    else
        echo -e "${YELLOW}⚠${NC} XYZ values differ from nominal (may be calibrated)"
    fi
else
    echo -e "${RED}✗${NC} Could not find joint_1 origin"
    exit 1
fi
echo ""

echo -e "${YELLOW}Test 4: Test with custom calibration file${NC}"
echo "--------------------------------------------"
# Create a temporary custom calibration file
TEMP_CALIB="/tmp/test_calibration_$$.yaml"
cat > "$TEMP_CALIB" << 'EOF'
serial_number: "TEST_SERIAL"

joint_1:
  xyz: "0.001 0.002 0.053"
  rpy: "3.141592653589793 0.0 0.0"

joint_2:
  xyz: "0.11024 -0.069257 -0.1375"
  rpy: "-1.5707963267948966 0.0 0.0"

joint_3:
  xyz: "0.0 0.485 0.0"
  rpy: "-3.141592653589793 0.0 0.0"

joint_4:
  xyz: "0.0 -0.15216 -0.091704"
  rpy: "-1.5707963267948966 0.0 0.0"

joint_5:
  xyz: "0.0 -0.062957 -0.22275"
  rpy: "-1.5707963267948966 0.0 0.0"

joint_6:
  xyz: "0.087028 0.086 -0.076922"
  rpy: "0.0 -1.5707963267948966 0.0"
EOF

URDF_CUSTOM=$(ros2 run xacro xacro \
    "$PKG_SHARE/urdf/link6_dynamic_calib.urdf.xacro" \
    gripper:="" \
    calibration_file:="$TEMP_CALIB" \
    2>&1)

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} URDF processing with custom calibration successful"

    JOINT1_CUSTOM=$(echo "$URDF_CUSTOM" | grep -A 1 'joint name="joint_1"' | grep "origin")

    # Check if custom values were applied
    if echo "$JOINT1_CUSTOM" | grep -q "0.001 0.002 0.053"; then
        echo -e "${GREEN}✓${NC} Custom calibration values applied correctly"
        echo "  $JOINT1_CUSTOM"
    else
        echo -e "${RED}✗${NC} Custom calibration values not applied"
        echo "  Expected: xyz containing '0.001 0.002 0.053'"
        echo "  Got: $JOINT1_CUSTOM"
        rm "$TEMP_CALIB"
        exit 1
    fi
else
    echo -e "${RED}✗${NC} URDF processing with custom calibration failed"
    echo "$URDF_CUSTOM"
    rm "$TEMP_CALIB"
    exit 1
fi

# Clean up
rm "$TEMP_CALIB"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All tests passed!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Dynamic calibration system is working correctly."
echo ""

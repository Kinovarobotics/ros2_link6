#!/bin/bash
# Test script for calibration feature
# This script validates that the calibration process works correctly

set -e  # Exit on error

echo "============================================"
echo "Testing Link6 Calibration Feature"
echo "============================================"
echo ""

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

# Define paths
MACRO_FILE="$PKG_DIR/urdf/link6_macro.xacro"
CALIB_FILE="$PKG_DIR/test/calibration/calib.xml"
OUTPUT_FILE="/tmp/link6_calibrated_macro_test.xacro"
PYTHON_SCRIPT="$SCRIPT_DIR/calibrated_urdf_generator.py"

echo "1. Checking input files..."
if [ ! -f "$MACRO_FILE" ]; then
    echo "   ERROR: Macro file not found: $MACRO_FILE"
    exit 1
fi
echo "   ✓ Macro file found: $MACRO_FILE"

if [ ! -f "$CALIB_FILE" ]; then
    echo "   ERROR: Calibration file not found: $CALIB_FILE"
    exit 1
fi
echo "   ✓ Calibration file found: $CALIB_FILE"

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "   ERROR: Python script not found: $PYTHON_SCRIPT"
    exit 1
fi
echo "   ✓ Python script found: $PYTHON_SCRIPT"
echo ""

echo "2. Running calibration script..."
python3 "$PYTHON_SCRIPT" \
    --urdf_path "$MACRO_FILE" \
    --calibration_file "$CALIB_FILE" \
    --output_file "$OUTPUT_FILE"

if [ $? -ne 0 ]; then
    echo "   ERROR: Calibration script failed"
    exit 1
fi
echo "   ✓ Calibration script completed successfully"
echo ""

echo "3. Validating output file..."
if [ ! -f "$OUTPUT_FILE" ]; then
    echo "   ERROR: Output file not created: $OUTPUT_FILE"
    exit 1
fi
echo "   ✓ Output file created: $OUTPUT_FILE"

# Check that the output is valid XML
xmllint "$OUTPUT_FILE" > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "   ERROR: Output file is not valid XML"
    exit 1
fi
echo "   ✓ Output file is valid XML"

# Check that macro structure is preserved
if ! grep -q '<xacro:macro name="link6"' "$OUTPUT_FILE"; then
    echo "   ERROR: Macro definition not found in output"
    exit 1
fi
echo "   ✓ Macro definition preserved"

# Check that prefix variables are preserved
if ! grep -q '\${prefix}' "$OUTPUT_FILE"; then
    echo "   ERROR: Prefix variables not found in output"
    exit 1
fi
echo "   ✓ Prefix variables preserved"

# Check that all 6 joints are present
for i in {1..6}; do
    if ! grep -q "joint_$i" "$OUTPUT_FILE"; then
        echo "   ERROR: joint_$i not found in output"
        exit 1
    fi
done
echo "   ✓ All 6 joints present"
echo ""

echo "4. Testing xacro expansion..."
EXPANDED_URDF="/tmp/link6_calibrated_expanded_test.urdf"
xacro "$OUTPUT_FILE" gripper:=false robot_ip:=192.168.1.10 -o "$EXPANDED_URDF"
if [ $? -ne 0 ]; then
    echo "   ERROR: Xacro expansion failed"
    exit 1
fi
echo "   ✓ Xacro expanded successfully"

# Validate expanded URDF
xmllint "$EXPANDED_URDF" > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "   ERROR: Expanded URDF is not valid XML"
    exit 1
fi
echo "   ✓ Expanded URDF is valid XML"
echo ""

echo "============================================"
echo "✓ All tests passed!"
echo "============================================"
echo ""
echo "Test files created:"
echo "  - Calibrated macro: $OUTPUT_FILE"
echo "  - Expanded URDF: $EXPANDED_URDF"
echo ""
echo "You can inspect these files to verify the calibration was applied correctly."

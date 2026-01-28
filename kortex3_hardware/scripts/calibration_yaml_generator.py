#!/usr/bin/env python3
"""
This module reads a calibration file (XML or ZIP) and exports robot calibration
as a YAML file for dynamic loading in xacro files.

The YAML format allows calibration data to be loaded dynamically without
regenerating new URDF files.

Functions:
    export_calibration_to_yaml: Exports calibration data as YAML
"""
import argparse
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List
from zipfile import ZipFile, is_zipfile

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R
import yaml

np.set_printoptions(precision=6, suppress=False)


class CalibrationFile:
    """
    A class to parse and store calibration data from a given calibration file.
    """

    def __init__(self, xml_file: os.PathLike) -> None:
        """
        Initialize the CalibrationFile object by parsing the given XML file.

        Args:
            xml_file (os.PathLike): Path to a calibration XML file as exported from the Link 6 controller.

        Raises:
            FileNotFoundError: If the provided file path is incorrect.
        """
        file_path = Path(xml_file).resolve()

        if file_path.is_file():
            parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
            self.tree = ET.parse(str(file_path), parser=parser)
        else:
            print(f"{file_path} does not exist or is not a file.")
            raise FileNotFoundError("No calibration file found")

        self.serial_number = self._parse_serial_number()
        self.geometric_calibration = self._parse_geometric_calibration()

    def _parse_serial_number(self) -> str:
        """
        Reads the robot arm serial number from the calibration file.

        Returns:
            str: The serial number of the robot arm.
        """
        root_element = self.tree.getroot()
        serial_number = root_element.get("serialNumber", "UnknownSerialNumber")
        print(f"Found serial number: {serial_number}")
        return serial_number

    def _parse_geometric_calibration(self) -> List[npt.ArrayLike]:
        """
        Parses the joint geometric offsets and returns them as a list of homogeneous transformation matrices.

        Returns:
            List[npt.ArrayLike]: List of 4x4 numpy arrays representing the geometric calibration for each joint.
        """
        xpath_for_geometric_calibration = "./ArchitecturalCalibration/Transform"
        geometric_elements = self.tree.findall(xpath_for_geometric_calibration)

        geometric_calibration_matrices = [None] * len(geometric_elements)

        for element in geometric_elements:
            attributes = element.attrib
            joint_index = int(attributes.pop("index"))

            matrix = np.eye(4, dtype=np.float64, order="C")
            for key, value in attributes.items():
                row, col = int(key[1]), int(key[2])
                if not (row, col) <= matrix.shape:
                    raise ValueError(f"Input matrix wrong size: {(row, col)}")
                matrix[row, col] = np.float64(value)
            geometric_calibration_matrices[joint_index] = matrix

        return geometric_calibration_matrices


# Nominal joint transforms from the original URDF
NOMINAL_JOINT_TRANSFORMS = [
    {"xyz": [0, 0, 0.053], "rpy": [3.141592653589793, 0, 0]},  # joint_1
    {"xyz": [0.11024, -0.069257, -0.1375], "rpy": [-1.5707963267948966, 0, 0]},  # joint_2
    {"xyz": [0, 0.485, 0], "rpy": [-3.141592653589793, 0, 0]},  # joint_3
    {"xyz": [0, -0.15216, -0.091704], "rpy": [-1.5707963267948966, 0, 0]},  # joint_4
    {"xyz": [0, -0.062957, -0.22275], "rpy": [-1.5707963267948966, 0, 0]},  # joint_5
    {"xyz": [0.087028, 0.086, -0.076922], "rpy": [0, -1.5707963267948966, 0]},  # joint_6
]


def construct_transform_matrix(xyz: List[float], rpy: List[float]) -> np.ndarray:
    """
    Construct a 4x4 homogeneous transformation matrix from xyz and rpy.

    Args:
        xyz: Translation [x, y, z]
        rpy: Rotation in roll-pitch-yaw [r, p, y]

    Returns:
        4x4 homogeneous transformation matrix
    """
    T = np.eye(4, dtype=np.float64)
    rot = R.from_euler("xyz", rpy, degrees=False)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = xyz
    return T


def export_calibration_to_yaml(calib: CalibrationFile, output_path: Path) -> None:
    """
    Export calibration data as YAML for dynamic loading in xacro files.

    Args:
        calib: CalibrationFile object containing calibration data
        output_path: Path where the YAML file should be written
    """

    yaml_data = {
        "serial_number": calib.serial_number,
    }

    # Apply calibration to each joint
    for idx in range(6):
        joint_idx = idx + 1
        joint_name = f"joint_{joint_idx}"

        # Get nominal transformation
        nominal = NOMINAL_JOINT_TRANSFORMS[idx]
        T_nom = construct_transform_matrix(nominal["xyz"], nominal["rpy"])

        # Apply calibration: T_cal = T_calib * T_nom
        T_calib = calib.geometric_calibration[idx]
        T_cal = T_calib @ T_nom

        # Extract xyz and rpy from calibrated transform
        xyz = T_cal[:3, 3]
        rot = R.from_matrix(T_cal[:3, :3])
        rpy = rot.as_euler("xyz", degrees=False)

        # Store with high precision (17 decimal places for double precision)
        yaml_data[joint_name] = {
            "xyz": f"{xyz[0]:.17f} {xyz[1]:.17f} {xyz[2]:.17f}",
            "rpy": f"{rpy[0]:.17f} {rpy[1]:.17f} {rpy[2]:.17f}",
        }

        print(f"Calibrated {joint_name}: xyz=[{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]")

    # Write YAML file
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        # Add header comment
        f.write("# Robot-specific calibration data for Link6\n")
        f.write(f"# Serial number: {calib.serial_number}\n")
        f.write("# This file is auto-generated by calibration_yaml_generator.py\n")
        f.write("#\n")
        f.write("# To use this calibration in your launch file:\n")
        f.write("#   ros2 launch link6_bringup real_robot.launch.py \\\n")
        f.write(f"#       calibration_file:={output_path}\n")
        f.write("#\n\n")
        yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

    print(f"✓ Calibration YAML written to: {output_path}")


if __name__ == "__main__":
    # -------------------------------------------------------------------------
    # 1.  Command-line interface
    # -------------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        description="Generate robot-specific calibration data in YAML format for dynamic loading."
    )
    parser.add_argument(
        "--calibration_file",
        required=True,
        help="Path to the calibration bundle from the controller (either *.zip or calib.xml).",
    )
    parser.add_argument(
        "--output_file",
        default=None,
        help="Where to write the YAML file. If omitted, writes to calibration.yaml in the same directory as the input file.",
    )
    args = parser.parse_args()

    # -------------------------------------------------------------------------
    # 2.  Resolve paths
    # -------------------------------------------------------------------------
    cal_path = Path(args.calibration_file).expanduser().resolve()
    if not cal_path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {cal_path}")

    # -------------------------------------------------------------------------
    # 3.  Parse calibration bundle
    # -------------------------------------------------------------------------
    print(f"Loading calibration from: {cal_path}")
    if is_zipfile(cal_path):
        print("Extracting calib.xml from ZIP archive...")
        with ZipFile(cal_path) as zf, tempfile.TemporaryDirectory() as tmp:
            xml_tmp = Path(tmp) / "calib.xml"
            zf.extract("calib.xml", path=xml_tmp.parent)
            calib = CalibrationFile(xml_tmp)
    else:
        calib = CalibrationFile(cal_path)

    # -------------------------------------------------------------------------
    # 4.  Determine output path
    # -------------------------------------------------------------------------
    if args.output_file:
        out_path = Path(args.output_file).expanduser().resolve()
    else:
        # Default: place calibration.yaml next to calibration file
        out_path = cal_path.parent / "calibration.yaml"

    # -------------------------------------------------------------------------
    # 5.  Export to YAML
    # -------------------------------------------------------------------------
    try:
        export_calibration_to_yaml(calib, out_path)
        print("✓ Calibration export completed successfully")
    except Exception as e:
        print(f"✗ Calibration export failed: {e}")
        raise

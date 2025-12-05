"""
This module reads a calibration file (XML or ZIP) and applies robot calibration
offsets to a Xacro macro file. It preserves the macro structure including prefix
variables, properties, conditionals, and gripper integration.

Functions:
    apply_calibration_to_macro: Applies calibration data to a Xacro macro file.
"""
import argparse
import os
import tempfile
import tkinter as tk
import xml.etree.ElementTree as ET
from pathlib import Path
from tkinter import filedialog
from typing import List, Optional
from zipfile import ZipFile, is_zipfile

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R
import logging

logger = logging.getLogger("calibrated_urdf")
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
    handlers=[logging.FileHandler("calibrated_urdf.log"), logging.StreamHandler()],
)
np.set_printoptions(precision=6, suppress=False)


class CalibrationFile:
    """
    A class to parse and store calibration data from a given calibration file.
    """

    def __init__(self, zip_file: os.PathLike) -> None:
        """
        Initialize the CalibrationFile object by parsing the given zip file.

        Args:
            zip_file (os.PathLike): Path to a calibration signature file as exported from the Link 6 controller.

        Raises:
            FileNotFoundError: If the provided file path is incorrect.
        """
        file_path = Path(zip_file).resolve()

        if file_path.is_file():
            logger.info(f"Found calibration package at: {file_path}")
            parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
            self.tree = ET.parse(str(file_path), parser=parser)
        else:
            logger.error(f"{file_path} does not exist or is not a file.")
            raise FileNotFoundError("No calibration file found")

        self.serial_number = self._parse_serial_number()
        self.geometric_calibration = self._parse_geometric_calibration()
        self.elasto_calibration = self._parse_elasto_calibration()

    def _parse_serial_number(self) -> str:
        """
        Reads the robot arm serial number from the calibration file.

        Returns:
            str: The serial number of the robot arm.
        """
        root_element = self.tree.getroot()
        serial_number = root_element.get("serialNumber", "UnknownSerialNumber")
        logger.info(f"Found serial number: {serial_number}")
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

    def _parse_elasto_calibration(self) -> List[npt.ArrayLike]:
        """
        Parses the joint Elasto-Static calibration weights and returns them as a list of 6x6 weight arrays.

        Returns:
            List[npt.ArrayLike]: List of 6x6 numpy arrays representing the Elasto-Static calibration for each joint.
        """
        xpath_for_elasto_calibration = "./ElastoStaticCalibration/Compliance"
        elasto_elements = self.tree.findall(xpath_for_elasto_calibration)

        elasto_calibration_matrices = [None] * len(elasto_elements)

        for element in elasto_elements:
            attributes = element.attrib
            joint_index = int(attributes.pop("index"))

            matrix = np.eye(6, dtype=np.float64, order="C")
            for key, value in attributes.items():
                row, col = int(key[1]), int(key[2])
                if not (row, col) <= matrix.shape:
                    raise ValueError(f"Input matrix wrong size: {(row, col)}")
                matrix[row, col] = np.float64(value)
            elasto_calibration_matrices[joint_index] = matrix

        return elasto_calibration_matrices


def find_joint_in_xacro(root: ET.Element, joint_name: str) -> Optional[ET.Element]:
    """
    Find a joint element in a Xacro file by name, handling both regular names
    and names with prefix variables like ${prefix}joint_1.

    Args:
        root: Root element of the XML tree or macro element
        joint_name: Name of the joint to find (e.g., "joint_1" or "${prefix}joint_1")

    Returns:
        The joint element if found, None otherwise
    """
    # Try to find joint with exact name match
    for joint in root.iter("joint"):
        if joint.get("name") == joint_name:
            return joint

    # If not found and joint_name doesn't have prefix, try with ${prefix}
    if not joint_name.startswith("${prefix}"):
        prefixed_name = f"${{prefix}}{joint_name}"
        for joint in root.iter("joint"):
            if joint.get("name") == prefixed_name:
                return joint

    return None


def parse_origin_to_matrix(origin_elem: ET.Element) -> np.ndarray:
    """
    Parse an origin element's xyz and rpy attributes into a 4x4 transformation matrix.

    Args:
        origin_elem: The origin XML element

    Returns:
        4x4 homogeneous transformation matrix
    """
    xyz_str = origin_elem.get("xyz", "0 0 0")
    rpy_str = origin_elem.get("rpy", "0 0 0")

    xyz = np.array([float(x) for x in xyz_str.split()])
    rpy = np.array([float(x) for x in rpy_str.split()])

    # Create transformation matrix
    rot = R.from_euler("xyz", rpy, degrees=False)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = xyz

    return T


def update_origin_element(origin_elem: ET.Element, T: np.ndarray) -> None:
    """
    Update an origin element's xyz and rpy attributes from a transformation matrix.

    Args:
        origin_elem: The origin XML element to update
        T: 4x4 homogeneous transformation matrix
    """
    xyz = T[:3, 3]
    rot = R.from_matrix(T[:3, :3])
    rpy = rot.as_euler("xyz", degrees=False)

    # Format with high precision to maintain calibration accuracy
    xyz_str = f"{xyz[0]:.17f} {xyz[1]:.17f} {xyz[2]:.17f}"
    rpy_str = f"{rpy[0]:.17f} {rpy[1]:.17f} {rpy[2]:.17f}"

    origin_elem.set("xyz", xyz_str)
    origin_elem.set("rpy", rpy_str)


def apply_calibration_to_macro(
    macro_path: Path,
    calib: "CalibrationFile",
    output_path: Path
) -> None:
    """
    Apply calibration offsets to joints in a Xacro macro file.
    Preserves the macro structure including prefix variables, properties, and conditionals.

    Args:
        macro_path: Path to the input Xacro macro file
        calib: CalibrationFile object containing calibration data
        output_path: Path where the calibrated macro should be written
    """
    logger.info(f"Loading Xacro macro from: {macro_path}")

    # Register xacro namespace to preserve it in output
    ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")

    # Parse the Xacro file as XML (don't expand it)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(str(macro_path), parser=parser)
    root = tree.getroot()

    logger.info("Applying calibration to joints...")

    # Apply calibration to each joint (joint_1 through joint_6)
    joints_calibrated = 0
    for idx in range(6):
        joint_idx = idx + 1
        joint_name = f"joint_{joint_idx}"

        # Try to find the joint (handles both "joint_N" and "${prefix}joint_N")
        joint = find_joint_in_xacro(root, joint_name)

        if joint is None:
            logger.warning(f"Could not find {joint_name} in macro")
            continue

        # Find the origin element
        origin = joint.find("origin")
        if origin is None:
            logger.warning(f"{joint_name} has no origin element")
            continue

        # Get nominal transformation
        T_nom = parse_origin_to_matrix(origin)

        # Apply calibration: T_cal = T_calib * T_nom
        T_calib = calib.geometric_calibration[idx]
        T_cal = T_calib @ T_nom

        # Update the origin element
        update_origin_element(origin, T_cal)

        logger.info(f"Calibrated {joint.get('name')}: xyz={T_cal[:3, 3]}, "
                   f"rpy={R.from_matrix(T_cal[:3, :3]).as_euler('xyz', degrees=False)}")
        joints_calibrated += 1

    if joints_calibrated != 6:
        logger.warning(f"Only calibrated {joints_calibrated}/6 joints")
    else:
        logger.info("Successfully calibrated all 6 joints")

    # Write the calibrated macro
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Write with proper XML declaration and formatting
    with open(output_path, "wb") as f:
        tree.write(f, encoding="utf-8", xml_declaration=True)

    logger.info(f"Calibrated macro written to: {output_path}")


def get_calibration_file_path() -> Path:
    """
    Opens a file dialog to select the calibration package file and returns its path.

    Returns:
        Path: The file path of the selected calibration package.

    Raises:
        FileNotFoundError: If the selected file does not exist.
    """
    root = tk.Tk()
    root.withdraw()

    file_name = filedialog.askopenfilename(
        defaultextension=".zip",
        title="Select path to calibration bundle file",
        filetypes=(
            ("Zip files", "*.zip"),
            ("XML files", "*.xml"),
            ("all files", "*.*"),
        ),
    )

    root.destroy()

    # Check if the user has selected a file or cancelled the dialog
    if not file_name:
        raise ValueError("No file selected. Exiting program.")

    selected_file_path = Path(file_name)

    if selected_file_path.is_file():
        return selected_file_path
    else:
        raise FileNotFoundError(
            f"{selected_file_path} does not exist or is not a file."
        )


if __name__ == "__main__":
    # -------------------------------------------------------------------------
    # 1.  Command-line interface
    # -------------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        description="Generate a robot-specific calibrated Xacro macro."
    )
    parser.add_argument(
        "--urdf_path", required=True,
        help="Absolute path to the nominal Xacro macro file (e.g., link6_macro.xacro)."
    )
    parser.add_argument(
        "--calibration_file", required=True,
        help="Path to the calibration bundle from the controller "
             "(either *.zip or calib.xml)."
    )
    parser.add_argument(
        "--output_file", default=None,
        help="Where to write the calibrated macro. "
             "If omitted, it is placed next to the input file with "
             "'_calibrated' suffix (e.g., link6_calibrated_macro.xacro)."
    )
    args = parser.parse_args()

    # -------------------------------------------------------------------------
    # 2.  Resolve paths
    # -------------------------------------------------------------------------
    cal_path = Path(args.calibration_file).expanduser().resolve()
    if not cal_path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {cal_path}")

    macro_path = Path(args.urdf_path).expanduser().resolve()
    if not macro_path.is_file():
        raise FileNotFoundError(f"Macro file not found: {macro_path}")
    logger.info(f"Input macro file: {macro_path}")

    # -------------------------------------------------------------------------
    # 3.  Parse calibration bundle
    # -------------------------------------------------------------------------
    if is_zipfile(cal_path):
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
        # Generate output name from input (e.g., link6_macro.xacro -> link6_calibrated_macro.xacro)
        stem = macro_path.stem  # e.g., "link6_macro"
        if stem.endswith("_macro"):
            out_stem = stem.replace("_macro", "_calibrated_macro")
        else:
            out_stem = f"{stem}_calibrated"
        out_path = macro_path.with_name(f"{out_stem}.xacro")

    logger.info(f"Output will be written to: {out_path}")

    # -------------------------------------------------------------------------
    # 5.  Apply calibration to macro
    # -------------------------------------------------------------------------
    try:
        apply_calibration_to_macro(macro_path, calib, out_path)
        logger.info("✓ Calibration completed successfully")
    except Exception as e:
        logger.error(f"✗ Calibration failed: {e}")
        raise
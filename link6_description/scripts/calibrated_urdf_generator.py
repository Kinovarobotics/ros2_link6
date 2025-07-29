"""
This module reads a zip file containing an XML file with joint offsets and a nominal URDF file. It then applies the robot calibration offset found in the XML file and outputs a modified URDF with the specific calibration information.

Functions:
    apply_calibration(zip_file: str) -> str:
        Takes in a filepath to a zip file containing an XML file with joint offsets and a nominal URDF file.
        Returns a string of the modified URDF with the specific calibration information applied.
"""
import argparse
import os
import sys                     
import tempfile
import tkinter as tk
import xml.etree.ElementTree as ET
from pathlib import Path
from tkinter import filedialog
from typing import List
from zipfile import ZipFile, is_zipfile

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R
import logging

SCRIPT_DIR = Path(__file__).resolve().parent   
sys.path.insert(0, str(SCRIPT_DIR))          
from urdf_parser import urdf                 



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
        description="Generate a robot-specific calibrated Xacro."
    )
    parser.add_argument(
        "--urdf", default="link6_nominal",
        help="Base name (without extension) of the *nominal* URDF inside "
             "`<pkg>/urdf/`. Ignored when --urdf_path is given."
    )
    parser.add_argument(
        "--urdf_path", default=None,
        help="Absolute path to the nominal URDF file. "
             "Takes precedence over --urdf."
    )
    parser.add_argument(
        "--calibration_file", required=True,
        help="Path to the bundle coming from the controller "
             "(either *.zip or calib.xml)."
    )
    parser.add_argument(
        "--output_file", default=None,
        help="Where to write the calibrated model. "
             "If omitted, it is placed next to the nominal URDF and given "
             'the name "link6_calibrated.xacro".'
    )
    args = parser.parse_args()

    # -------------------------------------------------------------------------
    # 2.  Resolve paths
    # -------------------------------------------------------------------------
    cal_path = Path(args.calibration_file).expanduser().resolve()
    if not cal_path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {cal_path}")

    # nominal URDF (already expanded from Xacro by the C++ side)
    if args.urdf_path:
        urdf_file_path = Path(args.urdf_path).expanduser().resolve()
    else:
        urdf_file_path = Path("urdf").resolve() / f"{args.urdf}.urdf"
    if not urdf_file_path.is_file():
        raise FileNotFoundError(f"Nominal URDF missing: {urdf_file_path}")
    logger.debug("Nominal URDF: %s", urdf_file_path)

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
    # 4.  Load nominal model & apply offsets
    # -------------------------------------------------------------------------
    robot = urdf.Robot.from_xml_file(urdf_file_path)
    for idx, joint in enumerate(robot.joints):
        T_nom = joint.origin.to_matrix()
        T_cal = calib.geometric_calibration[idx] @ T_nom

        rot = R.from_matrix(T_cal[:3, :3])
        joint.origin.xyz = T_cal[:3, 3].tolist()
        joint.origin.rpy = rot.as_euler("xyz", degrees=False).tolist()

        logger.debug("joint_%d xyz=%s rpy=%s",
                     idx + 1, joint.origin.xyz, joint.origin.rpy)

    # -------------------------------------------------------------------------
    # 5.  Write calibrated **Xacro**
    # -------------------------------------------------------------------------
    from urdf_parser.xml_reflection.basics import xml_string

    # Build new XML tree
    root = robot.to_xml()
    root.set("name", "link6_calibrated")
    root.set("xmlns:xacro", "http://ros.org/wiki/xacro")

    # Decide output location
    if args.output_file:
        out_path = Path(args.output_file).expanduser().resolve()
    else:
        out_path = urdf_file_path.with_name("link6_calibrated.xacro")
    out_path = out_path.with_suffix(".xacro")  # enforce suffix
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(out_path, "w") as f:
        f.write('<?xml version="1.0" encoding="utf-8"?>\n')
        f.write(xml_string(root, addHeader=False))

    logger.info("Calibrated Xacro written to %s", out_path)
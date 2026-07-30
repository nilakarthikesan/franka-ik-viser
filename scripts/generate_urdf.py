"""Generate the FR3 URDF from the franka_description checkout (M0).

franka_description builds URDFs from xacro files that use ROS's
`$(find franka_description)` substitution. That substitution needs
`ament_index_python`, which is a ROS package not published on PyPI. Since the
only thing xacro calls is `get_package_share_directory`, we shim that single
function to resolve `franka_description` to our checkout, then run the
official `scripts/create_urdf.py` unmodified.

Afterwards we rewrite `package://franka_description/...` mesh URIs to paths
relative to the URDF file, so yourdfpy / Viser can load meshes without any
ROS machinery (DESIGN.md §3 flags absolute paths as a known pitfall).

Usage:  python scripts/generate_urdf.py
Output: assets/franka_description/urdfs/fr3_franka_hand.urdf
"""

import runpy
import sys
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FRANKA_DESC = REPO_ROOT / "assets" / "franka_description"


def install_ament_shim() -> None:
    packages = types.ModuleType("ament_index_python.packages")

    def get_package_share_directory(package_name: str) -> str:
        if package_name == "franka_description":
            return str(FRANKA_DESC)
        raise KeyError(f"shim only resolves franka_description, got {package_name!r}")

    packages.get_package_share_directory = get_package_share_directory
    ament = types.ModuleType("ament_index_python")
    ament.packages = packages
    sys.modules["ament_index_python"] = ament
    sys.modules["ament_index_python.packages"] = packages


def generate() -> Path:
    install_ament_shim()
    import os

    os.chdir(FRANKA_DESC)  # create_urdf.py expects to run from the repo root
    sys.argv = ["create_urdf.py", "fr3", "--robot-ee", "franka_hand"]
    runpy.run_path(str(FRANKA_DESC / "scripts" / "create_urdf.py"), run_name="__main__")

    urdfs = sorted((FRANKA_DESC / "urdfs").glob("*.urdf"))
    assert urdfs, "create_urdf.py produced no URDF"
    return urdfs[-1]


def rewrite_mesh_paths(urdf_path: Path) -> None:
    # URDF lives in urdfs/, meshes in ../meshes -> make URIs relative to the URDF.
    text = urdf_path.read_text()
    n = text.count("package://franka_description/")
    text = text.replace("package://franka_description/", "../")
    urdf_path.write_text(text)
    print(f"Rewrote {n} package:// URIs to relative paths in {urdf_path}")


if __name__ == "__main__":
    urdf = generate()
    rewrite_mesh_paths(urdf)
    print(f"Generated: {urdf}")

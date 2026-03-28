import time
import rclpy
from rclpy.node import Node

from pathlib import Path
from typing import Optional, List


class MapManager:

    def __init__(self, node: Node, package_map_dir: str):
        self._node = node
        base_dir = Path.home() / ".phaseshift" / "maps"

        self.runtime_2d_dir = base_dir / "2d"
        self.runtime_3d_dir = base_dir / "3d"

        self.runtime_2d_dir.mkdir(parents=True, exist_ok=True)
        self.runtime_3d_dir.mkdir(parents=True, exist_ok=True)

        self.package_map_dir = Path(package_map_dir)

    # ==================================================
    # 2D MAP (Nav2)
    # ==================================================

    def generate_2d_map_path(self, name: str) -> str:
        timestamp = int(time.time())
        base = self.runtime_2d_dir / f"{name}_{timestamp}"
        return str(base)

    def has_runtime_2d_map(self) -> bool:
        count = len(list(self.runtime_2d_dir.glob("*.yaml")))
        self._log_message(f"Currently there are {count} runtime maps")
        return count > 0

    def get_latest_2d_map(self) -> Optional[str]:
        maps = list(self.runtime_2d_dir.glob("*.yaml"))
        if not maps:
            self._log_message("There are no runtime maps...")
            return None
        
        self._log_message("Trying to get runtime map...")
        return str(max(maps, key=lambda p: p.stat().st_mtime))

    def resolve_2d_map(self, name: str, use_example: bool = False) -> Optional[str]:

        if use_example:
            map_yaml = self.package_map_dir / f"{name}.yaml"
            if map_yaml.exists():
                return str(map_yaml)
            return None

        # runtime
        map_yaml = self.runtime_2d_dir / f"{name}.yaml"
        if map_yaml.exists():
            return str(map_yaml)
        
        return None

    def list_all_2d_maps(self) -> List[str]:
        runtime = [p.stem for p in self.runtime_2d_dir.glob("*.yaml")]
        builtin = [p.stem for p in self.package_map_dir.glob("*.yaml")]
        return list(set(runtime + builtin))

    # ==================================================
    # 3D MAP (Visualization only)
    # ==================================================

    def generate_3d_map_path(self, name: str) -> str:
        timestamp = int(time.time())
        base = self.runtime_3d_dir / f"{name}_{timestamp}"
        return str(base)

    def list_3d_maps(self) -> List[str]:
        return [p.stem for p in self.runtime_3d_dir.glob("*")]

    def resolve_3d_map(self, name: str) -> Optional[str]:
        for ext in [".bt", ".pcd"]:
            path = self.runtime_3d_dir / f"{name}{ext}"
            if path.exists():
                return str(path)
        return None
    
    # ==================================================
    # UTILS
    # ==================================================
    def _log_message(self, msg: str):
        self._node.get_logger().info(f"\033[93m[{msg}]\033[0m")

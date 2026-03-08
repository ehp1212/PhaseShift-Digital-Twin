import os
import glob
import time

class MapManager:
    def __init__(self):
        self.map_directory = os.path.expanduser("~/maps")
        os.makedirs(self.map_directory, exist_ok=True)

    # ------------------------------
    # Map Save Path
    # ------------------------------
    def create_map_paths(self, map_name: str):
        timestamp = int(time.time())
        base_name = f"{map_name}_{timestamp}"

        base_path = os.path.join(self.map_directory, base_name)

        return{
            "base": base_path,
            "yaml": base_path + "./yaml",
            "pgm": base_path + "./pgm"
        }

    # ------------------------------
    # Base -> Yaml
    # ------------------------------
    def base_to_yaml(self, base_path: str):
        return base_path + "./yaml"
    
    # ------------------------------
    # Check map exists
    # ------------------------------
    def has_map(self) -> bool:
        maps = glob.glob(os.path.join(self.map_directory, "*.yaml")) 
        return len(maps) > 0
    
    # ------------------------------
    # Latest map
    # ------------------------------
    def get_latest_map(self):
        maps = glob.glob(os.path.join(self.map_directory, "*.yaml")) 
        if not maps:
            return None
        
        maps.sort(key=os.path.getmtime, reverse=True)
        return maps[0]
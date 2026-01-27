"""
Unified Data Loader.
Handles loading of YAML configurations and CSV flight data.
Supports Absolute Paths, Relative Paths, and Project-Relative Paths.

统一数据加载器。
支持绝对路径、相对路径和项目相对路径。
"""
import yaml
import pandas as pd
import os

class Loader:
    @staticmethod
    def _get_project_root():
        """
        Finds the project root directory.
        Assumes this file is located in <ProjectRoot>/tools/
        """
        current_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.dirname(current_dir)

    @staticmethod
    def _resolve_path(path: str, search_paths: list = None):
        """
        Helper to find a file in multiple potential locations.
        Priority:
        1. Absolute Path (e.g., /Users/river/data.csv)
        2. Relative to Current Working Directory (e.g., ./data.csv)
        3. Relative to Project Root (e.g., configs/aircraft.yaml)
        4. Custom search paths (e.g., examples/)
        """
        # 1. Check if it's an absolute path
        if os.path.isabs(path):
            if os.path.exists(path):
                return path
            else:
                # If absolute path provided but not found, fail immediately
                return None

        # 2. Check relative to Current Working Directory (CWD)
        if os.path.exists(path):
            return os.path.abspath(path)

        # 3. Check relative to Project Root
        root = Loader._get_project_root()
        project_path = os.path.join(root, path)
        if os.path.exists(project_path):
            return project_path

        # 4. Check custom search paths (if provided)
        if search_paths:
            for sub_dir in search_paths:
                # Try root/sub_dir/path
                candidate = os.path.join(root, sub_dir, path)
                if os.path.exists(candidate):
                    return candidate

                # Try cwd/sub_dir/path (less likely but possible)
                candidate_cwd = os.path.join(os.getcwd(), sub_dir, path)
                if os.path.exists(candidate_cwd):
                    return candidate_cwd

        return None

    @staticmethod
    def load_yaml(path: str):
        """
        Loads a YAML config file.
        """
        # Configs are usually in 'configs/' folder if not found directly
        resolved_path = Loader._resolve_path(path, search_paths=['configs'])

        if not resolved_path:
            raise FileNotFoundError(f"Config file not found: {path}")

        with open(resolved_path, 'r') as f:
            return yaml.safe_load(f)

    @staticmethod
    def load_csv(path: str):
        """
        Loads a CSV file into a Pandas DataFrame.
        """
        # Data might be in 'examples/' or root
        resolved_path = Loader._resolve_path(path, search_paths=['examples'])

        if not resolved_path:
            raise FileNotFoundError(f"CSV file not found: {path}")

        print(f"Loading data from: {resolved_path}")
        return pd.read_csv(resolved_path)
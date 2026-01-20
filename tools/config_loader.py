"""
Utility to load configuration parameters from YAML files.

负责从 YAML 文件加载配置参数。
"""
import yaml
import os

class ConfigLoader:
    @staticmethod
    def load_yaml(filepath: str):
        """
        Loads a YAML file and returns a dictionary.
        加载 YAML 文件并返回字典。
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Config file not found: {filepath}")
        with open(filepath, 'r') as f:
            return yaml.safe_load(f)
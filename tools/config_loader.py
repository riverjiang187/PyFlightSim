"""
Utility to load configuration parameters from YAML files.
Handles absolute path resolution to ensure robustness across different execution contexts.

负责从 YAML 文件加载配置参数。
处理绝对路径解析，确保在不同执行环境下（如跨目录运行）的健壮性。
"""
import yaml
import os

class ConfigLoader:
    @staticmethod
    def load_yaml(relative_path: str):
        """
        Loads a YAML file and returns a dictionary.
        Resolves path relative to the Project Root.

        加载 YAML 文件并返回字典。
        基于项目根目录解析路径。

        Args:
            relative_path: Path relative to project root (e.g., "configs/aircraft.yaml")
        """
        # 1. Get the directory of this script (tools/)
        #    获取当前脚本 (tools/) 的绝对路径
        current_script_dir = os.path.dirname(os.path.abspath(__file__))

        # 2. Go up one level to get Project Root (FlightSim/)
        #    向上一级获取项目根目录 (FlightSim/)
        project_root = os.path.dirname(current_script_dir)

        # 3. Construct full absolute path
        #    构建完整的绝对路径
        full_path = os.path.join(project_root, relative_path)

        if not os.path.exists(full_path):
            # Print current working directory to help debugging
            # 打印当前工作目录以辅助调试
            print(f"DEBUG: CWD is {os.getcwd()}")
            raise FileNotFoundError(f"Config file not found at: {full_path}")

        with open(full_path, 'r') as f:
            return yaml.safe_load(f)
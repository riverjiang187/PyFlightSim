import os

# ================= 配置区域 =================
# 输出文件名
OUTPUT_FILE = "project_source_code.md"

# 要忽略的文件夹 (黑名单)
IGNORE_DIRS = {
    '__pycache__',
    '.git',
    '.idea',
    '.vscode',
    'venv',
    '.venv',
    'build',
    'dist',
    'egg-info'
}

# 要忽略的文件后缀 (黑名单)
# 注意：一定要忽略 .csv，因为飞行数据太长了，不需要给 AI 看
IGNORE_EXTENSIONS = {
    '.pyc', '.pyd', '.so', '.dll',  # 编译文件
    '.png', '.jpg', '.jpeg', '.gif', '.ico',  # 图片
    '.csv', '.log', '.txt', '.md',  # 数据和日志 (排除 .md 防止递归读取自己)
    '.DS_Store'  # Mac 系统文件
}

# 指定需要包含的特定后缀 (白名单)
# 如果只想导出代码和配置，可以严格限制这里
INCLUDE_EXTENSIONS = {'.py', '.yaml', '.yml', '.json', '.ini'}


# ===========================================

def get_tree_structure(startpath):
    """生成项目的目录树字符串"""
    tree_str = f"{os.path.basename(os.path.abspath(startpath))}/\n"

    for root, dirs, files in os.walk(startpath):
        # 修改 dirs 列表以跳过忽略的目录 (原地修改，影响 os.walk 的后续遍历)
        dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]

        level = root.replace(startpath, '').count(os.sep)
        indent = '│   ' * (level)
        subindent = '├── '

        if root != startpath:
            tree_str += f"{indent}{os.path.basename(root)}/\n"

        # 过滤文件
        filtered_files = []
        for f in files:
            _, ext = os.path.splitext(f)
            if f == os.path.basename(__file__) or f == OUTPUT_FILE:
                continue
            if ext in IGNORE_EXTENSIONS:
                continue
            # 如果定义了白名单，只包含白名单内的文件
            if INCLUDE_EXTENSIONS and ext not in INCLUDE_EXTENSIONS:
                continue
            filtered_files.append(f)

        for i, f in enumerate(sorted(filtered_files)):
            if root != startpath:
                file_indent = '│   ' * (level + 1) + '├── '
            else:
                file_indent = '├── '
            tree_str += f"{file_indent}{f}\n"

    return tree_str


def get_file_content(filepath):
    """读取文件内容"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception as e:
        return f"Error reading file: {e}"


def get_language_tag(filename):
    """根据后缀返回 Markdown 代码块的语言标签"""
    if filename.endswith('.py'): return 'python'
    if filename.endswith(('.yaml', '.yml')): return 'yaml'
    if filename.endswith('.json'): return 'json'
    return ''


def main():
    root_dir = os.getcwd()
    print(f"正在扫描项目: {root_dir}")

    # 1. 生成目录树
    print("正在生成目录结构图...")
    tree = get_tree_structure(root_dir)

    content_buffer = []

    # 写入头部信息
    content_buffer.append(f"# Project Export: {os.path.basename(root_dir)}\n")
    content_buffer.append("## Project Structure\n")
    content_buffer.append("```text")
    content_buffer.append(tree)
    content_buffer.append("```\n")
    content_buffer.append("---\n")
    content_buffer.append("## File Contents\n")

    # 2. 遍历读取文件
    print("正在提取文件内容...")
    file_count = 0

    for root, dirs, files in os.walk(root_dir):
        # 过滤目录
        dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]

        for file in sorted(files):
            _, ext = os.path.splitext(file)

            # 过滤逻辑
            if file == os.path.basename(__file__) or file == OUTPUT_FILE: continue
            if ext in IGNORE_EXTENSIONS: continue
            if INCLUDE_EXTENSIONS and ext not in INCLUDE_EXTENSIONS: continue

            filepath = os.path.join(root, file)
            rel_path = os.path.relpath(filepath, root_dir)

            print(f"  -> Reading: {rel_path}")

            file_content = get_file_content(filepath)
            lang = get_language_tag(file)

            # 格式化为 Markdown
            content_buffer.append(f"### File: `{rel_path}`\n")
            content_buffer.append(f"```{lang}")
            content_buffer.append(file_content)
            content_buffer.append("```\n")
            content_buffer.append("---\n")
            file_count += 1

    # 3. 写入最终文件
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write('\n'.join(content_buffer))

    print(f"\n✅ 完成！")
    print(f"共提取了 {file_count} 个文件。")
    print(f"结果已保存至: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
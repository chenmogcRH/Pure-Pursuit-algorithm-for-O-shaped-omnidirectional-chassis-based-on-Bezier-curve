# 发布到 GitHub 前的清单

这个目录目前嵌在原比赛工程里。**不要在原工程根目录运行 `git add .`、修改其 `origin`，或把原工程历史推送到新仓库。**

1. 确认根目录的 `LICENSE` 中“崔晓”是你希望公开显示的署名；本项目已按 MIT License 准备好许可文件。
2. 将整个 `open-source-pure-pursuit` 目录**复制到原工程之外**，作为一个全新文件夹。复制时不用带上被 `.gitignore` 忽略的 `build/`。
3. 在新文件夹内重新运行 README 中的构建与测试命令，核对示例输出和文档。
4. 在 GitHub 新建**空仓库**，不要勾选初始化 README、许可证或 `.gitignore`（本目录已有 README 与 `.gitignore`）。
5. 使用 GitHub Desktop 的“Add an Existing Repository”并按提示创建仓库、提交、发布；或在新文件夹中运行以下命令，将 URL 换成自己的空仓库地址：

   ```sh
   git init -b main
   git add README.md PUBLISHING.md CMakeLists.txt .gitignore include src examples tests docs LICENSE
   git diff --cached --stat
   git commit -m "Initial algorithm release"
   git remote add origin https://github.com/YOUR_NAME/YOUR_REPO.git
   git push -u origin main
   ```

6. 发布后检查 GitHub 文件清单：只应有算法源码、文档、示例和测试，不应出现原比赛路线、设备配置、构建产物或原工程提交历史。

此仓库源码只使用 C++ 标准库；CMake 负责构建，不需要特定机器人 SDK。

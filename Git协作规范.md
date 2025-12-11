# Git 协作规范与冲突处理指南

## 目录
1. [提交前检查](#1-提交前检查)
2. [正确的Git工作流程](#2-正确的git工作流程)
3. [遇到冲突时的正确处理](#3-遇到冲突时的正确处理)
4. [使用分支工作（推荐）](#4-使用分支工作推荐)
5. [团队协作规范](#5-团队协作规范)
6. [配置Git忽略编译产物](#6-配置git忽略编译产物)
7. [紧急情况处理](#7-紧急情况处理)
8. [使用可视化工具](#8-使用可视化工具)
9. [关键原则](#9-关键原则)

---

## 1. 提交前检查

每次提交代码前，先检查是否有冲突标记：

```bash
# 检查是否有冲突标记
git diff --check

# 或者搜索冲突标记
git grep "<<<<<<< HEAD"
git grep "======="
git grep ">>>>>>>"
```

⚠️ **如果发现冲突标记，绝对不要提交！**

---

## 2. 正确的Git工作流程

### 开始工作前

```bash
# 1. 先拉取最新代码
git pull origin main

# 2. 如果有冲突，会提示你
# 3. 解决冲突后再开始工作
```

### 提交代码时

```bash
# 1. 查看修改了哪些文件
git status

# 2. 添加要提交的文件（不要用 git add .）
git add Core/Src/main.c
git add Core/Inc/main.h

# 3. 提交前再次检查
git diff --cached

# 4. 提交
git commit -m "描述你的修改"

# 5. 推送前先拉取
git pull origin main

# 6. 如果有冲突，解决后再推送
git push origin main
```

---

## 3. 遇到冲突时的正确处理

当 `git pull` 或 `git merge` 出现冲突时：

### 步骤 1：查看冲突文件

```bash
git status
```

### 步骤 2：理解冲突标记

打开冲突文件，你会看到：

```c
<<<<<<< HEAD
你的代码
=======
别人的代码
>>>>>>> branch-name
```

### 步骤 3：手动解决冲突

- 删除冲突标记（`<<<<<<<`、`=======`、`>>>>>>>`）
- 保留正确的代码（可能需要合并两边的修改）
- 确保代码语法正确

### 步骤 4：标记冲突已解决

```bash
git add 文件名
```

### 步骤 5：完成合并

```bash
git commit
```

### 步骤 6：推送

```bash
git push origin main
```

---

## 4. 使用分支工作（推荐）

使用分支可以有效避免冲突：

```bash
# 1. 创建自己的分支
git checkout -b feature/your-name

# 2. 在自己的分支上工作
git add .
git commit -m "your changes"

# 3. 推送到自己的分支
git push origin feature/your-name

# 4. 在GitHub上创建Pull Request
# 5. 合并前解决冲突
# 6. 合并到main分支
```

### 分支命名规范

- `feature/功能名` - 新功能开发
- `fix/问题描述` - Bug修复
- `docs/文档名` - 文档更新
- `refactor/模块名` - 代码重构

---

## 5. 团队协作规范

### 文件分工

- ✅ **避免多人同时修改同一个文件**
- ✅ 如果必须修改，提前沟通
- ✅ 使用模块化设计，每人负责不同的文件

### 提交频率

- ✅ **小步提交**：每完成一个小功能就提交
- ✅ **频繁拉取**：每天开始工作前先 `git pull`
- ✅ **及时推送**：完成后尽快推送，避免积累太多改动

### 提交信息规范

```bash
# 好的提交信息
git commit -m "添加OLED显示功能"
git commit -m "修复I2C通信超时问题"
git commit -m "更新README文档"

# 不好的提交信息
git commit -m "update"
git commit -m "fix"
git commit -m "test"
```

---

## 6. 配置Git忽略编译产物

确保 `.gitignore` 包含这些内容：

```gitignore
# Keil编译产物
MDK-ARM/DebugConfig/
MDK-ARM/RTE/
MDK-ARM/*.uvguix.*
MDK-ARM/test1119/
*.o
*.d
*.crf
*.lst
*.map
*.axf
*.htm
*.lnp
*.bak
*.dep
*.uvguix
*.uvopt

# 其他临时文件
*.log
*.tmp
*~

# 操作系统文件
.DS_Store
Thumbs.db
```

---

## 7. 紧急情况处理

### 情况1：不小心提交了冲突标记到远程仓库

```bash
# 1. 回退到上一个正确的提交
git log  # 找到正确的commit ID
git reset --hard <commit-id>

# 2. 强制推送（谨慎使用！会覆盖远程仓库）
git push origin main --force

# 3. 通知团队成员重新拉取
```

### 情况2：本地代码混乱，想要重新开始

```bash
# 1. 备份当前修改（如果需要）
git stash

# 2. 强制重置到远程最新状态
git fetch origin
git reset --hard origin/main

# 3. 清理未跟踪的文件
git clean -fd

# 4. 如果需要恢复之前的修改
git stash pop
```

### 情况3：误删了重要文件

```bash
# 恢复单个文件
git checkout HEAD -- 文件路径

# 恢复所有文件
git reset --hard HEAD
```

---

## 8. 使用可视化工具

推荐使用Git GUI工具，更容易发现和解决冲突：

### VS Code（推荐）
- 内置Git支持
- 冲突高亮显示
- 一键解决冲突
- 安装插件：GitLens

### 其他工具
- **GitHub Desktop** - 简单易用，适合初学者
- **SourceTree** - 功能强大，可视化分支管理
- **GitKraken** - 界面美观，跨平台

### VS Code 解决冲突步骤

1. 打开有冲突的文件
2. VS Code会高亮显示冲突区域
3. 点击 "Accept Current Change" 或 "Accept Incoming Change"
4. 或者点击 "Accept Both Changes" 然后手动编辑
5. 保存文件
6. 在源代码管理面板中暂存文件
7. 提交

---

## 9. 关键原则

### ✅ 应该做的

- ✅ **永远不要提交包含冲突标记的代码**
- ✅ **提交前先拉取，推送前再拉取**
- ✅ **小步提交，频繁同步**
- ✅ **遇到冲突立即解决，不要拖延**
- ✅ **使用分支隔离开发**
- ✅ **写清晰的提交信息**
- ✅ **提交前检查代码能否编译**
- ✅ **定期清理本地分支**

### ❌ 不应该做的

- ❌ **不要使用 `git add .` 盲目添加所有文件**
- ❌ **不要长时间不同步代码**
- ❌ **不要在有冲突时强制推送**
- ❌ **不要提交编译产物和临时文件**
- ❌ **不要直接在main分支上开发**
- ❌ **不要忽略Git的警告信息**

---

## 快速参考命令

```bash
# 日常工作流
git pull origin main          # 拉取最新代码
git status                    # 查看状态
git add 文件名                # 添加文件
git commit -m "提交信息"      # 提交
git push origin main          # 推送

# 分支操作
git branch                    # 查看分支
git checkout -b 分支名        # 创建并切换分支
git checkout main             # 切换到main分支
git merge 分支名              # 合并分支

# 撤销操作
git checkout -- 文件名        # 撤销文件修改
git reset HEAD 文件名         # 取消暂存
git reset --hard HEAD         # 重置所有修改

# 查看历史
git log                       # 查看提交历史
git log --oneline             # 简洁查看历史
git diff                      # 查看修改

# 冲突处理
git status                    # 查看冲突文件
git add 文件名                # 标记冲突已解决
git commit                    # 完成合并
```

---

## 常见问题 FAQ

### Q: 为什么会出现冲突？
A: 当两个人修改了同一个文件的同一部分，Git无法自动合并时就会产生冲突。

### Q: 如何避免冲突？
A: 频繁同步代码、使用分支、做好文件分工、小步提交。

### Q: 冲突标记是什么？
A: `<<<<<<< HEAD`、`=======`、`>>>>>>>` 是Git用来标记冲突位置的特殊符号。

### Q: 可以自动解决冲突吗？
A: 简单的冲突可以用工具自动解决，但复杂的冲突需要人工判断。

### Q: 强制推送会有什么后果？
A: 会覆盖远程仓库的历史，可能导致其他人的代码丢失，谨慎使用！

---

## 团队约定

请所有团队成员：

1. ✅ 阅读并遵守本规范
2. ✅ 每天开始工作前先 `git pull`
3. ✅ 遇到问题及时沟通
4. ✅ 不确定时先问，不要盲目操作
5. ✅ 定期备份重要代码

---

**最后更新：** 2025年

**维护者：** 项目团队

**问题反馈：** 遇到Git问题请及时在团队群里讨论

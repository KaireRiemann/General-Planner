# Release 版本管理与提交约定

`general_planner_release/` 是需要维护和提交的发布目录。打包脚本原地同步后出现的有效差异应进入版本管理，不应为了保持工作树干净而恢复旧的发布文件。

## 提交范围

- 发布配置、launch、启动包装脚本及辅助脚本。
- 消息定义、生成的 C++ 头文件和 Python 消息接口。
- 发布程序 `src/general_planner_release/bin/*.bin` 和运行所需的共享库。
- 发布目录现有的依赖代码、资源、许可证和说明。

修改源码或运行参数后，使用 `sh_files/build_general_planner_release.sh` 刷新发布目录，检查差异，将相应源码和发布产物一起提交，避免新程序配旧配置或旧接口。生成头文件中仅空白的差异也可以随发布更新提交，不必反复恢复为旧版本。

根目录 `.gitignore` 对发布程序设置了 `*.bin` 的例外；其他位置的 `.bin` 仍保持原忽略规则。配置和消息接口原本已被跟踪，不需要额外放行。

## 保持忽略的内容

构建中间目录、Python 缓存、bag、日志及临时测试产物不作为发布版本提交。`general_planner_release.tar.gz` 是发布目录的打包副本，继续忽略。

Tracking detector 的 `.pt` / `.onnx` 权重沿用该组件现有的单独分发约定，本次没有改为 Git 管理；新 checkout 部署仍需按组件说明准备权重。

## 提交前检查

```bash
git status --short
git diff -- general_planner_release
git add -- general_planner_release
git diff --cached --stat
```

随后将本次对应的源码和规则修改一并暂存、审阅并提交。不要使用 `assume-unchanged` 或 `skip-worktree` 隐藏发布目录更新，也不要忽略整个发布目录。

打包脚本成功仅表示编译和同步完成，运行验证结果应另行记录。

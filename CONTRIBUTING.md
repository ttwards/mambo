# 贡献指南

## 快速开始

### 安装 Pre-commit

```bash
./setup-precommit.sh
```

或手动安装：

```bash
pip install pre-commit
pre-commit install
```

### 提交代码

```bash
git add .
git commit -m "feat: 你的功能描述"  # pre-commit 自动运行检查
```

## 提交类型

- `feat`: 新功能
- `fix`: Bug 修复
- `docs`: 文档更新
- `style`: 代码格式
- `refactor`: 重构
- `test`: 测试
- `chore`: 构建/工具

## 常用命令

```bash
# Pre-commit
pre-commit run --all-files    # 检查所有文件
pre-commit autoupdate          # 更新 hooks

# 编译测试
west build -b robomaster_board_c samples/motor/dji_m3508_demo
west build -b dm_mc02 samples/motor/dm_demo
```

## 代码规范

- 每个文件必须包含 SPDX 许可证头：`// SPDX-License-Identifier: Apache-2.0`
- 使用 `.clang-format` 格式化 C/C++ 代码
- 提交前运行 `pre-commit run --all-files`

## CI 流程

推送到 GitHub 后自动运行：

1. 代码格式检查
2. 多开发板编译测试
3. 静态代码分析
4. 文档检查

## 常见问题

**Q: Pre-commit 失败？**

```bash
make format  # 自动修复格式问题
```

**Q: 临时跳过检查？**

```bash
git commit --no-verify  # 不推荐！
```

**Q: 更新依赖？**

```bash
west update
west build -t pristine
```

---

详细的 Zephyr 开发指南请参考 `Documents/` 目录。

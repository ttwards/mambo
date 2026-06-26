# Radxa 上迁移并配置本 Zephyr 工作空间

本文用于在一台新的 Radxa 设备上复现当前 Zephyr 开发环境。目标是保持和现有机器相同的工作空间布局：

```text
~/zephyrproject/
├── .venv/          # Python virtualenv, west 在这里
├── .west/          # west 工作空间配置
├── bootloader/
├── modules/
├── zephyr/
└── zephyr_ws/      # 本仓库, manifest repo
```

当前仓库状态：

- manifest 仓库路径：`zephyr_ws`
- 当前 Git 远端：`https://github.com/NiZenMeZhiDao/zephyr_ws`
- 当前分支：`vcan_update`
- 常用开发板：`robomaster_board_c`
- 当前 west manifest：`zephyr_ws/west.yml`
- 当前 west topdir：`~/zephyrproject`
- 当前 Zephyr commit：`e3672da9b3bd`

Zephyr 官方最新文档要求 Ubuntu 24.04 LTS 或更新版本，并要求 Python >= 3.12、CMake >= 3.20.5、DTC >= 1.4.6。Radxa 是 AArch64/ARM64 设备，安装依赖时不要装 `gcc-multilib` 和 `g++-multilib`，这两个包通常不可用。

参考：

- Zephyr Getting Started: https://docs.zephyrproject.org/latest/develop/getting_started/index.html
- Linux host dependencies: https://docs.zephyrproject.org/latest/develop/getting_started/installation_linux.html

## 0. 迁移前检查旧电脑

在旧电脑上确认代码已经提交并推到远端：

```sh
cd ~/zephyrproject/zephyr_ws
git status
git branch -vv
git push origin vcan_update
```

如果要求新旧电脑的 `zephyr/` 和 `modules/` commit 也完全一致，先在旧电脑保存一份冻结 manifest：

```sh
cd ~/zephyrproject
west manifest --freeze > ~/zephyrproject/zephyr_ws/west-lock.yml
```

当前 `west.yml` 中 Zephyr 使用 `revision: main`，干净重建时会拉取执行迁移当天的 `main`。如果只是保持本仓库一致，用下面的推荐流程即可；如果要严格复现旧电脑所有依赖版本，请使用完整拷贝方式，或把 `west-lock.yml` 作为迁移时的临时 manifest。

注意：当前仓库的 `.gitignore` 忽略了 `.vscode/`，所以 `.vscode/tasks.json` 不会随 Git 自动迁移。如果新 Radxa 也要完全一样的 VS Code 任务，需要单独复制旧电脑上的：

```text
~/zephyrproject/zephyr_ws/.vscode/tasks.json
```

## 1. Radxa 系统准备

推荐使用 Ubuntu 24.04 或基于 Ubuntu 24.04 的 Radxa 官方系统。先更新系统：

```sh
sudo apt update
sudo apt upgrade
```

安装 Zephyr 主机依赖。AArch64 上不要带 `gcc-multilib` / `g++-multilib`：

```sh
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
  python3-dev python3-venv python3-tk xz-utils file make gcc g++ \
  libsdl2-dev libmagic1 openocd
```

检查版本：

```sh
python3 --version
cmake --version
dtc --version
```

如果 Python 不是 3.12 或更高，不要继续用系统 Python 硬跑。先换 Ubuntu 24.04 系统，或安装并用 Python 3.12 创建虚拟环境。

## 2. 推荐迁移方式：干净重建 workspace

这种方式最稳：Git 只迁移本仓库，`zephyr/`、`modules/`、`bootloader/` 由 `west update` 重新拉取。

```sh
cd ~
python3 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate
pip install --upgrade pip
pip install west
```

初始化和当前机器同名的 west 工作空间：

```sh
west init -m https://github.com/NiZenMeZhiDao/zephyr_ws --mr vcan_update ~/zephyrproject
cd ~/zephyrproject
west update
west zephyr-export
west packages pip --install
```

完成后应能看到：

```sh
west topdir
west manifest --path
west list
```

期望结果：

```text
west topdir         -> /home/<radxa-user>/zephyrproject
west manifest path  -> /home/<radxa-user>/zephyrproject/zephyr_ws/west.yml
```

## 3. 安装 Zephyr SDK

在虚拟环境中执行：

```sh
source ~/zephyrproject/.venv/bin/activate
cd ~/zephyrproject/zephyr
west sdk install
```

如果只想减少下载量，当前主要目标是 STM32/ARM 板，可以优先安装 ARM toolchain。具体参数以 `west sdk install --help` 为准。

确认 SDK 可被识别：

```sh
west sdk list
```

如果后续构建报找不到 SDK，检查是否错误设置了旧环境变量：

```sh
echo "$ZEPHYR_TOOLCHAIN_VARIANT"
echo "$ZEPHYR_SDK_INSTALL_DIR"
```

一般不需要手动设置这两个变量；如果它们指向旧路径，先从 shell 配置里移除。

## 4. USB、烧录和调试权限

把当前用户加入常用串口/调试权限组：

```sh
sudo usermod -aG dialout,plugdev $USER
```

然后注销并重新登录，或重启 Radxa。

连接开发板后检查：

```sh
lsusb
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

首次使用 CMSIS-DAP、ST-Link、J-Link 等调试器时，如果 `west flash` 报权限错误，需要安装对应 udev 规则。Zephyr 官方文档也提醒 Linux 首次使用 debug probe 通常需要配置 udev rules。

## 5. VS Code 配置

推荐在 Radxa 本机 VS Code 或远程 SSH VS Code 中打开：

```text
~/zephyrproject/zephyr_ws
```

当前 `.vscode/tasks.json` 是本地忽略文件，不会由 Git clone 自动出现。可选做法：

1. 从旧电脑复制 `.vscode/tasks.json` 到新电脑同一路径。
2. 或者从 `VSC_sample_configs/tasks.json` 复制一份再改命令。

当前任务里的虚拟环境路径是：

```sh
source ~/zephyrproject/.venv/bin/activate
```

所以只要新 Radxa 也使用 `~/zephyrproject/.venv`，大部分任务不需要改路径。

当前 Monitor 任务使用 `pyocd rtt -t STM32F407IGHx`。如果新环境里没有 `pyocd`，在虚拟环境中安装：

```sh
source ~/zephyrproject/.venv/bin/activate
pip install pyocd
```

## 6. 构建验证

先用命令行验证，不要一开始就依赖 VS Code：

```sh
source ~/zephyrproject/.venv/bin/activate
cd ~/zephyrproject/zephyr_ws
west build -b robomaster_board_c src/2026_R2usb_armv2 \
  -d build/2026_R2usb_armv2 \
  -p always \
  -- -DUSER_CACHE_DIR=$PWD/build/zephyr-cache -DUSE_CCACHE=0
```

成功后应生成：

```text
build/2026_R2usb_armv2/zephyr/zephyr.elf
build/2026_R2usb_armv2/zephyr/zephyr.bin
build/2026_R2usb_armv2/zephyr/zephyr.hex
```

如果要验证一个更小的包，也可以构建：

```sh
west build -b robomaster_board_c src/lk_rom_zero \
  -d build/lk_rom_zero \
  -p always \
  -- -DUSER_CACHE_DIR=$PWD/build/zephyr-cache -DUSE_CCACHE=0
```

这里保留 `-DUSER_CACHE_DIR=$PWD/build/zephyr-cache` 和 `-DUSE_CCACHE=0` 是为了避免缓存路径和 ccache 临时目录导致的环境问题。Radxa 本机如果 ccache 正常，也可以去掉 `-DUSE_CCACHE=0` 再测试。

## 7. 烧录验证

连接 RoboMaster Board C 后：

```sh
source ~/zephyrproject/.venv/bin/activate
cd ~/zephyrproject/zephyr_ws
west flash --runner openocd
```

如果使用别的调试器：

```sh
west flash --runner stlink
west flash --runner jlink
```

如果 `openocd` 找不到或权限不足，先检查：

```sh
which openocd
groups
lsusb
```

## 8. 完整拷贝方式

如果 Radxa 网络慢，或者想尽量离线迁移，也可以从旧电脑完整复制 `~/zephyrproject`。但建议不要复制旧的 `build/`，它体积大且容易带来平台差异。

示例：

```sh
rsync -a --delete \
  --exclude 'zephyr_ws/build/' \
  --exclude 'build/' \
  --exclude '.cache/' \
  old-host:~/zephyrproject/ ~/zephyrproject/
```

复制后仍建议重新激活虚拟环境并检查：

```sh
source ~/zephyrproject/.venv/bin/activate
cd ~/zephyrproject
west topdir
west update
west zephyr-export
west packages pip --install
```

如果旧电脑和 Radxa 的 CPU 架构不同，旧的 `.venv/` 也不要复用，删除后重建：

```sh
rm -rf ~/zephyrproject/.venv
python3 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate
pip install --upgrade pip
pip install west
cd ~/zephyrproject
west packages pip --install
```

## 9. 常见问题

### Python 版本不够

现象：

```text
Could NOT find Python3: Found unsuitable version "3.10.x", but required is at least "3.12"
```

处理：

- 确认 Radxa 系统是否为 Ubuntu 24.04 或更新。
- 用 `~/zephyrproject/.venv/bin/west`，不要混用系统里旧的 `west`。

### AArch64 依赖安装失败

现象：

```text
Unable to locate package gcc-multilib
Unable to locate package g++-multilib
```

处理：

- Radxa 是 ARM64，安装依赖时删掉这两个包。

### west 找不到 manifest

现象：

```text
FATAL ERROR: no west workspace found
```

处理：

```sh
cd ~/zephyrproject
west topdir
west manifest --path
```

如果这两个命令失败，说明没有正确执行 `west init`，或只复制了 `zephyr_ws/` 而没有 `.west/`。

### VS Code 能打开但任务不能用

处理：

- 确认打开的是 `~/zephyrproject/zephyr_ws`。
- 确认 `.vscode/tasks.json` 已单独复制。
- 确认 `~/zephyrproject/.venv/bin/west` 存在。

### 烧录权限不足

处理：

- 用户加入 `dialout` 和 `plugdev` 后需要重新登录。
- 安装对应调试器 udev rules。
- 用 `lsusb` 确认调试器已经被 Radxa 识别。

## 10. 最小验收清单

新 Radxa 上至少跑通以下命令，才算环境迁移完成：

```sh
source ~/zephyrproject/.venv/bin/activate
cd ~/zephyrproject
west topdir
west manifest --path
west list

cd ~/zephyrproject/zephyr_ws
west build -b robomaster_board_c src/2026_R2usb_armv2 \
  -d build/2026_R2usb_armv2 \
  -p always \
  -- -DUSER_CACHE_DIR=$PWD/build/zephyr-cache -DUSE_CCACHE=0
```

硬件在手时再补充：

```sh
west flash --runner openocd
```

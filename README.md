<p align="right">
  <a href="README.md.eng">English</a>
</p>

<p align="center">
  <img src="assets/logo.svg" width="620" alt="Mambo">
</p>

<p align="center">
  <a href="https://github.com/ttwards/mambo/actions/workflows/ci.yml"><img src="https://img.shields.io/github/actions/workflow/status/ttwards/mambo/ci.yml?branch=master&style=for-the-badge&label=CI&logo=githubactions&logoColor=white" alt="CI"></a>
  <a href="https://opensource.org/license/mit/"><img src="https://img.shields.io/badge/license-MIT-blue?style=for-the-badge" alt="MIT License"></a>
  <a href="https://github.com/ttwards/mambo/stargazers"><img src="https://img.shields.io/github/stars/ttwards/mambo?style=for-the-badge&logo=github&label=Stars" alt="Stars"></a>
  <a href="https://github.com/ttwards/mambo/forks"><img src="https://img.shields.io/github/forks/ttwards/mambo?style=for-the-badge&logo=github&label=Forks" alt="Forks"></a>
</p>

Mambo 是一个面向机器人下位机开发的 Zephyr RTOS 模块。项目通过设备树描述电机、底盘、传感器、通信接口等硬件关系，并提供对应的驱动、协议库和样例程序。

## 项目结构

- `boards/`: 自定义开发板定义。
- `dts/bindings/`: 驱动和模块使用的设备树绑定。
- `drivers/`: 电机、底盘、传感器、VCAN、SBUS 等 Zephyr 驱动。
- `lib/ares/`: ARES 通信、协议、IMU 算法和板级辅助库。
- `samples/`: 可直接构建的示例应用。
- `template/`: 新应用模板。
- `Documents/`: 设计说明和专项文档。

整体结构可参考 `structure.png`，设备树描述示例可参考 `example.dts`。

## 环境准备

以下命令以 Ubuntu 22.04/24.04 为例：

```shell
sudo apt update
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
```

建议在虚拟环境中安装 West 和 Zephyr 的 Python 依赖：

```shell
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip wheel
python3 -m pip install west
```

如果本机已有受管理的 Python 环境，也可以使用等价的 venv、pipx 或系统包管理方案；建议避免把 Zephyr 构建依赖混入全局 Python 环境。

## 初始化工作区

```shell
west init -m https://github.com/ttwards/mambo --mr master my-workspace
cd my-workspace
west update
west zephyr-export
python3 -m pip install -r zephyr/scripts/requirements.txt
west sdk install -t arm-zephyr-eabi
```

初始化完成后，工作区根目录包含以下项目目录：

- `mambo/`: 当前 manifest repository。
- `zephyr/`: `west.yml` 固定的 Zephyr 源码。
- `modules/hal/cmsis/`
- `modules/hal/cmsis_6/`
- `modules/lib/cmsis-dsp/`
- `modules/hal/st/`
- `modules/hal/stm32/`
- `modules/debug/segger/`
- `bootloader/mcuboot/`

`west sdk install -t arm-zephyr-eabi` 只安装当前 ARM Cortex-M 目标所需的 GNU 工具链。需要构建其他架构时，再按 Zephyr SDK 文档补充对应 toolchain。

## 构建与烧录

构建 Robomaster Board C 上的 DJI M3508 示例：

```shell
cd mambo
west build -b robomaster_board_c samples/motor/dji_m3508_demo
```

烧录：

```shell
west flash
```

默认 runner 取决于开发板配置。需要指定调试器时可使用：

```shell
west flash --runner stlink
west flash --runner jlink
```

其他样例可按相同方式构建，例如：

```shell
west build -b dm_mc02 samples/motor/dm_demo
west build -b robomaster_board_c samples/vcan/vcan_host_demo
```

## 开发工具

项目提供 pre-commit 配置和 GitHub Actions CI。首次开发前建议安装 hooks：

```shell
./setup-precommit.sh
```

手动运行检查：

```shell
pre-commit run --all-files
```

VS Code 示例配置位于 `VSC_sample_configs/`。贡献流程和提交规范见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 文档

- 通用开发和模块说明：`Documents/`
- VCAN 设计与调试说明：`Documents/vcan/`
- 样例使用说明：各 `samples/**/README.*`

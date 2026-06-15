#!/bin/bash
# build.sh - PFLIGHT_26 ビルドスクリプト
# 使用方法: bash build.sh

set -e  # エラーで終了

# STM32CubeIDE ツールパス
MAKE_PATH="D:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
GNU_TOOLS_PATH="D:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin"

export PATH="${GNU_TOOLS_PATH}:${MAKE_PATH}:$PATH"

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEBUG_DIR="$PROJECT_DIR/Debug"

echo "=================================================="
echo "STM32 Project Build Script"
echo "=================================================="
echo "Project: PFLIGHT_26"
echo "Debug Dir: $DEBUG_DIR"
echo "Toolchain: GNU Tools for STM32 v13.3"
echo "=================================================="
echo ""

# Clean ビルド (オプション -c フラグで指定)
if [ "$1" = "-c" ] || [ "$1" = "--clean-only" ]; then
    echo "[CLEAN] Removing build artifacts..."
    cd "$DEBUG_DIR"
    make clean
    echo "✅ Clean completed"
    exit 0
fi

# ビルド
echo "[BUILD] Compiling..."
cd "$DEBUG_DIR"

if [ "$1" = "--no-clean" ]; then
    echo "[INFO] Skipping clean..."
else
    echo "[CLEAN] Removing old build artifacts..."
    make clean
fi

make all -j24

if [ $? -eq 0 ]; then
    echo ""
    echo "=================================================="
    echo "✅ Build succeeded!"
    echo "=================================================="
    echo ""
    echo "Output files:"
    echo "  - PFLIGHT_26.elf  (Executable)"
    echo "  - PFLIGHT_26.map  (Memory map)"
    echo "  - PFLIGHT_26.list (Disassembly)"
    echo ""

    # バイナリサイズ表示
    echo "Binary size:"
    arm-none-eabi-size PFLIGHT_26.elf
    echo ""
else
    echo ""
    echo "=================================================="
    echo "❌ Build failed!"
    echo "=================================================="
    exit 1
fi

exit 0

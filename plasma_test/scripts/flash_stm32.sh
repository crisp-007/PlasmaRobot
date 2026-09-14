#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
firmware_hex="${project_dir}/plasma_test/MDK-ARM/plasma_test/plasma_test.hex"
local_stlink_root="/home/larusxu/.local/stlink-tools"
local_stlink_bin="${local_stlink_root}/usr/bin"
local_stlink_lib="${local_stlink_root}/usr/lib/aarch64-linux-gnu"

if command -v st-info >/dev/null 2>&1 && command -v st-flash >/dev/null 2>&1; then
    st_info="$(command -v st-info)"
    st_flash="$(command -v st-flash)"
elif [[ -x "${local_stlink_bin}/st-info" && -x "${local_stlink_bin}/st-flash" ]]; then
    st_info="${local_stlink_bin}/st-info"
    st_flash="${local_stlink_bin}/st-flash"
    export LD_LIBRARY_PATH="${local_stlink_lib}:${LD_LIBRARY_PATH:-}"
else
    echo "未找到 st-info/st-flash。" >&2
    exit 1
fi

if [[ ! -f "${firmware_hex}" ]]; then
    echo "未找到固件: ${firmware_hex}" >&2
    exit 1
fi

echo "检测 ST-Link 和目标芯片..."
probe_output="$("${st_info}" --probe 2>&1)"
echo "${probe_output}"
if grep -q "Found 0 stlink programmers" <<<"${probe_output}"; then
    echo "未检测到 ST-Link，请连接下载器后重试。" >&2
    exit 2
fi

if [[ "${1:-}" != "--write" ]]; then
    echo "探测完成，未写入 Flash。需要烧录时执行:"
    echo "  $0 --write"
    exit 0
fi

if ! command -v arm-none-eabi-objcopy >/dev/null 2>&1; then
    echo "未找到 arm-none-eabi-objcopy。" >&2
    exit 1
fi

firmware_bin="$(mktemp /tmp/plasma-firmware.XXXXXX.bin)"
trap 'rm -f "${firmware_bin}"' EXIT
arm-none-eabi-objcopy -I ihex -O binary "${firmware_hex}" "${firmware_bin}"

echo "烧录固件: ${firmware_hex}"
"${st_flash}" --reset write "${firmware_bin}" 0x08000000
echo "烧录和校验完成。若程序未自动运行，请按一下主板 RESET。"

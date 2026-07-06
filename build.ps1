$ErrorActionPreference = 'Stop'

# 优先从环境变量读取 ESP-IDF 路径；未设置时使用默认路径
$DefaultIdfPath = 'E:\Espressif\frameworks\esp-idf-v5.4.3'
$DefaultIdfToolsPath = 'E:\Espressif'
$DefaultProjectPath = 'F:\AI-detection'

$idfPath = if ($env:IDF_PATH) { $env:IDF_PATH } else { $DefaultIdfPath }
$idfToolsPath = if ($env:IDF_TOOLS_PATH) { $env:IDF_TOOLS_PATH } else { $DefaultIdfToolsPath }
$projectPath = if ($env:AI_DETECTION_PROJECT_PATH) { $env:AI_DETECTION_PROJECT_PATH } else { $DefaultProjectPath }

$env:IDF_PATH = $idfPath
$env:IDF_TOOLS_PATH = $idfToolsPath
$env:IDF_PYTHON_ENV_PATH = "$idfToolsPath\python_env\idf5.4_py3.11_env"

if (Test-Path Env:\MSYSTEM) { Remove-Item Env:\MSYSTEM }
cd $projectPath

# 把 ESP-IDF 必需工具加入 PATH
$toolPaths = @(
    "$idfToolsPath\python_env\idf5.4_py3.11_env\Scripts",
    "$idfToolsPath\tools\cmake\3.30.2\bin",
    "$idfToolsPath\tools\ninja\1.12.1",
    "$idfToolsPath\tools\ccache\4.12.1",
    "$idfToolsPath\tools\dfu-util\0.11",
    "$idfToolsPath\tools\esp-rom-elfs\20241011",
    "$idfToolsPath\tools\esp32ulp-elf\2.38_20240113",
    "$idfToolsPath\tools\idf-exe\1.0.3",
    "$idfToolsPath\tools\openocd-esp32\v0.12.0-esp32-20250707\bin",
    "$idfToolsPath\tools\riscv32-esp-elf\esp-14.2.0_20250730\riscv32-esp-elf\bin",
    "$idfToolsPath\tools\xtensa-esp-elf\esp-14.2.0_20250730\xtensa-esp-elf\bin"
)
$env:PATH = ($toolPaths -join ';') + ';' + $env:PATH

& "$idfToolsPath\python_env\idf5.4_py3.11_env\Scripts\python.exe" "$idfPath\tools\idf.py" -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_p4_function_ev_board build

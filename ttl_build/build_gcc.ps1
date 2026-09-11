$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$gcc = 'D:\motor\tools\arm-none-eabi\bin\arm-none-eabi-gcc.exe'
$objcopy = Join-Path (Split-Path $gcc) 'arm-none-eabi-objcopy.exe'
Push-Location -LiteralPath $root
try {
$out = Join-Path $root 'build'
New-Item -ItemType Directory -Force $out | Out-Null
$inc = @('-IAPP','-IBSP','-ICMSIS','-IDRIVERS','-ILIB\STM32F10x_StdPeriph_Lib_V3.5.0\inc')
$flags = @('-mcpu=cortex-m3','-mthumb','-Os','-g','-Werror=uninitialized','-Werror=maybe-uninitialized','-ffunction-sections','-fdata-sections','-std=gnu11','-DSTM32F10X_HD','-DUSE_STDPERIPH_DRIVER') + $inc
$sources = @('APP\main.c','APP\stm32f10x_it.c','APP\translator.c','BSP\usart.c','BSP\board.c','CMSIS\system_stm32f10x.c','LIB\STM32F10x_StdPeriph_Lib_V3.5.0\src\stm32f10x_gpio.c','LIB\STM32F10x_StdPeriph_Lib_V3.5.0\src\stm32f10x_rcc.c','LIB\STM32F10x_StdPeriph_Lib_V3.5.0\src\stm32f10x_usart.c','LIB\STM32F10x_StdPeriph_Lib_V3.5.0\src\misc.c')
$objects = @()
foreach ($source in $sources) {
  $obj = Join-Path $out ((Split-Path $source -Leaf) -replace '\.c$','.o')
  & $gcc @flags '-c' $source '-o' $obj
  if ($LASTEXITCODE) { throw "compile failed: $source" }
  $objects += $obj
}
$linkFlags = @('-T','gcc_flash.ld','startup_gcc.s','-Wl,--gc-sections','-Wl,-Map=build\ttl_control.map')
& $gcc @flags @linkFlags $objects '--specs=nosys.specs' '-lm' '-lc' '-lgcc' '-o' 'build\ttl_control.elf'
if ($LASTEXITCODE) { throw 'link failed' }
& $objcopy '-O' 'ihex' 'build\ttl_control.elf' 'build\ttl_control.hex'
if ($LASTEXITCODE) { throw 'hex conversion failed' }
& $objcopy '-O' 'binary' 'build\ttl_control.elf' 'build\ttl_control.bin'
if ($LASTEXITCODE) { throw 'bin conversion failed' }
Write-Host "Built $out\ttl_control.hex"
} finally {
  Pop-Location
}

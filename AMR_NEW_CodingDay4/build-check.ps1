$ErrorActionPreference='Stop'
$root=$PSScriptRoot
Set-Location -LiteralPath $root
$gcc=(Get-ChildItem 'C:\ST' -Recurse -Filter arm-none-eabi-gcc.exe | Select-Object -First 1).FullName
if (!$gcc) { throw 'ARM GCC not found under C:\ST' }
$out=Join-Path $root 'BuildCheck'
New-Item -ItemType Directory -Force $out | Out-Null
$rtos='Middlewares/Third_Party/FreeRTOS/Source'
$includes=@('Core/Inc','Drivers/STM32F4xx_HAL_Driver/Inc','Drivers/STM32F4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32F4xx/Include','Drivers/CMSIS/Include',"$rtos/include","$rtos/portable/GCC/ARM_CM4F","$rtos/CMSIS_RTOS_V2") | ForEach-Object { "-I$_" }
$cpu=@('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard')
$sources=@(Get-ChildItem Core/Src,Drivers/STM32F4xx_HAL_Driver/Src,$rtos -Filter '*.c') + @(Get-Item "$rtos/portable/GCC/ARM_CM4F/port.c","$rtos/portable/MemMang/heap_4.c","$rtos/CMSIS_RTOS_V2/cmsis_os2.c") + @(Get-ChildItem Core/Startup -Filter '*.s')
$objects=foreach ($src in $sources) {
    $obj=Join-Path $out ($src.BaseName+'.o')
    & $gcc @cpu @includes '-DUSE_HAL_DRIVER' '-DSTM32F446xx' '-std=gnu11' '-Os' '-ffunction-sections' '-fdata-sections' '-Wall' '-Wextra' -c $src.FullName -o $obj 2>&1 | Out-Host
    if ($LASTEXITCODE) { throw "Compile failed: $($src.Name)" }
    $obj
}
& $gcc @cpu @objects '-TSTM32F446RETX_FLASH.ld' '--specs=nano.specs' '--specs=nosys.specs' '-Wl,--gc-sections' '-Wl,-Map=BuildCheck/AMR_NEW.map' '-lm' '-o' 'BuildCheck/AMR_NEW.elf'
if ($LASTEXITCODE) { throw 'Link failed' }
& (Join-Path (Split-Path $gcc) 'arm-none-eabi-size.exe') 'BuildCheck/AMR_NEW.elf' 2>&1 | Out-Host



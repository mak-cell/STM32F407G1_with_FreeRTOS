Here is a clean and professional `README.md` for your STM32 FreeRTOS projects.

You can copy this directly into your repository.

---

# 📘 STM32 FreeRTOS Projects

This repository contains STM32F407-based FreeRTOS example projects built using **STM32CubeIDE**.

Projects included:

```
001_basicrtos
002_freertos_with_systemview
```

---

# 🧠 Overview

FreeRTOS (Free Real-Time Operating System) is a lightweight, open-source RTOS kernel designed for microcontrollers.

Unlike traditional super-loop firmware:

```c
while(1)
{
   // everything runs sequentially
}
```

FreeRTOS allows splitting firmware into independent **tasks** that run concurrently using a scheduler.

Target MCU:

```
STM32F407 (Cortex-M4F)
```

IDE:

```
STM32CubeIDE 1.18.x
```

Compiler:

```
arm-none-eabi-gcc (13.x)
```

---

# 📂 Project Structure

## 1️⃣ 001_basicrtos

Basic FreeRTOS integration with STM32F407.

### Features:

* Task creation
* Task scheduling
* LED toggle example
* SysTick managed by FreeRTOS
* Stack overflow hook example

### Learning Focus:

* How FreeRTOS scheduler works
* Context switching
* Interrupt priority configuration
* FreeRTOSConfig setup

---

## 2️⃣ 002_freertos_with_systemview

FreeRTOS integrated with:

```
SEGGER SystemView
```

### Features:

* Real-time task monitoring
* Context switch tracing
* Interrupt tracing
* System performance visualization

### Learning Focus:

* Runtime tracing
* Debugging task execution
* Performance analysis

---

# ⚙️ How FreeRTOS Is Integrated

Two possible integration approaches:

1. Manual FreeRTOS (Non-CMSIS)
2. STM32CubeMX CMSIS-RTOS wrapper

These projects use:

```
Manual FreeRTOS integration
```

Include paths required:

```
Thirdparty/Freertos/include
Thirdparty/Freertos/portable/GCC/ARM_CM4F
```

---

# 🚨 Common Issues & Fixes

---

## ❌ 1. FreeRTOS.h: No such file or directory

### Cause:

Incorrect include path configuration.

### Fix:

Remove all manual macro paths like:

```
"${workspace_loc:/${ProjName}/Thirdparty/Freertos/include}"
```

Re-add using:

```
Project → Properties → C/C++ Build → Settings
→ MCU GCC Compiler → Include paths
→ Add → Workspace
```

Then:

```
Project → Clean → Build
```

---

## ❌ 2. Undefined reference to vApplicationStackOverflowHook

### Cause:

Stack overflow hook enabled but function not implemented.

### Fix Option A – Disable:

```c
#define configCHECK_FOR_STACK_OVERFLOW 0
```

### Fix Option B – Implement hook:

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    __disable_irq();
    while(1)
    {
        // optional: blink LED
    }
}
```

---

## ❌ 3. Debugger Stuck at configASSERT()

If debugger stops at:

```c
configASSERT( ucMaxSysCallPriority );
```

### Cause:

Invalid interrupt priority configuration.

### ❌ Wrong:

```c
#define configKERNEL_INTERRUPT_PRIORITY          0
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     0
```

Priority 0 is highest and cannot be masked.

### ✅ Correct Configuration (Cortex-M4):

```c
#define configKERNEL_INTERRUPT_PRIORITY          (15 << 4)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (5 << 4)
#define configMAX_API_CALL_INTERRUPT_PRIORITY    configMAX_SYSCALL_INTERRUPT_PRIORITY
```

---

# 🧩 Important Notes for STM32F4 + FreeRTOS

* SysTick is managed by FreeRTOS
* Do NOT override:

  * PendSV_Handler
  * SVC_Handler
  * SysTick_Handler
* Disable them inside `stm32f4xx_it.c`
* Use alternative HAL timebase (TIMx) if needed

---

# 🔍 Debugging

For ITM / SWO debugging:

* Ensure interrupt priorities are valid
* Enable SWO in debugger configuration
* Avoid priority 0 for any FreeRTOS critical section

---

# 🧵 Task States (FreeRTOS)

A task can be:

* Running
* Ready
* Blocked
* Suspended

Scheduler performs context switching automatically.

---

# 🛠 Build Instructions

1. Open project in STM32CubeIDE
2. Verify include paths
3. Clean project
4. Build
5. Flash using ST-Link

---

# 📚 References

* FreeRTOS Official Documentation
* CMSIS-RTOS2 API
* STM32F4 Reference Manual
* SEGGER SystemView Documentation

---

# 🎯 Learning Goals

By completing these projects, you will understand:

* FreeRTOS scheduler internals
* Cortex-M interrupt priority model
* Stack overflow handling
* Context switching
* RTOS debugging
* System tracing with SystemView

---

# 👨‍💻 Author

STM32 + FreeRTOS learning experiments
Cortex-M4 Embedded Development

---

If you want, I can also:

* Make a more professional GitHub-style version
* Add build badges
* Add architecture diagram
* Add memory map explanation
* Add interrupt priority visual diagram
* Add SystemView setup guide

Just tell me 👍

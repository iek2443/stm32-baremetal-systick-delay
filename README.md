# STM32 Bare-Metal: SysTick Delay and LED Toggle (PD15)

---

## 📌 Summary

This project demonstrates how to implement a **millisecond-level delay** using the **SysTick timer** on the STM32F4 Discovery board.  
The LED connected to **PD15 (Blue LED)** is toggled on and off every second using a custom SysTick-based delay function.  
The entire configuration is performed at the **register level**, without using HAL or CMSIS.

---

## 🔁 Previous Lesson

If you haven’t completed the previous lesson where we performed an ADC read from an analog pin, check it out here:

👉 [Previous Lesson: ADC Read from PA1 (Channel 1)](https://github.com/iek2443/stm32-baremetal-adc-read)

---

## 🧠 What You Will Learn

- How to configure and use the **SysTick timer** for periodic delays
- How to toggle an LED with precise timing
- How to monitor the `COUNTFLAG` status for timing operations

---

## ⚙️ Key Registers Used

- `RCC->AHB1ENR` → Enables clock to GPIOD
- `GPIOD->MODER` → Configures PD15 as output
- `GPIOD->BSRR`  → Sets or resets PD15 output
- `SysTick->CSR` → Controls SysTick enable, clock source, and count status
- `SysTick->RVR` → Sets the reload value for timing
- `SysTick->CVR` → Clears current value and forces reload

---

## 🔧 Requirements

- STM32F4 Discovery Board
- ARM GCC Toolchain
- USB Mini-B cable

---

📁 Project Structure
--------------------

stm32-baremetal-systick-delay/\
├── src/\
│   └── main.c         --> Bare-metal SysTick delay and LED toggle\
├── inc/               --> (Optional: header files)\
└── README.md

---

🧭 Pin Mapping

| Function         | Port | Pin | Description         |
|------------------|------|-----|---------------------|
| LED (Blue)       | D    | 15  | Toggled with delay   |

---

## ⏳ SysTick Timing Flow

	1.	Set SysTick reload value for 1ms delay (16000 cycles at 16MHz)
	2.	Enable SysTick and use processor clock
	3.	Wait until COUNTFLAG is set
	4.	Repeat for desired delay period (e.g., 1000 times for 1 second)
	5.	Toggle LED output
 ---
 

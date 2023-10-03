#![no_main]
#![no_std]

use panic_halt as _; // panic handler

use cortex_m_rt::entry;
use stm32f3_discovery::stm32f3xx_hal::pac::usart1;

use stm32f3_discovery::stm32f3xx_hal;

use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::{
    serial::Serial,
    pac::{self, USART1},
};

#[entry]
fn main() -> ! {
    // Initialization of peripherals
    let device_periphs = pac::Peripherals::take().unwrap();
    let mut reset_and_clock_control = device_periphs.RCC.constrain();

    let mut flash = device_periphs.FLASH.constrain();
    let clocks = reset_and_clock_control.cfgr.freeze(&mut flash.acr);
    let mut gpioa = device_periphs.GPIOA.split(&mut reset_and_clock_control.ahb);

    // USART TX and RX pins
    let tx = gpioa.pa9.into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let rx = gpioa.pa10.into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    // Initialization  of UART
    Serial::new(device_periphs.USART1, (tx, rx), 115_200.Bd(), clocks, &mut reset_and_clock_control.apb2);
    let usart: &'static mut usart1::RegisterBlock = unsafe {&mut *(USART1::ptr() as *mut _)};

    // Send a single character
    usart
        .tdr
        .write(|w| w.tdr().bits(u16::from(b'X')) );

    loop {}
}

#![no_main]
#![no_std]

use power_safety_board as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");
    defmt::println!("STM32G431KB is running!");
    
    let mut counter = 0;
    loop {
        defmt::println!("Counter: {}", counter);
        counter += 1;
        
        // Simple delay
        cortex_m::asm::delay(8_000_000); // ~1 second at default clock
    }
}

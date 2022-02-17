#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicBool, Ordering};

use arrayvec::ArrayString;
use embassy::blocking_mutex::raw::NoopRawMutex;
use embassy::channel::mpsc::{self, Channel, Receiver, Sender};
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{DMA1_CH3, PB0, PC13, PG1, USART3};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::Config;
use embassy_stm32::Peripherals;
use embassy_stm32::exti::ExtiInput;

fn config() -> Config {
    let mut config = Config::default();

    config.rcc.hse = Some(Hertz(8_000_000));
    config.rcc.bypass_hse = false;
    config.rcc.pll48 = false;
    config.rcc.sys_ck = Some(Hertz(180_000_000));
    config.rcc.hclk = Some(Hertz(180_000_000));
    config.rcc.pclk1 = Some(Hertz(45_000_000));
    config.rcc.pclk2 = Some(Hertz(90_000_000));

    config
}

#[embassy::main(config = "config()")]
async fn main(spawner: Spawner, p: Peripherals) -> ! {
    static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);
    static UART_QUEUE: Forever<Channel<NoopRawMutex, ArrayString<32>, 8>> = Forever::new();

    let led1 = Output::new(p.PB0, Level::Low, Speed::VeryHigh);
    let button = Input::new(p.PC13, Pull::None);
    let button = ExtiInput::new(button, p.EXTI13);
    let usart = Uart::new(
        p.USART3,
        p.PD9,
        p.PD8,
        p.DMA1_CH3,
        embassy_stm32::dma::NoDma,
        usart::Config::default(),
    );

    let button_interrupt = Output::new(p.PG0, Level::Low, Speed::VeryHigh);
    let button_processed = Output::new(p.PG1, Level::Low, Speed::VeryHigh);
    // We're going to use this pin inside embassy, so now that it is set up right, we should forget it
    core::mem::forget(button_interrupt);

    let uart_queue = UART_QUEUE.put(Channel::new());
    let (sender, receiver) = mpsc::split(uart_queue);

    spawner.spawn(blink_led(led1, &BUTTON_PRESSED)).unwrap();
    spawner.spawn(uart_writer(usart, receiver)).unwrap();
    spawner
    .spawn(button_waiter(
        button,
        &BUTTON_PRESSED,
        sender,
        button_processed,
    ))
    .unwrap();


    // Main is done, run this future that never finishes
    loop {
        let () = core::future::pending().await;
    }
}

#[embassy::task]
async fn blink_led(mut led: Output<'static, PB0>, button_high: &'static AtomicBool) {
    loop {
        Timer::after(Duration::from_millis(100)).await;
        if !button_high.load(Ordering::SeqCst) {
            led.set_high();
        }
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
    }
}

#[embassy::task]
async fn button_waiter(
    mut button: ExtiInput<'static, PC13>,
    button_pressed: &'static AtomicBool,
    sender: Sender<'static, NoopRawMutex, ArrayString<32>, 8>,
    mut button_processed: Output<'static, PG1>,
) {
    let mut trigger_count = 0;

    fn format_message(trigger_count: i32, button_pressed: bool) -> ArrayString<32> {
        use core::fmt::Write;

        let mut string = ArrayString::new();
        core::writeln!(
            string,
            "Button is {} ({})",
            button_pressed as i32,
            trigger_count,
        )
        .unwrap();
        string
    }

    loop {
        button_processed.set_low();
        button.wait_for_rising_edge().await;
        button_processed.set_high();

        trigger_count += 1;
        button_pressed.store(true, Ordering::SeqCst);
        if sender.send(format_message(trigger_count, true)).await.is_err() {
            panic!("SendError");
        }

        button_processed.set_low();
        button.wait_for_falling_edge().await;
        button_processed.set_high();

        trigger_count += 1;
        button_pressed.store(false, Ordering::SeqCst);
        if sender.send(format_message(trigger_count, false)).await.is_err() {
            panic!("SendError");
        }
    }
}

#[embassy::task]
async fn uart_writer(
    mut usart: Uart<'static, USART3, DMA1_CH3>,
    mut receiver: Receiver<'static, NoopRawMutex, ArrayString<32>, 8>,
) {
    loop {
        let message = receiver.recv().await.unwrap();
        usart.write(message.as_bytes()).await.unwrap();
    }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("hardfault");
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

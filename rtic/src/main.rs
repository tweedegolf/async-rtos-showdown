#![no_main]
#![no_std]

use stm32f4xx_hal as _; // memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [CAN1_TX, LCD_TFT])]
mod app {
    use core::fmt::Write;
    use arrayvec::ArrayString;
    use rtic_monotonic::Monotonic;
    use stm32f4xx_hal::gpio::gpiob::PB0;
    use stm32f4xx_hal::gpio::gpiog::{PG1, PG0};
    use stm32f4xx_hal::{
        fugit::MonoTimer,
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, Output, Floating, PushPull},
        pac::{self, USART3},
        prelude::*,
        serial::{Serial, Tx},
    };

    #[monotonic(binds = TIM5, default = true)]
    type Tonic = MonoTimer<pac::TIM5, 45_000_000>;

    #[shared]
    struct Shared {
        button_pressed: bool,
    }

    #[local]
    struct Local {
        btn: PC13<Input<Floating>>,
        tx: Tx<USART3, u8>,
        led: PB0<Output<PushPull>>,
        button_processed: PG1<Output<PushPull>>,
        button_interrupt: PG0<Output<PushPull> > ,
        trigger_count: i32,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(180.mhz()).freeze();

        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();
        let gpiod = ctx.device.GPIOD.split();
        let gpiog = ctx.device.GPIOG.split();

        // Set up the LED.
        let led = gpiob.pb0.into_push_pull_output();
        let button_interrupt = gpiog.pg0.into_push_pull_output();
        let button_processed = gpiog.pg1.into_push_pull_output();

        // Set up the button.
        let mut btn = gpioc.pc13.into_floating_input();
        let mut sys_cfg = ctx.device.SYSCFG.constrain();
        btn.make_interrupt_source(&mut sys_cfg);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::RisingFalling);

        // Set up serial.
        let tx_pin = gpiod.pd8.into_alternate();
        let tx: Tx<USART3, u8> =
            Serial::tx(ctx.device.USART3, tx_pin, 115200.bps(), &clocks).unwrap();

        // Set up monotonic timer.
        let mono = ctx.device.TIM5.monotonic(&clocks);

        blink_led::spawn().ok();

        (
            Shared {
                button_pressed: false,
            },
            Local {
                btn,
                tx,
                led,
                button_processed,
                button_interrupt,
                trigger_count: 0,
            },
            init::Monotonics(mono),
        )
    }

    // Button interrupt
    #[task(binds = EXTI15_10, priority = 10, local = [btn, button_interrupt], shared = [button_pressed])]
    fn on_exti(mut ctx: on_exti::Context) {
        ctx.local.button_interrupt.set_high();
        ctx.local.btn.clear_interrupt_pending_bit();
        process_button::spawn().ok();

        ctx.shared.button_pressed.lock(|button_pressed| *button_pressed = ctx.local.btn.is_high());
        ctx.local.button_interrupt.set_low();
    }

    #[task(priority = 9, local = [button_processed, trigger_count], shared = [button_pressed])]
    fn process_button(mut ctx: process_button::Context) {
        ctx.local.button_processed.set_high();

        *ctx.local.trigger_count += 1;

        write_serial::spawn(format_message(
            *ctx.local.trigger_count,
            ctx.shared.button_pressed.lock(|button_pressed| *button_pressed),
        )).ok();

        ctx.local.button_processed.set_low();
    }

    #[task(local = [tx], capacity = 8)]
    fn write_serial(ctx: write_serial::Context, msg: ArrayString<32> ) {
        write!(ctx.local.tx, "{}", msg.as_str()).ok();
    }

    #[task(shared = [button_pressed], local = [led])]
    fn blink_led(mut ctx: blink_led::Context) {
        ctx.shared.button_pressed.lock(|button_pressed| {
            if *button_pressed {
                ctx.local.led.set_low();
            } else {
                ctx.local.led.toggle();
            }
        });

        blink_led::spawn_after(fugit::Duration::<u32, 1_u32, 45000000_u32>::millis(100)).ok();
    }

    fn format_message(trigger_count: i32, button_pressed: bool) -> ArrayString<32> {
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

#![no_main]
#![no_std]

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32f4xx_hal as _; // memory layout

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [CAN1_TX])]
mod app {
    use core::fmt::Write;
    use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use heapless::String;
    use rtic_monotonic::Monotonic;
    use stm32f4xx_hal::delay::Delay;
    use stm32f4xx_hal::gpio::gpioa::PA8;
    use stm32f4xx_hal::{
        fugit::MonoTimer,
        gpio::{gpioa::PA5, gpioc::PC13, Edge, ExtiPin, Input, Output, PullUp, PushPull},
        pac::{self, USART1},
        prelude::*,
        serial::{Serial, Tx},
    };

    #[monotonic(binds = TIM5, default = true)]
    type Tonic = MonoTimer<pac::TIM5, 45_000_000>;

    static BTN_COUNT: AtomicUsize = AtomicUsize::new(0);
    static OVERRIDE: AtomicBool = AtomicBool::new(false);

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        btn: PC13<Input<PullUp>>,
        tx: Tx<USART1, u8>,
        delay: Delay,
        led: PA5<Output<PushPull>>,
        exti_bench: PA8<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(180.mhz()).freeze();

        // Set up the LED.
        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();
        let exti_bench = gpioa.pa8.into_push_pull_output();

        // Set up the button.
        let gpioc = ctx.device.GPIOC.split();
        let mut btn = gpioc.pc13.into_pull_up_input();
        let mut sys_cfg = ctx.device.SYSCFG.constrain();
        btn.make_interrupt_source(&mut sys_cfg);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::RisingFalling);

        // Set up serial.
        let tx_pin = gpioa.pa9.into_alternate();
        let tx: Tx<USART1, u8> =
            Serial::tx(ctx.device.USART1, tx_pin, 115200.bps(), &clocks).unwrap();

        // Set up monotonic timer.
        let mono = ctx.device.TIM5.monotonic(&clocks);

        let delay = Delay::new(ctx.core.SYST, &clocks);

        (
            Shared {},
            Local {
                btn,
                tx,
                delay,
                led,
                exti_bench,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [delay, led])]
    fn idle(ctx: idle::Context) -> ! {
        loop {
            ctx.local.led.set_low();
            ctx.local.delay.delay_ms(200u32);
            if !OVERRIDE.load(Ordering::Relaxed) {
                ctx.local.led.set_high();
            }
            ctx.local.delay.delay_ms(100u32);
        }
    }

    #[task(binds = EXTI15_10, local = [btn, exti_bench])]
    fn on_exti(ctx: on_exti::Context) {
        ctx.local.exti_bench.set_high();
        ctx.local.btn.clear_interrupt_pending_bit();

        OVERRIDE.store(!ctx.local.btn.is_high(), Ordering::Relaxed);
        serial::spawn(format_message(
            BTN_COUNT.fetch_add(1, Ordering::Relaxed),
            ctx.local.btn.is_high(),
        ))
        .ok();
        ctx.local.exti_bench.set_low();
    }

    #[task(local = [tx], capacity = 8)]
    fn serial(ctx: serial::Context, msg: String<32>) {
        write!(ctx.local.tx, "{}", msg.as_str()).ok();
    }

    fn format_message(trigger_count: usize, button_pressed: bool) -> String<32> {
        let mut result = String::new();
        let _ = core::writeln!(
            result,
            "Button is {} ({trigger_count})",
            button_pressed as usize,
        );
        result
    }
}

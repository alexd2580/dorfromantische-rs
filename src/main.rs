use compute_shade_rs::{error, event_loop};

mod app;
mod data;

use app::App;

fn run_main() -> error::VResult<i32> {
    let event_loop = event_loop::EventLoop::default();
    let mut app = App::new(&event_loop)?;
    Ok(event_loop.run(&mut app))
}

fn main() {
    simple_logger::init_with_level(log::Level::Debug).unwrap();
    log::info!("Initializing...");
    if let Err(err) = run_main() {
        log::error!("{}", err);
    }
    log::info!("Terminating...");
}

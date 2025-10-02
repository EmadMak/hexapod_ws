use rclrs::{vendor::example_interfaces, *};
use std::time::Duration;

fn main() -> Result<(), rclrs::RclrsError> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("example_node")?;

    let publisher = node.create_publisher::<example_interfaces::msg::String>("example_topic")?;
    // let logger = node.logger();
    let mut count = 0;

    loop {
        let message = example_interfaces::msg::String {
            data: format!("Hello from Rust! Count: {}", count),
        };
        publisher.publish(&message)?;
        count += 1;
        std::thread::sleep(Duration::from_secs(1));
    }
}
use std::time::Duration;

use rclrs::*;
use std_msgs::msg::Float64MultiArray as FloatArray;
use std_msgs::msg::MultiArrayLayout;

const R_COXA: f64 = 0.022;
const L_FEMUR: f64 = 0.07;
const L_TIBIA: f64 = 0.0825;


fn ik_solve(x: f64, y: f64, z: f64) -> Vec<f64> {
    let theta_coxa = (y).atan2(x);

    let r = (x*x + y*y).sqrt();
    let x_prime = r - R_COXA;

    let a = (z.powi(2) + (x_prime - R_COXA).powi(2)).sqrt();

    let theta_f1 = (x_prime - R_COXA).atan2(z.abs());
    let theta_f2 = ((L_FEMUR.powi(2) + a.powi(2) - L_TIBIA.powi(2)) / (2.0 * a * L_FEMUR)).acos();
    let theta_femur = theta_f1 + theta_f2 - 90.0_f64.to_radians() - 20.0_f64.to_radians();

    let theta_tibia = ((L_FEMUR.powi(2) + L_TIBIA.powi(2) - a.powi(2)) / (2.0 * L_FEMUR * L_TIBIA)).acos() - 90.0_f64.to_radians();

    vec![theta_coxa, theta_femur, theta_tibia, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, theta_coxa, theta_femur, theta_tibia]
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let ik_node = executor.create_node("ik_node")?;

    let publisher = ik_node.create_publisher::<FloatArray>("/legs_controller/commands")?;

    // let _subscription = ik_node.create_subscription::<ImuMsg, _>(
    //     "imu", 
    //     move |msg: sensor_msgs::msg::Imu| {
    //         println!("I heard {:?}", msg.orientation);
    //     }
    // )?;

    // executor.spin(SpinOptions::default());
    //Ok(())
    let mut y_dir = 0.1;
    let mut step = 1.0;
    loop {
        let message = FloatArray {
            layout: MultiArrayLayout::default(),
            data: ik_solve(0.116, y_dir, -0.0536),
        };

        if y_dir >= 0.1 {
            step = -1.0;
        } else if y_dir <= -0.1 {
            step = 1.0;
        }

        y_dir += step * 0.02;

        publisher.publish(&message)?;
        std::thread::sleep(Duration::from_secs(1));
    }

}
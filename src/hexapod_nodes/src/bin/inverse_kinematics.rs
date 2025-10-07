use std::time::Duration;

use rclrs::*;
use std_msgs::msg::Float64MultiArray;
use std_msgs::msg::MultiArrayLayout;

const R_COXA: f64 = 0.022;
const L_FEMUR: f64 = 0.07;
const L_TIBIA: f64 = 0.0825;

const STEP_PERIOD: f64 = 1.0;
const BETA: f64 = 0.6;
const CLEARANCE_HEIGHT: f64 = 0.05;
const GROUND_HEIGHT: f64 = -0.0536;
const DT: f64 = 0.01;

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

fn stance(phi: f64, x_touch: f64, x_lift: f64, y_touch: f64, y_lift: f64) -> Float64MultiArray {
    let mut s = phi / BETA;
    s = 3.0*s.powi(2) - 2.0*s.powi(3);

    Float64MultiArray {
        layout: MultiArrayLayout::default(),
        data: ik_solve(
            x_touch + s * (x_lift - x_touch), 
            y_touch + s * (y_lift - y_touch), 
            GROUND_HEIGHT
        )
    }
}

fn swing(phi: f64, x_touch: f64, x_lift: f64, y_touch: f64, y_lift: f64) -> Float64MultiArray {
    let s = (1.0 - phi) / (1.0 - BETA);
    let psi = 10.0 * s.powi(3) - 15.0 * s.powi(4) + 6.0 * s.powi(5);

    Float64MultiArray {
        layout: MultiArrayLayout::default(),
        data: ik_solve(
            x_lift + psi * (x_touch - x_lift), 
            y_lift + psi * (y_touch - y_lift), 
            GROUND_HEIGHT + CLEARANCE_HEIGHT * 4.0*s*(1.0-s)
        )
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let ik_node = executor.create_node("ik_node")?;

    let publisher = ik_node.create_publisher::<Float64MultiArray>("/legs_controller/commands")?;

    let mut phi = 0.0;
    loop {
        if phi > 1.0 {
            phi = 0.0;
        };

        let mut _message: Float64MultiArray;
        
        if phi <= BETA {
            _message = stance(phi, 0.116, 0.116, 0.1, -0.1);
        } else {
            _message = swing(phi, 0.116, 0.116, -0.1, 0.1);
        }

        publisher.publish(&_message)?;
        std::thread::sleep(Duration::from_millis(100));
        phi += DT;
    }

}
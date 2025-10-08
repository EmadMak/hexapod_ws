use std::time::{Duration, Instant};

use rclrs::*;
use std_msgs::msg::Float64MultiArray;
use std_msgs::msg::MultiArrayLayout;

const R_COXA: f64 = 0.022;
const L_FEMUR: f64 = 0.07;
const L_TIBIA: f64 = 0.0825;

const STEP_PERIOD: f64 = 2.0;
const BETA: f64 = 0.6;
const CLEARANCE_HEIGHT: f64 = 0.05;
const GROUND_HEIGHT: f64 = -0.0536;
const DT: f64 = 0.001;

enum Leg {
    Lf, Lm, Lb,
    Rf, Rm, Rb
}

impl Leg {
    fn start_index(self) -> usize {
        match self {
            Leg::Lm => 0,
            Leg::Lf => 3,
            Leg::Lb => 6,
            Leg::Rf => 9,
            Leg::Rb => 12,
            Leg::Rm => 15
        }
    }
}

type JointTriplet = [f64; 3];

fn ik_leg(x: f64, y: f64, z: f64) -> JointTriplet {
    let theta_coxa = (y).atan2(x);

    let r = (x*x + y*y).sqrt();
    let x_prime = r - R_COXA;

    let a = (z.powi(2) + (x_prime - R_COXA).powi(2)).sqrt();

    let theta_f1 = (x_prime - R_COXA).atan2(z.abs());
    let theta_f2 = ((L_FEMUR.powi(2) + a.powi(2) - L_TIBIA.powi(2)) / (2.0 * a * L_FEMUR)).acos();
    let theta_femur = theta_f1 + theta_f2 - 90.0_f64.to_radians() - 20.0_f64.to_radians();

    let theta_tibia = ((L_FEMUR.powi(2) + L_TIBIA.powi(2) - a.powi(2)) / (2.0 * L_FEMUR * L_TIBIA)).acos() - 90.0_f64.to_radians();

    [theta_coxa, theta_femur, theta_tibia]
}

fn set_leg(buf: &mut [f64; 18], leg: Leg, joints: JointTriplet) {
    let i = leg.start_index();
    buf[i..i+3].copy_from_slice(&joints);
}

fn stance(phi: f64, x_touch: f64, x_lift: f64, y_touch: f64, y_lift: f64) -> [f64; 3] {
    let mut s = phi / BETA;
    s = 3.0*s.powi(2) - 2.0*s.powi(3);

    [
        x_touch + s * (x_lift - x_touch), 
        y_touch + s * (y_lift - y_touch), 
        GROUND_HEIGHT
    ]
}

fn swing(phi: f64, x_touch: f64, x_lift: f64, y_touch: f64, y_lift: f64) -> [f64; 3] {
    let s = (1.0 - phi) / (1.0 - BETA);
    let psi = 10.0 * s.powi(3) - 15.0 * s.powi(4) + 6.0 * s.powi(5);

    [
        x_lift + psi * (x_touch - x_lift), 
        y_lift + psi * (y_touch - y_lift), 
        GROUND_HEIGHT + CLEARANCE_HEIGHT * 4.0*s*(1.0-s)
    ]
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let ik_node = executor.create_node("ik_node")?;
    let publisher = ik_node.create_publisher::<Float64MultiArray>("/legs_controller/commands")?;

    let t0 = Instant::now();

    let tick = Duration::from_secs_f64(DT);
    let mut next_tick = t0 + tick;
    let mut legs_buffer = [0.0; 18];

    let offset = 0.5;
    
    loop {
        let t = t0.elapsed().as_secs_f64();

        let  phi = (t / STEP_PERIOD) % 1.0;
        let phi2 = (phi + offset) % 1.0;

        let leg_coords = if phi <= BETA {
            stance(phi, 0.116, 0.116, 0.05, -0.05)
        } else {
            swing(phi, 0.116, 0.116, -0.05, 0.05)
        };

        let other_leg_coords = if phi2 <= BETA {
            stance(phi2, 0.116, 0.116, 0.05, -0.05)
        } else {
            swing(phi2, 0.116, 0.116, -0.05, 0.05)
        };

        let joints = ik_leg(leg_coords[0], leg_coords[1], leg_coords[2]);
        let other_joints = ik_leg(other_leg_coords[0], other_leg_coords[1], other_leg_coords[2]);
        
        set_leg(&mut legs_buffer, Leg::Lb, joints);
        set_leg(&mut legs_buffer, Leg::Rm, joints);
        set_leg(&mut legs_buffer, Leg::Lf, joints);
        set_leg(&mut legs_buffer, Leg::Rb, other_joints);
        set_leg(&mut legs_buffer, Leg::Lm, other_joints);
        set_leg(&mut legs_buffer, Leg::Rf, other_joints); 

        let _message = Float64MultiArray {
            layout: MultiArrayLayout::default(),
            data: legs_buffer.to_vec()
        };

        publisher.publish(&_message)?;

        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        }
        next_tick += tick;
    }

}
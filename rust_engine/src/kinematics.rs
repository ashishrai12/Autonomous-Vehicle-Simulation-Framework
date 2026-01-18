use nalgebra::{Vector3, Rotation3};

pub struct BicycleModel {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub v: f64,
    pub wheelbase: f64,
}

impl BicycleModel {
    pub fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64) -> Self {
        Self { x, y, yaw, v, wheelbase }
    }

    pub fn step(&mut self, acceleration: f64, steering_angle: f64, dt: f64) {
        self.x += self.v * self.yaw.cos() * dt;
        self.y += self.v * self.yaw.sin() * dt;
        self.yaw += self.v / self.wheelbase * steering_angle.tan() * dt;
        self.v += acceleration * dt;
    }

    pub fn get_state(&self) -> (f64, f64, f64, f64) {
        (self.x, self.y, self.yaw, self.v)
    }
}

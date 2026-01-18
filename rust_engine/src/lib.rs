use pyo3::prelude::*;
mod kinematics;
mod collision;
mod raycast;

#[pyclass]
struct RustSimulationEngine {
    vehicle: kinematics::BicycleModel,
}

#[pymethods]
impl RustSimulationEngine {
    #[new]
    fn new(x: f64, y: f64, yaw: f64, v: f64, wheelbase: f64) -> Self {
        Self {
            vehicle: kinematics::BicycleModel::new(x, y, yaw, v, wheelbase),
        }
    }

    fn step(&mut self, acceleration: f64, steering_angle: f64, dt: f64) {
        self.vehicle.step(acceleration, steering_angle, dt);
    }

    fn get_vehicle_state(&self) -> (f64, f64, f64, f64) {
        self.vehicle.get_state()
    }

    fn check_collisions(&self, vehicle_width: f64, vehicle_height: f64, obstacles: Vec<(f64, f64, f64, f64)>) -> Vec<bool> {
        let (x, y, yaw, _) = self.vehicle.get_state();
        let vehicle_rect = collision::Rect { x, y, width: vehicle_width, height: vehicle_height, yaw };
        let obs_rects: Vec<collision::Rect> = obstacles.iter().map(|(ox, oy, w, h)| collision::Rect { x: *ox, y: *oy, width: *w, height: *h, yaw: 0.0 }).collect();
        collision::check_collisions(&vehicle_rect, &obs_rects)
    }

    fn cast_rays(&self, num_rays: usize, max_range: f64, obstacles: Vec<(f64, f64, f64, f64)>) -> Vec<f64> {
        let (x, y, yaw, _) = self.vehicle.get_state();
        let obs: Vec<raycast::Obstacle> = obstacles.iter().map(|(ox, oy, w, h)| raycast::Obstacle { x: *ox, y: *oy, width: *w, height: *h }).collect();
        
        let mut results = Vec::with_capacity(num_rays);
        for i in 0..num_rays {
            let angle = yaw + (i as f64 / num_rays as f64 - 0.5) * std::f64::consts::PI;
            let ray = raycast::Ray { origin_x: x, origin_y: y, angle };
            results.push(ray.cast(&obs, max_range));
        }
        results
    }
}

#[pymodule]
fn rust_engine(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<RustSimulationEngine>()?;
    Ok(())
}

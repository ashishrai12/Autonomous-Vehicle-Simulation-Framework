pub struct Rect {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
    pub yaw: f64,
}

impl Rect {
    pub fn intersects(&self, other: &Rect) -> bool {
        // Simplified AABB for now, can be upgraded to SAT (Separating Axis Theorem) for OBB
        let dx = (self.x - other.x).abs();
        let dy = (self.y - other.y).abs();
        let combined_half_width = (self.width + other.width) / 2.0;
        let combined_half_height = (self.height + other.height) / 2.0;

        dx < combined_half_width && dy < combined_half_height
    }
}

pub fn check_collisions(vehicle_rect: &Rect, obstacles: &[Rect]) -> Vec<bool> {
    obstacles.iter().map(|obs| vehicle_rect.intersects(obs)).collect()
}

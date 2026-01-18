pub struct Ray {
    pub origin_x: f64,
    pub origin_y: f64,
    pub angle: f64,
}

pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

impl Ray {
    pub fn cast(&self, obstacles: &[Obstacle], max_range: f64) -> f64 {
        let mut min_dist = max_range;
        let dx = self.angle.cos();
        let dy = self.angle.sin();

        for obs in obstacles {
            // Very simplified ray-box intersection
            // Checks intersection with AABB
            if let Some(dist) = self.intersect_aabb(obs) {
                if dist < min_dist {
                    min_dist = dist;
                }
            }
        }
        min_dist
    }

    fn intersect_aabb(&self, obs: &Obstacle) -> Option<f64> {
        let x_min = obs.x - obs.width / 2.0;
        let x_max = obs.x + obs.width / 2.0;
        let y_min = obs.y - obs.height / 2.0;
        let y_max = obs.y + obs.height / 2.0;

        let mut tmin = (x_min - self.origin_x) / self.angle.cos();
        let mut tmax = (x_max - self.origin_x) / self.angle.cos();

        if tmin > tmax { std::mem::swap(&mut tmin, &mut tmax); }

        let mut tymin = (y_min - self.origin_y) / self.angle.sin();
        let mut tymax = (y_max - self.origin_y) / self.angle.sin();

        if tymin > tymax { std::mem::swap(&mut tymin, &mut tymax); }

        if (tmin > tymax) || (tymin > tmax) {
            return None;
        }

        if tymin > tmin {
            tmin = tymin;
        }

        if tmax < 0.0 {
            return None;
        }

        Some(tmin.max(0.0))
    }
}

// middleware/src/kalman.rs

pub struct KalmanFilter {
    angle: f32,
    bias: f32,
    p: [[f32; 2]; 2],
    q_angle: f32,
    q_bias: f32,
    r_measure: f32,
}

impl KalmanFilter {
    pub fn new() -> Self {
        Self {
            angle: 0.0,
            bias: 0.0,
            p: [[0.0, 0.0], [0.0, 0.0]],
            q_angle: 0.001, 
            q_bias: 0.003,
            r_measure: 0.03,
        }
    }

    pub fn predict(&mut self, gyro_rate: f32, dt: f32) {
        let rate = gyro_rate - self.bias;
        self.angle += dt * rate;

        self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_angle);
        self.p[0][1] -= dt * self.p[1][1];
        self.p[1][0] -= dt * self.p[1][1];
        self.p[1][1] += self.q_bias * dt;
    }

    pub fn update(&mut self, accel_angle: f32) -> f32 {
        let y = accel_angle - self.angle; 
        let s = self.p[0][0] + self.r_measure; 
        let k = [self.p[0][0] / s, self.p[1][0] / s]; 

        self.angle += k[0] * y;
        self.bias += k[1] * y;

        let p00_temp = self.p[0][0];
        let p01_temp = self.p[0][1];

        self.p[0][0] -= k[0] * p00_temp;
        self.p[0][1] -= k[0] * p01_temp;
        self.p[1][0] -= k[1] * p00_temp;
        self.p[1][1] -= k[1] * p01_temp;

        self.angle
    }

    // This is the "Getter" that allows main.rs to see the current state
    pub fn get_angle(&self) -> f32 {
        self.angle
    }
}

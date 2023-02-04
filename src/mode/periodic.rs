/// Periodic reading where reading returns the last available data
pub struct Periodic {}

#[allow(dead_code)]
pub enum MPS {
    Half = 0x20,
    Normal = 0x21,
    Double = 0x22,
    X4 = 0x23,
    X10 = 0x27,
}

impl Periodic {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {}
    }
}

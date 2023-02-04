/// Periodic reading where reading returns the last available data
#[derive(Default, Copy, Clone, Debug)]
pub struct Periodic {
    mps: MPS
}

/// Stands for measurements per second
#[allow(dead_code)]
#[derive(Default, Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum MPS {
    Half = 0x20,
    #[default]
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

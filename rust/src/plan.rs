//! The build plan handed to the CAD side (same JSON as the Python worker).

use serde::Serialize;

#[derive(Serialize, Clone, Debug)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Loop {
    Circle { cx: f64, cy: f64, r: f64 },
    Polyline { pts: Vec<[f64; 2]> },
    /// Exact lines and three-point arcs, in order, closed.
    Path { pieces: Vec<PathPiece> },
}

#[derive(Serialize, Clone, Debug)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum PathPiece {
    Line { a: [f64; 2], b: [f64; 2] },
    Arc { a: [f64; 2], m: [f64; 2], b: [f64; 2], cx: f64, cy: f64, r: f64 },
}

#[derive(Serialize, Clone, Debug)]
pub struct Region {
    pub outer: Loop,
    pub holes: Vec<Loop>,
    pub area: f64,
    pub centroid: [f64; 2],
}

#[derive(Serialize, Clone, Debug)]
pub struct LayerOut {
    pub z0: f64,
    pub z1: f64,
    pub regions: Vec<Region>,
}

#[derive(Serialize, Clone, Debug, Default)]
pub struct DevOut {
    pub max_mm: f64,
    pub mean_mm: f64,
    pub p99_mm: f64,
    pub volume_diff_pct: f64,
    pub samples: usize,
}

#[derive(Serialize, Debug)]
pub struct Plan {
    pub version: u32,
    pub units: &'static str,
    pub strategy: String,
    pub tolerance_mm: f64,
    pub frame: Option<[[f64; 4]; 4]>,
    pub layers: Vec<LayerOut>,
    pub deviation: Option<DevOut>,
    pub deviation_sharp: Option<DevOut>,
    pub treatments: Vec<serde_json::Value>,
    pub circle_fit_error_mm: f64,
    pub simplify_error_mm: f64,
    pub review: Vec<String>,
    pub notes: Vec<String>,
    pub snapped: Vec<serde_json::Value>,
    pub llm_calls: u32,
    pub stats: serde_json::Value,
}

impl Plan {
    pub fn new(tol: f64) -> Plan {
        Plan {
            version: 1, units: "mm", strategy: "mesh_convert".into(), tolerance_mm: tol, frame: None,
            layers: vec![], deviation: None, deviation_sharp: None, treatments: vec![],
            circle_fit_error_mm: 0.0, simplify_error_mm: 0.0, review: vec![], notes: vec![],
            snapped: vec![], llm_calls: 0, stats: serde_json::json!({}),
        }
    }
}

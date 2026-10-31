//! stl2cad-worker: STL in, build-plan JSON out.
//!     stl2cad-worker part.stl --out plan.json [--tol 0.05] [--max-edge 6]

use stl2cad::analyze::analyze_with;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (mut stl, mut out, mut tol, mut max_edge) = (None, None, 0.05f64, 6.0f64);
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--out" => { out = args.get(i + 1).cloned(); i += 2; }
            "--max-edge" => { max_edge = args.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or(6.0); i += 2; }
            "--tol" => { tol = args.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or(0.05); i += 2; }
            a if !a.starts_with("--") && stl.is_none() => { stl = Some(a.to_string()); i += 1; }
            a => { eprintln!("unknown argument: {a}"); std::process::exit(2); }
        }
    }
    let (Some(stl), Some(out)) = (stl, out) else {
        eprintln!("usage: stl2cad-worker part.stl --out plan.json [--tol 0.05]");
        std::process::exit(2);
    };
    match analyze_with(&stl, tol, max_edge) {
        Ok(plan) => {
            std::fs::write(&out, serde_json::to_string_pretty(&plan).unwrap()).expect("write plan");
            println!("{}", serde_json::json!({
                "strategy": plan.strategy, "layers": plan.layers.len(),
                "deviation": plan.deviation, "llm_calls": plan.llm_calls }));
        }
        Err(e) => { eprintln!("error: {e}"); std::process::exit(1); }
    }
}

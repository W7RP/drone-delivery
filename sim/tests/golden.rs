//! Golden summary of the A-B example (seed 42). Any behaviour change shows up here; if the
//! change is intended, regenerate with `UPDATE_GOLDEN=1 cargo test --test golden` and review
//! the diff before committing.

mod common;
use common::*;

const GOLDEN: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/golden/ab_synthetic_seed42.json"
);

#[test]
fn ab_synthetic_summary_matches_golden() {
    let r = run_text(&std::fs::read_to_string(EXAMPLE).unwrap(), None);
    let got = format!("{}\n", serde_json::to_string_pretty(&r.summary).unwrap());
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::write(GOLDEN, &got).unwrap();
        return;
    }
    let want =
        std::fs::read_to_string(GOLDEN).expect("golden file exists (UPDATE_GOLDEN=1 creates it)");
    assert_eq!(got, want, "summary differs from the golden file");
}

//! The Rust enums and event kinds must match the schemas exactly: names and order (the order
//! is the binary code, spec section 0).

use drone_sim::ids::*;
use drone_sim::trace::KIND_NAMES;
use serde_json::Value;

fn schema(name: &str) -> Value {
    let path = format!("{}/../schemas/{name}", env!("CARGO_MANIFEST_DIR"));
    serde_json::from_str(&std::fs::read_to_string(path).expect("schema readable"))
        .expect("schema is JSON")
}

fn names<T: Copy>(all: &[T], name: fn(T) -> &'static str) -> Vec<String> {
    all.iter().map(|&x| name(x).to_owned()).collect()
}

macro_rules! check {
    ($common:expr, $($ty:ident),+) => {
        $(
            let expected: Vec<String> = $common["$defs"][stringify!($ty)]["enum"]
                .as_array()
                .unwrap_or_else(|| panic!("schema has no enum {}", stringify!($ty)))
                .iter()
                .map(|v| v.as_str().unwrap().to_owned())
                .collect();
            assert_eq!(names($ty::ALL, $ty::name), expected, "enum {}", stringify!($ty));
            for (i, v) in $ty::ALL.iter().enumerate() {
                assert_eq!(usize::from(v.code()), i);
            }
        )+
    };
}

#[test]
fn enums_match_common_schema() {
    let c = schema("common.schema.json");
    check!(
        c,
        PadMode,
        Spot,
        VisitState,
        VisitOutcome,
        DroneMode,
        MissionKind,
        OrderKind,
        OrderStatus,
        PodLocation,
        HoldReason,
        ServiceOutcome,
        AlertCode,
        AlertSeverity,
        EntityKind,
        EventClass,
        TimerKind,
        FlightSource,
        ServiceMode,
        ReturnPolicy,
        ChargingMode,
        OutEmptyPolicy,
        InOccupiedAction,
        FaultKind,
        Preset,
        ConfigStatus,
        MatrixSource,
        NetworkTopology
    );
}

#[test]
fn event_kinds_match_events_schema_and_fit_a_record() {
    let e = schema("events.schema.json");
    let kinds: Vec<&str> = e["properties"]["kind"]["enum"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    assert_eq!(kinds, KIND_NAMES.to_vec());
    let width = |t: &str| match t {
        "u8" | "bool" => 1,
        "u16" => 2,
        "u32" | "f32" => 4,
        "u64" | "f64" => 8,
        other => panic!("unexpected x-type {other}"),
    };
    let common = schema("common.schema.json");
    for k in &kinds {
        let def = &e["$defs"][*k];
        let bytes: usize = def["required"]
            .as_array()
            .unwrap()
            .iter()
            .map(|f| {
                let prop = &def["properties"][f.as_str().unwrap()];
                let t = prop["x-type"]
                    .as_str()
                    .map(str::to_owned)
                    .unwrap_or_else(|| {
                        let r = prop["$ref"].as_str().unwrap().rsplit('/').next().unwrap();
                        common["$defs"][r]["x-type"].as_str().unwrap().to_owned()
                    });
                width(&t)
            })
            .sum();
        assert!(
            bytes <= 32,
            "{k} payload of {bytes} bytes does not fit the 32-byte args area"
        );
    }
}

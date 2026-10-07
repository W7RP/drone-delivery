//! Load a scenario end to end: parse and validate the config, build the network, and apply
//! the rules that need routes (R12, R16). Then build a `World` ready to run.

use std::path::Path;

use crate::config::{self, Config, ConfigError};
use crate::flight::EnergyRules;
use crate::ids::PadId;
use crate::network::Network;
use crate::output::Outputs;
use crate::world::World;

pub struct Scenario {
    pub cfg: Config,
    pub net: Network,
}

impl Scenario {
    /// A scenario from its text alone; a `network_files` scenario needs `from_toml_in`.
    pub fn from_toml(text: &str) -> Result<Scenario, ConfigError> {
        Scenario::from_toml_in(text, None)
    }

    /// `scenario_dir` resolves `network_files.dir` (spec 8.4).
    pub fn from_toml_in(text: &str, scenario_dir: Option<&Path>) -> Result<Scenario, ConfigError> {
        let cfg = config::parse_in(text, scenario_dir)?;
        let net = Network::build(&cfg);
        check_energy_rules(&cfg, &net)?;
        Ok(Scenario { cfg, net })
    }

    pub fn load(path: &Path) -> Result<Scenario, ConfigError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| ConfigError(format!("{}: {e}", path.display())))?;
        let scenario = Scenario::from_toml_in(&text, path.parent())?;
        let dir_name = path
            .parent()
            .and_then(Path::file_name)
            .and_then(|n| n.to_str());
        if dir_name.is_some_and(|n| n != scenario.cfg.name) {
            return Err(ConfigError(format!(
                "meta.name: R14 \"{}\" must equal the scenario directory name",
                scenario.cfg.name
            )));
        }
        Ok(scenario)
    }

    /// Override the master seed (CLI `--seed`); the generated network keeps the config seed.
    pub fn with_seed(mut self, seed: u64) -> Scenario {
        self.cfg.seed = seed;
        self
    }

    pub fn into_world(self, replication: u32, out: Outputs) -> World {
        World::new(self.cfg, self.net, replication, out)
    }
}

/// R12 (every directed lane) and R16 (every demand flow) must pass `dispatch_ok` with the gross
/// payload from `drone.charge_target_soc`, all pads online.
fn check_energy_rules(cfg: &Config, net: &Network) -> Result<(), ConfigError> {
    let rules = EnergyRules::from_config(cfg);
    let usable = rules.usable_j(cfg.drone.charge_target_soc);
    let m = cfg.pod_gross_mass_kg;
    let need =
        |a: PadId, b: PadId| rules.dispatch_need_j(net, a, b, m, |_| true, |q| cfg.has_charger(q));
    for l in 0..net.lane_to.len() {
        let (a, b) = (net.lane_from[l], net.lane_to[l]);
        let n = need(a, b);
        if n > usable {
            return Err(ConfigError(format!(
                "R12: lane {} -> {} needs {n:.0} J above reserve, battery gives {usable:.0} J",
                cfg.pads[usize::from(a)].name,
                cfg.pads[usize::from(b)].name
            )));
        }
    }
    for f in &cfg.flows {
        let n = need(f.origin, f.dest);
        if f.rate_per_s > 0.0 && n > usable {
            return Err(ConfigError(format!(
                "R16: flow {} -> {} needs {n:.0} J above reserve, battery gives {usable:.0} J",
                cfg.pads[usize::from(f.origin)].name,
                cfg.pads[usize::from(f.dest)].name
            )));
        }
    }
    Ok(())
}

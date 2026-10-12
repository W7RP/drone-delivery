# Merge plan

How to bring the phase branches into `main`. Nothing here has been run; the merges are for the
maintainer to do. The repository stays private (it holds OSM-derived files).

## State on 2026-10-09

Every phase branch was cut from the tip of the one before it and none has been rebased, so the
history is one straight line:

```
main (ba98d6f, Phase 1 spec)
 -> feat/phase2-sim-core            10 commits
 -> feat/phase3-pittsburgh-pipeline 13
 -> feat/phase4-px4-single-lane     12
 -> feat/phase5-flight-calibration  16
 -> feat/phase6a-network-realism    12
 -> feat/phase6b-weather-traffic    12
 -> feat/phase7-vtol                the rest of Phase 7
```

Checked with `git merge-base --is-ancestor <previous> <next>` for each pair (all true) and
`git rev-list --count <next>..<previous>` (all 0: no branch has commits its successor lacks).
`main` has had no commits since Phase 1.

## Order and commands

Merge in phase order, one merge commit per phase, so `git log --first-parent main` reads as the
list of phases. Run the verification suite on `main` after each merge; every phase branch
passed its own suite when it was pushed.

```
git fetch origin
git switch main
git pull --ff-only origin main

git merge --no-ff feat/phase2-sim-core            -m "merge: phase 2 simulation core"
scripts/verify.sh
git merge --no-ff feat/phase3-pittsburgh-pipeline -m "merge: phase 3 Pittsburgh pipeline"
scripts/verify.sh
git merge --no-ff feat/phase4-px4-single-lane     -m "merge: phase 4 PX4 single lane"
scripts/verify.sh
git merge --no-ff feat/phase5-flight-calibration  -m "merge: phase 5 flight calibration"
scripts/verify.sh
git merge --no-ff feat/phase6a-network-realism    -m "merge: phase 6a network realism"
scripts/verify.sh
git merge --no-ff feat/phase6b-weather-traffic    -m "merge: phase 6b weather and traffic"
scripts/verify.sh
git merge --no-ff feat/phase7-vtol                -m "merge: phase 7 VTOL"
scripts/verify.sh

git push origin main
```

`scripts/verify.sh` at a phase's merge checks only what that phase's suite knows about; the
suite grows with the phases. The PX4 flights are not part of it (they need the SITL stack and
several minutes of wall time per flight, docs/flight-runs.md), so a merge does not re-fly them.

If a straight line is preferred over merge commits, `git merge --ff-only feat/phase7-vtol` on
`main` gives the same tree in one step.

## Expected conflicts

None while the state above holds: each merge only adds the next branch's commits on top of
what `main` already has, so git never has two sides to reconcile.

Conflicts appear only if something is committed to `main` or to an earlier phase branch before
the merges. In that case merge (do not rebase or amend, the branches are pushed) and expect
them in the files every phase touches:

- `docs/spec.md`: the version history table at the end and the assumption list (A-numbers).
  Keep both sides' rows in version order; renumber nothing, assumptions are cited by number.
- `sim/src/config.rs` (`SCHEMA_VERSION`, `VERSIONED_KEYS`) and `schemas/*.schema.json`: keep
  the highest schema version and the union of the keys.
- `scripts/verify.sh`, `README.md`, `docs/running.md`: keep both sides' steps.
- Golden outputs under `sim/tests/golden/`: never hand-merge. Take the newer phase's file and
  rerun `cargo test`; a golden that changes is a behavior change to explain, not to accept.

Generated files (`configs/*/derived/`, `pipeline/calib/out/`, `analysis/out/`) are rebuilt by
their scripts rather than merged by hand; each records the revision that wrote it.

## After the merge

- Tag the result if it is to be a release (`git tag -a v0.7.0 -m "phase 7"`), which also needs
  a CHANGELOG.md in Keep a Changelog form; the project has none yet.
- Delete the merged phase branches only once `main` is pushed and verified.

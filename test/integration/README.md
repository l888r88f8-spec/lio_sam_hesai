# Localization stability validation

This directory contains the external-data regression checks for the Hesai
simulation.  The bag and map are deliberately not stored in Git.  Place them
under the same data root:

```text
/home/robot/rosbag2/hesai_sim
/home/robot/map/hesai_sim2
```

The default data root is `/home/robot`; set `LIO_SAM_TESTDATA_ROOT` when the
two directories are mounted elsewhere.  The current bag requires about 1.5
GiB and the map about 8 MiB, so provision at least 1.6 GiB for the inputs plus
space for build products and temporary logs.

From the repository root, use the supported entrypoint:

```bash
scripts/run_localization_stability.sh quick
scripts/run_localization_stability.sh record-baseline
scripts/run_localization_stability.sh memory
```

`quick` replays the reviewed window and compares it with
`test/integration/baselines/hesai_sim.yaml`.  `record-baseline` replays the
same data but writes a candidate baseline for review.  `memory` loop-replays
the input and checks its RSS result against the existing baseline; it never
updates a baseline.

Each run creates a temporary result directory named
`/tmp/lio-sam-hesai-stability.XXXXXX`.  Set `LIO_SAM_KEEP_RESULTS=1` to retain
it after the command exits.  Quick and record runs write `launch.log`,
`bag-play.log`, the generated runtime parameter file, and `observed.json`.
Record mode additionally writes `candidate-baseline.yaml`.  Memory runs write
the same process logs and runtime parameter file, plus `rss.csv` and
`rss-summary.json`.

Review a candidate's dataset identity and every observed metric before copying
it into `test/integration/baselines/hesai_sim.yaml`.  Make that baseline update
in a separate commit; do not combine it with collector or workflow changes.
Only explicit `record-baseline` mode may produce a candidate.  CI runs quick
and memory validation only and never updates a baseline.

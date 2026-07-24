# Serial Early-Exit Relocalization Design

## Goal

Reduce typical automatic relocalization latency below 500 ms by evaluating
Scan Context candidates in score order, while preserving all configured
`sc_top_n` candidates as a fallback.

## Design

`ScanContext::Query` already returns candidates in ascending descriptor
distance, so `Localization::TryAutoRelocalize` consumes that order directly.
For each candidate it prepares only that candidate's ground-constrained pose
and 50 m local map, creates one independent matcher, and runs fine registration.
Candidate-level `std::async` parallelism is removed; the IVOX matcher's existing
`std::execution::par_unseq` point-level parallelism remains enabled.

After each successful match:

- If its finite fitness is below `sc_fitness_thres`, retain it when it is the
  best result seen so far.
- If its fitness is also below `sc_accept_fitness`, stop immediately and return
  it.
- Otherwise continue until all candidates have been evaluated, then return the
  best result below `sc_fitness_thres`.

An exception or unusable local map rejects only the current candidate. A manual
initial pose that arrives during evaluation prevents additional automatic
candidates from starting; it is consumed by the existing initialization path
on the next processing cycle.

## Observability

Log each candidate's one-based rank, Scan Context score, preparation time,
registration time, and resulting fitness. Log the total number prepared and
evaluated, total preparation and registration time, and whether evaluation
stopped early.

## Testing

Add a source-surface regression test proving that the automatic relocalization
path no longer calls `RunBoundedAsync`, evaluates candidates in index order, and
uses `ShouldEarlyAcceptRelocalization` inside that loop. Rebuild the
relocalization executable and run the existing relocalization unit tests.

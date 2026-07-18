# Test evidence policy

Evidence documents use `docs/test-evidence/schema/evidence.schema.json` and are
validated by `tools/port/validate_evidence.py`. Required metadata captures the
package/scenario, source revisions, UTC start time, environment, commands,
artifacts, and structured pass/fail/skip/error totals.

Tracked evidence contains compact JSON, summaries, hashes, and small golden
traces. Large serial logs, packet captures, binaries, maps, core dumps, and HIL
recordings stay in CI artifact storage and are referenced by immutable artifact
name plus SHA-256. Release evidence is retained for the life of its release;
ordinary branch evidence is retained for at least 30 days. Never include
credentials, usernames, home directories, device serial numbers, private host
names, or absolute workstation paths.

HIL runs additionally record board type, firmware checksum, debugger/probe
class (not serial number), CAN bitrate/mode, reset method, fixture revision, and
per-test isolation result. A skipped hardware test needs a machine-readable
reason and never counts as a pass.

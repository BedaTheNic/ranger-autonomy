# jazzy-r5.1 validation

Application sources, configured controller limits and gateway speed-cap removal
are unchanged from jazzy-r5; its native/Jazzy unit and gateway results apply to
the same source hashes. The integration fixture now waits for the lifecycle
manager to become active before querying individual node states, and retries
read-only readiness requests that time out during initialization. It still
asserts that every required node is active and verifies all planning, control,
stale-scan, cancellation and logging behavior.

Local adaptive, holonomic and standalone Ackermann integration tests passed.
An additional adaptive test with the planner limited to half a CPU passed,
including its longer Hybrid initialization. This is software validation with
synthetic inputs, not physical steering or autonomous-drive validation.

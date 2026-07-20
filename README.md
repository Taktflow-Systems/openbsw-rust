# Eclipse openBSW Rust port

This workspace is an allocation-free Rust port of the mandatory behavior
surveyed at Eclipse openBSW commit
`be0029bbb79fe901048a24c2665f2ba854328734`. It provides a POSIX reference
application and shared production compositions for STM32F413 and STM32G474.

Start with:

- `docs/architecture/production-reference.md` for ownership and composition;
- `docs/port/developer-guide.md` for the host build and reference app;
- `docs/port/board-setup.md` for the supported STM32 boards;
- `docs/port/test-guide.md` for host, target, HIL, Miri, and fuzz gates;
- `docs/port/release-build.md` for reproducible release artifacts;
- `docs/port/status.md` for the generated 37/37 parity ledger.

The functional-safety mechanisms are diagnostic support building blocks, not
a certification claim. Their capabilities and exclusions are documented in
`docs/port/safety-mechanisms-2026-07-18.md`.

The repository is licensed under Apache-2.0; see `LICENSE` and `NOTICE`.

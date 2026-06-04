# Hardware Tests

`tests/` contains no_std/no_main firmware images for hardware checks. These are not libtest unit tests.

Build or flash a hardware test only when explicitly enabling the feature:

```bash
cargo test --test servo --features hardware-test
```

Without `hardware-test`, normal `cargo test` does not build the servo hardware-test target.

# dx_stream upstream PRs (base: dev @ a7a95c9)

Each directory is one PR: a `git format-patch` series plus `PR.md`. Commits stay
one per defect / change so each can be reviewed or reverted on its own.
Submit in this order.

| # | dir | type | base | commits |
|---|---|---|---|---|
| 1 | 1-dynamic-source-lifecycle | **Bug fix** | dev | 10 |
| 2 | 2-inputselector-live-deadline | **Bug fix** (live behaviour change) | dev | 3 |
| 3 | 3-silent-drops | **Feature** (includes 1 bug fix) | 1 + 2 | 3 |
| 4 | 4-message-identity-time | **Feature** | 3 | 3 |

    git checkout -b <name> origin/dev   # 3, 4: branch from their base
    git am <dir>/*.patch

Verified: dev + 1..4 applied in this order builds; all 59 element and metadata
test suites pass (320 checks) and the pydxs tests pass (40/40). The same code
ran a 16-camera pipeline for 24 h plus fault injection (source freeze, RTSP
server restart, unreachable source, stream id reuse, main/sub switch, mid-stream
resolution change, Kafka paused 30 s).

Local test note: `test/common/meta_helpers.hpp` includes the installed headers;
set `INSTALL_PREFIX` to an install of the tree under test.

## Recreating the branches

Only the patches are published here, not the `dev` history. On a clone that has
`dev` @ `a7a95c9`:

    git checkout -b fix/dynamic-source-lifecycle    a7a95c9 && git am 1-dynamic-source-lifecycle/*.patch
    git checkout -b fix/inputselector-live-deadline a7a95c9 && git am 2-inputselector-live-deadline/*.patch
    git checkout -b feature/silent-drops fix/dynamic-source-lifecycle && git am 2-inputselector-live-deadline/*.patch 3-silent-drops/*.patch
    git checkout -b feature/message-identity-time feature/silent-drops && git am 4-message-identity-time/*.patch

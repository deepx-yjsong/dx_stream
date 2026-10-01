## Title

feat(msgconv,inputselector,msgbroker): identify the source and its capture time in each message

**Type: Feature** — additive; existing pipelines, JSON keys and custom libraries built against the old headers keep working.

## Summary

A message says which pad a frame came through, but not which source that is,
which node produced it, or when the frame was captured. With sources added and
removed at runtime, or several nodes on one broker, consumers cannot tell
messages apart or place them in time.

| Addition | How |
|---|---|
| Source identity | `dxinputselector` sink pads get a `sensor-id` property (mutable in PLAYING), stamped as `DXFrameMeta._sensor_id`. Unlike the pad index it is not reused. Set before linking and the first frame carries it. |
| Node identity | `dxmsgconv` `node-id`. |
| Capture time | From the buffer's `GstReferenceTimestampMeta`, **only** `timestamp/x-unix` or `timestamp/x-ntp` (as rtspsrc attaches), converted to the Unix epoch. Other references (e.g. PTP) are not UTC and are ignored. Buffer PTS passed as well. |
| Message key | `dxmsgconv` sets the sensor id as the key on `GstDxMsgMeta`; the Kafka adapter sends it, so one source keeps its order on one partition. No sensor id → no key (unchanged). MQTT ignores it. |
| Wire format | `dxmsgconv` `payload-type` enum (`json` default, `protobuf`). `dx_msgconvl` emits `dxstream.SensorFrame` (`dx_msgconvl/proto/dx_frame.proto`); `capture_time` is `google.protobuf.Timestamp` (as in the NVIDIA Metropolis schema). No proto3 `optional`, so protoc 3.6+ builds it. Protobuf backend optional at build time (`-Dprotobuf=auto`). New JSON keys (`sensorId`, `nodeId`, `ptsNs`, `captureTime` RFC 3339) use proto3 JSON names; existing keys unchanged. |

| Python access | `pydxs` exposes `DXFrameMeta.sensor_id`, so a Python pad probe can tell sources apart (the pad index is reused). |

**Custom libraries built against the old headers keep working**: new fields
are appended to `GstDxMsgMetaInfo` and `DXFrameMeta`; `DxMsgPayload` and
`DxMsgContext`, which libraries allocate, are unchanged (the key lives on the
element-owned `GstDxMsgMeta`).

**Depends on** PR 3 (applies on top; touches the same files).

## Verification

`test_dxmsgconv_identity` (sensorId/nodeId, key, capture time from x-ntp and
x-unix, PTP ignored — mutation-checked: dropping the NTP offset or the
reference filter fails a case), pad `sensor-id` stamping, `ptsNs`,
`DXFrameMeta._sensor_id` copy, `dx_msgconvl` `test_protobuf_convert`
(round trip with and without times), and pydxs CPY2/CPY7 (the sensor id
dxinputselector stamps from its pad is read back from Python; 40/40). Field: 16 cameras publishing protobuf to
Kafka for 24 h, four consumers (Python, Ruby/Logstash) decoding `capture_time`;
keys equal sensor ids on every message checked. A Python probe on the
dxmsgbroker sink pad counts output per camera through `pydxs` (16 cameras).

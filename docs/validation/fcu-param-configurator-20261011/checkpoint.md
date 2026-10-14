# FCU parameter compatibility diagnostic — RETAINED, review corrections

MowgliNext feat/mavros-refresh HEAD35986cd unchanged; MVRbf5bb0d and both
gitlinks unchanged. Existing unrelated modifications preserved. No commit/push.

Review clarification applied: launch defaults255/1, manually entered bindings
and observed FCU identity do not establish effective MAVROS IDs/GCS acceptance.
MAV_SYSID/MAV_GCS_SYSID always read-only To verify, no proposed renumbering,
including multirobot IDs, default matches and GCS ranges. Resolution can belong
in MAVROS. No additional confirmation/authorization protocol introduced.
Only identified serial-protocol compatibility correction remains available.
Other provisional output/CAN/RPM/safety/navigation items unchanged To verify;
calibrations/preferences remain exact; YAML builder and tabs preserved.

Full personal Pixhawk export15896bytes SHA256
c4f1162d20fabcfe1c583304f9cee924c368a5ca93a2c6b6714bebd0c89e8a8e
moved without byte changes into local-only/ (Git-ignored). Prior evidence,
including the old diff embedding the export, saved under local-only/pre-review/.
Neither belongs in a public commit without explicit operator authorization.
Public tests now use independent synthetic-runtime.param:12 synthetic parameters,
BOM/comments/tabs/CRLF/no final newline. Public diff regenerated without export.

Review tests PASS3 files35 cases (36.07s), including unconfirmed SYSID rejection,
observed identity with no SYSID correction, selective serial export and exact
conservation. TypeScript/targeted strict lint PASS. Existing full baseline49
cases and Vite build retained; unrelated suites/build not rerun for targeted
review fixes. See results.json, review-*.log, source.sha256, preservation.json,
whitespace.json and review.diff. HARDWARE_REQUIRED criteria remain in README.md.
No hardware/device writes/firmware/submodule changes. No commit authorized.

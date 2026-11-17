#!/usr/bin/env python3
"""Exercise extension lifecycle operations in one private Docker Compose project."""

import argparse
import json
import os
import platform
import re
import secrets
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

DOCKER = ["docker", "--host=unix:///var/run/docker.sock"]
READINESS = "http://127.0.0.1:4006/api/system/update-readiness"
IMAGE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9./_:-]*@sha256:[a-f0-9]{64}$")
REPO_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*/[A-Za-z0-9][A-Za-z0-9_.-]*$")
REV_RE = re.compile(r"^[a-f0-9]{40}$")


class Failure(Exception):
    pass


def run(command, timeout=150):
    redirects = {"DOCKER_HOST", "DOCKER_CONTEXT", "DOCKER_TLS", "DOCKER_TLS_VERIFY",
                 "DOCKER_CERT_PATH", "DOCKER_DEFAULT_PLATFORM"}
    env = {k: v for k, v in os.environ.items() if k not in redirects and not k.startswith("COMPOSE_")}
    try:
        return subprocess.run(command, text=True, capture_output=True, timeout=timeout, env=env, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise Failure(f"command could not complete ({type(exc).__name__})") from None


def checked(command, label):
    result = run(command)
    if result.returncode:
        raise Failure(f"{label} failed (exit {result.returncode})")
    return result


def ready():
    try:
        with urllib.request.urlopen(READINESS, timeout=5) as response:
            value = json.load(response)
    except (OSError, ValueError) as exc:
        raise Failure(f"robot readiness check failed ({type(exc).__name__})") from None
    if not isinstance(value, dict) or value.get("ready") is not True or value.get("maintenance") is not False:
        raise Failure("robot readiness is not idle and ready")


def json_file(path, value):
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    path.chmod(0o600)


def containers():
    result = checked(DOCKER + ["ps", "-a", "--no-trunc", "--format", "{{.Names}} {{.ID}}"], "container inventory")
    items = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) != 2:
            raise Failure("Docker returned an invalid container identity list")
        items[fields[0]] = fields[1]
    return items


def started_at(items):
    if not items:
        return {}
    output = checked(DOCKER + ["inspect", "--format", "{{.Name}} {{.State.StartedAt}}", *sorted(items.values())], "start-time inventory").stdout
    values = {}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) != 2:
            raise Failure("Docker returned an invalid start-time projection")
        values[fields[0].removeprefix("/")] = fields[1]
    if len(values) != len(items):
        raise Failure("Docker start-time inventory was incomplete")
    return values


def bundle(extension_id, version, health, seconds, image, repository, revision, project, data_dir, platform_name, volume_name=None):
    service = {"image": image, "user": "65532", "entrypoint": ["/bin/sleep"], "command": [seconds],
               "read_only": True, "cap_drop": ["ALL"], "security_opt": ["no-new-privileges:true"],
               "restart": "unless-stopped", "healthcheck": {"disable": True}, "networks": {"private": {}},
               "volumes": [{"type": "bind", "source": str(data_dir), "target": "/data"}]}
    compose = {"name": project, "services": {"probe": service},
               "networks": {"private": {"name": project + "_private", "driver": "bridge", "internal": True}}}
    if volume_name:
        service["volumes"] = [{"type": "volume", "source": "data", "target": "/data"}]
        compose["volumes"] = {"data": {"name": volume_name, "external": True}}
    manifest = {"schema": 1, "id": extension_id, "version": version,
                "source": {"repository": repository, "revision": revision}, "runtime": {"kind": "compose"},
                "requires": {"manager_api": 1, "platforms": [platform_name], "contracts": []},
                "grants": ["storage:extension-data"], "data_schema": 1, "health": health, "profile": "isolated"}
    return {"manifest": manifest, "compose": compose}


def need(condition, message):
    if not condition:
        raise Failure(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, help="built mowgli-extensions executable")
    parser.add_argument("--image", required=True, help="preloaded Linux image ending in @sha256:<64 lowercase hex>")
    parser.add_argument("--source-repository", required=True, help="owner/repository manifest source")
    parser.add_argument("--source-revision", required=True, help="full lowercase 40-character commit")
    parser.add_argument("--robot-readiness", action="store_true", help="check local GUI update-readiness before each mutation")
    parser.add_argument("--storage", choices=("bind", "volume"), default="bind", help="exercise a private bind directory or external named volume")
    parser.add_argument("--keep-files", action="store_true", help="retain this run's temporary files for debugging")
    args = parser.parse_args()
    binary = Path(args.binary).resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error("--binary must be executable")
    if not IMAGE_RE.fullmatch(args.image):
        parser.error("--image must be an immutable @sha256:<64 lowercase hex> reference")
    if not REPO_RE.fullmatch(args.source_repository):
        parser.error("--source-repository must be owner/repository")
    if not REV_RE.fullmatch(args.source_revision):
        parser.error("--source-revision must be 40 lowercase hexadecimal characters")

    phases, root, compose_cmd = [], None, None
    cleanup_needed = cleanup_ok = False
    error = None
    project = "extension-lifecycle-" + secrets.token_hex(6)
    volume_name = project + "_data" if args.storage == "volume" else None
    volume_created = False
    helper_name = project + "-data-check"
    test_label = "garden.mowgli.extension-test"
    extension_id = "engine-probe-" + secrets.token_hex(6)
    baseline_ids, baseline_starts = {}, {}
    identities_preserved = data_preserved = None

    def volume_marker(seed=False):
        command = DOCKER + ["run", "--rm", "--name", helper_name, "--label", f"{test_label}={project}",
                            "--pull", "never", "--network", "none", "--read-only", "--cap-drop", "ALL",
                            "--security-opt", "no-new-privileges:true", "--user", "65532"]
        mount = f"type=volume,src={volume_name},dst=/data"
        if seed:
            command += ["--mount", mount, "--entrypoint", "/bin/sh", args.image,
                        "-c", "umask 022; printf 'private extension test data\\n' > /data/retained.txt"]
        else:
            command += ["--mount", mount + ",readonly", "--entrypoint", "/bin/cat",
                        args.image, "/data/retained.txt"]
        return checked(command, "private volume marker").stdout

    def record(phase, **detail):
        nonlocal identities_preserved, data_preserved
        current = containers()
        starts = started_at(current)
        identities_preserved = identities_preserved is not False and not any(
            current.get(n) != i or starts.get(n) != baseline_starts.get(n) for n, i in baseline_ids.items())
        need(identities_preserved,
             "an unrelated container identity or start time changed")
        marker = volume_marker() if volume_name else (root / "data" / "retained.txt").read_text(encoding="utf-8")
        data_preserved = data_preserved is not False and marker == "private extension test data\n"
        need(data_preserved,
             "private data marker changed or disappeared")
        phases.append({"phase": phase, "ok": True, **detail})

    def mutate(action, *action_args, expected_failure=False):
        if args.robot_readiness:
            ready()
        command = [str(binary), "--config", str(root / "host.json"), "--state-dir", str(root / "state"),
                   "--timeout", "90s", action, *action_args]
        result = run(command)
        if expected_failure and result.returncode:
            return result
        if result.returncode:
            raise Failure(f"extension action failed (exit {result.returncode})")
        if expected_failure:
            raise Failure("application-health failure unexpectedly succeeded")
        return result

    def status():
        output = checked([str(binary), "--config", str(root / "host.json"), "--state-dir", str(root / "state"),
                          "status"], "extension status").stdout
        try:
            entries = json.loads(output)
        except json.JSONDecodeError:
            raise Failure("extension status returned invalid JSON") from None
        return next((entry for entry in entries if entry.get("id") == extension_id), None)

    try:
        plat = {"x86_64": "linux/amd64", "amd64": "linux/amd64",
                "aarch64": "linux/arm64", "arm64": "linux/arm64"}.get(platform.machine().lower())
        need(platform.system() == "Linux" and plat is not None, "runner supports only Linux AMD64 or ARM64")
        root = Path(tempfile.mkdtemp(prefix="mowgli-extension-engine-"))
        root.chmod(0o700)
        for name in ("config", "data", "state"):
            (root / name).mkdir(mode=0o700)
        config_dir, data_dir = root / "config", root / "data"
        json_file(root / "host.json", {
            "schema": 1, "installation": project, "platform": plat, "contracts": [],
            "trusted_repositories": [args.source_repository], "approved_grants": ["storage:extension-data"],
            "external_units": [],
        })
        (data_dir / "retained.txt").write_text("private extension test data\n", encoding="utf-8")
        (data_dir / "retained.txt").chmod(0o600)
        image_plat = checked(DOCKER + ["image", "inspect", "--format", "{{.Os}}/{{.Architecture}}", args.image], "preloaded image check").stdout.strip()
        need(image_plat == plat, "preloaded image platform differs from this host")
        for args0 in (
            ["ps", "-a", "--filter", f"label=com.docker.compose.project={project}", "--format", "{{.ID}}"],
            ["network", "ls", "--filter", f"label=com.docker.compose.project={project}", "--format", "{{.Name}}"],
            ["volume", "ls", "--filter", f"label=com.docker.compose.project={project}", "--format", "{{.Name}}"],
        ):
            if checked(DOCKER + args0, "private project collision check").stdout.strip():
                raise Failure("random Compose project already has Docker resources")
        baseline_ids = containers()
        baseline_starts = started_at(baseline_ids)
        if volume_name:
            names = checked(DOCKER + ["volume", "ls", "--format", "{{.Name}}"], "volume collision check").stdout.splitlines()
            need(volume_name not in names, "random named volume already exists")
            checked(DOCKER + ["volume", "create", "--label", f"{test_label}={project}", volume_name], "private volume creation")
            volume_created = True
            cleanup_needed = True
            volume_marker(seed=True)
        phases.append({"phase": "preflight", "ok": True, "host_platform": plat,
                       "baseline_containers": len(baseline_ids), "image_preloaded": True, "storage": args.storage})

        v1 = bundle(extension_id, "1.0.0", "process-only", "3600", args.image, args.source_repository,
                    args.source_revision, project, data_dir, plat, volume_name)
        v2 = bundle(extension_id, "1.0.1", "process-only", "3601", args.image, args.source_repository,
                    args.source_revision, project, data_dir, plat, volume_name)
        v3 = bundle(extension_id, "1.0.3", "process-only", "3603", args.image, args.source_repository,
                    args.source_revision, project, data_dir, plat, volume_name)
        no_probe = bundle(extension_id, "1.0.2", "application", "3602", args.image, args.source_repository,
                          args.source_revision, project, data_dir, plat, volume_name)
        manual_file = root / "manual-compose.json"
        v1_file, v2_file, bad_file = root / "v1.bundle.json", root / "v2.bundle.json", root / "no-probe.bundle.json"
        json_file(manual_file, v1["compose"])
        json_file(v1_file, v1)
        json_file(v2_file, v2)
        v3_file = root / "v3.bundle.json"
        json_file(v3_file, v3)
        json_file(bad_file, no_probe)
        compose_cmd = DOCKER + ["compose", "--project-name", project, "--project-directory", str(root), "-f", str(manual_file)]

        if args.robot_readiness:
            ready()
        cleanup_needed = True
        checked(compose_cmd + ["up", "-d", "--no-build", "--pull", "never"], "private Compose start")
        record("manual_private_start")

        mutate("adopt", "--bundle", str(v1_file), "--project", project,
               "--config-dir", str(config_dir), "--data-dir", str(data_dir), extension_id)
        adopted = status()
        need(adopted and adopted["enabled"] and adopted["observation"]["process"] == "running",
             "exact adoption did not register the running private service")
        record("exact_adopt", version=adopted["version"])

        mutate("upgrade", "--bundle", str(v2_file), extension_id)
        upgraded = status()
        need(upgraded and upgraded["version"] == "1.0.1" and
             upgraded["observation"]["identity"] != adopted["observation"]["identity"],
             "upgrade did not activate a new runtime identity")
        record("upgrade", version=upgraded["version"], new_runtime_identity=True)

        mutate("rollback", extension_id)
        rolled = status()
        need(rolled and rolled["version"] == "1.0.0", "rollback did not restore the adopted release")
        record("rollback", version=rolled["version"])

        failed = mutate("upgrade", "--bundle", str(bad_file), extension_id, expected_failure=True)
        if "recovery completed" not in failed.stderr:
            raise Failure("failed application health did not report automatic recovery")
        recovered = status()
        need(recovered and recovered["version"] == "1.0.0" and recovered["observation"]["process"] == "running"
             and not recovered["recovery_required"], "failed application health did not restore the prior release")
        record("automatic_recovery", expected_rejection=True, restored_version=recovered["version"])

        mutate("disable", extension_id)
        need(status()["observation"]["process"] == "stopped", "disable did not stop the private service")
        record("disable")
        for version, path in (("1.0.1", v2_file), ("1.0.3", v3_file)):
            mutate("upgrade", "--bundle", str(path), extension_id)
            stopped = status()
            need(stopped and stopped["version"] == version and not stopped["enabled"]
                 and stopped["observation"]["process"] == "stopped", "disabled upgrade lost release identity")
            record("disabled_upgrade", version=version)
        mutate("rollback", extension_id)
        rolled = status()
        need(rolled and rolled["version"] == "1.0.1" and not rolled["enabled"]
             and rolled["observation"]["process"] == "stopped", "disabled rollback changed the stopped state")
        record("disabled_rollback")
        mutate("enable", extension_id)
        need(status()["observation"]["process"] == "running", "disabled upgrades stranded the runtime")
        record("enable_after_disabled_upgrades")
        mutate("disable", extension_id)
        record("disable_after_disabled_upgrades")
        mutate("remove", extension_id)
        need(status() is None, "remove left the extension registered")
        record("remove_retaining_data")

        mutate("install", "--bundle", str(v1_file), "--project", project,
               "--config-dir", str(config_dir), "--data-dir", str(data_dir), extension_id)
        installed = status()
        need(installed and not installed["enabled"], "install after remove did not retain a disabled registration")
        record("install_retained_data")
        mutate("enable", extension_id)
        need(status()["observation"]["process"] == "running", "enable did not start the retained installation")
        record("enable")
        mutate("disable", extension_id)
        record("final_disable")
        mutate("remove", extension_id)
        need(status() is None, "final remove left the extension registered")
        record("final_remove")
    except Failure as exc:
        error = str(exc)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        error = f"unexpected runner failure ({type(exc).__name__})"
    finally:
        if root is not None and cleanup_needed:
            readiness_ok = None
            if args.robot_readiness:
                try:
                    ready()
                    readiness_ok = True
                except Failure:
                    readiness_ok = False
            try:
                cleanup_ok = compose_cmd is None or run(compose_cmd + ["down"]).returncode == 0
                helper = checked(DOCKER + ["ps", "-aq", "--filter", f"label={test_label}={project}"], "helper cleanup inventory").stdout.split()
                if helper:
                    checked(DOCKER + ["rm", "-f", *helper], "private helper cleanup")
            except Failure:
                cleanup_ok = False
            phases.append({"phase": "private_compose_cleanup", "ok": cleanup_ok, "readiness_ready": readiness_ok})
            if not cleanup_ok:
                error = error or "private Compose cleanup failed; temporary files retained"
            else:
                try:
                    record("post_cleanup_invariants")
                    for query in (["ps", "-aq"], ["network", "ls", "-q"]):
                        leftovers = checked(DOCKER + query + ["--filter", f"label=com.docker.compose.project={project}"],
                                            "private cleanup verification").stdout.strip()
                        need(not leftovers, "private project resources remain after cleanup")
                except Failure as exc:
                    cleanup_ok = False
                    error = error or str(exc)
            if volume_created and cleanup_ok and error is None:
                try:
                    owner = checked(DOCKER + ["volume", "inspect", "--format",
                                              '{{index .Labels "' + test_label + '"}}', volume_name], "private volume ownership").stdout.strip()
                    need(owner == project, "private volume ownership changed; refusing cleanup")
                    checked(DOCKER + ["volume", "rm", volume_name], "private volume cleanup")
                    phases.append({"phase": "private_volume_cleanup", "ok": True})
                except Failure as exc:
                    cleanup_ok = False
                    error = error or str(exc)
        elif root is not None:
            cleanup_ok = True

    artifacts, root_path = "not-created", None
    if root is not None:
        if args.keep_files or not cleanup_ok or error is not None:
            artifacts, root_path = "retained", str(root)
        else:
            try:
                shutil.rmtree(root)
                artifacts = "removed"
            except OSError:
                artifacts, root_path = "partially-retained", str(root)
                error = error or "private temporary files could not be fully removed"
    report = {
        "success": error is None, "project": project, "storage": args.storage, "phases": phases, "artifacts": artifacts,
        "invariants": {"unrelated_container_ids_and_start_times_preserved": identities_preserved,
                       "private_data_marker_retained": data_preserved, "cleanup_succeeded": cleanup_ok},
    }
    if root_path:
        report["temporary_root"] = root_path
    if error:
        report["error"] = error
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if error is None else 1


if __name__ == "__main__":
    sys.exit(main())

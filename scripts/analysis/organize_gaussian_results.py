#!/usr/bin/env python3
"""Organize the completed 2026-11-23 Gaussian experiments; preview unless --apply."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil


def files_under(folder):
    if not folder.is_dir() or folder.is_symlink():
        return
    for parent, directories, files in os.walk(folder, followlinks=False):
        directories[:] = [name for name in directories
                          if not (Path(parent) / name).is_symlink()]
        for name in files:
            path = Path(parent) / name
            if path.is_file() and not path.is_symlink():
                yield path


def disk_summary(paths):
    unique = {}
    apparent = 0
    for path in paths:
        stat = path.stat()
        apparent += stat.st_size
        unique[(stat.st_dev, stat.st_ino)] = stat.st_blocks * 512
    return dict(files=len(paths), apparent_bytes=apparent,
                allocated_unique_inode_bytes=sum(unique.values()))


def rewrite_json(value, replacements):
    if isinstance(value, str):
        for old, new in replacements:
            value = value.replace(old, new)
        return value
    if isinstance(value, list):
        return [rewrite_json(item, replacements) for item in value]
    if isinstance(value, dict):
        updated = {}
        for key, item in value.items():
            new_key = rewrite_json(key, replacements)
            if new_key in updated:
                raise ValueError(f"Path replacement would duplicate JSON key: {new_key}")
            updated[new_key] = rewrite_json(item, replacements)
        return updated
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True,
                        help="Host project containing build/gs_work, e.g. /home/lhx/docker/MY-LIVO")
    parser.add_argument("--workspace-root", type=Path, required=True,
                        help="Current authoritative source worktree")
    parser.add_argument("--apply", action="store_true", help="Apply the printed organization and pruning")
    args = parser.parse_args()
    project = args.project_root.resolve()
    workspace = args.workspace_root.resolve()
    old_work = project / "build/gs_work"
    destination = project / "results/gaussian/experiments_20261123"
    if not old_work.is_dir() or old_work.is_symlink():
        parser.error(f"Expected the original real experiment directory: {old_work}")
    if destination.exists() or destination.is_symlink():
        parser.error(f"Destination already exists; this is a one-time migration: {destination}")
    if not (workspace / "AGENTS.md").is_file():
        parser.error(f"Expected the source worktree with AGENTS.md: {workspace}")

    deletes = []
    reasons = {}
    retained = []
    model_files = ("point_cloud.ply", "state.npy", "bounds.npy", "gaussian_centers.pcd")
    retired = [
        "depth_ab_20261123/supported120",
        "depth_ab_20261123/supported1280_long",
        "depth_ab_20261123/depth120",
        "depth_ab_20261123/depth60",
        "depth_ab_20261123/multires",
        "fill_ab_20261123/control",
        "fill_ab_20261123/weak_rgb",
        "fill_ab_20261123/weak_rgb_plane",
    ]
    for name in retired:
        for filename in model_files:
            path = old_work / name / filename
            if path.is_file() and not path.is_symlink():
                deletes.append(path)
                reasons[path] = "Unselected completed comparison; saved renders, metrics and logs are retained"
    for name in ("v00_base", "v01_far", "v02_far_plane"):
        for filename in ("state.npy", "bounds.npy", "gaussian_centers.pcd"):
            path = old_work / name / filename
            if path.is_file() and not path.is_symlink():
                deletes.append(path)
                reasons[path] = "Historical PLY/render baseline retained; optimizer checkpoint is no longer needed"
        retained.append(str(old_work / name / "point_cloud.ply"))
    for name in ("fill_ab_20261123/plane_only", "depth_ab_20261123/supported640_long"):
        for filename in model_files:
            path = old_work / name / filename
            if not path.is_file():
                parser.error(f"Required accepted or resumable product missing: {path}")
            retained.append(str(path))

    prepared = old_work / "depth_ab_20261123/prep1280"
    pruned_training = []
    heldout = []
    if prepared.is_dir():
        for manifest in sorted((prepared / "input").glob("submap_*/frames.txt")):
            for line in manifest.read_text().splitlines()[1:]:
                fields = line.split()
                if len(fields) < 3 or not re.fullmatch(r"frame_[0-9]+", fields[0]):
                    parser.error(f"Unexpected prepared frame record: {manifest}")
                name, validation = fields[0], int(fields[1])
                if validation:
                    heldout.append(name)
                    for suffix in (".png", ".bin", "_sky.png", "_dynamic.png"):
                        path = manifest.parent / (name + suffix)
                        if not path.is_file():
                            parser.error(f"Required 720p held-out evaluation input missing: {path}")
                    continue
                pruned_training.append(name)
                for suffix in (".png", ".bin", "_sky.png", "_dynamic.png"):
                    path = manifest.parent / (name + suffix)
                    if path.is_file() and not path.is_symlink():
                        deletes.append(path)
                        reasons[path] = "720p training input is reproducible; all held-out evaluation inputs are retained"

    moves = [(old_work, destination)]
    compiler_destination = project / "build/gaussian/experiments_20261123"
    for variant in ("binary", "binary_final", "binary_accepted"):
        directory = old_work / "fill_ab_20261123" / variant
        if not directory.is_dir():
            continue
        if variant == "binary_accepted":
            executable = directory / "my_livo_gaussian"
            if not executable.is_file():
                parser.error(f"Required final executable missing: {executable}")
            moves.insert(0, (executable, compiler_destination / variant / executable.name))
            retained.append(str(executable))
        for path in directory.iterdir():
            if path.is_file() and (path.suffix == ".o" or
                                  variant != "binary_accepted" and path.name == "my_livo_gaussian"):
                deletes.append(path)
                reasons[path] = "Rebuildable object or unselected historical experimental executable"

    preview = project / "build/gs_tmp"
    if preview.is_dir() and not preview.is_symlink():
        moves.append((preview, destination / "preview_history"))
    delivery = project / "build/gs_delivery/st"
    prototypes = destination / "browser_prototypes"
    for name in ("out", "logs"):
        path = delivery / name
        if path.is_dir() and not path.is_symlink():
            moves.append((path, prototypes / name))
    temporary = delivery / "tmp"
    if temporary.is_dir() and not temporary.is_symlink():
        for path in files_under(temporary):
            deletes.append(path)
            reasons[path] = "Reproducible browser conversion intermediate; final SOG products and performance logs retained"

    adapter = project / "build/gaussian"
    legacy = destination / "legacy_adapter"
    for name in ("forward_cache_gradient_input", "forward_cache_gradient_output"):
        path = adapter / name
        if path.is_dir() and not path.is_symlink():
            moves.append((path, legacy / name))
    for path in sorted(adapter.iterdir()):
        if not path.is_file() or path.suffix not in (".log", ".csv"):
            continue
        if "build" in path.stem or path.name == "semantic_setup.log":
            continue
        moves.append((path, legacy / "logs" / path.name))

    # Specific compiler locations must be replaced before the enclosing gs_work prefix.
    path_moves = list(moves)
    for variant in ("binary", "binary_final", "binary_accepted"):
        path_moves.append((old_work / "fill_ab_20261123" / variant,
                           compiler_destination / variant))
    replacements = []
    for old, new in path_moves:
        replacements.append((str(old), str(new)))
        relative_old, relative_new = old.relative_to(project), new.relative_to(project)
        replacements.append((str(Path("/home/project/MY-LIVO") / relative_old),
                             str(Path("/home/project/MY-LIVO") / relative_new)))
    replacements = sorted(dict(replacements).items(), key=lambda item: len(item[0]), reverse=True)
    deletes = list(dict.fromkeys(deletes))
    for path in deletes:
        if not path.is_file() or path.is_symlink():
            parser.error(f"Deletion candidate changed or is not a regular file: {path}")
        if not (path.is_relative_to(old_work) or path.is_relative_to(temporary)):
            parser.error(f"Deletion escaped the explicitly selected experiment scope: {path}")
    for old, new in moves:
        if new.exists() or new.is_symlink():
            parser.error(f"Refusing to overwrite migration destination: {new}")
        if old.stat().st_dev != project.stat().st_dev:
            parser.error(f"Expected same-filesystem rename, not copy: {old}")

    stats = {path: path.stat() for path in deletes}
    removed_links = Counter((stat.st_dev, stat.st_ino) for stat in stats.values())
    released_inodes = {}
    for stat in stats.values():
        key = (stat.st_dev, stat.st_ino)
        if removed_links[key] >= stat.st_nlink:
            released_inodes[key] = stat.st_blocks * 512
    before_files = list(files_under(old_work)) + list(files_under(preview))
    before_files += list(files_under(delivery / "out")) + list(files_under(delivery / "logs"))
    before_files += list(files_under(temporary))
    for old, _ in moves:
        if old.is_file() and not old.is_symlink():
            before_files.append(old)
        elif old not in (old_work, preview, delivery / "out", delivery / "logs"):
            before_files += list(files_under(old))
    before = disk_summary(list(dict.fromkeys(before_files)))
    deletion_summary = dict(files=len(deletes),
                            apparent_bytes=sum(stat.st_size for stat in stats.values()),
                            estimated_released_inode_blocks_bytes=sum(released_inodes.values()),
                            links_with_external_owners=sum(
                                1 for stat in stats.values()
                                if removed_links[(stat.st_dev, stat.st_ino)] < stat.st_nlink))
    # Parse and serialize affected metadata before deleting or moving anything.
    # Logs and provenance source scripts are left verbatim.
    json_payloads = {}
    link_updates = {}
    dead_legacy_links = {}
    scan_roots = [old_work, project / "results", workspace / "results"]
    scan_roots += [old for old, _ in moves if old.is_dir() and old != old_work]
    for scan_root in scan_roots:
        if not scan_root.is_dir() or scan_root.is_symlink():
            continue
        for parent, directories, files in os.walk(scan_root, followlinks=False):
            for name in list(directories) + files:
                path = Path(parent) / name
                if not path.is_symlink():
                    continue
                target = os.readlink(path)
                old_input_prefix = "/home/project/MY-LIVO/results/xingye13_no_native_pcd_1108/gaussian/input/"
                if (path.is_relative_to(adapter / "forward_cache_gradient_input") and
                        target.startswith(old_input_prefix)):
                    host_target = project / Path(target).relative_to("/home/project/MY-LIVO")
                    if not host_target.exists():
                        stat = path.lstat()
                        dead_legacy_links[path] = dict(
                            original_target=target, allocated_bytes=stat.st_blocks * 512,
                            reason="Pre-existing dead link to absent 2026-11-08 input; old output and logs retained")
                        continue
                updated = rewrite_json(target, replacements)
                if updated != target:
                    link_updates[path] = (target, updated)
            directories[:] = [name for name in directories
                              if not (Path(parent) / name).is_symlink()]
            for name in files:
                path = Path(parent) / name
                if path.suffix != ".json" or path.is_symlink():
                    continue
                text = path.read_text()
                if not any(old in text for old, _ in replacements):
                    continue
                data = json.loads(text)
                updated = rewrite_json(data, replacements)
                if updated != data:
                    json_payloads[path] = json.dumps(
                        updated, indent=2, ensure_ascii=False, allow_nan=False) + "\n"

    preview_report = dict(mode="apply" if args.apply else "dry-run",
                          project_root=str(project), workspace_root=str(workspace),
                          destination=str(destination), before=before,
                          pruning=deletion_summary, retained_full_models=[
                              "fill_ab_20261123/plane_only",
                              "depth_ab_20261123/supported640_long"],
                          retained_ply_baselines=["v00_base", "v01_far", "v02_far_plane"],
                          retained_720p_heldout_frames=len(heldout),
                          removed_720p_training_frames=len(pruned_training),
                          operational_json_updates=len(json_payloads),
                          soft_link_updates=len(link_updates),
                          preexisting_dead_legacy_links=len(dead_legacy_links),
                          moves=[dict(source=str(old), destination=str(new)) for old, new in moves])
    print(json.dumps(preview_report, indent=2, ensure_ascii=False), flush=True)
    if not args.apply:
        return

    free_before = shutil.disk_usage(project).free
    removed = []
    for path in deletes:
        stat = stats[path]
        removed.append(dict(original_path=str(path),
                            current_parent=rewrite_json(str(path.parent), replacements),
                            filename=path.name, apparent_bytes=stat.st_size,
                            allocated_bytes=stat.st_blocks * 512, inode=stat.st_ino,
                            device=stat.st_dev, original_link_count=stat.st_nlink,
                            reason=reasons[path]))
        path.unlink()
    dead_links_removed = []
    for path, information in dead_legacy_links.items():
        dead_links_removed.append(dict(path=rewrite_json(str(path), replacements), **information))
        path.unlink()
    for old, new in moves:
        new.parent.mkdir(parents=True, exist_ok=True)
        old.rename(new)
    for variant in ("binary", "binary_final", "binary_accepted"):
        directory = destination / "fill_ab_20261123" / variant
        if directory.is_dir() and not any(directory.iterdir()):
            directory.rmdir()
    if temporary.is_dir() and not any(temporary.iterdir()):
        temporary.rmdir()

    json_changes = []
    links_changed = []
    for original, (target, updated) in link_updates.items():
        path = Path(rewrite_json(str(original), replacements))
        path.unlink()
        path.symlink_to(updated)
        links_changed.append(dict(path=str(path), before=target, after=updated))
    for original, text in json_payloads.items():
        path = Path(rewrite_json(str(original), replacements))
        temporary_json = path.with_name(path.name + ".organization_tmp")
        temporary_json.write_text(text)
        temporary_json.replace(path)
        json_changes.append(str(path))

    # This old local runner only uses SC for outputs; keep its tool dependencies in build.
    runner = delivery / "scripts/run_measured.sh"
    if runner.is_file():
        text = runner.read_text()
        old_assignment = "SC=" + str(delivery)
        if old_assignment in text:
            runner.write_text(text.replace(old_assignment, "SC=" + str(prototypes)))

    if prepared.is_dir():
        raise RuntimeError("Old 720p preparation unexpectedly survived the directory rename")
    pruned_preparation = destination / "depth_ab_20261123/prep1280"
    if pruned_preparation.is_dir():
        (pruned_preparation / "input_retention.json").write_text(json.dumps(dict(
            purpose="Preserve the original 30 held-out 1280x720 inputs for existing evaluation reports",
            training_available=False, heldout_frames=heldout,
            removed_training_frames=pruned_training,
            manifests_and_poses="Retained verbatim as the original reference frame lists",
            regenerate_training="Decode again from the recorded dataset using preparation.json parameters"),
            indent=2, ensure_ascii=False) + "\n")
    for name in retired:
        folder = destination / name
        if folder.is_dir():
            (folder / "retention.json").write_text(json.dumps(dict(
                product_status="Unselected comparison, geometry and optimizer checkpoint pruned",
                saved_renders_and_metrics=True, resumable=False,
                removed_files=list(model_files)), indent=2) + "\n")
    for name in ("v00_base", "v01_far", "v02_far_plane"):
        (destination / name / "retention.json").write_text(json.dumps(dict(
            product_status="Historical PLY and saved-render baseline retained",
            saved_ply=True, saved_renders_and_metrics=True, resumable=False,
            removed_files=["state.npy", "bounds.npy", "gaussian_centers.pcd"]), indent=2) + "\n")
    if dead_links_removed:
        (legacy / "input_retention.json").write_text(json.dumps(dict(
            missing_historical_source="xingye13_no_native_pcd_1108/gaussian/input",
            removed_dead_links=dead_links_removed, saved_render_output_and_logs=True), indent=2) + "\n")

    (destination / "README.md").write_text("""# 星业13高斯实验 · 2026-11-23

训练结果和评估资料已从 build 迁到此目录，原实验名称保留。

| 目录 | 保留内容与用途 |
|---|---|
| `fill_ab_20261123/plane_only/` | 采用的显式平面尺度版本，完整 PLY、中心 PCD、状态、边界、日志与30张留出图，可继续训练 |
| `depth_ab_20261123/supported640_long/` | 640长训起始基线，完整产品与状态，可继续训练 |
| `v00_base/`、`v01_far/`、`v02_far_plane/` | 历史 PLY 与留出图对照，已移除不再使用的优化器状态 |
| `prep640/` | 240帧完整640备料，现有静态评估可复算 |
| `depth_ab_20261123/prep1280/` | 30张原留出RGB、深度与掩码；210训练帧大文件已移除，原帧清单和位姿保留 |
| 其余 depth/fill 试验 | 保留日志、参数、逐视角指标与保存的留出图；未采用模型的大文件已精简，不能直接继续训练 |
| `image_geometry_audit_20261123/` | 固定相机的远山三角化与不确定性分析 |
| `g4_views/` | 指向此前完整地图留出图的比较入口 |
| `preview_history/` | 原路线分析、预览图与中间报告 |
| `browser_prototypes/` | 浏览转换最终SOG与性能记录，临时转换产物已移除 |
| `legacy_adapter/` | 历史真实运行与重渲染验证资料 |

最终可执行文件在 `build/gaussian/experiments_20261123/binary_accepted/`，
构建源码、第三方依赖、原扫描/位姿/标定及既有正式结果未移动或删除。
`organization.json` 记录每项移动与精简理由；原训练/构建日志保留历史路径，
操作用JSON和软链接已更新为当前路径。未来实验直接写入 results，
不再将训练模型、报告和预览放在 build。
旧的2026-11-08输入已不存在，历史验证目录的失效链接已清除；
原目标清单在 `legacy_adapter/input_retention.json`，旧输出和日志保留。
""")
    after_files = list(files_under(destination))
    after_files += list(files_under(compiler_destination))
    report = dict(preview_report, completed_at=datetime.now(timezone.utc).isoformat(),
                  after=disk_summary(after_files), removed_files=removed,
                  changed_json_files=json_changes, changed_links=links_changed,
                  removed_preexisting_broken_links=dead_links_removed,
                  path_replacements=[dict(before=old, after=new) for old, new in replacements],
                  disk_free_before_bytes=free_before,
                  disk_free_after_bytes=shutil.disk_usage(project).free)
    report["observed_disk_free_change_bytes"] = report["disk_free_after_bytes"] - free_before
    report["disk_measurement_note"] = (
        "Allocated byte estimates use inode blocks and link counts; free-space change may include concurrent activity. "
        "No hashes are calculated.")
    (destination / "organization.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps(dict(status="organized", destination=str(destination),
                          pruning=deletion_summary, changed_json_files=len(json_changes),
                          changed_links=len(links_changed), after=report["after"]),
                     indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

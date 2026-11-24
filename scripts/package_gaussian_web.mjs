import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { parseArgs } from 'node:util';

const { values, positionals } = parseArgs({ allowPositionals: true, options: {
    gpu: { type: 'string' }, 'refresh-viewer': { type: 'boolean' }, help: { type: 'boolean', short: 'h' }
} });
if (values.help || positionals.length !== 2) {
    console.log('Usage: node scripts/package_gaussian_web.mjs [--gpu INDEX | --refresh-viewer] point_cloud.ply WEB_DIRECTORY');
    process.exit(values.help ? 0 : 2);
}

const [model, output] = positionals.map(name => path.resolve(name));
const runtime = fileURLToPath(new URL('../build/gaussian/web/node_modules/', import.meta.url));
const cli = path.join(runtime, '.bin/splat-transform');
const viewer = path.join(runtime, '@playcanvas/supersplat-viewer');
if (!fs.existsSync(cli) || !fs.existsSync(viewer)) {
    throw Error('Install web tools: npm install --prefix build/gaussian/web @playcanvas/splat-transform@3.12.0 @playcanvas/supersplat-viewer@1.43.2');
}
if (!fs.statSync(model).isFile()) throw Error(`Missing source model: ${model}`);
if (!values['refresh-viewer'] && fs.existsSync(output)) throw Error(`Output already exists: ${output}`);
const source = path.dirname(model);
const readJson = filename => JSON.parse(fs.readFileSync(filename, 'utf8'));
const poses = readJson(path.join(source, 'camera_poses.json'));
const info = readJson(path.join(source, 'map_info.json'));
const { defaultSettings, validateSettings } = await import(pathToFileURL(path.join(viewer, 'dist/settings.js')));
const settings = defaultSettings('environment');
const transform = ([east, north, up]) => [east, up, -north];
const { width, height, fx, fy } = info.intrinsics ?? {};
if (![width, height, fx, fy].every(value => Number.isFinite(value) && value > 0)) throw Error('Invalid map intrinsics');
const horizontalFov = 360 / Math.PI * Math.atan(width / (2 * fx));
const verticalFov = 360 / Math.PI * Math.atan(height / (2 * fy));
const views = Object.create(null);
for (const {frame, T_world_camera: camera} of poses) {
    if (!/^frame_\d+$/.test(frame) || views[frame] || camera?.length !== 4 ||
        !camera.every(row => row.length === 4 && row.every(Number.isFinite))) throw Error(`Invalid recorded pose: ${frame}`);
    views[frame] = {
        position: transform(camera.slice(0, 3).map(row => row[3])),
        target: transform(camera.slice(0, 3).map(row => row[3] + 15 * row[2]))
    };
}
if (!poses.length) throw Error('No recorded camera poses');
settings.cameras = [{ initial: { ...views[poses[0].frame], fov: verticalFov } }];
settings.background.color = info.background_rgb ?? settings.background.color;
validateSettings(settings);

function publishViewer(site) {
    const lod = readJson(path.join(site, 'lod-meta.json'));
    if (lod.lodLevels !== 4 || lod.counts.length !== 4 || lod.counts[0] !== info.gaussians ||
        !lod.counts.every(count => Number.isSafeInteger(count) && count > 0) || !lod.filenames.length) {
        throw Error('LOD manifest does not match the source map');
    }
    for (const name of lod.filenames) {
        const chunk = path.join(site, name);
        const meta = readJson(chunk);
        for (const channel of ['means', 'scales', 'quats', 'sh0', ...(meta.shN ? ['shN'] : [])]) {
            for (const file of meta[channel].files) {
                if (!fs.statSync(path.join(path.dirname(chunk), file)).size) throw Error(`Empty chunk texture: ${name}/${file}`);
            }
        }
    }
    fs.writeFileSync(path.join(site, 'map-view.json'), JSON.stringify({ settings, horizontalFov }));
    fs.writeFileSync(path.join(site, 'views.json'), JSON.stringify(views));
    for (const name of ['index.js', 'index.css']) fs.copyFileSync(path.join(viewer, 'public', name), path.join(site, name));
    for (const name of ['LICENSE', 'THIRD_PARTY_NOTICES.md']) fs.copyFileSync(path.join(viewer, name), path.join(site, name));
    fs.copyFileSync(new URL('./gaussian_web.html', import.meta.url), path.join(site, 'index.html'));
    console.log(`Web map: ${output} (${lod.counts.join(' / ')} splats; ${lod.filenames.length} stream files)`);
}

if (values['refresh-viewer']) {
    publishViewer(output);
    process.exit(0);
}
const flags = values.gpu === undefined ? [] : ['--gpu', values.gpu];
const convert = args => execFileSync(cli, ['--no-tty', '--max-workers', '2', ...flags, ...args], { stdio: 'inherit' });
fs.mkdirSync(path.dirname(output), { recursive: true });
const work = fs.mkdtempSync(path.join(path.dirname(output), '.gaussian-web-'));
try {
    const site = path.join(work, 'site');
    fs.mkdirSync(site);
    const levels = [model];
    for (let level = 1; level <= 3; level++) {
        const reduced = path.join(work, `level${level}.ply`);
        convert(['--scratch-dir', work, levels.at(-1), '--decimate', '25%', reduced]);
        levels.push(reduced);
    }
    const manifest = path.join(site, 'lod-meta.json');
    convert(['--lod-chunk-count', '128', '--lod-file-count', '128',
        ...levels.flatMap((file, index) => [file, '--tag-lod', String(index)]),
        manifest, '--rotate', '0,0,180', '--rotate', '-90,0,0']);
    publishViewer(site);
    fs.renameSync(site, output);
} finally {
    fs.rmSync(work, { recursive: true, force: true });
}

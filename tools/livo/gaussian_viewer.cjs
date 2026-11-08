#!/usr/bin/env node
// Local-file viewing only: use SuperSplat's importer, never its publishing API.
const fs = require('node:fs');
const path = require('node:path');
const { chromium } = require(process.env.GAUSSIAN_PLAYWRIGHT);
const repo = path.resolve(__dirname, '../..');
const runtime = path.join(repo, 'build/gaussian/viewer');
const uid = process.getuid?.() ?? 1000;
const profile = path.join(runtime, `profile-${uid}`);
const port = Number(process.env.GAUSSIAN_VIEWER_PORT || (9327 + uid % 20000));
const endpoint = `http://127.0.0.1:${port}`;
let cleanup;

function findMap(input) {
    const p = path.resolve(input);
    if (fs.statSync(p).isFile()) return p;
    const projects = fs.readdirSync(p).filter(n => n.endsWith('.ssproj')).sort();
    if (projects.length) return path.join(p, projects[0]);
    if (fs.existsSync(path.join(p, 'point_cloud.ply'))) return path.join(p, 'point_cloud.ply');
    for (const folder of ['analysis', 'full_gaussian_analysis']) {
        const preview = path.join(p, folder, 'preview_regions/start/point_cloud.ply');
        if (fs.existsSync(preview)) return preview;
    }
    const fusion = path.join(p, 'Log/fusion');
    if (fs.existsSync(fusion)) {
        for (const n of fs.readdirSync(fusion).sort().reverse()) {
            const ply = path.join(fusion, n, 'gaussian/point_cloud.ply');
            if (fs.existsSync(ply)) return ply;
        }
    }
    throw Error(`No Gaussian PLY or SuperSplat project in ${p}`);
}

(async () => {
    const arg = process.argv[2];
    if (!arg || ['-h', '--help'].includes(arg)) {
        console.log('Usage: bash scripts/view_gaussian.sh <round-directory | point_cloud.ply | scene.ssproj>');
        return;
    }
    const file = findMap(arg);
    if (!/\.(ply|ssproj)$/i.test(file)) throw Error('Choose a Gaussian .ply or .ssproj file.');
    fs.mkdirSync(runtime, { recursive: true });
    let browser, context, ownsBrowser = false;
    // Reuse this launcher's own instance. Command-line GPU flags do not apply to an existing normal Chrome.
    try {
        browser = await chromium.connectOverCDP(endpoint, { timeout: 2000 });
    } catch { /* launch below */ }
    if (browser) {
        cleanup = () => browser.close();
        const cdp = await browser.newBrowserCDPSession();
        const { arguments: args } = await cdp.send('Browser.getBrowserCommandLine');
        if (!args.includes(`--user-data-dir=${profile}`)) throw Error(`Port ${port} belongs to another browser; set GAUSSIAN_VIEWER_PORT.`);
        context = browser.contexts()[0];
    } else {
        const env = { ...process.env };
        const args = ['--enable-automation', '--disable-dev-shm-usage', `--remote-debugging-port=${port}`, '--enable-unsafe-webgpu', '--ignore-gpu-blocklist'];
        if (uid === 0) args.push('--no-sandbox');
        const nvidia = '/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0';
        if (process.platform === 'linux' && fs.existsSync(nvidia)) {
            const icd = path.join(runtime, 'nvidia_icd.json');
            if (!fs.existsSync(icd)) fs.writeFileSync(icd, JSON.stringify({ file_format_version: '1.0.0', ICD: { library_path: nvidia, api_version: '1.3.194' } }));
            env.VK_ICD_FILENAMES = icd; env.VK_DRIVER_FILES = icd;
            args.push('--use-angle=vulkan', '--enable-features=Vulkan,VulkanFromANGLE,DefaultANGLEVulkan', '--disable-vulkan-surface');
        }
        context = await chromium.launchPersistentContext(profile, { headless: false, viewport: { width: 1440, height: 960 }, env, args });
        cleanup = () => context.close();
        ownsBrowser = true;
    }
    const page = ownsBrowser ? context.pages()[0] : await context.newPage();
    await page.goto('https://superspl.at/editor?lng=zh-CN', { waitUntil: 'domcontentloaded', timeout: 60000 });
    const iframe = await page.waitForSelector('iframe', { timeout: 60000 });
    const frame = await iframe.contentFrame();
    await frame.waitForFunction(() => !!window.scene, null, { timeout: 60000 });
    await frame.evaluate(() => {
        const input = document.createElement('input'); input.type = 'file'; input.id = 'my-livo-map'; input.hidden = true; document.body.append(input);
    });
    // CDP attaches the actual local file without serializing >50 MB through Playwright's remote upload path.
    const cdp = await context.newCDPSession(page);
    const element = await cdp.send('Runtime.evaluate', { expression: "document.querySelector('iframe').contentDocument.querySelector('#my-livo-map')" });
    await cdp.send('DOM.setFileInputFiles', { objectId: element.result.objectId, files: [file] });
    await frame.evaluate(async project => {
        const f = document.getElementById('my-livo-map').files[0];
        if (project) await window.scene.events.invoke('doc.load', f);
        else await window.scene.events.invoke('import', [{ filename: f.name, contents: f }], false, { throwOnError: true });
    }, file.toLowerCase().endsWith('.ssproj'));
    if (file.toLowerCase().endsWith('.ply')) {
        const poses = path.join(path.dirname(file), 'camera_poses.json');
        const pose = fs.existsSync(poses) ? JSON.parse(fs.readFileSync(poses, 'utf8')) : [];
        const infoPath = path.join(path.dirname(file), 'map_info.json');
        const info = fs.existsSync(infoPath) ? JSON.parse(fs.readFileSync(infoPath, 'utf8')) : {};
        await frame.evaluate(({ T, background, intrinsics }) => {
            const s = window.scene, model = s.elements.find(e => e.numSplats > 0);
            if (!model) throw Error('No Gaussian layer was imported.');
            // MY-LIVO is Z-up; browser navigation is Y-up. This affects the display transform only.
            model.move(undefined, model.entity.getLocalRotation().clone().set(-Math.SQRT1_2, 0, 0, Math.SQRT1_2));
            if (T) {
                const p = s.camera.mainCamera.getPosition().clone().set(T[0][3], T[2][3], -T[1][3]);
                const target = p.clone().set(p.x + 20*T[0][2], p.y + 20*T[2][2], p.z - 20*T[1][2]);
                const horizontal = s.camera.mainCamera.camera.horizontalFov;
                const aspect = s.camera.mainCamera.camera.aspectRatio || s.canvas.width / s.canvas.height;
                const tanH = intrinsics ? intrinsics.width / (2 * intrinsics.fx) : 1;
                const tanV = intrinsics ? intrinsics.height / (2 * intrinsics.fy) : 1;
                const halfFov = horizontal ? Math.min(tanH, tanV * aspect) : Math.min(tanV, tanH / aspect);
                const fov = intrinsics ? 2 * Math.atan(halfFov) * 180 / Math.PI : 75;
                s.events.fire('camera.setFov', fov); s.camera.setPose(p, target, 0);
            } else s.camera.focus();
            if (Array.isArray(background) && background.length === 3 && background.every(v => Number.isFinite(v) && v >= 0 && v <= 1)) {
                s.events.fire('setBgClr', s.camera.mainCamera.camera.clearColor.clone().set(...background, 1));
            }
            s.events.fire('grid.setVisible', false); s.camera.onUpdate(0); s.forceRender = true;
        }, { T: pose.length ? pose[Math.floor(pose.length / 2)].T_world_camera : null, background: info.background_rgb, intrinsics: info.intrinsics });
    }
    const state = await frame.evaluate(async () => {
        const a = await navigator.gpu.requestAdapter();
        return { vendor: a?.info?.vendor, architecture: a?.info?.architecture, gaussians: window.scene.elements.reduce((n,e) => n + (e.numSplats || 0), 0) };
    });
    if (!state.gaussians) throw Error('The file did not produce a Gaussian layer.');
    await page.evaluate(name => { document.title = `MY-LIVO 高斯地图 · ${name}`; }, path.basename(file));
    await page.bringToFront();
    console.log(JSON.stringify({ file, ...state }, null, 2));
    console.log('Ready. Use this browser window to view maps; File > Open restores .ssproj camera settings.');
    if (ownsBrowser) await new Promise(resolve => context.on('close', resolve));
    else await browser.close(); // disconnect only; keep the existing viewer open
})().catch(async error => {
    console.error(`Gaussian viewer: ${error.message}`);
    if (cleanup) await cleanup().catch(() => {});
    process.exitCode = 1;
});

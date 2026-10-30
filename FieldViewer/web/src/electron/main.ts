import { app, BrowserWindow, shell } from 'electron';
import { spawn, type ChildProcess } from 'node:child_process';
import { appendFileSync, existsSync, mkdirSync } from 'node:fs';
import * as http from 'node:http';
import * as path from 'node:path';

// The window is the TypeScript UI; the simulation and robot link run in the Java backend
// (FieldViewer/src/main/java) so they use the exact same pure pursuit code as the robot.

const PORT = 5218;
const BACKEND_URL = `http://localhost:${PORT}`;

// dist/electron -> dist -> web -> FieldViewer -> repo root
const WEB_DIST = path.resolve(__dirname, '..');
const FIELD_VIEWER_DIR = path.resolve(__dirname, '..', '..', '..');
const REPO_ROOT = path.resolve(FIELD_VIEWER_DIR, '..');
const WINDOWS = process.platform === 'win32';

let backend: ChildProcess | null = null;
let window: BrowserWindow | null = null;
const backendLog: string[] = [];
let quitting = false;
let starting: Promise<void> | null = null;
let recentRestarts: number[] = [];

// everything the backend prints (and why it stopped) also goes here, for debugging
const LOG_FILE = path.join(FIELD_VIEWER_DIR, 'build', 'fieldviewer-backend.log');

function log(line: string): void {
  backendLog.push(line);
  if (backendLog.length > 40) backendLog.shift();
  console.log(`[backend] ${line}`);
  try {
    mkdirSync(path.dirname(LOG_FILE), { recursive: true });
    appendFileSync(LOG_FILE, `${new Date().toISOString()} ${line}\n`);
  } catch {
    // logging must never take the app down
  }
}

function ping(): Promise<boolean> {
  return new Promise((resolve) => {
    const request = http.get(`${BACKEND_URL}/api/health`, (response) => {
      response.resume();
      resolve(response.statusCode === 200);
    });
    request.on('error', () => resolve(false));
    request.setTimeout(1000, () => {
      request.destroy();
      resolve(false);
    });
  });
}

function run(command: string, args: string[], cwd: string): ChildProcess {
  const child = spawn(command, args, { cwd, shell: WINDOWS, env: process.env });
  const remember = (data: Buffer) => {
    for (const line of data.toString().split(/\r?\n/)) {
      if (line.trim()) {
        log(line);
      }
    }
  };
  child.stdout?.on('data', remember);
  child.stderr?.on('data', remember);
  return child;
}

function waitForExit(child: ChildProcess): Promise<number> {
  return new Promise((resolve) => child.on('exit', (code) => resolve(code ?? 1)));
}

async function waitForBackend(timeoutMs: number): Promise<boolean> {
  const start = Date.now();
  while (Date.now() - start < timeoutMs) {
    if (await ping()) return true;
    if (backend && backend.exitCode !== null) return false;
    await new Promise((r) => setTimeout(r, 300));
  }
  return false;
}

const SCRIPT = path.join(FIELD_VIEWER_DIR, 'build', 'install', 'FieldViewer', 'bin', WINDOWS ? 'FieldViewer.bat' : 'FieldViewer');

/**
 * Uses a backend that's already running, or builds the Java side with Gradle and starts it.
 * Safe to call again at any time; only one start runs at once.
 */
function ensureBackend(build: boolean, showProgress: boolean): Promise<void> {
  if (!starting) {
    starting = startBackend(build, showProgress).finally(() => {
      starting = null;
    });
  }
  return starting;
}

async function startBackend(build: boolean, showProgress: boolean): Promise<void> {
  if (await ping()) {
    return; // e.g. started with ./gradlew :FieldViewer:run
  }
  if (build || !existsSync(SCRIPT)) {
    if (showProgress) setStatus('Building the simulator…');
    const gradlew = WINDOWS ? 'gradlew.bat' : './gradlew';
    const gradle = run(gradlew, [':FieldViewer:installDist', '-q', '--console=plain'], REPO_ROOT);
    if ((await waitForExit(gradle)) !== 0) {
      throw new Error('Gradle build failed');
    }
  }

  if (showProgress) setStatus('Starting the simulator…');
  if (!existsSync(SCRIPT)) {
    throw new Error(`missing ${SCRIPT}`);
  }
  const child = run(SCRIPT, ['--no-browser', '--port', String(PORT), '--web-root', WEB_DIST], FIELD_VIEWER_DIR);
  backend = child;
  child.on('exit', (code, signal) => {
    if (!quitting) {
      log(`backend stopped unexpectedly (exit code ${code}, signal ${signal}); restarting`);
    }
  });
  if (!(await waitForBackend(60_000))) {
    throw new Error('the Java backend did not start');
  }
}

/** Keeps the backend alive: if it stops answering, start a new one (the page reconnects on its own). */
function superviseBackend(): void {
  setInterval(async () => {
    if (quitting || starting || await ping()) {
      return;
    }
    if (backend && backend.exitCode === null && backend.signalCode === null) {
      return; // ours is still alive, maybe just busy
    }
    const now = Date.now();
    recentRestarts = recentRestarts.filter((t) => now - t < 60_000);
    if (recentRestarts.length >= 5) {
      return; // something is badly wrong; the page shows the error, don't loop forever
    }
    recentRestarts.push(now);
    ensureBackend(false, false).catch((e) => log(`restart failed: ${(e as Error).message}`));
  }, 3000);
}

function setStatus(text: string): void {
  void window?.webContents.executeJavaScript(
    `document.getElementById('status') && (document.getElementById('status').textContent = ${JSON.stringify(text)})`);
}

async function createWindow(): Promise<void> {
  window = new BrowserWindow({
    width: 1480,
    height: 920,
    minWidth: 1000,
    minHeight: 640,
    title: 'FieldViewer',
    backgroundColor: '#07090c',
    titleBarStyle: process.platform === 'darwin' ? 'hiddenInset' : 'default',
    autoHideMenuBar: true,
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
    },
  });

  // links open in the real browser; the window itself only ever shows the local UI
  window.webContents.setWindowOpenHandler(({ url }) => {
    void shell.openExternal(url);
    return { action: 'deny' };
  });
  window.webContents.on('will-navigate', (event, url) => {
    if (!url.startsWith(BACKEND_URL) && !url.startsWith('file:')) {
      event.preventDefault();
    }
  });

  // if the page ever crashes, note why and bring it back instead of leaving a blank window
  window.webContents.on('render-process-gone', (_event, details) => {
    log(`window crashed (${details.reason}); reloading`);
    setTimeout(() => void window?.loadURL(BACKEND_URL), 500);
  });

  await window.loadFile(path.join(WEB_DIST, 'loading.html'));
  try {
    await ensureBackend(true, true);
    await window.loadURL(BACKEND_URL);
    superviseBackend();
  } catch (e) {
    const message = `${(e as Error).message}\n\n${backendLog.slice(-15).join('\n')}`;
    await window.loadFile(path.join(WEB_DIST, 'loading.html'), { query: { error: message } });
  }
}

function stopBackend(): void {
  quitting = true;
  if (backend && backend.exitCode === null) {
    if (WINDOWS && backend.pid) {
      spawn('taskkill', ['/pid', String(backend.pid), '/T', '/F']);
    } else {
      backend.kill();
    }
  }
}

app.whenReady().then(createWindow).catch((e) => console.error(e));
app.on('window-all-closed', () => app.quit());
app.on('before-quit', stopBackend);
process.on('exit', stopBackend);

const http = require("http");
const fs = require("fs");
const { execSync, spawn } = require("child_process");
const path = require("path");

let yaml = null;
try {
    yaml = require("js-yaml");
} catch (error) {
    yaml = null;
}

const BACKEND_PORT = Number.parseInt(process.env.CONSOLE_BACKEND_PORT || "3031", 10);
const CONSOLE_DIR = path.resolve(__dirname, "..");
const APP_JS_DIR = path.resolve(CONSOLE_DIR, "src");
const SRC_DIR = path.resolve(CONSOLE_DIR, "..");
const WS_DIR = path.resolve(SRC_DIR, "..");
const SERIAL_LOG_PATH = "/tmp/r2_console_serial_bridge.log";

const jsonResponse = (res, statusCode, payload) => {
    res.writeHead(statusCode, {
        "Content-Type": "application/json; charset=utf-8",
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET,POST,OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    });
    res.end(JSON.stringify(payload));
};

const sleepMs = (ms) => {
    const start = Date.now();
    while (Date.now() - start < ms) {
        // short blocking wait used only for shutdown confirmation loops.
    }
};

const findSerialBridgeProcesses = () => {
    try {
        const output = execSync("ps -eo pid=,pgid=,args=", {
            encoding: "utf8",
            stdio: ["ignore", "pipe", "ignore"],
        });

        const rows = output
            .split("\n")
            .map((row) => row.trim())
            .filter((row) => row.length > 0);

        return rows
            .map((row) => {
                const match = row.match(/^(\d+)\s+(\d+)\s+(.+)$/);
                if (!match) {
                    return null;
                }
                return {
                    pid: Number.parseInt(match[1], 10),
                    pgid: Number.parseInt(match[2], 10),
                    command: match[3],
                };
            })
            .filter((item) => {
                if (!item) {
                    return false;
                }
                if (!item.command.includes("serial_bridge_node")) {
                    return false;
                }
                if (item.command.includes("console_backend.js")) {
                    return false;
                }
                return Number.isInteger(item.pid) && item.pid > 0;
            })
            .sort((a, b) => b.pid - a.pid);
    } catch (error) {
        return [];
    }
};

const listPortsUsedByProcess = (pid) => {
    if (!Number.isInteger(pid) || pid <= 0) {
        return [];
    }

    const fdDirPath = `/proc/${pid}/fd`;
    let fdEntries = [];
    try {
        fdEntries = fs.readdirSync(fdDirPath);
    } catch (error) {
        return [];
    }

    const ports = new Set();
    fdEntries.forEach((fdName) => {
        try {
            const target = fs.readlinkSync(`${fdDirPath}/${fdName}`);
            if (/^\/dev\/tty(USB|ACM|AMA|S)\d+$/.test(target)) {
                ports.add(target);
            }
        } catch (error) {
            // FD close timing races can happen; ignore transient errors.
        }
    });

    return Array.from(ports).sort();
};

const getRunningInfo = () => {
    const processes = findSerialBridgeProcesses();
    if (processes.length === 0) {
        return { running: false, pid: null, command: "" };
    }

    const primary = processes[0];
    return {
        running: true,
        pid: primary.pid,
        command: primary.command,
    };
};

const startSerialBridge = () => {
    const runningInfo = getRunningInfo();
    if (runningInfo.running) {
        return {
            started: false,
            running: true,
            pid: runningInfo.pid,
            message: "serial_bridge は既に起動中です",
        };
    }

    const command = [
        "set -e",
        "source /opt/ros/jazzy/setup.bash",
        `[ -f '${WS_DIR}/install/setup.bash' ] && source '${WS_DIR}/install/setup.bash' || true`,
        "exec ros2 run serial_bridge serial_bridge_node",
    ].join("; ");

    const child = spawn("bash", ["-lc", command], {
        cwd: WS_DIR,
        detached: true,
        stdio: ["ignore", fs.openSync(SERIAL_LOG_PATH, "a"), fs.openSync(SERIAL_LOG_PATH, "a")],
    });
    child.unref();

    return {
        started: true,
        running: true,
        pid: child.pid,
        message: "serial_bridge を起動しました",
    };
};

const stopSerialBridge = () => {
    const processes = findSerialBridgeProcesses();
    if (processes.length === 0) {
        return {
            stopped: false,
            running: false,
            pid: null,
            message: "serial_bridge は起動していません",
        };
    }

    const pids = Array.from(new Set(processes.map((item) => item.pid)));
    const pgids = Array.from(new Set(processes.map((item) => item.pgid)));

    try {
        // Kill process groups only for detached leaders to avoid affecting terminal sessions.
        pgids.forEach((pgid) => {
            if (!Number.isInteger(pgid) || pgid <= 0) {
                return;
            }
            const hasGroupLeader = processes.some((item) => item.pid === pgid);
            if (!hasGroupLeader) {
                return;
            }
            try {
                process.kill(-pgid, "SIGTERM");
            } catch (error) {
                // Group may already be gone or inaccessible; individual pid kill follows.
            }
        });

        pids.forEach((pid) => {
            try {
                process.kill(pid, "SIGTERM");
            } catch (error) {
                // Process may have already exited.
            }
        });

        let remaining = findSerialBridgeProcesses();
        for (let i = 0; i < 30 && remaining.length > 0; i += 1) {
            sleepMs(100);
            remaining = findSerialBridgeProcesses();
        }

        if (remaining.length > 0) {
            const remainPids = Array.from(new Set(remaining.map((item) => item.pid)));
            const remainPgids = Array.from(new Set(remaining.map((item) => item.pgid)));

            remainPgids.forEach((pgid) => {
                if (!Number.isInteger(pgid) || pgid <= 0) {
                    return;
                }
                const hasGroupLeader = remaining.some((item) => item.pid === pgid);
                if (!hasGroupLeader) {
                    return;
                }
                try {
                    process.kill(-pgid, "SIGKILL");
                } catch (error) {
                    // Best effort.
                }
            });

            remainPids.forEach((pid) => {
                try {
                    process.kill(pid, "SIGKILL");
                } catch (error) {
                    // Best effort.
                }
            });

            sleepMs(300);
            remaining = findSerialBridgeProcesses();
        }

        if (remaining.length > 0) {
            const remainingPids = remaining.map((item) => item.pid).join(", ");
            return {
                stopped: false,
                running: true,
                pid: remaining[0]?.pid || null,
                message: `serial_bridge の停止に失敗しました (残存PID: ${remainingPids})`,
            };
        }

        return {
            stopped: true,
            running: false,
            pid: pids[0] || null,
            message: "serial_bridge を停止しました",
        };
    } catch (error) {
        return {
            stopped: false,
            running: true,
            pid: pids[0] || null,
            message: `serial_bridge の停止に失敗しました: ${error.message}`,
        };
    }
};

const readLogTail = (lineLimit = 200) => {
    let fileText = "";
    try {
        fileText = fs.readFileSync(SERIAL_LOG_PATH, "utf8");
    } catch (error) {
        if (error && error.code === "ENOENT") {
            return [];
        }
        throw error;
    }

    const lines = fileText.split(/\r?\n/);
    if (lines.length > 0 && lines[lines.length - 1] === "") {
        lines.pop();
    }
    return lines.slice(-lineLimit);
};

const forceShutdownBackend = () => {
    try {
        stopSerialBridge();
    } catch (error) {
        // Best effort: backend shutdown continues even if serial bridge stop fails.
    }

    setTimeout(() => {
        process.exit(0);
    }, 200);
};

const readJsonBody = (req) => new Promise((resolve, reject) => {
    const chunks = [];
    req.on("data", (chunk) => {
        chunks.push(chunk);
    });
    req.on("end", () => {
        try {
            const raw = Buffer.concat(chunks).toString("utf8");
            if (!raw) {
                resolve({});
                return;
            }
            resolve(JSON.parse(raw));
        } catch (error) {
            reject(error);
        }
    });
    req.on("error", (error) => reject(error));
});

const sanitizeSegment = (text, fallback) => {
    const cleaned = String(text || "")
        .trim()
        .toLowerCase()
        .replace(/[^a-z0-9_-]+/g, "-")
        .replace(/^-+|-+$/g, "");
    return cleaned || fallback;
};

const buildTimestampForFile = () => {
    const now = new Date();
    const yyyy = String(now.getFullYear());
    const mm = String(now.getMonth() + 1).padStart(2, "0");
    const dd = String(now.getDate()).padStart(2, "0");
    const hh = String(now.getHours()).padStart(2, "0");
    const min = String(now.getMinutes()).padStart(2, "0");
    const ss = String(now.getSeconds()).padStart(2, "0");
    const ms = String(now.getMilliseconds()).padStart(3, "0");
    return `${yyyy}${mm}${dd}_${hh}${min}${ss}_${ms}`;
};

const saveJsonExportToAppDir = ({ category, exportData }) => {
    const safeCategory = sanitizeSegment(category, "export");
    const now = new Date();
    const yyyy = String(now.getFullYear());
    const mm = String(now.getMonth() + 1).padStart(2, "0");
    const dateFolder = path.join(APP_JS_DIR, `${safeCategory}_exports`, `${yyyy}${mm}`);

    // Create date-based folder structure
    fs.mkdirSync(dateFolder, { recursive: true });

    const stamp = buildTimestampForFile();
    const fileName = `${safeCategory}_${stamp}.json`;
    const targetPath = path.join(dateFolder, fileName);
    const json = JSON.stringify(exportData, null, 2);
    fs.writeFileSync(targetPath, `${json}\n`, "utf8");
    return {
        fileName,
        filePath: targetPath,
        relativePath: path.relative(APP_JS_DIR, targetPath),
    };
};

const findLatestJsonExport = (category) => {
    const safeCategory = sanitizeSegment(category, "export");
    const categoryFolder = path.join(APP_JS_DIR, `${safeCategory}_exports`);

    if (!fs.existsSync(categoryFolder)) {
        return null;
    }

    let allFiles = [];
    try {
        const dateFolders = fs.readdirSync(categoryFolder).sort((a, b) => b.localeCompare(a));
        for (const dateFolder of dateFolders) {
            const dateDir = path.join(categoryFolder, dateFolder);
            if (!fs.statSync(dateDir).isDirectory()) continue;

            const files = fs.readdirSync(dateDir);
            for (const file of files) {
                if (file.endsWith(".json")) {
                    allFiles.push(path.join(dateDir, file));
                }
            }
            if (allFiles.length > 0) break; // Found files in the latest date folder
        }
    } catch (error) {
        return null;
    }

    allFiles.sort((a, b) => b.localeCompare(a));

    for (const filePath of allFiles) {
        try {
            const text = fs.readFileSync(filePath, "utf8");
            const parsed = JSON.parse(text);
            return {
                fileName: path.basename(filePath),
                filePath,
                data: parsed,
            };
        } catch (error) {
            // Ignore broken json and keep scanning older files.
        }
    }

    return null;
};

const listExportFilesByCategory = (category, extensions = [".json"]) => {
    const safeCategory = sanitizeSegment(category, "export");
    const categoryFolder = path.join(APP_JS_DIR, `${safeCategory}_exports`);
    if (!fs.existsSync(categoryFolder)) {
        return [];
    }

    const normalizedExtensions = new Set(
        extensions.map((ext) => String(ext || "").toLowerCase())
    );
    const files = [];

    const dateFolders = fs.readdirSync(categoryFolder).sort((a, b) => b.localeCompare(a));
    for (const dateFolder of dateFolders) {
        const dateDir = path.join(categoryFolder, dateFolder);
        if (!fs.statSync(dateDir).isDirectory()) {
            continue;
        }

        const fileNames = fs.readdirSync(dateDir).sort((a, b) => b.localeCompare(a));
        for (const fileName of fileNames) {
            const ext = path.extname(fileName).toLowerCase();
            if (!normalizedExtensions.has(ext)) {
                continue;
            }
            const filePath = path.join(dateDir, fileName);
            const stat = fs.statSync(filePath);
            files.push({
                filePath,
                fileName,
                ext,
                mtimeMs: stat.mtimeMs,
            });
        }
    }

    files.sort((a, b) => {
        if (b.mtimeMs !== a.mtimeMs) {
            return b.mtimeMs - a.mtimeMs;
        }
        return b.filePath.localeCompare(a.filePath);
    });
    return files;
};

const parsePlannerStateConfigFile = (filePath) => {
    const text = fs.readFileSync(filePath, "utf8");
    const ext = path.extname(filePath).toLowerCase();

    if (ext === ".json") {
        return JSON.parse(text);
    }

    if (ext === ".yaml" || ext === ".yml") {
        if (!yaml) {
            throw new Error("js-yaml is not installed");
        }
        const parsed = yaml.load(text);
        if (!parsed || typeof parsed !== "object") {
            throw new Error("yaml root must be an object");
        }
        return parsed;
    }

    throw new Error(`unsupported config extension: ${ext}`);
};

const findLatestPlannerStateConfig = (category) => {
    const files = listExportFilesByCategory(category, [".yaml", ".yml", ".json"]);
    for (const file of files) {
        try {
            const parsed = parsePlannerStateConfigFile(file.filePath);
            return {
                fileName: file.fileName,
                filePath: file.filePath,
                data: parsed,
                format: file.ext === ".json" ? "json" : "yaml",
            };
        } catch (error) {
            // Ignore broken files and continue scanning older files.
        }
    }
    return null;
};

const server = http.createServer((req, res) => {
    const url = new URL(req.url, `http://${req.headers.host}`);

    if (req.method === "OPTIONS") {
        jsonResponse(res, 200, { ok: true });
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/serial-bridge/status") {
        const runningInfo = getRunningInfo();
        const usedPorts = listPortsUsedByProcess(runningInfo.pid);
        jsonResponse(res, 200, {
            running: runningInfo.running,
            pid: runningInfo.pid,
            command: runningInfo.command,
            ports: usedPorts,
            logPath: SERIAL_LOG_PATH,
            workspace: WS_DIR,
        });
        return;
    }

    if (req.method === "POST" && url.pathname === "/api/serial-bridge/start") {
        const startInfo = startSerialBridge();
        jsonResponse(res, 200, startInfo);
        return;
    }

    if (req.method === "POST" && url.pathname === "/api/serial-bridge/stop") {
        const stopInfo = stopSerialBridge();
        jsonResponse(res, 200, stopInfo);
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/serial-bridge/logs") {
        const rawLines = url.searchParams.get("lines");
        const parsedLines = Number.parseInt(rawLines || "200", 10);
        const lineLimit = Number.isFinite(parsedLines)
            ? Math.max(10, Math.min(1000, parsedLines))
            : 200;

        try {
            const lines = readLogTail(lineLimit);
            jsonResponse(res, 200, {
                logPath: SERIAL_LOG_PATH,
                lines,
                lineCount: lines.length,
            });
        } catch (error) {
            jsonResponse(res, 500, {
                message: "ログ取得に失敗しました",
                error: String(error?.message || error),
            });
        }
        return;
    }

    if (req.method === "POST" && url.pathname === "/api/backend/force-shutdown") {
        jsonResponse(res, 200, {
            shuttingDown: true,
            message: "console backend を強制シャットダウンします",
        });
        forceShutdownBackend();
        return;
    }

    if (req.method === "POST" && url.pathname === "/api/json-exports/save") {
        readJsonBody(req)
            .then((body) => {
                if (!body || typeof body !== "object") {
                    jsonResponse(res, 400, { message: "invalid request body" });
                    return;
                }

                const category = body.category;
                const exportData = body.data;
                if (!category || typeof category !== "string") {
                    jsonResponse(res, 400, { message: "category is required" });
                    return;
                }
                if (exportData === undefined) {
                    jsonResponse(res, 400, { message: "data is required" });
                    return;
                }

                try {
                    const saved = saveJsonExportToAppDir({ category, exportData });
                    jsonResponse(res, 200, {
                        saved: true,
                        directory: APP_JS_DIR,
                        fileName: saved.fileName,
                        filePath: saved.filePath,
                    });
                } catch (error) {
                    jsonResponse(res, 500, {
                        saved: false,
                        message: "json export save failed",
                        error: String(error?.message || error),
                    });
                }
            })
            .catch((error) => {
                jsonResponse(res, 400, {
                    message: "invalid json body",
                    error: String(error?.message || error),
                });
            });
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/json-exports/latest") {
        const category = url.searchParams.get("category");
        if (!category) {
            jsonResponse(res, 400, { message: "category is required" });
            return;
        }

        const latest = findLatestJsonExport(category);
        if (!latest) {
            jsonResponse(res, 404, {
                found: false,
                message: "latest json export not found",
                category,
            });
            return;
        }

        jsonResponse(res, 200, {
            found: true,
            category,
            fileName: latest.fileName,
            filePath: latest.filePath,
            data: latest.data,
        });
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/json-exports/list") {
        const category = url.searchParams.get("category");
        if (!category) {
            jsonResponse(res, 400, { message: "category is required" });
            return;
        }

        const safeCategory = sanitizeSegment(category, "export");
        const categoryFolder = path.join(APP_JS_DIR, `${safeCategory}_exports`);

        if (!fs.existsSync(categoryFolder)) {
            jsonResponse(res, 200, {
                category,
                files: [],
                message: "no exports found for this category",
            });
            return;
        }

        try {
            const files = [];
            const dateFolders = fs.readdirSync(categoryFolder).sort((a, b) => b.localeCompare(a));
            for (const dateFolder of dateFolders) {
                const dateDir = path.join(categoryFolder, dateFolder);
                if (!fs.statSync(dateDir).isDirectory()) continue;

                const fileNames = fs.readdirSync(dateDir)
                    .filter((f) => f.endsWith(".json"))
                    .sort((a, b) => b.localeCompare(a));

                for (const fileName of fileNames) {
                    const filePath = path.join(dateDir, fileName);
                    const stat = fs.statSync(filePath);
                    files.push({
                        fileName,
                        dateFolder,
                        timestamp: stat.mtime.toISOString(),
                        size: stat.size,
                        relativePath: path.relative(APP_JS_DIR, filePath),
                    });
                }
            }
            jsonResponse(res, 200, {
                category,
                files,
                count: files.length,
            });
        } catch (error) {
            jsonResponse(res, 500, {
                category,
                message: "failed to list exports",
                error: String(error?.message || error),
            });
        }
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/json-exports/get") {
        const category = url.searchParams.get("category");
        const relativePath = url.searchParams.get("path");
        if (!category || !relativePath) {
            jsonResponse(res, 400, { message: "category and path are required" });
            return;
        }

        try {
            const safeCategory = sanitizeSegment(category, "export");
            const filePath = path.join(APP_JS_DIR, `${safeCategory}_exports`, relativePath);
            const realPath = path.resolve(filePath);
            const basePath = path.resolve(path.join(APP_JS_DIR, `${safeCategory}_exports`));

            // Security check: ensure path is within the category folder
            if (!realPath.startsWith(basePath)) {
                jsonResponse(res, 403, { message: "access denied" });
                return;
            }

            if (!fs.existsSync(realPath)) {
                jsonResponse(res, 404, { message: "file not found", path: relativePath });
                return;
            }

            const text = fs.readFileSync(realPath, "utf8");
            const parsed = JSON.parse(text);
            jsonResponse(res, 200, {
                found: true,
                category,
                fileName: path.basename(realPath),
                data: parsed,
            });
        } catch (error) {
            jsonResponse(res, 500, {
                message: "failed to read export file",
                error: String(error?.message || error),
            });
        }
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/planner-state-config/latest") {
        const category = url.searchParams.get("category") || "planner_state_config";

        const latest = findLatestPlannerStateConfig(category);
        if (!latest) {
            jsonResponse(res, 404, {
                found: false,
                category,
                message: "latest planner state config not found",
            });
            return;
        }

        jsonResponse(res, 200, {
            found: true,
            category,
            fileName: latest.fileName,
            filePath: latest.filePath,
            format: latest.format,
            data: latest.data,
            yamlEnabled: Boolean(yaml),
        });
        return;
    }


    jsonResponse(res, 404, { message: "not found" });
});

server.listen(BACKEND_PORT, "0.0.0.0", () => {
    console.log(`Console backend listening on :${BACKEND_PORT}`);
});

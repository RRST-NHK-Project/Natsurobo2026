#!/usr/bin/env node

const fs = require("fs");
const path = require("path");
const { execSync } = require("child_process");

const projectRoot = path.resolve(__dirname, "..");
const packageJsonPath = path.join(projectRoot, "package.json");

function readBaseVersion() {
    try {
        const raw = fs.readFileSync(packageJsonPath, "utf8");
        const pkg = JSON.parse(raw);
        return pkg.version || "0.0.0";
    } catch (error) {
        return "0.0.0";
    }
}

function run(command) {
    return execSync(command, {
        cwd: projectRoot,
        encoding: "utf8",
        stdio: ["ignore", "pipe", "ignore"],
    }).trim();
}

function buildVersion() {
    const base = readBaseVersion();

    try {
        const count = run("git rev-list --count HEAD");
        const describe = run("git describe --tags --always --dirty").replace(/\s+/g, "-");
        return `${base}+${count}-${describe}`;
    } catch (error) {
        return base;
    }
}

console.log(buildVersion());

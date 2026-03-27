// @ts-nocheck
import { spawn } from "child_process";
import path from "path";
import fs from "node:fs";

const hiCanGeneratorFileLocation = path.resolve(
  "../shared/hi-can-generator/hi-can-generator.py",
);
const hiCanAddressFileLocation = path.resolve(
  "../shared/hi-can/include/hi_can_address.hpp",
);
const canLookupFilePath = path.resolve("src/lib/canLookup.json");

const canBus = "can0";

let timeoutId;
let running = false;
let stopCandump = null;
let cleaned = false;

let canLookup = null;
const buffer = [];
const MAX_BUFFER = 1000;

export async function canSocket(io) {
  if (!running) {
    // Create lookup table file on launch
    await generateFile();

    // Start processing candump output (after file generated)
    stopCandump = startCanDump(canBus);

    // Send messages as they are received
    timeoutId = setInterval(() => {
      if (buffer.length === 0) return;
      const batch = buffer.splice(0, buffer.length);
      io.emit("can-data", batch);
    }, 2000);

    running = true;

    process.on("SIGINT", () => {
      cleanup();
      process.exit(0);
    });

    process.on("SIGTERM", () => {
      console.log("PROCESS ENDED\n");
      cleanup();
      process.exit(0);
    });
  }
}

function cleanup() {
  if (cleaned) return;
  cleaned = true;

  stopCandump?.();
  stopCandump = null;

  if (timeoutId) {
    clearInterval(timeoutId);
    timeoutId = null;
  }
}

async function generateFile() {
  try {
    const data = await lookupTable();

    canLookup = data;
    fs.writeFileSync(canLookupFilePath, JSON.stringify(data, null, 2));
  } catch (err) {
    console.error("Error:", err);
  }
}

const lookupTable = () => {
  return new Promise((resolve, reject) => {
    const pythonProcess = spawn("python3", [
      hiCanGeneratorFileLocation,
      hiCanAddressFileLocation,
    ]);

    let output = "";
    let error = "";

    pythonProcess.stdout.on("data", (data) => {
      output += data.toString();
    });

    pythonProcess.stderr.on("data", (data) => {
      error += data.toString();
    });

    pythonProcess.on("close", (code) => {
      if (code !== 0) {
        reject(error);
        return;
      }

      try {
        resolve(JSON.parse(output));
      } catch (err) {
        console.error(err);
        reject(`Invalid JSON\n`);
      }
    });
  });
};

export function startCanDump(iface) {
  const proc = spawn("candump", [iface]);

  let leftover = "";
  
  proc.on('error', (err) => {
    console.error('Failed to start candump:', err.message);
  });

  proc.stdout.on("data", (data) => {
    leftover += data.toString();

    const lines = leftover.split("\n");
    leftover = lines.pop() || "";

    for (const line of lines) {
      if (!line.trim()) continue;

      const parsed = parseCandump(line);

      if (!parsed || !parsed.details || !parsed.data.length) {
        // Unknown CAN or no data
        continue;
      }

      buffer.push(parsed);

        // avoid memory overflow
        if (buffer.length > MAX_BUFFER) {
          buffer.shift();
        }
    }
  });

  proc.stderr.on("data", (d) => {
    if (d.toString().includes("SIOCGIFINDEX: No such device")) {
      console.error(`Failed to start candump: can bus "${canBus}" cannot be found or is incorrect`);
    } else {
      console.error("STDERR:", d.toString());
    }
    cleanup();
  });

  return () => proc.kill(); // stop function
}

function parseCandump(line) {
  const parts = line.trim().split(/\s+/);
  if (parts.length < 4 || !canLookup) {
    return null;
  }
  const address = `0x${parseInt(parts[1], 16).toString(16).padStart(8, "0")}`;
  const canDetails = canLookup[address];

  const timestamp = formatTimestamp(Date.now());

  const parsed = {
    iface: parts[0],
    address: address,
    timestamp: timestamp,
    details: canDetails,
    data: parts.slice(3),
  };

  return parsed;
}

function formatTimestamp(ts) {
  const d = new Date(ts);

  const hours = String(d.getHours()).padStart(2, "0");
  const minutes = String(d.getMinutes()).padStart(2, "0");
  const seconds = String(d.getSeconds()).padStart(2, "0");
  const ms = String(d.getMilliseconds()).padStart(3, "0");

  return `${hours}:${minutes}:${seconds}.${ms}`;
}

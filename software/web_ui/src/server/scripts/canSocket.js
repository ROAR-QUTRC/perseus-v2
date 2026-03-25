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

let timeoutId;
let running = false;
let stopCandump = null;

let canLookup = null;
const buffer = [];
const MAX_BUFFER = 1000;

export async function canSocket(io) {
  if (!running) {
    // Create lookup table file on launch
    await generateFile();

    // Start processing candump output (after file generated)
    stopCandump = startCanDump("vcan0");

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
  console.log("Cleaned");
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
        canLookup = JSON.parse(output);
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

  proc.stdout.on("data", (data) => {
    leftover += data.toString();

    const lines = leftover.split("\n");
    leftover = lines.pop() || "";

    for (const line of lines) {
      if (!line.trim()) continue;

      const parsed = parseCandump(line);

      if (!parsed.details || !parsed.data.length) {
        // Unknown CAN or no data
        continue;
      }
      if (parsed) {
        buffer.push(parsed);

        // avoid memory overflow
        if (buffer.length > MAX_BUFFER) {
          buffer.shift();
        }
      }
    }
  });

  proc.stderr.on("data", (d) => {
    console.error("STDERR:", d.toString());
  });

  return () => proc.kill(); // stop function
}

function parseCandump(line) {
  const parts = line.trim().split(/\s+/);
  if (parts.length < 4) {
    return null;
  }
  const address = `0x${parseInt(parts[1], 16).toString(16).padStart(8, "0")}`;
  const canDetails = canLookup[address];

  const parsed = {
    iface: parts[0],
    address: address,
    details: canDetails,
    data: parts.slice(3),
  };

  return parsed;
}

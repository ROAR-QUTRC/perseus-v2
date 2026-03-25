// @ts-nocheck
import { spawn } from "child_process";
import path from "path";
import fs from "node:fs";
import { response } from "express";

const hiCanGeneratorFileLocation = path.resolve(
  "../shared/hi-can-generator/hi-can-generator.py",
);
const hiCanAddressFileLocation = path.resolve(
  "../shared/hi-can/include/hi_can_address.hpp",
);
const canLookupFilePath = path.resolve("src/lib/canLookup.json");

let timeoutId;
let running = false;

let canLookup = null;
const buffer = [];
const MAX_BUFFER = 1000;

export const canSocket = (socket) => {
  if (!running) {
    console.log("Can Socket");
    // Create lookup table file on launch
    generateFile();

    // Start processing candump output
    const stopCandump = startCanDump("vcan0");

    // Send messages as they are received
    timeoutId = setInterval(() => {
      if (buffer.length === 0) return;
      const batch = buffer.splice(0, buffer.length);
      socket.emit("can-data", batch);
    }, 2000);

    running = true;
  }
};

async function generateFile() {
  try {
    const data = await lookupTable();

    canLookup = data;
    fs.writeFileSync(canLookupFilePath, JSON.stringify(data, null, 2));

    console.log("File Written");
  } catch (err) {
    console.error("Error:", err);
  }
}

const lookupTable = () => {
  console.log("LOOKUP TABLE");
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

      // console.log(output.slice(output.length-200,output.length), "\n");

      try {
        canLookup = JSON.parse(output);
        console.log("CAN LOOKUP TYPE:", typeof canLookup);
        resolve(JSON.parse(output));
      } catch (err) {
        console.log(typeof output);
        console.error(err);
        reject(
          `Invalid JSON\n ${output.slice(0, 200)}\n...\n${output.slice(output.length - 200, output.length)}`,
        );
      }
    });
  });
};

const getCanData = () => {
  return new Promise((resolve, reject) => {
    const temp = latestInfo;
    latestInfo = [];
    resolve(latestInfo);
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
      if (parsed) {
        buffer.push(parsed);
        console.log("Added: ", parsed);

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

  // console.log("Parsing input", parsed);

  return parsed;
}

import type { RequestHandler } from "@sveltejs/kit";
import { exec } from "child_process";

let prevIdle: number = 0;
let prevActive: number = 0;

let prevRxBytes: number = 0;
let prevTxBytes: number = 0;

export const POST: RequestHandler = async ({ request }) => {
  let cpuPromise = new Promise((resolve, reject) => {
    exec("cat /proc/stat && free -t", (error, stdout, stderr) => {
      const times: number[] = stdout
        .split("\n")[0]
        .replace("cpu  ", "") // the two traliing spaces look weird and may break things
        .split(" ")
        .map((time) => parseInt(time));

      // algorithm reference mainly from: https://stackoverflow.com/questions/23367857/accurate-calculation-of-cpu-usage-given-in-percentage-in-linux
      // check the man page for /proc/stat for the order of the values
      const idle: number = times[3] + times[4]; // calculate idle time
      const active: number =
        times[0] + times[1] + times[2] + times[5] + times[6] + times[7]; // calculate active time

      // calculate the delta between the current and previous values
      const idleDelta: number = idle - prevIdle;
      const activeDelta: number = active - prevActive;

      // calculate the cpu usage
      const cpuUsage: number = activeDelta / (activeDelta + idleDelta);

      prevIdle = idle;
      prevActive = active;

      resolve(JSON.stringify({ cpuUsage: Math.round(cpuUsage * 10000) / 100 }));
    });
  });

  let memPromise = new Promise((resolve, reject) => {
    exec("free -tb", (error, stdout, stderr) => {
      // For both totals and swap:
      // 0: total
      // 1: used
      // 2: free
      // all values in bytes
      const totals: string[] = stdout
        .split("\n")[3]
        .split(" ")
        .filter((info) => info !== "" && info !== "Total:");
      const swap: string[] = stdout
        .split("\n")[2]
        .split(" ")
        .filter((info) => info !== "" && info !== "Swap:");

      resolve(JSON.stringify({ total: totals, swap: swap }));
    });
  });

  let netPromise = new Promise(async (resolve, reject) => {
    const { networkIf } = await request.json();
    if (networkIf === undefined) {
      resolve(JSON.stringify({ rx: 0, tx: 0 }));
      return;
    }
    exec(
      `cat /sys/class/net/${networkIf}/statistics/rx_bytes && cat /sys/class/net/${networkIf}/statistics/tx_bytes`,
      (error, stdout) => {
        const rxBytes: number = parseInt(stdout.split("\n")[0]);
        const txBytes: number = parseInt(stdout.split("\n")[1]);

        const rxDelta: number = rxBytes - prevRxBytes;
        const txDelta: number = txBytes - prevTxBytes;
        prevRxBytes = rxBytes;
        prevTxBytes = txBytes;

        resolve(JSON.stringify({ rx: rxDelta, tx: txDelta }));
      },
    );
  });

  return new Response(
    JSON.stringify(await Promise.all([cpuPromise, memPromise, netPromise])),
  );
};

export const GET: RequestHandler = async () => {
  return new Response(
    JSON.stringify(
      await new Promise((resolve, reject) => {
        exec("ls /sys/class/net", (error, stdout, stderr) => {
          resolve(stdout.split("\n").filter((iface) => iface !== ""));
        });
      }),
    ),
  );
};

// @ts-nocheck
import { exec } from "child_process";

let timeoutId;
let running = false;

let prevIdle = 0;
let prevActive = 0;

let prevNetworkData = {};

export const resourceMonitor = (socket) => {
  if (!running) {
    timeoutId = setInterval(async () => {
      socket.emit("monitor-data", {
        cpu: await cpuData(),
        memory: await memoryData(),
        network: await networkData(),
      });
    }, 1000);
    running = true;
  }
};

const cpuData = () => {
  return new Promise((resolve, reject) => {
    exec("cat /proc/stat", (error, stdout, stderr) => {
      const times = stdout
        .split("\n")[0]
        .replace("cpu  ", "") // the two traliing spaces look weird and may break things
        .split(" ")
        .map((time) => parseInt(time));

      // algorithm reference mainly from: https://stackoverflow.com/questions/23367857/accurate-calculation-of-cpu-usage-given-in-percentage-in-linux
      // check the man page for /proc/stat for the order of the values
      const idle = times[3] + times[4]; // calculate idle time
      const active =
        times[0] + times[1] + times[2] + times[5] + times[6] + times[7]; // calculate active time

      // calculate the delta between the current and previous values
      const idleDelta = idle - prevIdle;
      const activeDelta = active - prevActive;

      // calculate the cpu usage
      const cpuUsage = activeDelta / (activeDelta + idleDelta);

      prevIdle = idle;
      prevActive = active;

      // console.log('new: ', { idle: idle, active: active, prevIdle: prevIdle, prevActive: prevActive, usage: Math.round(cpuUsage * 10000) / 100})

      resolve(Math.round(cpuUsage * 10000) / 100);
    });
  });
};

const memoryData = () => {
  return new Promise((resolve, reject) => {
    exec("free -tb", (error, stdout, stderr) => {
      // For both totals and swap:
      // 0: total
      // 1: used
      // 2: free
      // all values in bytes
      const total = stdout
        .split("\n")[3]
        .split(" ")
        .filter((info) => info !== "" && info !== "Total:")
        .map((value) => Number(value));
      const swap = stdout
        .split("\n")[2]
        .split(" ")
        .filter((info) => info !== "" && info !== "Swap:")
        .map((value) => Number(value));
      const normal = total.map((n, i) => n - swap[i]);

      resolve({
        total: { total: total[0], used: total[1] },
        normal: { total: normal[0], used: normal[1] },
        swap: { total: swap[0], used: swap[1] },
      });
    });
  });
};

const networkData = () => {
  return new Promise(async (resolve, reject) => {
    exec("ls /sys/class/net/", async (_, stdout) => {
      const networkIf = stdout
        .split("\n")
        .filter((networkIf) => networkIf !== "");

      let networkData = {};
      for (const network of networkIf) {
        const data = await execPromise(
          `cat /sys/class/net/${network}/statistics/rx_bytes && cat /sys/class/net/${network}/statistics/tx_bytes`,
        );

        if (prevNetworkData[network] === undefined) {
          prevNetworkData[network] = {
            rxBytes: 0,
            txBytes: 0,
          };
        }

        const rxBytes = parseInt(data.split("\n")[0]);
        const txBytes = parseInt(data.split("\n")[1]);

        const rxDelta = rxBytes - prevNetworkData[network].rxBytes;
        const txDelta = txBytes - prevNetworkData[network].txBytes;

        prevNetworkData[network] = {
          rxBytes: rxBytes,
          txBytes: txBytes,
        };

        networkData[network] = {
          rxBytes: rxDelta,
          txBytes: txDelta,
        };
      }

      resolve(networkData);
    });
  });
};

const execPromise = (command) => {
  return new Promise((resolve, reject) => {
    exec(command, (error, stdout, stderr) => {
      if (error) {
        reject(error);
      } else {
        resolve(stdout);
      }
    });
  });
};

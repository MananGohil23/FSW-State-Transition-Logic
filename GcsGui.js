const missionTime = "00:00:00";
const fswState = "None";
let mode = "None";

document.getElementById("mission-time").textContent = missionTime;
document.getElementById("fsw-state").textContent = fswState;
document.getElementById("mode").textContent = mode;

const cxOnButton = document.querySelector("#cx-on");
const cxOffButton = document.querySelector("#cx-off");

const commandModes = {
  "cx-on": "CX ON",
  "cx-off": "CX OFF",
  "sim-enable": "SIM ENABLE",
  "sim-activate": "SIM ACTIVATE",
  "sim-disable": "SIM DISABLE",
  "mec-dep-on": "MEC DEP ON",
  calibrate: "CALIBRATE",
  "set-gps-time": "SET GPS TIME",
};

Object.entries(commandModes).forEach(([buttonId, nextMode]) => {
  const button = document.querySelector(`#${buttonId}`);

  if (!button) {
    return;
  }

  button.addEventListener("click", () => {
    mode = nextMode;
    document.getElementById("mode").textContent = mode;
    document.getElementById("cmd-echo").textContent = mode;
  });
});

let totalSeconds = 0;
let intervalId = null;

function updateInterval() {
  totalSeconds++;

  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;

  const formattedTime =
    String(hours).padStart(2, "0") +
    ":" +
    String(minutes).padStart(2, "0") +
    ":" +
    String(seconds).padStart(2, "0");

  document.getElementById("mission-time").textContent = formattedTime;
}

cxOnButton.addEventListener("click", () => {
  if (!intervalId) {
    intervalId = setInterval(updateInterval, 1000);
  }
});

cxOffButton.addEventListener("click", () => {
  if (intervalId) {
    clearInterval(intervalId);
    intervalId = null;
  }
});

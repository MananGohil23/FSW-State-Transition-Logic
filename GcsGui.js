const missionTime = "00:00:00";
const fswState = "None";
let mode = "None";

document.getElementById("mission-time").textContent = missionTime;
document.getElementById("fsw-state").textContent = fswState;
document.getElementById("mode").textContent = mode;

const cxOnButton = document.querySelector("#cx-on");
const cxOffButton = document.querySelector("#cx-off");

const commands = {
  "cx-on": "CX ON",
  "cx-off": "CX OFF",
  "sim-enable": "SIM ENABLE",
  "sim-activate": "SIM ACTIVATE",
  "sim-disable": "SIM DISABLE",
  "mec-dep-on": "MEC DEP ON",
  calibrate: "CALIBRATE",
  "set-gps-time": "SET GPS TIME",
};

Object.entries(commands).forEach(([buttonId, command]) => {
  const button = document.querySelector(`#${buttonId}`);

  if (!button) {
    return;
  }

  button.addEventListener("click", () => {
    document.getElementById("cmd-echo").textContent = command;
  });
});

const modes = {
  "cx-on": "Flight",
  "sim-activate": "Simulation",
  "sim-disable": "None",
}

Object.entries(modes).forEach(([buttonId, modeValue]) => {
  const button = document.querySelector(`#${buttonId}`);

  if (!button) {
    return;
  }

  button.addEventListener("click", () => {
    mode = modeValue;
    document.getElementById("mode").textContent = mode;
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

cxOffButton.addEventListener("dblclick", () => {
  if (totalSeconds !== 0) {
    formattedTime = "00:00:00";
    totalSeconds = 0;
    document.getElementById("mission-time").textContent = formattedTime;
  }
});

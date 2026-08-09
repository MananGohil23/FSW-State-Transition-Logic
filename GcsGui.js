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
let stateId = null;
let altitude = 0;
let isAscending = true;

function updateTimer() {
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

function updateState() {
  if (isAscending) {
    altitude++;
    document.getElementById("altitude").textContent = altitude;
    if (altitude >= 100) {
      isAscending = false;
    }
  } else {
    altitude--;
    document.getElementById("altitude").textContent = altitude;
    if (altitude <= 0) {
      altitude = 0;
      isAscending = true;
      if (stateId) {
        clearInterval(stateId);
        stateId = null;
      }
    }
  }
}

cxOnButton.addEventListener("click", () => {
  if (!intervalId) {
    intervalId = setInterval(updateTimer, 1000);
  }
  if (!stateId) {
    stateId = setInterval(updateState, 300);
  }
});

cxOffButton.addEventListener("click", () => {
  if (intervalId) {
    clearInterval(intervalId);
    intervalId = null;
  }

  if (stateId) {
    clearInterval(stateId);
    stateId = null;
  }

  const currentMissionTime = document.getElementById("mission-time").textContent;
  document.getElementById("last-packet-time").textContent = currentMissionTime;
});

cxOffButton.addEventListener("dblclick", () => {
  document.getElementById("mode").textContent = "None";

  if (totalSeconds !== 0) {
    const formattedTime = "00:00:00";
    totalSeconds = 0;
    document.getElementById("mission-time").textContent = formattedTime;
    document.getElementById("last-packet-time").textContent = formattedTime;
  }

  altitude = 0;
  isAscending = true;
  document.getElementById("altitude").textContent = altitude;
});

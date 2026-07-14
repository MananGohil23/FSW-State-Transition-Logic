const missionTime = "00:00:00";
const fswState = "None";
let mode = "None";

document.getElementById("mission-time").textContent = missionTime;
document.getElementById("fsw-state").textContent = fswState;
document.getElementById("mode").textContent = mode;

const cxOnButton = document.querySelector("#cx-on");

const commandModes = {
  "cx-on": "CX ON",
  "cx-off": "CX OFF",
  "sim-enable": "SIM ENABLE",
  "sim-activate": "SIM ACTIVATE",
  "sim-disable": "SIM DISABLE",
  "mec-dep-on": "MEC DEP ON",
  "calibrate": "CALIBRATE",
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

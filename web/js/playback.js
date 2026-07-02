import { PointCloudViewer } from "./point-cloud-viewer.js";
import { groupUnitsIntoGofs, parseV3cSampleStream } from "./v3c-sample-stream.js";
import { unitToRtpPackets } from "./v3c-rtp-packet.js";

const statusEl = document.getElementById("status");
const fileInput = document.getElementById("fileInput");
const playBtn = document.getElementById("playBtn");
const pauseBtn = document.getElementById("pauseBtn");
const stopBtn = document.getElementById("stopBtn");
const fpsInput = document.getElementById("fpsInput");
const gofSlider = document.getElementById("gofSlider");
const infoEl = document.getElementById("info");

const viewer = new PointCloudViewer(document.getElementById("viewerHost"));

const worker = new Worker(new URL("./v3c-decoder-worker.js", import.meta.url), { type: "module" });
worker.onmessage = (event) => {
  const message = event.data;
  if (message.type === "error") {
    statusEl.textContent = `Decode: ${message.message}`;
    return;
  }
  if (message.type === "video-frame") {
    statusEl.textContent = `V-PCC ${message.track} frame (file replay)`;
    return;
  }
  if (message.type === "atlas") {
    statusEl.textContent = `Atlas GOF metadata (${message.byteLength} B)`;
  }
};

let parsedFile = null;
let gofs = [];
let playing = false;
let playTimer = null;
let currentGof = 0;
let rtpState = { sequence: 0, timestamp: 0 };

function setInfo(html) {
  infoEl.innerHTML = html;
}

function resetWorker() {
  worker.postMessage({ type: "reset" });
  rtpState = { sequence: 0, timestamp: 0 };
}

function sendGofToWorker(gofIndex) {
  const gof = gofs[gofIndex];
  if (!gof) {
    return;
  }

  rtpState.timestamp = gofIndex * 3000;
  for (const unit of gof) {
    const packets = unitToRtpPackets(unit.data, unit.unitType, rtpState);
    for (const packet of packets) {
      worker.postMessage({ type: "rtp-packet", data: packet.buffer }, [packet.buffer]);
    }
  }

  statusEl.textContent = `Playing GOF ${gofIndex + 1} / ${gofs.length}`;
  gofSlider.value = String(gofIndex);
}

function stopPlayback() {
  playing = false;
  if (playTimer) {
    clearInterval(playTimer);
    playTimer = null;
  }
}

function startPlayback() {
  if (!gofs.length) {
    return;
  }

  stopPlayback();
  playing = true;
  const fps = Math.max(1, Number(fpsInput.value) || 5);
  const intervalMs = 1000 / fps;

  sendGofToWorker(currentGof);
  playTimer = setInterval(() => {
    currentGof += 1;
    if (currentGof >= gofs.length) {
      stopPlayback();
      statusEl.textContent = `Finished (${gofs.length} GOF(s))`;
      return;
    }
    sendGofToWorker(currentGof);
  }, intervalMs);
}

fileInput.addEventListener("change", async () => {
  stopPlayback();
  resetWorker();
  currentGof = 0;

  const file = fileInput.files?.[0];
  if (!file) {
    return;
  }

  statusEl.textContent = `Loading ${file.name}...`;
  const buffer = await file.arrayBuffer();
  const bytes = new Uint8Array(buffer);

  try {
    parsedFile = parseV3cSampleStream(bytes);
    gofs = groupUnitsIntoGofs(parsedFile.units);
    gofSlider.min = "0";
    gofSlider.max = String(Math.max(0, gofs.length - 1));
    gofSlider.value = "0";
    gofSlider.disabled = gofs.length === 0;

    const unitCounts = {};
    for (const unit of parsedFile.units) {
      unitCounts[unit.name] = (unitCounts[unit.name] ?? 0) + 1;
    }

    setInfo(
      `<strong>${file.name}</strong> (${(bytes.byteLength / 1024).toFixed(1)} KB)<br>` +
        `V3C units: ${parsedFile.units.length}, GOFs: ${gofs.length}, size precision: ${parsedFile.precision}<br>` +
        Object.entries(unitCounts)
          .map(([name, count]) => `${name}: ${count}`)
          .join(" · "),
    );

    statusEl.textContent = `Loaded ${gofs.length} GOF(s). Press Play or scrub.`;
    playBtn.disabled = gofs.length === 0;
  } catch (error) {
    statusEl.textContent = `Parse error: ${error.message}`;
    setInfo("");
    playBtn.disabled = true;
  }
});

playBtn.addEventListener("click", () => {
  if (!playing) {
    startPlayback();
  }
});

pauseBtn.addEventListener("click", () => {
  stopPlayback();
  statusEl.textContent = `Paused at GOF ${currentGof + 1} / ${gofs.length}`;
});

stopBtn.addEventListener("click", () => {
  stopPlayback();
  currentGof = 0;
  resetWorker();
  gofSlider.value = "0";
  statusEl.textContent = "Stopped";
});

gofSlider.addEventListener("input", () => {
  stopPlayback();
  currentGof = Number(gofSlider.value);
  resetWorker();
  sendGofToWorker(currentGof);
});

function animate() {
  requestAnimationFrame(animate);
  viewer.render();
}
animate();

window.addEventListener("resize", () => {
  viewer.resize(window.innerWidth, window.innerHeight - 120);
});

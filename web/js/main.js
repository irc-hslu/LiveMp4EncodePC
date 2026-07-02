import { PointCloudViewer } from "./point-cloud-viewer.js";

const statusEl = document.getElementById("status");
const serverListEl = document.getElementById("serverList");
const profileStatsEl = document.getElementById("profileStats");
const wsUrlsInput = document.getElementById("wsUrls");
const connectBtn = document.getElementById("connectBtn");

const PCV1_MAGIC = 0x31564350;
const PROFILE_INTERVAL_MS = 5000;

function formatMbps(bytes, intervalSec) {
  if (intervalSec <= 0) {
    return "0.00";
  }
  return ((bytes * 8) / (intervalSec * 1_000_000)).toFixed(2);
}

function createReceiveStats() {
  return {
    previewBytes: 0,
    previewPackets: 0,
    previewFrames: 0,
    rtpBytes: 0,
    rtpPackets: 0,
    renderFrames: 0,
    windowStart: performance.now(),
  };
}

const receiveStats = createReceiveStats();

function isPcv1FrameComplete(bytes) {
  if (bytes.byteLength < 12) {
    return false;
  }
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (view.getUint32(0, true) !== PCV1_MAGIC) {
    return false;
  }
  const chunkIndex = view.getUint16(8, true);
  const chunkCount = view.getUint16(10, true);
  return chunkCount > 0 && chunkIndex === chunkCount - 1;
}

function notePreviewPacket(conn, payload, renderedFrame) {
  receiveStats.previewBytes += payload.byteLength + 1;
  receiveStats.previewPackets += 1;
  conn.previewBytes = (conn.previewBytes || 0) + payload.byteLength + 1;
  conn.previewPackets = (conn.previewPackets || 0) + 1;
  if (isPcv1FrameComplete(payload)) {
    receiveStats.previewFrames += 1;
    conn.previewFrames = (conn.previewFrames || 0) + 1;
  }
  if (renderedFrame) {
    receiveStats.renderFrames += 1;
    conn.renderFrames = (conn.renderFrames || 0) + 1;
  }
}

function noteRtpPacket(conn, payload) {
  receiveStats.rtpBytes += payload.byteLength + 1;
  receiveStats.rtpPackets += 1;
  conn.rtpBytes = (conn.rtpBytes || 0) + payload.byteLength + 1;
  conn.rtpPackets = (conn.rtpPackets || 0) + 1;
}

function renderProfileStats() {
  const intervalSec = Math.max(0.001, (performance.now() - receiveStats.windowStart) / 1000);
  const lines = [
    `preview WS  ${(receiveStats.previewFrames / intervalSec).toFixed(1)} fps | ${formatMbps(receiveStats.previewBytes, intervalSec)} Mbps | ${receiveStats.previewPackets} pkts`,
    `render      ${(receiveStats.renderFrames / intervalSec).toFixed(1)} fps`,
    `rtp WS      ${(receiveStats.rtpPackets / intervalSec).toFixed(1)} pkts/s | ${formatMbps(receiveStats.rtpBytes, intervalSec)} Mbps`,
  ];
  profileStatsEl.textContent = lines.join("\n");

  Object.assign(receiveStats, createReceiveStats());
  for (const conn of connections.values()) {
    conn.previewBytes = 0;
    conn.previewPackets = 0;
    conn.previewFrames = 0;
    conn.rtpBytes = 0;
    conn.rtpPackets = 0;
    conn.renderFrames = 0;
    conn.windowStart = performance.now();
  }
}

setInterval(renderProfileStats, PROFILE_INTERVAL_MS);

const viewer = new PointCloudViewer(document.body);

/** Live preview (0x02 PCV1) does not need WebCodecs; RTP V-PCC decode is optional. */
const v3cWebCodecsAvailable = typeof globalThis.VideoDecoder !== "undefined";

const worker = new Worker(new URL("./v3c-decoder-worker.js", import.meta.url), { type: "module" });
let anyPreviewReceived = false;
let webCodecsWarned = false;

worker.onmessage = (event) => {
  const message = event.data;
  if (message.type === "error") {
    const isWebCodecs = message.message.includes("WebCodecs");
    if (isWebCodecs) {
      if (!webCodecsWarned) {
        webCodecsWarned = true;
        console.warn(message.message);
      }
      if (!anyPreviewReceived) {
        statusEl.textContent =
          "Connected — waiting for preview (V-PCC HEVC decode needs Chrome/Edge; preview uses UDP PCV1)";
      }
      return;
    }
    if (!anyPreviewReceived) {
      statusEl.textContent = `V-PCC decode: ${message.message}`;
    }
    return;
  }

  if (message.type === "video-frame") {
    statusEl.textContent = `V-PCC ${message.track} frame @ ${Math.round(message.timestamp / 1000)} ms`;
    return;
  }

  if (message.type === "atlas") {
    statusEl.textContent = `V-PCC atlas (${message.byteLength} B, RTP packets=${message.stats.packets}) — waiting for preview...`;
  }
};

/** @type {Map<string, { url: string, socket: WebSocket, packetCount: number, previewCount: number, lastPreviewWallMs: number, previewFpsEstimate: number, waitingTimer: ReturnType<typeof setInterval>|null, status: string }>} */
const connections = new Map();

function shortUrl(url) {
  try {
    const parsed = new URL(url);
    return parsed.host;
  } catch {
    return url;
  }
}

function captureFpsLabel(conn) {
  return conn.previewFpsEstimate > 0 ? conn.previewFpsEstimate.toFixed(1) : "?";
}

function notePreviewFrame(conn) {
  const now = performance.now();
  if (conn.lastPreviewWallMs > 0) {
    const dt = now - conn.lastPreviewWallMs;
    if (dt > 1) {
      const instant = 1000 / dt;
      conn.previewFpsEstimate =
        conn.previewFpsEstimate > 0 ? conn.previewFpsEstimate * 0.85 + instant * 0.15 : instant;
    }
  }
  conn.lastPreviewWallMs = now;
}

function renderServerList() {
  if (connections.size === 0) {
    serverListEl.textContent = "";
    return;
  }

  const lines = [];
  for (const [id, conn] of connections) {
    const host = shortUrl(conn.url);
    const winSec = Math.max(0.001, (performance.now() - (conn.windowStart || performance.now())) / 1000);
    const previewMbps = formatMbps(conn.previewBytes || 0, winSec);
    const previewFps =
      conn.previewFrames && winSec > 0 ? (conn.previewFrames / winSec).toFixed(1) : captureFpsLabel(conn);
    lines.push(`${host}: ${conn.status} | ${previewMbps} Mbps preview`);
  }
  serverListEl.textContent = lines.join("\n");
}

function updateOverallStatus() {
  const total = connections.size;
  const connected = [...connections.values()].filter((c) => c.socket.readyState === WebSocket.OPEN).length;
  const streaming = [...connections.values()].filter((c) => c.previewCount > 0).length;

  if (total === 0) {
    statusEl.textContent = "Idle";
    return;
  }

  if (connected === 0) {
    statusEl.textContent = `Connecting to ${total} server(s)...`;
    return;
  }

  statusEl.textContent = `${connected}/${total} connected, ${streaming} streaming`;
  renderServerList();
}

function disconnectAll() {
  for (const [id, conn] of connections) {
    if (conn.waitingTimer) {
      clearInterval(conn.waitingTimer);
    }
    conn.socket.close();
    viewer.removeLayer(id);
  }
  connections.clear();
  renderServerList();
}

function connectServer(url) {
  const id = url;

  const conn = {
    url,
    socket: null,
    packetCount: 0,
    previewCount: 0,
    previewBytes: 0,
    previewPackets: 0,
    previewFrames: 0,
    rtpBytes: 0,
    rtpPackets: 0,
    renderFrames: 0,
    windowStart: performance.now(),
    lastPreviewWallMs: 0,
    previewFpsEstimate: 0,
    waitingTimer: null,
    status: "Connecting...",
  };
  connections.set(id, conn);

  const socket = new WebSocket(url);
  socket.binaryType = "arraybuffer";
  conn.socket = socket;

  socket.onopen = () => {
    conn.status = "Connected, waiting for data...";
    updateOverallStatus();
    conn.waitingTimer = setInterval(() => {
      if (conn.packetCount === 0 && conn.previewCount === 0 && socket.readyState === WebSocket.OPEN) {
        conn.status = "No data yet (is streamer running?)";
        updateOverallStatus();
      }
    }, 3000);
  };

  socket.onmessage = (event) => {
    const data = new Uint8Array(event.data);
    if (data.length === 0) {
      return;
    }

    if (data[0] === 0x02) {
      const payload = data.subarray(1);
      conn.previewCount += 1;
      anyPreviewReceived = true;
      if (conn.waitingTimer) {
        clearInterval(conn.waitingTimer);
        conn.waitingTimer = null;
      }
      const parsed = viewer.updateFromPreview(id, payload);
      if (parsed && !parsed.partial) {
        notePreviewFrame(conn);
        notePreviewPacket(conn, payload, true);
        conn.status = `frame ${parsed.frameNumber} (${parsed.pointCount} pts @ ~${captureFpsLabel(conn)} fps)`;
      } else {
        notePreviewPacket(conn, payload, false);
      }
      updateOverallStatus();
      return;
    }

    const rtpPacket = data[0] === 0x01 ? data.subarray(1) : data;
    conn.packetCount += 1;
    noteRtpPacket(conn, rtpPacket);
    if (conn.packetCount === 1 && conn.previewCount === 0) {
      if (conn.waitingTimer) {
        clearInterval(conn.waitingTimer);
        conn.waitingTimer = null;
      }
      conn.status = "Receiving RTP packets...";
    } else if (conn.packetCount % 50 === 0 && conn.previewCount === 0) {
      conn.status = `Receiving RTP (${conn.packetCount} packets)...`;
    }
    updateOverallStatus();

    if (!v3cWebCodecsAvailable) {
      return;
    }

    const rtpCopy = new Uint8Array(rtpPacket);
    worker.postMessage({ type: "rtp-packet", data: rtpCopy.buffer }, [rtpCopy.buffer]);
  };

  socket.onerror = () => {
    conn.status = "WebSocket error";
    updateOverallStatus();
  };

  socket.onclose = () => {
    if (conn.waitingTimer) {
      clearInterval(conn.waitingTimer);
      conn.waitingTimer = null;
    }
    conn.status = "Disconnected";
    updateOverallStatus();
  };
}

function parseUrls(text) {
  const seen = new Set();
  const urls = [];
  for (const line of text.split(/\r?\n/)) {
    const url = line.trim();
    if (!url || seen.has(url)) {
      continue;
    }
    seen.add(url);
    urls.push(url);
  }
  return urls;
}

function connect() {
  const urls = parseUrls(wsUrlsInput.value);
  if (urls.length === 0) {
    statusEl.textContent = "Enter at least one WebSocket URL";
    return;
  }

  disconnectAll();
  anyPreviewReceived = false;
  webCodecsWarned = false;
  Object.assign(receiveStats, createReceiveStats());
  profileStatsEl.textContent = "";
  if (v3cWebCodecsAvailable) {
    worker.postMessage({ type: "reset" });
  }
  statusEl.textContent = `Connecting to ${urls.length} server(s)...`;
  serverListEl.textContent = "";

  for (const url of urls) {
    connectServer(url);
  }
  updateOverallStatus();
}

connectBtn.addEventListener("click", connect);
window.connect = connect;

function animate() {
  requestAnimationFrame(animate);
  viewer.render();
}
animate();

window.addEventListener("resize", () => {
  viewer.resize(window.innerWidth, window.innerHeight);
});

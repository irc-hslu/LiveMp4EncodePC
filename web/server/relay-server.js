/**
 * UDP relay for browser V-PCC viewer:
 *   RTP (V3C)     UDP 8890 -> WebSocket with 0x01 prefix
 *   Point preview UDP 8891 -> WebSocket with 0x02 prefix (PCV1 / Three.js)
 *
 * Usage:
 *   node relay-server.js --udp-port 8890 --preview-udp-port 8891 --ws-port 8081
 *   node relay-server.js --profile-interval 5
 */

import dgram from "node:dgram";
import http from "node:http";
import { WebSocketServer } from "ws";

const PCV1_MAGIC = 0x31564350;

function parseArgs(argv) {
  const config = {
    udpHost: "0.0.0.0",
    udpPort: 8890,
    previewUdpPort: 8891,
    wsHost: "0.0.0.0",
    wsPort: 8081,
    profileIntervalSec: 5,
    profileEnabled: true,
  };

  for (let i = 2; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === "--udp-port") {
      config.udpPort = Number(argv[++i]);
    } else if (arg === "--udp-host") {
      config.udpHost = argv[++i];
    } else if (arg === "--preview-udp-port") {
      config.previewUdpPort = Number(argv[++i]);
    } else if (arg === "--ws-port") {
      config.wsPort = Number(argv[++i]);
    } else if (arg === "--ws-host") {
      config.wsHost = argv[++i];
    } else if (arg === "--profile-interval") {
      config.profileIntervalSec = Number(argv[++i]);
    } else if (arg === "--no-profile") {
      config.profileEnabled = false;
    }
  }

  return config;
}

function createLane(name) {
  return {
    name,
    packets: 0,
    bytes: 0,
    frames: 0,
  };
}

function formatMbps(bytes, intervalSec) {
  if (intervalSec <= 0) {
    return "0.00 Mbps";
  }
  return `${((bytes * 8) / (intervalSec * 1_000_000)).toFixed(2)} Mbps`;
}

function formatFps(events, intervalSec) {
  if (intervalSec <= 0) {
    return "0.0 fps";
  }
  return `${(events / intervalSec).toFixed(1)} fps`;
}

function isPcv1FrameComplete(message) {
  if (message.length < 12) {
    return false;
  }
  if (message.readUInt32LE(0) !== PCV1_MAGIC) {
    return false;
  }
  const chunkIndex = message.readUInt16LE(8);
  const chunkCount = message.readUInt16LE(10);
  return chunkCount > 0 && chunkIndex === chunkCount - 1;
}

function noteLane(lane, message, countFrame = false) {
  lane.packets += 1;
  lane.bytes += message.length;
  if (countFrame && isPcv1FrameComplete(message)) {
    lane.frames += 1;
  }
}

const config = parseArgs(process.argv);
const clients = new Set();

const lanes = {
  rtpUdpIn: createLane("rtp_udp_in"),
  previewUdpIn: createLane("preview_udp_in"),
  rtpWsOut: createLane("rtp_ws_out"),
  previewWsOut: createLane("preview_ws_out"),
};

let profileStartedAt = Date.now();

function broadcastPrefixed(prefix, message, outLane) {
  if (clients.size === 0) {
    return;
  }
  const framed = Buffer.allocUnsafe(1 + message.length);
  framed[0] = prefix;
  message.copy(framed, 1);
  for (const client of clients) {
    if (client.readyState === client.OPEN) {
      client.send(framed, { binary: true });
      if (outLane) {
        outLane.bytes += framed.length;
        outLane.packets += 1;
        if (outLane === lanes.previewWsOut && isPcv1FrameComplete(message)) {
          outLane.frames += 1;
        }
      }
    }
  }
}

function reportProfile(finalReport = false) {
  if (!config.profileEnabled) {
    return;
  }
  const intervalSec = Math.max(0.001, (Date.now() - profileStartedAt) / 1000);
  const header = finalReport ? "[profile] Final summary" : "[profile]";
  console.log(
    `${header} (${intervalSec.toFixed(1)}s, ${clients.size} WS client(s))\n` +
      `  rtp_udp_in     ${formatFps(lanes.rtpUdpIn.packets, intervalSec)} pkts/s | ${formatMbps(lanes.rtpUdpIn.bytes, intervalSec)}\n` +
      `  preview_udp_in ${formatFps(lanes.previewUdpIn.frames, intervalSec)} frames/s | ${formatMbps(lanes.previewUdpIn.bytes, intervalSec)} | ${lanes.previewUdpIn.packets} pkts\n` +
      `  rtp_ws_out     ${formatFps(lanes.rtpWsOut.packets, intervalSec)} msgs/s | ${formatMbps(lanes.rtpWsOut.bytes, intervalSec)}\n` +
      `  preview_ws_out ${formatFps(lanes.previewWsOut.frames, intervalSec)} frames/s | ${formatMbps(lanes.previewWsOut.bytes, intervalSec)} | ${lanes.previewWsOut.packets} msgs`,
  );

  for (const lane of Object.values(lanes)) {
    lane.packets = 0;
    lane.bytes = 0;
    lane.frames = 0;
  }
  profileStartedAt = Date.now();
}

const rtpSocket = dgram.createSocket("udp4");
rtpSocket.on("message", (message, remote) => {
  noteLane(lanes.rtpUdpIn, message);
  if (lanes.rtpUdpIn.packets === 1) {
    console.log(
      `RTP packet #1 from ${remote.address}:${remote.port} (${message.length} bytes, ${clients.size} client(s))`,
    );
  }
  if (clients.size === 0 && lanes.rtpUdpIn.packets === 1) {
    console.warn("Received RTP but no browser WebSocket client is connected yet.");
  }
  broadcastPrefixed(0x01, message, lanes.rtpWsOut);
});

rtpSocket.bind(config.udpPort, config.udpHost, () => {
  console.log(`RTP relay listening on udp://${config.udpHost}:${config.udpPort}`);
});

const previewSocket = dgram.createSocket("udp4");
previewSocket.on("message", (message, remote) => {
  noteLane(lanes.previewUdpIn, message, true);
  if (lanes.previewUdpIn.packets === 1) {
    console.log(
      `Preview packet #1 from ${remote.address}:${remote.port} (${message.length} bytes, ${clients.size} client(s))`,
    );
  }
  broadcastPrefixed(0x02, message, lanes.previewWsOut);
});

previewSocket.bind(config.previewUdpPort, config.udpHost, () => {
  console.log(`Point cloud preview relay on udp://${config.previewUdpPort}`);
});

const server = http.createServer((_req, res) => {
  res.writeHead(200, { "Content-Type": "text/plain" });
  res.end("V-PCC RTP + point cloud preview relay online\n");
});

const wss = new WebSocketServer({ server });
wss.on("connection", (socket) => {
  clients.add(socket);
  console.log(`Browser connected (${clients.size} total)`);
  socket.on("close", () => {
    clients.delete(socket);
    console.log(`Browser disconnected (${clients.size} total)`);
  });
});

server.listen(config.wsPort, config.wsHost, () => {
  console.log(`WebSocket relay listening on ws://${config.wsHost}:${config.wsPort}`);
  if (config.profileEnabled) {
    console.log(`Profiling enabled (every ${config.profileIntervalSec}s)`);
    setInterval(() => reportProfile(false), config.profileIntervalSec * 1000);
  }
});

process.on("SIGINT", () => {
  if (config.profileEnabled) {
    reportProfile(true);
  }
  process.exit(0);
});

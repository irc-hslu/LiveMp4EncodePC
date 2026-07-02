import { V3cStreamDemuxer } from "./atlas-parser.js";

const demuxer = new V3cStreamDemuxer();
const decoders = new Map();

async function createHevcDecoder(track, onFrame) {
  const config = {
    codec: "hev1.1.6.L93.B0",
    optimizeForLatency: true,
  };

  if (!("VideoDecoder" in globalThis)) {
    postMessage({ type: "error", message: "WebCodecs VideoDecoder is unavailable in this browser." });
    return null;
  }

  const supported = await VideoDecoder.isConfigSupported(config);
  if (!supported.supported) {
    postMessage({ type: "error", message: `HEVC decode unsupported for ${track}` });
    return null;
  }

  const decoder = new VideoDecoder({
    output(frame) {
      createImageBitmap(frame).then((bitmap) => {
        onFrame(track, bitmap, frame.timestamp);
        frame.close();
      });
    },
    error(error) {
      postMessage({ type: "error", message: `${track} decoder error: ${error.message}` });
    },
  });

  decoder.configure(config);
  return decoder;
}

function ensureDecoder(track) {
  if (decoders.has(track)) {
    return decoders.get(track);
  }

  const promise = createHevcDecoder(track, (trackName, bitmap, timestamp) => {
    postMessage({ type: "video-frame", track: trackName, bitmap, timestamp }, [bitmap]);
  });
  decoders.set(track, promise);
  return promise;
}

async function decodeVideoTrack(track, payload) {
  const decoder = await ensureDecoder(track);
  if (!decoder) {
    return;
  }

  const chunk = new EncodedVideoChunk({
    type: "key",
    timestamp: performance.now() * 1000,
    data: payload,
  });
  decoder.decode(chunk);
}

self.onmessage = async (event) => {
  const message = event.data;
  if (message.type === "rtp-packet") {
    const packet = new Uint8Array(message.data);
    const parsed = demuxer.processPacket(packet);
    if (!parsed) {
      return;
    }

    if (parsed.type === "geometry" || parsed.type === "attribute" || parsed.type === "occupancy") {
      await decodeVideoTrack(parsed.type, parsed.payload);
      return;
    }

    postMessage({
      type: parsed.type,
      timestamp: parsed.timestamp,
      stats: demuxer.stats,
      atlas: demuxer.atlasParser.getLatestMetadata(),
      byteLength: parsed.payload.byteLength,
    });
  }

  if (message.type === "reset") {
    demuxer.stats.packets = 0;
    demuxer.stats.byUnit = {};
    for (const decoderPromise of decoders.values()) {
      const decoder = await decoderPromise;
      decoder?.close();
    }
    decoders.clear();
  }
};

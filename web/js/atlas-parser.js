const V3C_UNIT = {
  VPS: 0,
  AD: 1,
  OVD: 2,
  GVD: 3,
  AVD: 4,
};

export function ssrcToUnitType(ssrc) {
  return ssrc - 1;
}

export function parseRtpHeader(view) {
  if (view.byteLength < 12) {
    return null;
  }

  const version = view.getUint8(0) >> 6;
  if (version !== 2) {
    return null;
  }

  const csrcCount = view.getUint8(0) & 0x0f;
  const extension = (view.getUint8(0) & 0x10) !== 0;
  const headerSize = 12 + csrcCount * 4 + (extension ? 4 : 0);

  return {
    payloadType: view.getUint8(1) & 0x7f,
    sequenceNumber: view.getUint16(2),
    timestamp: view.getUint32(4),
    ssrc: view.getUint32(8),
    headerSize,
  };
}

export function parseV3cUnitHeader(bytes) {
  if (bytes.length < 4) {
    return null;
  }

  const unitType = bytes[0] >> 5;
  const atlasId = bytes[2];
  return { unitType, atlasId };
}

export class AtlasParser {
  constructor() {
    this.patchMetadata = [];
  }

  ingestAtlasPayload(payload) {
    this.patchMetadata.push({
      receivedAt: performance.now(),
      byteLength: payload.byteLength,
    });
  }

  getLatestMetadata() {
    return this.patchMetadata.at(-1) ?? null;
  }
}

export class NalReassembler {
  constructor() {
    this.buffers = new Map();
  }

  pushRtpPacket(packet, unitType) {
    const view = new DataView(packet.buffer, packet.byteOffset, packet.byteLength);
    const header = parseRtpHeader(view);
    if (!header) {
      return null;
    }

    const payload = packet.subarray(header.headerSize);
    if (payload.length < 2) {
      return null;
    }

    const nalHeader = (payload[0] << 8) | payload[1];
    const key = `${unitType}:${header.ssrc}:${header.timestamp}:${nalHeader}`;
    const previous = this.buffers.get(key) ?? new Uint8Array(0);
    const merged = new Uint8Array(previous.length + payload.length);
    merged.set(previous, 0);
    merged.set(payload, previous.length);
    this.buffers.set(key, merged);

    if (payload.length < 1200) {
      this.buffers.delete(key);
      return merged;
    }

    return null;
  }
}

export class V3cStreamDemuxer {
  constructor() {
    this.atlasParser = new AtlasParser();
    this.nalReassembler = new NalReassembler();
    this.stats = {
      packets: 0,
      byUnit: {},
    };
  }

  processPacket(packet) {
    const view = new DataView(packet.buffer, packet.byteOffset, packet.byteLength);
    const header = parseRtpHeader(view);
    if (!header) {
      return null;
    }

    const unitType = ssrcToUnitType(header.ssrc);
    this.stats.packets += 1;
    this.stats.byUnit[unitType] = (this.stats.byUnit[unitType] ?? 0) + 1;

    const payload = packet.subarray(header.headerSize);
    if (payload.length === 0) {
      return null;
    }

    switch (unitType) {
      case V3C_UNIT.AD:
        this.atlasParser.ingestAtlasPayload(payload);
        return { type: "atlas", payload, timestamp: header.timestamp };

      case V3C_UNIT.GVD:
      case V3C_UNIT.AVD:
      case V3C_UNIT.OVD: {
        const nal = this.nalReassembler.pushRtpPacket(packet, unitType);
        if (!nal) {
          return null;
        }
        return {
          type: unitType === V3C_UNIT.GVD ? "geometry" : unitType === V3C_UNIT.AVD ? "attribute" : "occupancy",
          payload: nal,
          timestamp: header.timestamp,
          unitType,
        };
      }

      case V3C_UNIT.VPS:
        return { type: "vps", payload, timestamp: header.timestamp };

      default:
        return null; 
    }
  }
}

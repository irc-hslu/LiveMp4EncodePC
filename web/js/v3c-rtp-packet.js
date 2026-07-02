/** Build minimal RTP packets for the browser V3C demuxer (matches live relay path). */

const MAX_RTP_PAYLOAD = 1200;

/**
 * @param {{ payload: Uint8Array, ssrc: number, sequenceNumber: number, timestamp: number }} params
 */
export function buildRtpPacket({ payload, ssrc, sequenceNumber, timestamp }) {
  const packet = new Uint8Array(12 + payload.length);
  packet[0] = 0x80;
  packet[1] = 96;
  packet[2] = (sequenceNumber >> 8) & 0xff;
  packet[3] = sequenceNumber & 0xff;
  packet[4] = (timestamp >> 24) & 0xff;
  packet[5] = (timestamp >> 16) & 0xff;
  packet[6] = (timestamp >> 8) & 0xff;
  packet[7] = timestamp & 0xff;
  packet[8] = (ssrc >> 24) & 0xff;
  packet[9] = (ssrc >> 16) & 0xff;
  packet[10] = (ssrc >> 8) & 0xff;
  packet[11] = ssrc & 0xff;
  packet.set(payload, 12);
  return packet;
}

/**
 * Split a V3C unit into RTP packets (NAL reassembler expects final chunk < 1200 B).
 * @param {Uint8Array} unitData
 * @param {number} unitType
 * @param {{ sequence: number, timestamp: number }} state mutable seq/timestamp
 */
export function unitToRtpPackets(unitData, unitType, state) {
  const ssrc = unitType + 1;
  const packets = [];

  if (unitData.length <= MAX_RTP_PAYLOAD) {
    packets.push(
      buildRtpPacket({
        payload: unitData,
        ssrc,
        sequenceNumber: state.sequence++,
        timestamp: state.timestamp,
      }),
    );
    return packets;
  }

  let offset = 0;
  while (offset < unitData.length) {
    const remaining = unitData.length - offset;
    let chunkSize = Math.min(MAX_RTP_PAYLOAD, remaining);
    if (remaining > MAX_RTP_PAYLOAD && remaining - chunkSize < MAX_RTP_PAYLOAD) {
      chunkSize = Math.ceil(remaining / 2);
    }

    const chunk = unitData.subarray(offset, offset + chunkSize);
    packets.push(
      buildRtpPacket({
        payload: chunk,
        ssrc,
        sequenceNumber: state.sequence++,
        timestamp: state.timestamp,
      }),
    );
    offset += chunkSize;
  }

  return packets;
}

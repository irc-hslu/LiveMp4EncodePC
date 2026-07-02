/** V3C sample stream parser (uvgVPCCenc file_writer / vpcclive_streamer --record-dir format). */

export const V3C_UNIT = {
  VPS: 0,
  AD: 1,
  OVD: 2,
  GVD: 3,
  AVD: 4,
};

const UNIT_NAMES = ["VPS", "AD", "OVD", "GVD", "AVD"];

export function unitTypeName(unitType) {
  return UNIT_NAMES[unitType] ?? `UNIT_${unitType}`;
}

/**
 * @param {Uint8Array} bytes Full .v3c file contents
 * @returns {{ precision: number, units: Array<{ unitType: number, name: string, data: Uint8Array }> }}
 */
export function parseV3cSampleStream(bytes) {
  if (bytes.byteLength < 1) {
    throw new Error("V3C file is empty");
  }

  const precision = (bytes[0] >> 5) + 1;
  let offset = 1;
  const units = [];

  while (offset + precision <= bytes.byteLength) {
    let size = 0;
    for (let i = 0; i < precision; i += 1) {
      size = (size << 8) | bytes[offset + i];
    }
    offset += precision;

    if (size === 0) {
      break;
    }
    if (offset + size > bytes.byteLength) {
      throw new Error(`Truncated V3C unit at offset ${offset} (size ${size})`);
    }

    const data = bytes.subarray(offset, offset + size);
    const unitType = data[0] >> 5;
    units.push({
      unitType,
      name: unitTypeName(unitType),
      data,
    });
    offset += size;
  }

  return { precision, units };
}

/**
 * Group units into GOFs: each AD (atlas) unit starts a new GOF.
 * @param {ReturnType<typeof parseV3cSampleStream>["units"]} units
 */
export function groupUnitsIntoGofs(units) {
  const gofs = [];
  let current = [];

  for (const unit of units) {
    if (unit.unitType === V3C_UNIT.AD && current.length > 0) {
      gofs.push(current);
      current = [];
    }
    current.push(unit);
  }

  if (current.length > 0) {
    gofs.push(current);
  }

  return gofs;
}

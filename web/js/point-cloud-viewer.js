import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import {
  createDistanceScaledPointsMaterial,
  NORMALIZED_CAMERA_DISTANCE,
} from "./distance-scaled-points-material.js";

const PCV1_MAGIC = 0x31564350; // 'PCV1' little-endian
const HEADER_BYTES = 32;
const MAX_POINTS = 500000;
const LAYER_SPREAD = 2.5;

const SHARED_POINTS_MATERIAL = createDistanceScaledPointsMaterial();

class PointCloudLayer {
  constructor(scene, layerIndex, layerCount) {
    this.layerIndex = layerIndex;
    this.layerCount = layerCount;

    this.positionBuffer = new Float32Array(MAX_POINTS * 3);
    this.colorBuffer = new Float32Array(MAX_POINTS * 3);
    this.geometry = new THREE.BufferGeometry();
    this.positionAttr = new THREE.BufferAttribute(this.positionBuffer, 3);
    this.colorAttr = new THREE.BufferAttribute(this.colorBuffer, 3);
    this.geometry.setAttribute("position", this.positionAttr);
    this.geometry.setAttribute("color", this.colorAttr);
    this.geometry.setDrawRange(0, 0);

    this.points = new THREE.Points(this.geometry, SHARED_POINTS_MATERIAL);
    this.applySpreadOffset();
    scene.add(this.points);

    this.frameNumber = -1;
    this.pointCount = 0;
    this.pending = null;
    this.stableCenter = null;
    this.stableSpan = null;
  }

  applySpreadOffset() {
    const offsetX = (this.layerIndex - (this.layerCount - 1) * 0.5) * LAYER_SPREAD;
    this.points.position.set(offsetX, 0, 0);
  }

  setLayout(layerIndex, layerCount) {
    this.layerIndex = layerIndex;
    this.layerCount = layerCount;
    this.applySpreadOffset();
  }

  dispose(scene) {
    scene.remove(this.points);
    this.geometry.dispose();
  }

  /**
   * @param {ArrayBuffer|Uint8Array} buffer PCV1 payload (relay 0x02 prefix already stripped).
   */
  updateFromPreview(buffer) {
    const bytes = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer);
    if (bytes.byteLength < HEADER_BYTES) {
      return null;
    }

    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (view.getUint32(0, true) !== PCV1_MAGIC) {
      return null;
    }

    const frameNumber = view.getUint32(4, true);
    const chunkIndex = view.getUint16(8, true);
    const chunkCount = view.getUint16(10, true);
    const origin = [
      view.getFloat32(12, true),
      view.getFloat32(16, true),
      view.getFloat32(20, true),
    ];
    const quantScale = view.getFloat32(24, true);
    const pointCount = view.getUint32(28, true);
    if (pointCount === 0 || quantScale <= 0) {
      return null;
    }

    const bytesPerPoint = 9;
    if (bytes.byteLength < HEADER_BYTES + pointCount * bytesPerPoint) {
      return null;
    }

    if (chunkIndex === 0 || !this.pending || this.pending.frameNumber !== frameNumber) {
      this.pending = {
        frameNumber,
        chunkCount,
        positions: [],
        colors: [],
      };
    }

    let offset = HEADER_BYTES;
    for (let i = 0; i < pointCount; i += 1) {
      const qx = view.getUint16(offset, true);
      offset += 2;
      const qy = view.getUint16(offset, true);
      offset += 2;
      const qz = view.getUint16(offset, true);
      offset += 2;
      const r = bytes[offset++];
      const g = bytes[offset++];
      const b = bytes[offset++];

      this.pending.positions.push(qx / quantScale + origin[0]);
      this.pending.positions.push(qy / quantScale + origin[1]);
      this.pending.positions.push(qz / quantScale + origin[2]);
      this.pending.colors.push(r / 255, g / 255, b / 255);
    }

    const totalPoints = this.pending.positions.length / 3;

    if (chunkIndex !== chunkCount - 1) {
      return { frameNumber, pointCount: totalPoints, chunkIndex, chunkCount, partial: true };
    }

    const positions = new Float32Array(this.pending.positions);
    const colors = new Float32Array(this.pending.colors);
    this.commitNormalizedCloud(positions, colors, totalPoints);
    this.frameNumber = frameNumber;
    this.pointCount = totalPoints;
    this.pending = null;

    return { frameNumber, pointCount: totalPoints, chunkIndex, chunkCount, partial: false };
  }

  commitNormalizedCloud(positions, colors, pointCount) {
    let minX = Infinity;
    let minY = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    let maxZ = -Infinity;

    for (let i = 0; i < pointCount; i += 1) {
      const x = positions[i * 3 + 0];
      const y = positions[i * 3 + 1];
      const z = positions[i * 3 + 2];
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      minZ = Math.min(minZ, z);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
      maxZ = Math.max(maxZ, z);
    }

    const cx = (minX + maxX) * 0.5;
    const cy = (minY + maxY) * 0.5;
    const cz = (minZ + maxZ) * 0.5;
    const span = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 1e-3);

    if (this.stableCenter === null || this.stableSpan === null) {
      this.stableCenter = new THREE.Vector3(cx, cy, cz);
      this.stableSpan = span;
    } else {
      const spanJump = Math.abs(span - this.stableSpan) / this.stableSpan;
      if (spanJump > 0.45) {
        this.stableCenter.set(cx, cy, cz);
        this.stableSpan = span;
      } else {
        const blend = 0.12;
        this.stableCenter.set(
          this.stableCenter.x * (1 - blend) + cx * blend,
          this.stableCenter.y * (1 - blend) + cy * blend,
          this.stableCenter.z * (1 - blend) + cz * blend,
        );
        this.stableSpan = this.stableSpan * (1 - blend) + span * blend;
      }
    }

    const scale = 2.0 / this.stableSpan;
    const n = Math.min(pointCount, MAX_POINTS);
    for (let i = 0; i < n; i += 1) {
      this.positionBuffer[i * 3 + 0] = (positions[i * 3 + 0] - this.stableCenter.x) * scale;
      this.positionBuffer[i * 3 + 1] = (positions[i * 3 + 1] - this.stableCenter.y) * scale;
      this.positionBuffer[i * 3 + 2] = (positions[i * 3 + 2] - this.stableCenter.z) * scale;
      this.colorBuffer[i * 3 + 0] = colors[i * 3 + 0];
      this.colorBuffer[i * 3 + 1] = colors[i * 3 + 1];
      this.colorBuffer[i * 3 + 2] = colors[i * 3 + 2];
    }

    this.positionAttr.needsUpdate = true;
    this.colorAttr.needsUpdate = true;
    this.geometry.setDrawRange(0, n);
    this.geometry.computeBoundingSphere();
  }
}

/**
 * Three.js RGBD point cloud viewer with distance-scaled splat shader.
 * Supports multiple simultaneous point-cloud sources (one per relay server).
 */
export class PointCloudViewer {
  constructor(container = document.body) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b0f14);

    this.camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.001, 15000);
    this.camera.position.set(0, 0, NORMALIZED_CAMERA_DISTANCE);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 0, 0);

    /** @type {Map<string, PointCloudLayer>} */
    this.layers = new Map();
    this.cameraInitialized = false;

    this.resize(window.innerWidth, window.innerHeight);
  }

  relayoutLayers() {
    const entries = [...this.layers.values()];
    const count = entries.length;
    entries.forEach((layer, index) => {
      layer.setLayout(index, count);
    });
  }

  ensureLayer(id) {
    let layer = this.layers.get(id);
    if (!layer) {
      layer = new PointCloudLayer(this.scene, this.layers.size, this.layers.size + 1);
      this.layers.set(id, layer);
      this.relayoutLayers();
    }
    return layer;
  }

  removeLayer(id) {
    const layer = this.layers.get(id);
    if (!layer) {
      return;
    }
    layer.dispose(this.scene);
    this.layers.delete(id);
    this.relayoutLayers();
    if (this.layers.size === 0) {
      this.cameraInitialized = false;
    }
  }

  removeAllLayers() {
    for (const id of [...this.layers.keys()]) {
      this.removeLayer(id);
    }
  }

  /**
   * @param {string} id Layer / server identifier.
   * @param {ArrayBuffer|Uint8Array} buffer PCV1 payload (relay 0x02 prefix already stripped).
   */
  updateFromPreview(id, buffer) {
    const layer = this.ensureLayer(id);
    const result = layer.updateFromPreview(buffer);
    if (result && !result.partial && !this.cameraInitialized) {
      this.snapCameraToCloud();
      this.cameraInitialized = true;
    }
    return result;
  }


  snapCameraToCloud() {
    this.camera.near = 0.001;
    this.camera.far = 15000;
    this.camera.updateProjectionMatrix();
    this.controls.target.set(0, 0, 0);
    this.camera.position.set(0, 0, NORMALIZED_CAMERA_DISTANCE);
    this.controls.update();
  }

  resize(width, height) {
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
    const mat = SHARED_POINTS_MATERIAL;
    if (mat instanceof THREE.ShaderMaterial && mat.uniforms.uViewportScale) {
      mat.uniforms.uViewportScale.value = height * 0.5;
    }
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

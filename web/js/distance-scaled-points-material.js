import * as THREE from "three";

/** Point sprite size for normalized radius≈1 clouds. */
export const NORMALIZED_POINT_SIZE = 0.005;
export const NORMALIZED_CAMERA_DISTANCE = 3.25;

const DEFAULT_CLIP_MIN_X = -4;
const DEFAULT_CLIP_MAX_X = 4;
const DEFAULT_CLIP_MIN_Z = -3;
const DEFAULT_CLIP_MAX_Z = 3;
const DEFAULT_DISTANCE_SCALE = 1.05;
const DEFAULT_MIN_POINT_SIZE = 0.01;

const DIST_SCALED_POINTS_VERT = /* glsl */ `
precision highp float;

uniform float uPointSize;
uniform float uDistanceScale;
uniform float uMinPointSize;
uniform float uClipMinX;
uniform float uClipMaxX;
uniform float uClipMinZ;
uniform float uClipMaxZ;
uniform float uViewportScale;

in vec3 color;
out vec3 vColor;
out float vKeep;

void main() {
  vColor = color;
  vKeep = 1.0;

  vec4 worldPos = modelMatrix * vec4(position, 1.0);
  if (worldPos.x > uClipMaxX || worldPos.x < uClipMinX || worldPos.z > uClipMaxZ || worldPos.z < uClipMinZ) {
    vKeep = 0.0;
  }

  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  float dist = length(mv.xyz);
  float base = dist * uDistanceScale * uPointSize + uMinPointSize;
  float denom = max(1e-3, -mv.z);
  gl_PointSize = base * (uViewportScale / denom);
  gl_Position = projectionMatrix * mv;
}
`;

const DIST_SCALED_POINTS_FRAG = /* glsl */ `
precision highp float;
uniform float uSplatSoftness;
in vec3 vColor;
in float vKeep;
out vec4 outColor;
void main() {
  if (vKeep < 0.5) discard;
  vec2 coord = gl_PointCoord * 2.0 - 1.0;
  float r = dot(coord, coord);
  if (r > 1.0) discard;
  float alpha = exp(-r * uSplatSoftness);
  if (alpha < 0.001) discard;
  outColor = vec4(vColor, alpha);
}
`;

/** Distance-scaled splat material (port of plylivestream StreamProvider shader). */
export function createDistanceScaledPointsMaterial() {
  const mat = new THREE.ShaderMaterial({
    glslVersion: THREE.GLSL3,
    uniforms: {
      uPointSize: { value: NORMALIZED_POINT_SIZE },
      uDistanceScale: { value: DEFAULT_DISTANCE_SCALE },
      uMinPointSize: { value: DEFAULT_MIN_POINT_SIZE },
      uClipMinX: { value: DEFAULT_CLIP_MIN_X },
      uClipMaxX: { value: DEFAULT_CLIP_MAX_X },
      uClipMinZ: { value: DEFAULT_CLIP_MIN_Z },
      uClipMaxZ: { value: DEFAULT_CLIP_MAX_Z },
      uViewportScale: { value: 100.0 },
      uSplatSoftness: { value: 1.2 },
    },
    vertexShader: DIST_SCALED_POINTS_VERT,
    fragmentShader: DIST_SCALED_POINTS_FRAG,
    transparent: false,
    depthWrite: true,
    depthTest: true,
    blending: THREE.CustomBlending,
    blendSrc: THREE.ZeroFactor,
    blendDst: THREE.ZeroFactor,
    blendEquation: THREE.MaxEquation,
  });
  mat.userData.sharedRgbdPointMaterial = true;
  return mat;
}

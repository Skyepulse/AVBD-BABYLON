import { mat3, vec3, quat, mat4 } from "gl-matrix";
import { Quaternion, Vector3 } from "@babylonjs/core";

//================================//
export function mat3Diag(x: number, y: number, z: number): mat3 {
  const out = mat3.create();
  out[0] = x; out[4] = y; out[8] = z;
  return out;
}

//================================//
export function quatFromBabylon(q: Quaternion): quat {
  const out = quat.fromValues(q.x, q.y, q.z, q.w);
  return quat.normalize(out, out);
}

//================================//
export function quatToBabylon(q: quat): Quaternion {
    return new Quaternion(q[0], q[1], q[2], q[3]);
}

//================================//
export function vec3FromBabylon(v: Vector3): vec3 {
  return vec3.fromValues(v.x, v.y, v.z);
}

//================================//
export function vec3ToBabylon(v: vec3): Vector3 {
  return new Vector3(v[0], v[1], v[2]);
}

//================================//
export function integrateQuat(q: quat, omega: vec3, dt: number, out: quat): quat {
  const ox = omega[0], oy = omega[1], oz = omega[2];
  if ((ox*ox + oy*oy + oz*oz) === 0) { quat.copy(out, q); return out; }

  // q_dot = 0.5 * [0, ω] ⊗ q
  const qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  const cx = oy*qz - oz*qy;
  const cy = oz*qx - ox*qz;
  const cz = ox*qy - oy*qx;
  const dot = ox*qx + oy*qy + oz*qz;

  const dx = 0.5 * (qw*ox + cx);
  const dy = 0.5 * (qw*oy + cy);
  const dz = 0.5 * (qw*oz + cz);
  const dw = 0.5 * (-dot);

  out[0] = qx + dx * dt;
  out[1] = qy + dy * dt;
  out[2] = qz + dz * dt;
  out[3] = qw + dw * dt;
  return quat.normalize(out, out);
}

//================================//
export function omegaFromQuatDelta(q0: quat, q1: quat, dt: number, out: vec3): vec3 {
    
  // q_delta = q1 * inverse(q0)
  const qInv = quat.invert(quat.create(), q0);
  const dq = quat.multiply(quat.create(), q1, qInv);
  quat.normalize(dq, dq);

  // Map to axis-angle
  const angle = 2 * Math.acos(Math.max(-1, Math.min(1, dq[3])));
  const s = Math.sqrt(Math.max(0, 1 - dq[3]*dq[3]));
  if (s < 1e-8 || angle < 1e-8) {
    vec3.set(out, 0, 0, 0);
  } else {
    const axis = vec3.fromValues(dq[0]/s, dq[1]/s, dq[2]/s);
    vec3.scale(out, axis, angle / dt);
  }
  return out;
}


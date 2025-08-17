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
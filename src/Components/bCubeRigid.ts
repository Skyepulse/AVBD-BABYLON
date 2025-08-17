import { Quaternion, Vector3 } from "@babylonjs/core";
import {
  Scene,
  Mesh,
  AbstractMesh,
  MeshBuilder,
  Material,
} from "@babylonjs/core";

import { mat3, vec3, mat4, quat } from "gl-matrix";
import type bForce from "./bForce";

import { mat3Diag, quatFromBabylon, vec3FromBabylon, vec3ToBabylon, quatToBabylon } from "./mathHelpers";

//================================//
class CubeRigidbody
{
    public forces: bForce[];

    public position: vec3;
    public rotation: quat;
    public rotationMatrix: mat4;

    public velocity: vec3;
    public angularVelocity: vec3;

    public previousVelocities: vec3[];
    public previousAngularVelocities: vec3[];

    public readonly size:vec3;
    public readonly mass: number;

    public linearMomentum: vec3;
    public angularMomentum: vec3;

    public InertiaTensor: mat3; // inertia tensor
    public Iinv: mat3; // inverse of inertia tensor

    public torqueAccum: vec3 = vec3.create();

    public readonly friction: number;
    public readonly detectionRadius: number;

    private _mesh?: AbstractMesh;
    private static _sharedBox?: Mesh;
    private material: Material | null;
    private renderMethod: string = "unique";

    //================================//
    public static setInstanceMaterial(material: Material): void {
        this._sharedBox = MeshBuilder.CreateBox("rb-source", { size: 1 });
        this._sharedBox.isVisible = false;
        this._sharedBox.material = material;
    }

    //================================//
    constructor(
        size: Vector3,
        density: number,
        friction: number,
        position: Vector3,
        velocity: Vector3 = new Vector3(0, 0, 0),
        angularVelocity: Vector3 = new Vector3(0, 0, 0),
        rotation: Quaternion = Quaternion.Identity(),
        material: Material | null = new Material("defaultMaterial", null),
        renderMethod: string = "unique",
    ) 
    {
        // -------- geometry / mass properties --------
        this.size = vec3FromBabylon(size);
        const volume = this.size[0] * this.size[1] * this.size[2];
        const mass = density * volume;
        this.mass = mass;
        this.friction = friction;
        this.detectionRadius = vec3.length(this.size) * 0.5;

        // render
        this.material = material;
        this.renderMethod = renderMethod;

        // -------- kinematic state --------
        this.position = vec3FromBabylon(position);
        this.rotation = quatFromBabylon(rotation);

        this.velocity = vec3FromBabylon(velocity);
        this.angularVelocity = vec3FromBabylon(angularVelocity);
        this.previousVelocities = [vec3.clone(this.velocity)];
        this.previousAngularVelocities = [vec3.clone(this.angularVelocity)];

        this.forces = [];

        // --- inertia (solid box about its center) ---
        // I0 = (1/12) m * diag( y^2+z^2, x^2+z^2, x^2+y^2 )
        const x2 = this.size[0] * this.size[0];
        const y2 = this.size[1] * this.size[1];
        const z2 = this.size[2] * this.size[2];
        const k = (1 / 12) * this.mass;
        const Ix = k * (y2 + z2);
        const Iy = k * (x2 + z2);
        const Iz = k * (x2 + y2);

        this.InertiaTensor = mat3Diag(Ix, Iy, Iz);

        // Rotation Matrix
        this.angularMomentum = vec3.create();
        this.rotationMatrix = mat4.create();
        this.Iinv = mat3.create();
        this.refreshInertiaCaches(true);

        // --- momentum ---
         this.linearMomentum = vec3.scale(vec3.create(), this.velocity, this.mass);
    }

    //================================//
    public refreshInertiaCaches(updateAngularMomentum = false): void {

        if (this.mass <= 0) { // Static
            this.InertiaTensor = mat3Diag(0,0,0);
            this.Iinv = mat3Diag(0,0,0);
            return;
        }

        // We need the rotation matrix to compute the inertia tensor
        mat4.fromQuat(this.rotationMatrix, this.rotation);

        const R = mat3.fromQuat(mat3.create(), this.rotation);
        const Rt = mat3.transpose(mat3.create(), R);

        // I0^{-1} (diagonal) — avoid divide-by-zero for static bodies
        // ( which should not happen we return early, still check).
        const I0inv = mat3Diag(
            (this.InertiaTensor[0] > 0) ? 1 / this.InertiaTensor[0] : 0,
            (this.InertiaTensor[4] > 0) ? 1 / this.InertiaTensor[4] : 0,
            (this.InertiaTensor[8] > 0) ? 1 / this.InertiaTensor[8] : 0
        );

        // Iinv_world = R * I0inv * R^T
        const tmp = mat3.multiply(mat3.create(), R, I0inv);
        mat3.multiply(this.Iinv, tmp, Rt);

        if (updateAngularMomentum) {
            // L = I_world * ω
            const IworldTmp         = mat3.multiply(mat3.create(), R, this.InertiaTensor);
            const Iworld            = mat3.multiply(mat3.create(), IworldTmp, Rt);
            const w                 = this.angularVelocity; // Omega
            const L                 = vec3.transformMat3(vec3.create(), w, Iworld);
            this.angularMomentum    = L;
        }
    }

    //================================//
    public setRotation(q: Quaternion, updateAngularMomentum = false): void {
        this.rotation = quat.clone(quatFromBabylon(q));
        this.refreshInertiaCaches(updateAngularMomentum);
    }

    //================================//
    public applyImpulse(pointWorld: Vector3, impulseWorld: Vector3): void {
        if (this.isImmovable) return;

        const pointW = vec3FromBabylon(pointWorld);
        const impulseW = vec3FromBabylon(impulseWorld);

        // p' = p + J
        vec3.add(this.linearMomentum, this.linearMomentum, impulseW);
        // v = p / m
        vec3.scale(this.velocity, this.linearMomentum, 1 / this.mass);

        // L' = L + r × J
        const r = vec3.subtract(vec3.create(), pointW, this.position);
        const dL = vec3.cross(vec3.create(), r, impulseW);
        vec3.add(this.angularMomentum, this.angularMomentum, dL); // Similar to Vector3.addInPlace

        // ω = Iinv_world * L
        const Lg = this.angularMomentum;
        const omega = vec3.transformMat3(vec3.create(), Lg, this.Iinv);
        this.angularVelocity = omega;
    }

    //================================//
    public addTorque(torqueWorld: vec3): void {
        vec3.add(this.torqueAccum, this.torqueAccum, torqueWorld);
    }

    //================================//
    public clearAccumulators(): void {
        vec3.set(this.torqueAccum, 0, 0, 0);
    }

    //================================//
    public addForceAtPoint(forceWorld: Vector3, pointWorld: Vector3): void {
        const forceW = vec3FromBabylon(forceWorld);
        const pointW = vec3FromBabylon(pointWorld);

        const r = vec3.subtract(vec3.create(), pointW, this.position);
        const tau = vec3.cross(vec3.create(), r, forceW);
        this.addTorque(tau);
    }

    //================================//
    public integrateOrientation(dt: number): void {
        const ω = this.angularVelocity;
        if (vec3.squaredLength(ω) === 0) return;

        // q_dot = 0.5 * [0, ω] ⊗ q
        const qx = this.rotation[0], qy = this.rotation[1], qz = this.rotation[2], qw = this.rotation[3];
        const ox = ω[0], oy = ω[1], oz = ω[2];

        // vector part v = (qx,qy,qz), scalar w = qw
        const cx = oy * qz - oz * qy; // ω × v
        const cy = oz * qx - ox * qz;
        const cz = ox * qy - oy * qx;
        const dot = ox * qx + oy * qy + oz * qz;

        const dx = 0.5 * (qw * ox + cx);
        const dy = 0.5 * (qw * oy + cy);
        const dz = 0.5 * (qw * oz + cz);
        const dw = 0.5 * (-dot);

        this.rotation[0] = qx + dx * dt;
        this.rotation[1] = qy + dy * dt;
        this.rotation[2] = qz + dz * dt;
        this.rotation[3] = qw + dw * dt;

        quat.normalize(this.rotation, this.rotation); // Normalize the quaternion

        // rotation changed → refresh Iinv (no need to update L here)
        this.refreshInertiaCaches(false);
    }

    //================================//
    public get isImmovable(): boolean {
        return !(this.mass > 0);
    }

    //================================//
    public clearForces(): void {
        this.forces.length = 0;
    }

    //================================//
    public addForce(f: bForce): void {
        this.forces.push(f);
    }

    //================================//
    public removeForce(f: bForce): void {
        const i = this.forces.indexOf(f);
        if (i >= 0) this.forces.splice(i, 1);
    }

    //================================//
    public draw(scene: Scene): void
    {
        if (!this._mesh)
        {
            if (this.renderMethod === "instance") {
                if (!CubeRigidbody._sharedBox) {
                    throw new Error("Shared box mesh not created. Make sure to set the instance material.");
                }
                this._mesh = CubeRigidbody._sharedBox.createInstance("rb-inst");
            } else {
                this._mesh = MeshBuilder.CreateBox("rb-unique", { size: 1 }, scene);
                this._mesh.material = this.material;
            }

            if(!this._mesh)
            {
                console.warn("Mesh not created correctly");
                return;
            }
            this._mesh.rotationQuaternion = quatToBabylon(this.rotation);
        }

        // position
        this._mesh.position.copyFrom(vec3ToBabylon(this.position));

        // rotation (Babylon quaternion already)
        this._mesh.rotationQuaternion!.copyFrom(quatToBabylon(this.rotation));

        // scale box from unit size to your dimensions
        this._mesh.scaling.copyFrom(vec3ToBabylon(this.size));

        if (this.isImmovable)
        {
            this._mesh.freezeWorldMatrix();
            return;
        }
    }
}

export default CubeRigidbody;

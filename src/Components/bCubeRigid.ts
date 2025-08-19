import { Quaternion, StandardMaterial, Vector3 } from "@babylonjs/core";
import {
  Scene,
  Mesh,
  AbstractMesh,
  MeshBuilder,
  Material,
  Color3,
  TransformNode
} from "@babylonjs/core";

import { mat3, vec3, mat4, quat } from "gl-matrix";
import type bForce from "./bForce";

import { mat3Diag, quatFromBabylon, vec3FromBabylon, vec3ToBabylon, quatToBabylon, integrateQuat } from "./mathHelpers";

//================================//
class CubeRigidbody
{
    public forces: bForce[] = [];

    public position: vec3;
    public rotation: quat;

    // Optional cache for now, not used
    public rotationMatrix: mat4;

    public velocity: vec3;
    public angularVelocity: vec3;

    public previousVelocities: vec3[];
    public previousAngularVelocities: vec3[];

    public initialPosition: vec3 = vec3.create();
    public initialRotation: quat = quat.create();
    public inertialPosition: vec3 = vec3.create();
    public inertialRotation: quat = quat.create();

    public readonly size:vec3;
    public readonly mass: number;

    public InertiaTensor: mat3; // inertia tensor I0
    public Iinv: mat3; // inverse of inertia tensor

    public forceAccum: vec3 = vec3.create();
    public torqueAccum: vec3 = vec3.create();

    public readonly friction: number;
    public readonly detectionRadius: number;

    private _root?: TransformNode;
    private _mesh?: AbstractMesh;
    private static _sharedBox?: Mesh;
    private _material: Material | null;
    private _renderMethod: string = "unique";

    private _vertexMeshes: AbstractMesh[] = [];
    private _detectionWireframe: AbstractMesh | null = null;
    private static _debugMaterial: Material | null = null;
    private static _debugWireframeMaterial: Material | null = null;
    private isDebug: boolean = false;

    //================================//
    public static setInstanceMaterial(material: Material): void {
        this._sharedBox = MeshBuilder.CreateBox("rb-source", { size: 1 });
        this._sharedBox.isVisible = false;
        this._sharedBox.material = material;
    }

    //================================//
    constructor(
        scene: Scene,
        size: Vector3,
        density: number,
        friction: number,
        position: Vector3,
        velocity: Vector3 = new Vector3(0, 0, 0),
        angularVelocity: Vector3 = new Vector3(0, 0, 0),
        rotation: Quaternion = Quaternion.Identity(),
        material: Material | null = new Material("defaultMaterial", null),
        renderMethod: "unique" | "instance" = "unique",
    ) 
    {
        // -------- geometry / mass properties --------
        this.size = vec3FromBabylon(size);
        const volume = this.size[0] * this.size[1] * this.size[2];
        const mass = density * volume;
        this.mass = mass;
        this.friction = friction;
        this.detectionRadius = 0.5 * Math.sqrt(this.size[0] ** 2 + this.size[1] ** 2 + this.size[2] ** 2);

        // render
        this._material = material;
        this._renderMethod = renderMethod;

        // -------- kinematic state --------
        this.position = vec3FromBabylon(position);
        this.rotation = quatFromBabylon(rotation);

        // -------- Derived kinematics --------
        this.velocity = vec3FromBabylon(velocity);
        this.angularVelocity = vec3FromBabylon(angularVelocity);
        this.previousVelocities = [vec3.clone(this.velocity)];
        this.previousAngularVelocities = [vec3.clone(this.angularVelocity)];

        // -------- AVBD per step poses --------
        vec3.copy(this.initialPosition, this.position);
        quat.copy(this.initialRotation, this.rotation);
        vec3.copy(this.inertialPosition, this.position);
        quat.copy(this.inertialRotation, this.rotation);

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
        this.rotationMatrix = mat4.create();
        this.Iinv = mat3.create();
        this.refreshInertiaCaches();

        // Edge Meshes
        this.initializeMesh(scene);
        this.createDebugMeshes(scene);
    }

    //================================//
    private createDebugMeshes(scene: Scene)
    {
        if (this.isImmovable) return;

        if (!CubeRigidbody._debugMaterial) 
        {
            const mat = new StandardMaterial("debugMaterial", scene);
            mat.emissiveColor = new Color3(1, 0, 0);
            mat.disableLighting = true;
            CubeRigidbody._debugMaterial = mat;
        }

        if (!CubeRigidbody._debugWireframeMaterial)
        {
            const mat = new StandardMaterial("debugWireframeMaterial", scene);
            mat.emissiveColor = new Color3(0, 0.8, 0.2);
            mat.wireframe = true;
            mat.disableLighting = true;
            CubeRigidbody._debugWireframeMaterial = mat;
        }

        this._vertexMeshes = [];
        for (let i = 0; i < 8; i++)
        {
            const edgeMesh = MeshBuilder.CreateSphere("_debugEdge" + i, { diameter: 0.3 }, scene);
            edgeMesh.material = CubeRigidbody._debugMaterial;
            edgeMesh.isVisible = false;
            this._vertexMeshes.push(edgeMesh);
        }

        this._detectionWireframe = MeshBuilder.CreateSphere("detectionWireframe", {diameter:this.detectionRadius * 2, segments: 2}, scene);
        this._detectionWireframe.material = CubeRigidbody._debugWireframeMaterial;
        this._detectionWireframe.isVisible = false;

        // Parent debug meshes to the main root transform node if it exists
        if (this._root) {
            for (const m of this._vertexMeshes) m.parent = this._root;
            this._detectionWireframe.parent = this._root;
        }

        const hx = this.size[0] / 2, hy = this.size[1] / 2, hz = this.size[2] / 2;
        const allVertexPositions = [1, -1].flatMap(ix =>
            [1, -1].flatMap(iy =>
            [1, -1].map(iz => new Vector3(ix * hx, iy * hy, iz * hz))
            )
        );

        for(let i = 0; i < allVertexPositions.length; i++) {
            this._vertexMeshes[i].position.copyFrom(allVertexPositions[i]);
        }

        this._detectionWireframe.position = new Vector3(0, 0, 0);
    }

    //================================//
    public isConstrainedTo(bodyB: CubeRigidbody): boolean
    {
        for (const force of this.forces)
        {
            if ((force.bodyA === this && force.bodyB === bodyB) || (force.bodyB === this && force.bodyA === bodyB))
                return true;
        }
        return false;
    }

    //================================//
    public toggleDebug(b: boolean)
    {
        if (this.isDebug === b || !this._detectionWireframe) return;

        this.isDebug = b;

        for(const debugMesh of this._vertexMeshes)
            debugMesh.isVisible = b;

        this._detectionWireframe.isVisible = b;
    }

    //================================//
    public refreshInertiaCaches(): void {

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
    }

    //================================//
    public setRotation(q: Quaternion): void {
        this.rotation = quat.clone(quatFromBabylon(q));
        this.refreshInertiaCaches(); // Need to update cache to stay coherent
    }

    //================================//
    public addForce(forceWorld: vec3): void {
        vec3.add(this.forceAccum, this.forceAccum, forceWorld);
    }

    //================================//
    public addTorque(torqueWorld: vec3): void {
        vec3.add(this.torqueAccum, this.torqueAccum, torqueWorld);
    }

    //================================//
    public clearAccumulators(): void {
        vec3.set(this.torqueAccum, 0, 0, 0);
        vec3.set(this.forceAccum, 0, 0, 0);
    }

    //================================//
    public addForceAtPoint(forceWorld: vec3, pointWorld: vec3): void {
        const r = vec3.subtract(vec3.create(), pointWorld, this.position);
        const tau = vec3.cross(vec3.create(), r, forceWorld);
        this.addForce(forceWorld);
        this.addTorque(tau);
    }

    //================================//
    public integrateOrientation(dt: number): void {
        integrateQuat(this.rotation, this.angularVelocity, dt, this.rotation);
    }

    //================================//
    public get isImmovable(): boolean {
        return !(this.mass > 0);
    }

    //================================//
    public getRandomEdgeWorld(): vec3 
    {
        const hx = this.size[0] * 0.5; const hy = this.size[1] * 0.5; const hz = this.size[2] * 0.5;
        const he = [hx, hy, hz];

        const edgeAxis = Math.floor(Math.random() * 3);
        const edgeNum = (Math.random() * 2 - 1) * he[edgeAxis];

        const other = [0, 1, 2].filter(a => a !== edgeAxis); // Will select axes not selected at random
        const s0 = Math.random() < 0.5 ? -1 : 1; // Pos or neg side on other axis 1
        const s1 = Math.random() < 0.5 ? -1 : 1; // Pos or neg side on other axis 2

        const localVec = vec3.fromValues(0,0,0);
        localVec[edgeAxis] = edgeNum;
        localVec[other[0]] = s0 * he[other[0]];
        localVec[other[1]] = s1 * he[other[1]];

        // Construct world position
        const R = mat3.fromQuat(mat3.create(), this.rotation);
        const rWorld = vec3.transformMat3(vec3.create(), localVec, R);
        return vec3.add(vec3.create(), this.position, rWorld); // World position of random edge
    }

    //================================//
    public addbForce(bForce: bForce): void
    {
        this.forces.push(bForce);
    }

    //================================//
    public removebForce(bForce: bForce): void
    {
        this.forces = this.forces.filter(f => f !== bForce);
    }

    //================================//
    public initializeMesh(scene: Scene): void
    {
        if(!this._root)
        {
            this._root = new TransformNode("rb-root", scene);
            this._root.rotationQuaternion = quatToBabylon(this.rotation);
            this._root.position.copyFrom(vec3ToBabylon(this.position));
        }

        if (!this._mesh)
        {
            if (this._renderMethod === "instance") {
                if (!CubeRigidbody._sharedBox) {
                    throw new Error("Shared box mesh not created. Make sure to set the instance material.");
                }
                this._mesh = CubeRigidbody._sharedBox.createInstance("rb-inst");
            } else {
                this._mesh = MeshBuilder.CreateBox("rb-unique", { size: 1 }, scene);
                this._mesh.material = this._material;
            }

            if(!this._mesh)
            {
                console.warn("Mesh not created correctly");
                return;
            }
            this._mesh.parent = this._root;
        }
    }

    //================================//
    public draw(): void
    {
        if (!this._mesh || !this._root) return;

        // position
        this._root.position.copyFrom(vec3ToBabylon(this.position));
        this._root.rotationQuaternion!.copyFrom(quatToBabylon(this.rotation));

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

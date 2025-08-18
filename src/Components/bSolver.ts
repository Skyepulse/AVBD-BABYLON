import CubeRigidbody from "./bCubeRigid";
import BabylonContext
 from "./bContext";
import { vec3, quat } from "gl-matrix";
import { omegaFromQuatDelta, integrateQuat } from "./mathHelpers";

class Solver
{
    public static instance: Solver;

    private rigidbodies: CubeRigidbody[] = [];
    private context: BabylonContext;

    public gravity = vec3.fromValues(0, -9.81, 0);
    public fixedDt: number = 1 / 60;
    public maxSubsteps: number = 2;

    private _accum: number = 0;

    private constructor(bcontext: BabylonContext)
    {
        this.context = bcontext;
    }

    public static getInstance(bcontext: BabylonContext): Solver
    {
        if (!Solver.instance)
        {
            if (!bcontext) {
                throw new Error("BabylonContext is required");
            }
            Solver.instance = new Solver(bcontext);
        }
        return Solver.instance;
    }

    public addRigidbody(rb: CubeRigidbody): void
    {
        this.rigidbodies.push(rb);
    }

    public removeRigidbody(rb: CubeRigidbody): void
    {
        const index = this.rigidbodies.indexOf(rb);
        if (index !== -1)
        {
            this.rigidbodies.splice(index, 1);
        }
    }

    public update(): void
    {
        const scene = this.context.getScene();
        const engine = this.context.getEngine();

        if (this.fixedDt != null) {
            // fixed-step with substepping
            const realDt = engine.getDeltaTime() / 1000;
            this._accum += realDt;
            let n = 0;
            while (this._accum >= this.fixedDt && n < this.maxSubsteps) {
                this.step(this.fixedDt);
                this._accum -= this.fixedDt;
                n++;
            }
        } else {
            // variable-step
            const dt = engine.getDeltaTime() / 1000;
            this.step(dt);
        }

        // draw
        for (const rb of this.rigidbodies) {
            rb.draw(scene);
        }
    }

    public applyImpulseAtCenterALL(J: vec3)
    {
        for (const rb of this.rigidbodies) {
            this.applyImpulseAtPoint(rb.position, J, rb);
        }
    }

    public applyImpulseAtRandomEdgeALL()
    {
        for (const rb of this.rigidbodies) {
            const pointW = rb.getRandomEdgeWorld();

            // Choose an impulse direction roughly perpendicular to r for stronger torque:
            const r = vec3.subtract(vec3.create(), pointW, rb.position);
            let rand = vec3.fromValues(Math.random() * 2 - 1, Math.random() * 2 - 1, Math.random() * 2 - 1);
            if (vec3.squaredLength(rand) < 1e-10) rand = vec3.fromValues(1, 0, 0);
            // make it perpendicular to r
            const proj = vec3.scale(vec3.create(), r, (vec3.dot(rand, r) / Math.max(vec3.squaredLength(r), 1e-10)));
            const dir = vec3.normalize(vec3.create(), vec3.subtract(vec3.create(), rand, proj));

            const Jmag = 2.0; // tune
            const J = vec3.scale(vec3.create(), dir, Jmag);

            this.applyImpulseAtPoint(pointW, J, rb);
        }
    }

    public applyImpulseAtPoint(pointWorld: vec3, J: vec3, rb: CubeRigidbody)
    {
        // Linear
        vec3.scaleAndAdd(rb.velocity, rb.velocity, J, 1 / rb.mass);

        // Angular
        rb.refreshInertiaCaches();
        const r = vec3.subtract(vec3.create(), pointWorld, rb.position);
        const dL = vec3.cross(vec3.create(), r, J);
        const dOmega = vec3.transformMat3(vec3.create(), dL, rb.Iinv); // Iinv is available after refresh inertia cache
        vec3.add(rb.angularVelocity, rb.angularVelocity, dOmega);
    }

    private step(dt: number): void {
        for (const rb of this.rigidbodies) {
            if (rb.isImmovable) continue;

            // POSITION BASED AVBD
            vec3.copy(rb.initialPosition, rb.position);
            quat.copy(rb.initialRotation, rb.rotation);

            // -- Translational innertial prediction : x̂ = x + v dt + (g + F/m) dt^2
            const externalAcceleration = vec3.clone(this.gravity);
            vec3.scaleAndAdd(externalAcceleration, externalAcceleration, rb.forceAccum, 1 / rb.mass);
            vec3.copy(rb.inertialPosition, rb.position);
            vec3.scaleAndAdd(rb.inertialPosition, rb.inertialPosition, rb.velocity, dt);
            vec3.scaleAndAdd(rb.inertialPosition, rb.inertialPosition, externalAcceleration, dt * dt);

            // -- Rotational innertial prediction : α = I⁻¹ τ ; ω* = ω + α dt ; q̂ = integrateQuat(q, ω*, dt)
            rb.refreshInertiaCaches();
            const alpha = vec3.transformMat3(vec3.create(), rb.torqueAccum, rb.Iinv);
            const omegaPredict = vec3.scaleAndAdd(vec3.create(), rb.angularVelocity, alpha, dt);
            integrateQuat(rb.rotation, omegaPredict, dt, rb.inertialRotation);

            // -- Adaptive Warmstarting
            const prevVelocity = rb.previousVelocities.length > 0 ? rb.previousVelocities[rb.previousVelocities.length - 1] : rb.velocity;
            const accel = vec3.scale(vec3.create(), vec3.subtract(vec3.create(), rb.velocity, prevVelocity), 1 / Math.max(dt, 1e-6));
            const gLen = vec3.length(this.gravity);
            let accelWeight = 0;
            if (gLen > 0) {
                const gDir = vec3.scale(vec3.create(), this.gravity, 1 / gLen);
                const accelExt = vec3.dot(accel, gDir);
                accelWeight = Math.max(0, Math.min(1, accelExt / gLen));
                if (!isFinite(accelWeight)) accelWeight = 0;
            }

            vec3.copy(rb.position, rb.initialPosition);
            vec3.scaleAndAdd(rb.position, rb.position, rb.velocity, dt);
            vec3.scaleAndAdd(rb.position, rb.position, this.gravity, accelWeight * dt * dt);
        }

        // -- SOLVER LOOP --
        for (const rb of this.rigidbodies)
        {
            if (rb.isImmovable) continue;
            vec3.copy(rb.position, rb.inertialPosition);
            quat.copy(rb.rotation, rb.inertialRotation);
        }

        // -- End of step velocities:
        for (const rb of this.rigidbodies) {
            rb.previousVelocities.push(vec3.clone(rb.velocity));
            if (rb.previousVelocities.length > 10) rb.previousVelocities.shift();
            rb.previousAngularVelocities.push(vec3.clone(rb.angularVelocity));
            if (rb.previousAngularVelocities.length > 10) rb.previousAngularVelocities.shift();

            if (!rb.isImmovable) {
                // linear v = (x - x_initial) / dt
                vec3.scale(
                rb.velocity,
                vec3.subtract(vec3.create(), rb.position, rb.initialPosition),
                1 / dt
                );
                // angular ω from quaternion delta
                omegaFromQuatDelta(rb.initialRotation, rb.rotation, dt, rb.angularVelocity);
            }

            // clear per-step external accumulators
            rb.clearAccumulators();
        }
    }
}

export default Solver;
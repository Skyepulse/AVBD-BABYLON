import CubeRigidbody from "./bCubeRigid";
import BabylonContext
 from "./bContext";
import { vec3 } from "gl-matrix";

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

    private step(dt: number): void {
        for (const rb of this.rigidbodies) {
            if (rb.isImmovable) continue;

            // =========================
            // LINEAR (momentum-centric)
            // =========================
            rb.previousVelocities.push(vec3.clone(rb.velocity));
            if (rb.previousVelocities.length > 10) rb.previousVelocities.shift();

            // p += (m g) dt
            vec3.scaleAndAdd(rb.linearMomentum, rb.linearMomentum, this.gravity, rb.mass * dt);
            // v = p / m
            vec3.scale(rb.velocity, rb.linearMomentum, 1 / rb.mass);
            // x += v dt
            vec3.scaleAndAdd(rb.position, rb.position, rb.velocity, dt);

            // -------------------------
            // ANGULAR (torque → L → ω)
            // -------------------------
            rb.previousAngularVelocities.push(vec3.clone(rb.angularVelocity));
            if (rb.previousAngularVelocities.length > 10) rb.previousAngularVelocities.shift();

            // 1) L ← L + τ dt
            if (vec3.squaredLength(rb.torqueAccum) > 0) {
                vec3.scaleAndAdd(rb.angularMomentum, rb.angularMomentum, rb.torqueAccum, dt);
            }

            // 2) ω = Iinv_world · L
            rb.angularVelocity = vec3.transformMat3(vec3.create(), rb.angularMomentum, rb.Iinv);

            // 3) integrate quaternion using ω
            rb.integrateOrientation(dt);

            // Clear per-step accumulators
            rb.clearAccumulators();
        }
    }
}

export default Solver;
import CubeRigidbody from "./bCubeRigid";

class bForce
{
    public bodyA: CubeRigidbody | null = null;
    public bodyB: CubeRigidbody | null = null;

    public constructor(bodyA: CubeRigidbody | null, bodyB: CubeRigidbody | null)
    {
        this.bodyA = bodyA;
        this.bodyB = bodyB;

        this.bodyA?.addbForce(this);
        this.bodyB?.addbForce(this);
    }
}

export default bForce;
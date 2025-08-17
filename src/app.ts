import './style.css'
import "@babylonjs/core/Debug/debugLayer";
import "@babylonjs/inspector";
import { Vector3, Quaternion, StandardMaterial, Color3 } from '@babylonjs/core';

import { initializeCanvas } from "./Components/bCanvas";
import BabylonContext from './Components/bContext';
import Solver from './Components/bSolver';
import CubeRigidbody from './Components/bCubeRigid';
import { vec3ToBabylon } from './Components/mathHelpers';
import { vec3 } from 'gl-matrix';

class App {
    constructor() {

        // create the canvas html element and attach it to the webpage
        const canvas = initializeCanvas();

        // initialize babylon scene and engine
        const bContext = new BabylonContext(canvas);
        const solver = Solver.getInstance(bContext);

        const scene = bContext.getScene();

        // Green
        const floorMaterial = new StandardMaterial("floorMaterial", bContext.getScene());
        floorMaterial.diffuseColor = new Color3(0, 1, 0);
        const floor = new CubeRigidbody(
            new Vector3(10, 1, 10),  // size
            0,                       // density → mass 0
            0.8,                     // friction (unused for now)
            new Vector3(0, -0.5, 0),  // position (top surface at y=0)
            new Vector3(0, 0, 0),     // velocity
            new Vector3(0, 0, 0),     // angularVelocity
            Quaternion.Identity(),    // rotation
            floorMaterial,            // material
            "unique"                  // renderMethod
        );

        // dynamic cube: density > 0
        // Red
        const cubeMaterial = new StandardMaterial("cubeMaterial", bContext.getScene());
        cubeMaterial.diffuseColor = new Color3(1, 0, 0);
        CubeRigidbody.setInstanceMaterial(cubeMaterial);

        const cube1 = new CubeRigidbody(
            new Vector3(1, 1, 1),
            1,                        // density
            0.5,
            new Vector3(0, 3, 0),     // start 3 units above floor
            new Vector3(0, 10, 0),    // velocity
            new Vector3(0, 0, 0),    // angularVelocity
            Quaternion.Identity(),    // rotation
            null,             // material
            "instance"                // renderMethod
        );

        const cube2 = new CubeRigidbody(
            new Vector3(1, 1, 1),
            1,                        // density
            0.5,
            new Vector3(2, 3, 0),     // start 3 units above floor
            new Vector3(0, 10, 0),    // velocity
            new Vector3(0, 0, 0),    // angularVelocity
            Quaternion.Identity(),    // rotation
            null,             // material
            "instance"                // renderMethod
        );

        solver.addRigidbody(floor);
        solver.addRigidbody(cube1);
        solver.addRigidbody(cube2);

        bContext.setZoom(50);

        window.addEventListener("keydown", (e) => {
            // Upward impulse at center (instant jump)
            if (e.code === "Space") {
                const center = vec3ToBabylon(cube1.position);               // world point (center)
                const J = new Vector3(0, cube1.mass * 20, 0);                 // tweak magnitude
                cube1.applyImpulse(center, J);
            }

            // Torque-y impulse at a random edge point
            if (e.key.toLowerCase() === "r") {
                // Pick a random edge in LOCAL space: two axes at ±half, one axis = 0
                const hx = cube1.size[0] * 0.5;
                const hy = cube1.size[1] * 0.5;
                const hz = cube1.size[2] * 0.5;

                const zeroAxis = Math.floor(Math.random() * 3); // 0=x,1=y,2=z
                const sx = Math.random() < 0.5 ? -1 : 1;
                const sy = Math.random() < 0.5 ? -1 : 1;
                const sz = Math.random() < 0.5 ? -1 : 1;

                const localEdge = vec3.fromValues(
                    zeroAxis === 0 ? 0 : sx * hx,
                    zeroAxis === 1 ? 0 : sy * hy,
                    zeroAxis === 2 ? 0 : sz * hz
                );

                // Transform edge to WORLD space: p = x + R * localEdge
                const offsetW = vec3.create();
                vec3.transformQuat(offsetW, localEdge, cube1.rotation);
                const pointW = vec3.create();
                vec3.add(pointW, cube1.position, offsetW);

                // Tangential impulse direction: t ⟂ radius
                const n = vec3.normalize(vec3.create(), offsetW);
                const arbitrary = Math.abs(n[1]) < 0.9 ? vec3.fromValues(0, 1, 0) : vec3.fromValues(1, 0, 0);
                const tangent = vec3.normalize(vec3.create(), vec3.cross(vec3.create(), n, arbitrary));

                // Impulse magnitude (tweak to taste)
                const Jmag = cube1.mass * 2;
                const Jw = vec3.scale(vec3.create(), tangent, Jmag);

                // Apply off-center impulse → adds angular + some linear impulse
                cube1.applyImpulse(vec3ToBabylon(pointW), vec3ToBabylon(Jw));
            }
            });

        scene.onBeforeRenderObservable.add(() => solver.update());
    }
}
new App();
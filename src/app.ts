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

        scene.onBeforeRenderObservable.add(() => solver.update());

        // APPLY IMPULSE WITH SPACE, RANDOM TORQUE WITH R ON SELECTED EDGE (DEBUG DRAW SELECTED EDGE)

        window.addEventListener("keydown", (ev) => {

            if (ev.code === "Space") {
                ev.preventDefault(); // Apparently avoids scrolling

                const JMagnitude = 5.0;
                const J = vec3.fromValues(0,JMagnitude,0);
                solver.applyImpulseAtCenterALL(J);
            }

            if (ev.key === "r" || ev.key === "R") {
                solver.applyImpulseAtRandomEdgeALL();
            }
        });
    }
}
new App();
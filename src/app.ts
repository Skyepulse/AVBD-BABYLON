import './style.css'
import "@babylonjs/core/Debug/debugLayer";
import "@babylonjs/inspector";
import { Engine, Scene, ArcRotateCamera, Vector3, HemisphericLight, Mesh, MeshBuilder } from "@babylonjs/core";

import { initializeCanvas } from "./Components/bCanvas";
import BabylonContext from './Components/bContext';

class App {
    constructor() {

        // create the canvas html element and attach it to the webpage
        const canvas = initializeCanvas();

        // initialize babylon scene and engine
        const bContext = new BabylonContext(canvas);
        const scene = bContext.getScene();

        var camera: ArcRotateCamera = new ArcRotateCamera("Camera", Math.PI / 2, Math.PI / 2, 2, Vector3.Zero(), scene);
        camera.attachControl(canvas, true);
        var light1: HemisphericLight = new HemisphericLight("light1", new Vector3(1, 1, 0), scene);
        var sphere: Mesh = MeshBuilder.CreateSphere("sphere", { diameter: 1 }, scene);
    }
}
new App();
import { Engine, Scene, ArcRotateCamera, Vector3, HemisphericLight, Mesh, MeshBuilder, Camera } from "@babylonjs/core";

/**
 * Babylon.js context wrapper.
 */
class BabylonContext {

    private bScene: Scene;
    private bEngine: Engine;
    private renderCallbacks: Array<() => void> = [];

    private camera: ArcRotateCamera;
    private light: HemisphericLight;

    constructor(canvas: HTMLCanvasElement) {
        this.bEngine = new Engine(canvas, true);
        this.bScene = new Scene(this.bEngine);

        this.camera =  new ArcRotateCamera("Camera", Math.PI / 2, Math.PI / 2, 2, Vector3.Zero(), this.bScene);
        this.camera.attachControl(canvas, true);

        this.light = new HemisphericLight("light1", new Vector3(1, 1, 0), this.bScene);

        // enable/disable the Inspector
        window.addEventListener("keydown", (ev) => {
            // Shift+Ctrl+Alt+I
            if (ev.shiftKey && ev.ctrlKey && ev.altKey && ev.keyCode === 73) {
                if (this.bScene.debugLayer.isVisible()) {
                    this.bScene.debugLayer.hide();
                } else {
                    this.bScene.debugLayer.show();
                }
            }
        });

        this.initRenderLoop();
    }

    /**
     * Gets the Babylon.js scene.
     * @returns The Babylon.js scene.
     */
    public getScene(): Scene {
        return this.bScene;
    }

    /**
     * Gets the Babylon.js engine.
     * @returns The Babylon.js engine.
     */
    public getEngine(): Engine {
        return this.bEngine;
    }

    /**
     * Adds a callback to the render loop.
     * @param callback The callback to add.
     */
    public addToRenderLoop(callback: () => void): void {
        this.renderCallbacks.push(callback);
    }

    /**
     * Removes a callback from the render loop.
     * @param callback The callback to remove.
     */
    public removeFromRenderLoop(callback: () => void): void {
        const index = this.renderCallbacks.indexOf(callback);
        if (index > -1) {
            this.renderCallbacks.splice(index, 1);
        }
    }

    /**
     * Initializes the render loop.
     */
    private initRenderLoop(): void {
        this.bEngine.runRenderLoop(() => {
            this.bScene.render();
            this.renderCallbacks.forEach(callback => callback());
        });
    }

    public setZoom(level: number): void {
        const minZoom = 0.1;
        const maxZoom = 100;
        this.camera.radius = Math.min(Math.max(level, minZoom), maxZoom);
    }
}

export default BabylonContext;
import { Engine, Scene, ArcRotateCamera, Vector3, HemisphericLight, Mesh, MeshBuilder } from "@babylonjs/core";

/**
 * Babylon.js context wrapper.
 */
class BabylonContext {
    private bScene: Scene;
    private bEngine: Engine;
    private renderCallbacks: Array<() => void> = [];


    constructor(canvas: HTMLCanvasElement) {
        this.bEngine = new Engine(canvas, true);
        this.bScene = new Scene(this.bEngine);

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
}

export default BabylonContext;
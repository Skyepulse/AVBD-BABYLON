
/**
 * Initializes the Babylon.js canvas.
 * @returns The created HTMLCanvasElement.
 */
export function initializeCanvas()
{
    const canvas : HTMLCanvasElement = document.createElement("canvas");

    canvas.style.width = "100%";
    canvas.style.height = "100%";
    canvas.id = "BabylonCanvas";

    document.body.appendChild(canvas);

    return canvas;
}
// Playground worker: boots Pyodide, installs the mechlib + manifold3d wheels,
// executes the shared gallery demos module, and regenerates parts on demand.
// Messages in:  {type:"init", pyodideVersion, wheelUrls, demosUrl}
//               {type:"generate", id, demo, params}
// Messages out: {type:"status", stage}
//               {type:"ready"}
//               {type:"result", id, ms, meshes:[{name, positions, indices, color}]}
//               {type:"error", id?, message}

let pyodide = null;
let runDemo = null;

function status(stage) {
  self.postMessage({ type: "status", stage });
}

async function init(config) {
  try {
    status("loading python runtime");
    importScripts(`https://cdn.jsdelivr.net/pyodide/v${config.pyodideVersion}/full/pyodide.js`);
    pyodide = await loadPyodide({
      indexURL: `https://cdn.jsdelivr.net/pyodide/v${config.pyodideVersion}/full/`
    });

    status("loading numpy + shapely");
    await pyodide.loadPackage(["numpy", "shapely", "micropip"]);

    status("installing mechlib wheels");
    const micropip = pyodide.pyimport("micropip");
    await micropip.install(["trimesh==4.12.2", ...config.wheelUrls]);

    status("loading demo definitions");
    const demosSource = await (await fetch(config.demosUrl, { cache: "no-store" })).text();
    pyodide.globals.set("DEMOS_SOURCE", demosSource);
    pyodide.runPython(`
import json
import numpy as np

NS = {}
exec(DEMOS_SOURCE, NS)

def run_demo(name, kwargs_json):
    parts = NS[name](**json.loads(kwargs_json))
    out = []
    for part_name, mesh, color in parts:
        out.append((
            part_name,
            np.ascontiguousarray(mesh.vertices, dtype=np.float32).tobytes(),
            np.ascontiguousarray(mesh.faces, dtype=np.uint32).tobytes(),
            [int(c) for c in color],
        ))
    return out
`);
    runDemo = pyodide.globals.get("run_demo");
    self.postMessage({ type: "ready" });
  } catch (error) {
    self.postMessage({ type: "error", message: `runtime boot failed: ${error.message}` });
  }
}

function generate(request) {
  if (!runDemo) {
    self.postMessage({ type: "error", id: request.id, message: "runtime not ready" });
    return;
  }
  try {
    const started = performance.now();
    const proxy = runDemo(request.demo, JSON.stringify(request.params));
    const parts = proxy.toJs();
    proxy.destroy();

    const meshes = [];
    const transfers = [];
    for (const [name, posBytes, idxBytes, color] of parts) {
      const positions = new Float32Array(
        posBytes.buffer.slice(posBytes.byteOffset, posBytes.byteOffset + posBytes.byteLength)
      );
      const indices = new Uint32Array(
        idxBytes.buffer.slice(idxBytes.byteOffset, idxBytes.byteOffset + idxBytes.byteLength)
      );
      meshes.push({ name, positions, indices, color });
      transfers.push(positions.buffer, indices.buffer);
    }
    self.postMessage(
      { type: "result", id: request.id, ms: performance.now() - started, meshes },
      transfers
    );
  } catch (error) {
    const message = String(error.message || error).split("\n").slice(-3).join(" ").trim();
    self.postMessage({ type: "error", id: request.id, message });
  }
}

self.onmessage = (event) => {
  const data = event.data;
  if (data.type === "init") init(data);
  else if (data.type === "generate") generate(data);
};

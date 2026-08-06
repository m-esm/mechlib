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

    // scipy is not optional: mechlib.meshutil imports scipy.spatial.cKDTree, so
    // without it roughly a fifth of the catalogue (including the `frustum`
    // primitive) fails on ModuleNotFoundError. matplotlib is loaded lazily
    // instead, because it is only needed by the two text parts and costs
    // several seconds.
    status("loading numpy + scipy + shapely");
    await pyodide.loadPackage(["numpy", "scipy", "networkx", "shapely", "micropip"]);

    status("installing mechlib wheels");
    const micropip = pyodide.pyimport("micropip");
    await micropip.install(["trimesh==4.12.2", ...config.wheelUrls]);

    status("loading demo definitions");
    const demosSource = await (await fetch(config.demosUrl, { cache: "no-store" })).text();
    pyodide.globals.set("DEMOS_SOURCE", demosSource);
    pyodide.runPython(`
import json
import traceback
import numpy as np

NS = {}
exec(DEMOS_SOURCE, NS)

def run_demo(name, kwargs_json):
    """Return (error_text, parts). Never raises.

    A Python exception thrown across the PyProxy boundary leaves the proxy
    unusable, so one bad parameter combination used to break every later
    regeneration until the page was reloaded. Catching here keeps the runtime
    alive: a failing demo reports its error and the next one still works.
    """
    try:
        parts = NS[name](**json.loads(kwargs_json))
        out = []
        for part_name, mesh, color in parts:
            out.append((
                part_name,
                np.ascontiguousarray(mesh.vertices, dtype=np.float32).tobytes(),
                np.ascontiguousarray(mesh.faces, dtype=np.uint32).tobytes(),
                [int(c) for c in color],
            ))
        return (None, out)
    except Exception as exc:
        traceback.print_exc()
        return ("%s: %s" % (type(exc).__name__, exc), None)
`);
    runDemo = pyodide.globals.get("run_demo");
    self.postMessage({ type: "ready" });
  } catch (error) {
    self.postMessage({ type: "error", message: `runtime boot failed: ${error.message}` });
  }
}

const MISSING_MODULE = /No module named '([A-Za-z0-9_.]+)'/;

async function generate(request, retried) {
  if (!runDemo) {
    self.postMessage({ type: "error", id: request.id, message: "runtime not ready" });
    return;
  }
  try {
    const started = performance.now();
    const proxy = runDemo(request.demo, JSON.stringify(request.params));
    if (!proxy || typeof proxy.toJs !== "function") {
      throw new Error("python runtime returned no result; reload the page");
    }
    const [pyError, pyParts] = proxy.toJs();
    proxy.destroy();
    if (pyError) {
      // A demo may reach a module Pyodide has not loaded yet (matplotlib for
      // the text parts). Load it once and retry rather than failing the part.
      const missing = MISSING_MODULE.exec(pyError);
      if (missing && !retried) {
        const packageName = missing[1].split(".")[0];
        status(`loading ${packageName}`);
        let loaded = false;
        try {
          await pyodide.loadPackage(packageName);
          loaded = true;
        } catch (loadError) {
          try {
            // Pure-Python packages outside the Pyodide distribution only come
            // through micropip.
            await pyodide.pyimport("micropip").install(packageName);
            loaded = true;
          } catch (pipError) {
            loaded = false;
          }
        }
        if (!loaded) {
          self.postMessage({
            type: "error",
            id: request.id,
            message: `${pyError} (could not install ${packageName} in the browser runtime)`
          });
          return;
        }
        return generate(request, true);
      }
      self.postMessage({ type: "error", id: request.id, message: pyError });
      return;
    }
    const parts = pyParts;

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
    const raw = String(error.message || error);
    // A GEOS/C++ abort is not catchable in Python and leaves the interpreter
    // dead: every later request would fail with an opaque proxy error. Tell the
    // page so it can throw this worker away and boot a fresh one.
    const fatal = /fatal error|NoGilError|returned no result|Attempted to use PyProxy|CppException|TopologyException|geos::/i.test(raw);
    const message = raw.split("\n").slice(-3).join(" ").trim();
    self.postMessage({ type: "error", id: request.id, message, fatal });
  }
}

self.onmessage = (event) => {
  const data = event.data;
  if (data.type === "init") init(data);
  else if (data.type === "generate") generate(data);
};

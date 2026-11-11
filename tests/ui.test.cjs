const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const { JSDOM } = require("jsdom");

const root = path.join(__dirname, "../src/manual_calibration/static");
const html = fs.readFileSync(path.join(root, "index.html"), "utf8");
const script = fs.readFileSync(path.join(root, "app.js"), "utf8");
const pause = (ms = 200) => new Promise((resolve) => setTimeout(resolve, ms));

function setup() {
  const requests = [];
  const dom = new JSDOM(html, {
    url: "http://localhost:8765",
    runScripts: "outside-only",
  });
  const w = dom.window;
  w.AbortController = AbortController;
  w.URL.createObjectURL = () => "blob:preview";
  w.URL.revokeObjectURL = () => {};
  w.fetch = async (url, options = {}) => {
    const request = options.body ? JSON.parse(options.body) : null;
    requests.push({ url, request });
    const cid =
      new URL(url, w.location.href).searchParams.get("camera") || "first";
    const rid =
      new URL(url, w.location.href).searchParams.get("recording") || "one";
    const settings = {
      camera_id: cid,
      recording_id: rid,
      camera_frame: cid + "_body",
      reference_frame: "base",
      camera_mount: { x: 0, y: 0, z: 1.5, roll: 0, pitch: 0, yaw: 0 },
      intrinsics: {
        camera_model: "plumb_bob",
        fx_fy_cx_cy: [500, 500, 400, 250],
        distortion: [0, 0, 0, 0, 0],
      },
      opacity: 0.78,
      point_spacing: 4,
    };
    const info = {
      camera_id: request?.settings.camera_id,
      recording_id: request?.settings.recording_id,
      points: 100,
      offset_seconds: 0,
      scan_offsets_ms: { lidar: 0 },
    };
    return {
      ok: true,
      json: async () => ({
        settings,
        frame_count: 12,
        lidar_count: 1,
        cameras: [
          { id: "first", label: "First" },
          { id: "second", label: "Second" },
        ],
        recordings: [
          { id: "one", label: "One" },
          { id: "two", label: "Two" },
        ],
      }),
      headers: { get: () => JSON.stringify(info) },
      blob: async () => new Blob(["preview"]),
    };
  };
  w.eval(script);
  return { dom, w, requests, $: (id) => w.document.getElementById(id) };
}

test("editing, lens validation, keyboard guards and per-camera edit persistence", async () => {
  const { dom, w, requests, $ } = setup();
  try {
    await pause();
    assert.equal($("image").getAttribute("src"), "blob:preview");
    $("pose-x").value = "0.25";
    $("pose-x").dispatchEvent(new w.Event("input"));
    await pause();
    assert.equal(requests.at(-1).request.settings.camera_mount.x, 0.25);
    $("pose-x").dispatchEvent(
      new w.KeyboardEvent("keydown", { key: "n", bubbles: true }),
    );
    assert.equal($("timeline").value, "0");
    w.document.body.dispatchEvent(
      new w.KeyboardEvent("keydown", { key: "n", bubbles: true }),
    );
    assert.equal($("timeline").value, "1");
    $("recording").value = "two";
    $("recording").dispatchEvent(new w.Event("change"));
    await pause();
    assert.equal($("pose-x").value, "0.25");
    assert.equal($("timeline").value, "0");
    $("camera").value = "second";
    $("camera").dispatchEvent(new w.Event("change"));
    await pause();
    assert.equal($("pose-x").value, "0");
    $("camera").value = "first";
    $("camera").dispatchEvent(new w.Event("change"));
    await pause();
    assert.equal($("pose-x").value, "0.25");
    $("model").value = "fisheye";
    $("model").dispatchEvent(new w.Event("change"));
    await pause();
    assert.match($("error").textContent, /4 finite coefficients/);
    assert.equal($("image").hasAttribute("src"), false);
    $("resetLens").click();
    await pause();
    assert.equal($("pose-x").value, "0.25");
    assert.equal($("image").getAttribute("src"), "blob:preview");
  } finally {
    dom.window.close();
  }
});

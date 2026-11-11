"use strict";

const $ = (id) => document.getElementById(id);
const clone = (value) => JSON.parse(JSON.stringify(value));
const axes = ["x", "y", "z", "roll", "pitch", "yaw"];
const token = document.querySelector('meta[name="calibration-token"]').content;
const edits = new Map();
let settings,
  baseline,
  count = 0,
  frame = 0,
  playing = false,
  busy = false;
let timer,
  controller,
  activeUrl,
  currentJob,
  selectionVersion = 0;

async function post(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      "X-Calibration-Token": token,
    },
    body: JSON.stringify(body),
  });
  if (!response.ok) throw new Error((await response.json()).error);
  return response;
}

function numbers(text) {
  return text
    .split(",")
    .map((value) => (value.trim() === "" ? NaN : Number(value)));
}

function validateLens() {
  const lens = settings.intrinsics;
  const n = lens.camera_model === "plumb_bob" ? 5 : 4;
  if (
    lens.fx_fy_cx_cy.length !== 4 ||
    !lens.fx_fy_cx_cy.every(Number.isFinite) ||
    lens.fx_fy_cx_cy[0] <= 0 ||
    lens.fx_fy_cx_cy[1] <= 0
  ) {
    throw new Error("Enter finite fx, fy, cx, cy with positive focal lengths.");
  }
  if (lens.distortion.length !== n || !lens.distortion.every(Number.isFinite)) {
    throw new Error(
      `The selected lens model requires ${n} finite coefficients.`,
    );
  }
}

function lensFields() {
  const lens = settings.intrinsics;
  $("model").value =
    lens.camera_model === "equidistant" ? "fisheye" : lens.camera_model;
  $("focal").value = lens.fx_fy_cx_cy.join(", ");
  $("distortion").value = lens.distortion.join(", ");
  $("lensHelp").textContent =
    lens.camera_model === "plumb_bob"
      ? "Order: k1, k2, p1, p2, k3."
      : "Fisheye order: k1, k2, k3, k4. Use coefficients calibrated for this model.";
}

function showSettings() {
  $("axes").replaceChildren();
  axes.forEach((axis, i) => {
    const div = document.createElement("div");
    div.className = "axis";
    const title = document.createElement("div");
    title.className = "axis-title";
    const label = document.createElement("label");
    label.textContent = `${axis} (${i < 3 ? "m" : "degrees"})`;
    label.htmlFor = `pose-${axis}`;
    const number = document.createElement("input");
    number.type = "number";
    number.id = `pose-${axis}`;
    number.step = i < 3 ? 0.005 : 0.05;
    number.value = settings.camera_mount[axis];
    const slider = document.createElement("input");
    slider.type = "range";
    const origin = baseline.camera_mount[axis];
    slider.min = origin - (i < 3 ? 1 : 10);
    slider.max = origin + (i < 3 ? 1 : 10);
    slider.step = number.step;
    slider.value = number.value;
    slider.setAttribute("aria-label", `${axis} adjustment slider`);
    number.oninput = () => {
      if (number.value !== "" && Number.isFinite(Number(number.value))) {
        settings.camera_mount[axis] = Number(number.value);
        slider.value = number.value;
        schedule();
      }
    };
    slider.oninput = () => {
      settings.camera_mount[axis] = Number(slider.value);
      number.value = slider.value;
      schedule();
    };
    const note = document.createElement("small");
    note.textContent = `Baseline: ${origin}`;
    title.append(label, number);
    div.append(title, slider, note);
    $("axes").append(div);
  });
  lensFields();
  $("opacity").value = settings.opacity;
  $("spacing").value = String(settings.point_spacing);
  schedule();
}

function options(id, entries, selected) {
  $(id).replaceChildren();
  for (const entry of entries) {
    const option = document.createElement("option");
    option.value = entry.id;
    option.textContent = entry.label;
    $(id).append(option);
  }
  $(id).value = selected;
}

async function select(recording, camera) {
  const version = ++selectionVersion;
  if (settings) edits.set(settings.camera_id, clone(settings));
  playing = false;
  $("play").textContent = "Play";
  if (controller) controller.abort();
  busy = false;
  try {
    const query = new URLSearchParams();
    if (recording) query.set("recording", recording);
    if (camera) query.set("camera", camera);
    const response = await fetch(`/config?${query}`);
    if (!response.ok) throw new Error((await response.json()).error);
    const data = await response.json();
    if (version !== selectionVersion) return;
    baseline = clone(data.settings);
    settings = clone(data.settings);
    const previous = edits.get(settings.camera_id);
    if (previous && previous.reference_frame === settings.reference_frame) {
      for (const key of [
        "camera_mount",
        "intrinsics",
        "opacity",
        "point_spacing",
      ]) {
        settings[key] = clone(previous[key]);
      }
    }
    count = data.frame_count;
    frame = 0;
    options("camera", data.cameras, settings.camera_id);
    options("recording", data.recordings, settings.recording_id);
    $("reference").textContent =
      `Mount relative to ${settings.reference_frame}. ${data.lidar_count} fixed LiDAR stream(s). Camera body: ${settings.camera_frame}.`;
    $("timeline").max = count - 1;
    $("timeline").value = frame;
    showSettings();
  } catch (error) {
    if (version === selectionVersion) $("error").textContent = error.message;
  }
}

function schedule() {
  clearTimeout(timer);
  timer = setTimeout(preview, 140);
}

async function preview() {
  if (!settings) return;
  if (controller) controller.abort();
  const local = new AbortController();
  controller = local;
  const version = selectionVersion;
  busy = true;
  $("error").textContent = "";
  try {
    validateLens();
    const response = await fetch("/preview", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Calibration-Token": token,
      },
      body: JSON.stringify({
        settings,
        frame,
        mode: $("mode").value,
        width: Number($("width").value),
      }),
      signal: local.signal,
    });
    if (!response.ok) throw new Error((await response.json()).error);
    const info = JSON.parse(response.headers.get("X-Overlay-Info"));
    const blob = await response.blob();
    if (version !== selectionVersion || local !== controller) return;
    const url = URL.createObjectURL(blob);
    $("image").src = url;
    if (activeUrl) URL.revokeObjectURL(activeUrl);
    activeUrl = url;
    $("time").textContent =
      `Frame ${frame + 1} / ${count} · ${info.offset_seconds.toFixed(2)} s`;
    const offsets = Object.entries(info.scan_offsets_ms)
      .map(([id, value]) => `${id}: ${value.toFixed(1)} ms`)
      .join(" · ");
    $("status").textContent =
      `${info.points} display points · scan − image: ${offsets} · no motion compensation`;
  } catch (error) {
    if (error.name !== "AbortError" && local === controller) {
      $("error").textContent = error.message;
      $("image").removeAttribute("src");
      if (activeUrl) URL.revokeObjectURL(activeUrl);
      activeUrl = null;
    }
  } finally {
    if (local === controller) busy = false;
  }
}

function move(value) {
  frame = Math.max(0, Math.min(count - 1, value));
  $("timeline").value = frame;
  schedule();
}

function download(blob, name) {
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 3000);
}

$("camera").onchange = () => select($("recording").value, $("camera").value);
$("recording").onchange = () => select($("recording").value, $("camera").value);
$("reset").onclick = () => {
  settings = clone(baseline);
  showSettings();
};
$("resetLens").onclick = () => {
  settings.intrinsics = clone(baseline.intrinsics);
  lensFields();
  schedule();
};
$("model").onchange = () => {
  settings.intrinsics.camera_model = $("model").value;
  settings.intrinsics.distortion = [];
  lensFields();
  schedule();
};
$("focal").oninput = () => {
  settings.intrinsics.fx_fy_cx_cy = numbers($("focal").value);
  schedule();
};
$("distortion").oninput = () => {
  settings.intrinsics.distortion = numbers($("distortion").value);
  schedule();
};
$("opacity").oninput = () => {
  settings.opacity = Number($("opacity").value);
  schedule();
};
$("spacing").onchange = () => {
  settings.point_spacing = Number($("spacing").value);
  schedule();
};
$("mode").onchange = schedule;
$("width").onchange = schedule;
$("timeline").oninput = () => move(Number($("timeline").value));
$("back").onclick = () => move(frame - 1);
$("next").onclick = () => move(frame + 1);
$("last").onclick = () => move(count - 1);
$("play").onclick = () => {
  playing = !playing;
  $("play").textContent = playing ? "Pause" : "Play";
};
setInterval(() => {
  if (playing && !busy) {
    if (frame >= count - 1) {
      playing = false;
      $("play").textContent = "Play";
    } else move(frame + 1);
  }
}, 180);
document.addEventListener("keydown", (event) => {
  if (
    event.defaultPrevented ||
    event.isComposing ||
    event.ctrlKey ||
    event.metaKey ||
    event.altKey ||
    event.repeat
  )
    return;
  if (
    event.target instanceof Element &&
    event.target.closest(
      'input,textarea,select,[contenteditable]:not([contenteditable="false"])',
    )
  )
    return;
  const button = { " ": "play", p: "back", n: "next" }[event.key.toLowerCase()];
  if (button && settings) {
    event.preventDefault();
    $(button).click();
  }
});
$("download").onclick = async () => {
  try {
    validateLens();
    const response = await post("/export", { settings });
    download(await response.blob(), "manual-calibration.json");
  } catch (error) {
    $("error").textContent = error.message;
  }
};
$("load").onclick = () => $("file").click();
$("file").onchange = async () => {
  try {
    if (!$("file").files.length) return;
    const value = JSON.parse(await $("file").files[0].text());
    if (
      value.camera_frame !== settings.camera_frame ||
      value.reference_frame !== settings.reference_frame
    ) {
      throw new Error(
        "Select the matching camera and reference frame before loading these values.",
      );
    }
    const candidate = {
      ...settings,
      camera_mount: value.camera_mount,
      intrinsics: value.intrinsics,
    };
    await post("/export", { settings: candidate });
    settings = clone(candidate);
    showSettings();
  } catch (error) {
    $("error").textContent = error.message;
  } finally {
    $("file").value = "";
  }
};
$("saveImage").onclick = async () => {
  if (activeUrl)
    download(await (await fetch(activeUrl)).blob(), "manual-overlay.jpg");
};
$("video").onclick = async () => {
  try {
    validateLens();
    const response = await post("/video", { settings });
    currentJob = (await response.json()).job;
    $("video").disabled = true;
    $("job").textContent = "Rendering selected recording…";
  } catch (error) {
    $("job").textContent = error.message;
  }
};
setInterval(async () => {
  if (!currentJob) return;
  try {
    const jobs = await (await fetch("/jobs")).json();
    const job = jobs[currentJob];
    if (job?.status === "complete") {
      const link = document.createElement("a");
      link.href = job.url;
      link.textContent = "Download MP4";
      link.download = "manual-overlay.mp4";
      $("job").replaceChildren(link);
      currentJob = null;
      $("video").disabled = false;
    } else if (job?.status === "failed") {
      $("job").textContent = job.error;
      currentJob = null;
      $("video").disabled = false;
    }
  } catch (error) {
    $("job").textContent = error.message;
  }
}, 2000);
select();

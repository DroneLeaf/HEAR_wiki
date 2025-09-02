# Embedded Python IDE

This document describes the WebApp UX requirements of LeafScript and the two approaches for running Python code inside its web-based IDE.  
It lays out the **UX requirements**, covers **implementation specifications** (frontend + backend) and **pros & cons** of each.

---

## UX Requirements

**1. IDE**

It needs to have an embedded IDE where users are able to easily script custom missions using LeafSDK mission planning library.

**2. Mission Validation and Loading**

It will have two buttons for validating a planned mission and loading a validated mission. Load mission button will be hidden until the mission is validated.

**3. User Library Section**

The users will be able to save their custom missions in their user library for later use. They should also be able to browse their user library and load a saved mission in their current session.

**4. Pre-Defined Mission Library Section** 

It will have a library section where we store predefined missions, for example, delivery, surveying, etc.

**5. Mission Visualization**

Loaded missions will be visualized to the user with a graph diagram. We need a hidable visualization section that shows this visualization.

---

## Approach 1 — In-Browser with Pyodide

### Implementation Specifications

**Frontend (IDE layer)**  
1. **Editor**:  
   - Use **Monaco Editor** (`monaco-editor` NPM package).  
   - Initialize with language `"python"`.  
   - Add Run button.

2. **Linting / Type Checking**:  
   - Use **Pyright** (browser build) inside a Web Worker.  
   - Feed Monaco text → receive diagnostics → set Monaco markers.

3. **Formatter / Linter**:  
   - Use **Ruff WASM** in a separate Web Worker.  
   - Trigger lint on idle, format on save.

4. **Runtime**:  
   - Load **Pyodide** with `<script src=".../pyodide.js">`.  
   - Use `await loadPyodide({ indexURL: "..." })`.  
   - Install pure-Python packages via `micropip`.  
   - Optionally persist environment using IndexedDB.

**Backend**  
- None required for execution.  
- Only serves static assets (Pyodide dist) or uses CDN.  
- Optional logging API if run history is needed.

### Pros
- ✅ **Zero backend infra** (no servers required).  
- ✅ **Safe for your infra** (code runs in user's browser).  
- ✅ **Offline capable** once cached.  
- ✅ **Fast startup** for small scripts and pure-Python packages.  
- ✅ **Scales automatically** (each user runs locally).  

### Cons
- ❌ **Limited packages** (only pure-Python + Pyodide prebuilt stack).  
- ❌ **No native extensions** (PyTorch, OpenCV, TensorFlow not supported).  
- ❌ **Performance overhead** (~2–10× slower than native CPython).  
- ❌ **Resource constrained** (browser memory, no GPU).  
- ❌ **Ephemeral** (packages reset unless persisted manually).

---

## Approach 2 — Isolated Containers on AWS

### Implementation Specifications

**Frontend (IDE layer)**  
1. **Editor**:  
   - Monaco Editor with Pyright (lint) and Ruff (format).  
   - Run button → send code to backend.

2. **API Contract**:  
   - `POST /api/run { code: str, requirements: list[str], timeout_sec: int }`.  
   - Response: `{ stdout: str, stderr: str, exit: int, runtime: "server" }`.

**Backend (Control Plane API)**  
1. **Language & Framework**: FastAPI or Flask.  
2. **Runner Container**:  
   - Base: `python:3.12-slim`.  
   - Install `uv` for fast dependency installs.  
   - Entrypoint script:  
     - Create venv → install requirements → run code → return JSON.  
3. **AWS Infra**:  
   - **ECS Fargate** task (simple, serverless) or **EKS Job** (for GPU/advanced).  
   - Runner image stored in **ECR**.  
   - Packages via **AWS CodeArtifact** (PyPI) or **S3 wheelhouse**.  
   - Logs streamed to **CloudWatch**.  
4. **Security**:  
   - Run as non-root user.  
   - Read-only root FS, drop all capabilities, `no-new-privileges`.  
   - Network default-deny egress (allow only package source).  
   - Resource quotas: CPU/memory/pid/time/output caps.

### Pros
- ✅ **Full PyPI support** (native extensions allowed).  
- ✅ **Native performance** (no WASM overhead).  
- ✅ **GPU support** possible (EKS GPU nodes).  
- ✅ **Scalable** (many containers/jobs in parallel).  
- ✅ **Persistent options** (cached wheels, prebuilt environments).  
- ✅ **Observability** (CloudWatch logs, metrics, quotas).  

### Cons
- ❌ **Infrastructure complexity** (ECS/EKS setup, IAM, networking).  
- ❌ **Costs** (pay per container runtime).  
- ❌ **Latency** (cold starts 1–10s).  
- ❌ **Security responsibility** (must harden sandbox properly).  

---

# Summary

- **Use Pyodide** → for instant, offline, pure-Python or common scientific libs.  
- **Use AWS containers** → for full PyPI, performance, or GPU.  

---

# Minimal Working Example: In-Browser IDE

This code implements a browser-based Python IDE powered by Pyodide and the Monaco editor. When the page loads, it initializes a Pyodide runtime in a Web Worker to keep the UI responsive, preloads scientific packages, and silently installs **LeafSDK** (along with its dependency ``networkx``) in the background. The editor lets users write and execute Python code directly in the browser, with ``stdout``/``stderr`` output streamed live into a console panel. A status bar shows the readiness of both the Python runtime and LeafSDK, while controls are provided to run code, stop/restart the runtime, and clear output. This setup enables interactive Python development—including LeafSDK functionality—completely client-side, without any server.

```html
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Pyodide IDE (LeafSDK silent install)</title>
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <style>
    :root { --bg:#0b0f14; --fg:#cbd5e1; --ok:#22c55e; --err:#ef4444; --warn:#eab308; }
    * { box-sizing: border-box; }
    body { margin:0; font:14px system-ui, -apple-system, Segoe UI, Roboto, sans-serif; }
    header { padding:10px 14px; border-bottom:1px solid #eee; display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
    header button { padding:8px 10px; border:1px solid #ddd; border-radius:8px; cursor:pointer; background:#fff; }
    header button:disabled { opacity:.6; cursor:not-allowed; }
    .pill { background:#f4f4f5; padding:4px 8px; border-radius:999px; border:1px solid #e5e7eb; }
    #app { display:grid; grid-template-rows: 1fr 220px; height: calc(100vh - 70px); }
    #editor { width:100%; height:100%; }
    #log { white-space:pre-wrap; background:var(--bg); color:var(--fg); padding:10px; overflow:auto; }
    .ok{color:var(--ok);} .err{color:var(--err);} .warn{color:var(--warn);}
  </style>
  <!-- Monaco loader -->
  <script src="https://unpkg.com/monaco-editor@0.49.0/min/vs/loader.js"></script>
</head>
<body>
  <header>
    <span class="pill" id="status">status: loading…</span>
    <span class="pill" id="leafsdk">LeafSDK: pending</span>
    <button id="btn-run" disabled>Run (Ctrl/⌘+Enter)</button>
    <button id="btn-stop" disabled>Stop</button>
    <button id="btn-clear">Clear Output</button>
  </header>

  <div id="app">
    <div id="editor"></div>
    <div id="log"></div>
  </div>

  <script>
    // ---------- utilities ----------
    const logEl = document.getElementById('log');
    const statusEl = document.getElementById('status');
    const leafsdkEl = document.getElementById('leafsdk');
    const btnRun = document.getElementById('btn-run');
    const btnStop = document.getElementById('btn-stop');
    const btnClear = document.getElementById('btn-clear');

    const log = (m, cls='') => { const d=document.createElement('div'); if(cls) d.className=cls; d.textContent=m; logEl.appendChild(d); logEl.scrollTop=logEl.scrollHeight; };
    const setStatus = (s) => statusEl.textContent = 'status: ' + s;
    const setLeafStatus = (s) => leafsdkEl.textContent = 'LeafSDK: ' + s;

    // ---------- worker (classic) ----------
    function createWorker() {
      const workerCode = `
        let ready = false;
        let leafsdkReady = false;
        let pipesSuppressed = false;

        function send(type, payload={}){ postMessage({type, ...payload}); }

        function suppressLogs(enable) {
          pipesSuppressed = enable;
          if (enable) {
            pyodide.setStdout({ batched: (s)=>{} });
            pyodide.setStderr({ batched: (s)=>{} });
          } else {
            pyodide.setStdout({ batched: (s)=>send('stdout',{msg:s}) });
            pyodide.setStderr({ batched: (s)=>send('stderr',{msg:s}) });
          }
        }

        async function init(){
          try{
            send('status',{msg:'loading…'});
            importScripts('https://cdn.jsdelivr.net/pyodide/v0.28.2/full/pyodide.js');
            self.pyodide = await loadPyodide({ indexURL: 'https://cdn.jsdelivr.net/pyodide/v0.28.2/full' });
            // default pipes (visible)
            pyodide.setStdout({ batched: (s)=>send('stdout',{msg:s}) });
            pyodide.setStderr({ batched: (s)=>send('stderr',{msg:s}) });
            ready = true;
            const pyver = pyodide.runPython('import sys; sys.version.split()[0]');
            send('ready', {py: pyver, pyo:'0.28.2'});
            // Immediately start a QUIET background install flow for LeafSDK
            quietInstallLeafSDK();
          }catch(e){
            send('error',{msg:'failed to load pyodide: '+e});
          }
        }

        async function quietInstallLeafSDK(){
          if (!ready || leafsdkReady) return;
          // Temporarily silence stdout/stderr to keep terminal clean
          suppressLogs(true);
          try {
            send('leaf_status',{msg:'installing…'});
            // Preload packages quietly (includes networkx required by LeafSDK)
            await pyodide.loadPackage(['micropip','numpy','scipy','lxml','networkx']);
            // Silent install (no prints from Python)
            await pyodide.runPythonAsync(\`
import micropip
await micropip.install("LeafSDK==0.1.10", deps=False)
import leafsdk
\`);
            leafsdkReady = true;
            send('leaf_ready',{});
          } catch (e) {
            send('leaf_error',{msg:String(e)});
          } finally {
            // Restore visible pipes
            suppressLogs(false);
          }
        }

        onmessage = async (ev)=>{
          const {cmd} = ev.data||{};
          if (cmd==='init') return init();
          if (!ready) return send('error',{msg:'runtime not ready'});
          if (cmd==='run'){
            try{
              send('log',{msg:'[RUN] executing…'});
              const value = await pyodide.runPythonAsync(ev.data.code||'');
              let jsVal = value;
              try{ if(value && typeof value.toJs==='function') jsVal = value.toJs({dict_converter:Object.fromEntries}); }catch{}
              send('result',{value: jsVal});
              send('done',{});
            }catch(e){ send('error',{msg:String(e)}); }
          }
          if (cmd==='stop'){ /* no-op in worker; host recreates */ }
        };
      `;
      const blob = new Blob([workerCode], {type:'application/javascript'});
      return new Worker(URL.createObjectURL(blob)); // classic worker
    }

    let worker = createWorker();
    let isReady = false;
    let busy = false;
    let leafsdkReady = false;

    const setBusy = (b) => {
      busy = b;
      btnRun.disabled = !(isReady && !busy);   // allow running even if LeafSDK isn't needed
      btnStop.disabled = !(isReady && busy);
    };

    worker.onmessage = (ev)=>{
      const {type, msg, value, py, pyo} = ev.data;
      if (type==='status') setStatus(msg);
      else if (type==='ready'){
        isReady = true; setStatus('ready (Python '+py+', Pyodide '+pyo+')');
        setBusy(false);
        // show that LeafSDK is being handled in background
        setLeafStatus('installing…');
      }
      else if (type==='leaf_status'){ setLeafStatus(msg); }
      else if (type==='leaf_ready'){ leafsdkReady = true; setLeafStatus('ready ✓'); }
      else if (type==='leaf_error'){ setLeafStatus('error'); log('LeafSDK install error: ' + msg, 'err'); }
      else if (type==='stdout'){ log(msg); }
      else if (type==='stderr'){ log(msg, 'err'); }
      else if (type==='log'){ log(msg); }
      else if (type==='error'){ log(msg, 'err'); setBusy(false); }
      else if (type==='result'){
        const pretty=(typeof value==='object')?JSON.stringify(value,null,2):String(value);
        log('⟵ result:\n'+pretty,'ok');
      } else if (type==='done'){ setBusy(false); log('[OK] done','ok'); }
    };

    // init worker (starts Pyodide; LeafSDK install runs silently in background)
    worker.postMessage({cmd:'init'});
    setStatus('loading…'); setLeafStatus('pending'); setBusy(true);

    // ---------- Monaco editor ----------
    let editor;
    require.config({ paths: { 'vs': 'https://unpkg.com/monaco-editor@0.49.0/min/vs' }});
    require(['vs/editor/editor.main'], function () {
      const key='py-ide-content';
      const initialCode = `print("Hello from Pyodide!")
import sys
print("LeafSDK is imported!" if "leafsdk" in sys.modules else "LeafSDK not imported yet")
# The last expression is returned as the result:
sys.version`;
      editor = monaco.editor.create(document.getElementById('editor'), {
        value: localStorage.getItem(key) || initialCode,
        language:'python', automaticLayout:true, theme:'vs-dark', fontSize:14, minimap:{enabled:false}
      });
      editor.onDidChangeModelContent(()=>localStorage.setItem(key, editor.getValue()));
      editor.addCommand(monaco.KeyMod.CtrlCmd | monaco.KeyCode.Enter, () => runCode());
    });

    // ---------- UI actions ----------
    function runCode(){
      if(!isReady || busy) return;
      setBusy(true);
      log('[RUN] — start —','warn');
      worker.postMessage({cmd:'run', code: editor.getValue()});
    }
    btnRun.onclick = runCode;

    btnStop.onclick = () => {
      if (!isReady) return;
      worker.terminate(); isReady=false; setBusy(false); leafsdkReady=false;
      log('[WARN] execution stopped; restarting runtime…','warn');
      worker = createWorker();
      worker.onmessage = (e)=>{ const {type,msg,value,py,pyo}=e.data;
        if (type==='status') setStatus(msg);
        else if (type==='ready'){ isReady=true; setStatus('ready (Python '+py+', Pyodide '+pyo+')'); setBusy(false); setLeafStatus('installing…'); }
        else if (type==='leaf_status'){ setLeafStatus(msg); }
        else if (type==='leaf_ready'){ leafsdkReady=true; setLeafStatus('ready ✓'); }
        else if (type==='leaf_error'){ setLeafStatus('error'); log('LeafSDK install error: ' + msg, 'err'); }
        else if (type==='stdout'){ log(msg); } else if (type==='stderr'){ log(msg,'err'); }
        else if (type==='log'){ log(msg); } else if (type==='error'){ log(msg,'err'); setBusy(false); }
        else if (type==='result'){ const pretty=(typeof value==='object')?JSON.stringify(value,null,2):String(value); log('⟵ result:\n'+pretty,'ok'); }
        else if (type==='done'){ setBusy(false); log('[OK] done','ok'); }
      };
      worker.postMessage({cmd:'init'}); setStatus('loading…'); setLeafStatus('pending');
    };

    btnClear.onclick = () => { logEl.textContent = ''; };
  </script>
</body>
</html>
```
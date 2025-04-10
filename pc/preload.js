const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("electronAPI", {
  runAnalysis: () => ipcRenderer.invoke("run-analysis"),
  onControllerData: (callback) =>
    ipcRenderer.on("controller-data", (event, data) => callback(data)),
});
const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("node:path");
const { execFile } = require("child_process");
const { SerialPort, ReadlineParser } = require("serialport");

let mainWindow;

const createWindow = () => {
  initSerialPort();
  mainWindow = new BrowserWindow({
    width: 1200,
    height: 1000,
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
    },
  });
  mainWindow.loadFile("index.html");
};

function initSerialPort() {
  const portPath = "/dev/cu.usbmodem101"; // Менять порт!!!
  const port = new SerialPort({ path: portPath, baudRate: 115200 }, (err) => {
    if (err) {
      console.error("Ошибка открытия порта:", err.message);
    }
  });

  const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));
  parser.on("data", (line) => {
    try {
      const controllerData = JSON.parse(line);
      if (mainWindow && mainWindow.webContents) {
        mainWindow.webContents.send("controller-data", controllerData);
        // console.log(controllerData);
      }
    } catch (error) {
      console.error("Ошибка парсинга данных с контроллера:", error);
    }
  });
}

app.whenReady().then(() => {
  createWindow();
  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

ipcMain.handle("run-analysis", async () => {
  return new Promise((resolve, reject) => {
    const pythonExecutable = "/Users/zabojeb/.pyenv/shims/python";
    const scriptPath = app.isPackaged
      ? path.join(process.resourcesPath, "app.asar.unpacked", "analyst.py")
      : path.join(__dirname, "analyst.py");

    execFile(
      pythonExecutable,
      [scriptPath],
      { shell: true },
      (error, stdout, stderr) => {
        if (error) {
          console.error("Ошибка запуска python-скрипта:", error);
          reject(error);
          return;
        }
        const analysisWindow = new BrowserWindow({
          width: 1200,
          height: 1000,
        });
        analysisWindow.loadURL(
          "data:text/html;charset=utf-8," + encodeURIComponent(stdout)
        );
        resolve("success");
      }
    );
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});
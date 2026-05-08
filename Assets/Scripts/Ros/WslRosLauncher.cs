using System;
using System.Diagnostics;
using UnityEngine;

public class WslRosLauncher : MonoBehaviour
{
    [Header("WSL")]
    public string distroName = "Ubuntu";

    [Header("ROS")]
    public string scriptPath = "/home/reece/rugged_rover_ws/launch_unity_sim.sh";

    [Header("Debug")]
    public bool launchOnStart = true;
    public bool stopOnQuit = true;
    public bool showWslWindow = true;

    private Process rosProcess;

    void Start()
    {
        if (!launchOnStart)
        {
            UnityEngine.Debug.Log("[WslRosLauncher] launchOnStart is false. ROS launch skipped.");
            return;
        }

        StartRosLaunch();
    }

    void OnApplicationQuit()
    {
        if (stopOnQuit)
        {
            StopRosLaunch();
        }
    }

    public void StartRosLaunch()
    {
        if (rosProcess != null && !rosProcess.HasExited)
        {
            UnityEngine.Debug.LogWarning(
                $"[WslRosLauncher] ROS launch already running. PID: {rosProcess.Id}");
            return;
        }

        string arguments = $"-d {distroName} -- bash -lc \"{scriptPath}\"";

        UnityEngine.Debug.Log("[WslRosLauncher] Starting ROS launch script.");
        UnityEngine.Debug.Log($"[WslRosLauncher] Distro: {distroName}");
        UnityEngine.Debug.Log($"[WslRosLauncher] Script: {scriptPath}");
        UnityEngine.Debug.Log($"[WslRosLauncher] Full WSL arguments: {arguments}");

        try
        {
            ProcessStartInfo startInfo = new ProcessStartInfo
            {
                FileName = "wsl.exe",
                Arguments = arguments,
                UseShellExecute = showWslWindow,
                CreateNoWindow = !showWslWindow,
                RedirectStandardOutput = !showWslWindow,
                RedirectStandardError = !showWslWindow
            };

            rosProcess = new Process
            {
                StartInfo = startInfo,
                EnableRaisingEvents = true
            };

            rosProcess.Exited += OnRosProcessExited;

            if (!showWslWindow)
            {
                rosProcess.OutputDataReceived += (sender, args) =>
                {
                    if (!string.IsNullOrWhiteSpace(args.Data))
                    {
                        UnityEngine.Debug.Log($"[ROS stdout] {args.Data}");
                    }
                };

                rosProcess.ErrorDataReceived += (sender, args) =>
                {
                    if (!string.IsNullOrWhiteSpace(args.Data))
                    {
                        UnityEngine.Debug.LogWarning($"[ROS stderr] {args.Data}");
                    }
                };
            }

            bool started = rosProcess.Start();

            if (!started)
            {
                UnityEngine.Debug.LogError("[WslRosLauncher] Failed to start wsl.exe process.");
                return;
            }

            UnityEngine.Debug.Log($"[WslRosLauncher] ROS launch process started. PID: {rosProcess.Id}");

            if (!showWslWindow)
            {
                rosProcess.BeginOutputReadLine();
                rosProcess.BeginErrorReadLine();
            }
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"[WslRosLauncher] Exception while starting ROS launch: {ex}");
        }
    }

    public void StopRosLaunch()
    {
        if (rosProcess == null)
        {
            UnityEngine.Debug.Log("[WslRosLauncher] No ROS process to stop.");
            return;
        }

        try
        {
            if (rosProcess.HasExited)
            {
                UnityEngine.Debug.Log(
                    $"[WslRosLauncher] ROS process already exited with code {rosProcess.ExitCode}.");
                CleanupProcess();
                return;
            }

            UnityEngine.Debug.Log($"[WslRosLauncher] Stopping ROS launch process. PID: {rosProcess.Id}");

            rosProcess.Kill();

            UnityEngine.Debug.Log("[WslRosLauncher] Kill signal sent to ROS launch process.");
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"[WslRosLauncher] Exception while stopping ROS launch: {ex}");
        }
        finally
        {
            CleanupProcess();
        }
    }

    private void OnRosProcessExited(object sender, EventArgs e)
    {
        if (rosProcess == null)
            return;

        try
        {
            UnityEngine.Debug.LogWarning(
                $"[WslRosLauncher] ROS launch process exited. PID: {rosProcess.Id}, ExitCode: {rosProcess.ExitCode}");
        }
        catch
        {
            UnityEngine.Debug.LogWarning("[WslRosLauncher] ROS launch process exited.");
        }
    }

    private void CleanupProcess()
    {
        if (rosProcess == null)
            return;

        rosProcess.Exited -= OnRosProcessExited;
        rosProcess.Dispose();
        rosProcess = null;

        UnityEngine.Debug.Log("[WslRosLauncher] ROS process reference cleaned up.");
    }
}

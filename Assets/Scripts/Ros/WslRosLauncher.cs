using System;
using System.Diagnostics;
using UnityEngine;

public class WslRosLauncher : MonoBehaviour
{
    public enum LaunchTarget
    {
        Wsl,
        Ssh
    }

    [Header("Launch Target")]
    public LaunchTarget launchTarget = LaunchTarget.Wsl;

    [Header("WSL")]
    public string distroName = "Ubuntu";

    [Header("Local WSL ROS")]
    public string scriptPath = "/home/reece/rugged_rover_ws/launch_unity_sim.sh";

    [Header("Remote SSH ROS")]
    public string sshHost = "ros-laptop";
    public string remoteLaunchCommand =
        "pkill -f default_server_endpoint || true; pkill -f ros_tcp_endpoint || true; nohup /home/reece/rugged_rover_ws/launch_unity_sim.sh > /tmp/rugged_rover_unity_sim.log 2>&1 & echo started";
    public string remoteStopCommand =
        "pkill -f launch_unity_sim.sh || true; pkill -f default_server_endpoint || true; pkill -f ros_tcp_endpoint || true";

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

        ProcessStartInfo startInfo = CreateStartInfo();

        UnityEngine.Debug.Log("[WslRosLauncher] Starting ROS launch script.");
        UnityEngine.Debug.Log($"[WslRosLauncher] Launch target: {launchTarget}");
        UnityEngine.Debug.Log($"[WslRosLauncher] Command: {startInfo.FileName} {startInfo.Arguments}");

        try
        {
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
                UnityEngine.Debug.LogError(
                    $"[WslRosLauncher] Failed to start {startInfo.FileName} process.");
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

                if (launchTarget == LaunchTarget.Ssh && !string.IsNullOrWhiteSpace(remoteStopCommand))
                {
                    RunRemoteStopCommand();
                }

                CleanupProcess();
                return;
            }

            UnityEngine.Debug.Log($"[WslRosLauncher] Stopping ROS launch process. PID: {rosProcess.Id}");

            if (launchTarget == LaunchTarget.Ssh && !string.IsNullOrWhiteSpace(remoteStopCommand))
            {
                RunRemoteStopCommand();
            }

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

    private ProcessStartInfo CreateStartInfo()
    {
        string fileName;
        string arguments;

        if (launchTarget == LaunchTarget.Ssh)
        {
            fileName = "ssh.exe";
            arguments = $"{sshHost} bash -lc {QuoteForProcessArgument(BashSingleQuote(remoteLaunchCommand))}";
        }
        else
        {
            fileName = "wsl.exe";
            arguments = $"-d {distroName} -- bash -lc {QuoteForProcessArgument(BashSingleQuote(scriptPath))}";
        }

        return new ProcessStartInfo
        {
            FileName = fileName,
            Arguments = arguments,
            UseShellExecute = showWslWindow,
            CreateNoWindow = !showWslWindow,
            RedirectStandardOutput = !showWslWindow,
            RedirectStandardError = !showWslWindow
        };
    }

    private void RunRemoteStopCommand()
    {
        try
        {
            string arguments =
                $"{sshHost} bash -lc {QuoteForProcessArgument(BashSingleQuote(remoteStopCommand))}";

            ProcessStartInfo stopInfo = new ProcessStartInfo
            {
                FileName = "ssh.exe",
                Arguments = arguments,
                UseShellExecute = false,
                CreateNoWindow = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true
            };

            Process stopProcess = Process.Start(stopInfo);
            if (stopProcess == null)
            {
                UnityEngine.Debug.LogWarning("[WslRosLauncher] Failed to start remote stop process.");
                return;
            }

            stopProcess.WaitForExit(5000);

            UnityEngine.Debug.Log(
                $"[WslRosLauncher] Remote stop command sent. ExitCode: {stopProcess.ExitCode}");

            stopProcess.Dispose();
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogWarning(
                $"[WslRosLauncher] Failed to run remote stop command: {ex.Message}");
        }
    }

    private string BashSingleQuote(string value)
    {
        return $"'{value.Replace("'", "'\\''")}'";
    }

    private string QuoteForProcessArgument(string value)
    {
        return $"\"{value.Replace("\"", "\\\"")}\"";
    }
}

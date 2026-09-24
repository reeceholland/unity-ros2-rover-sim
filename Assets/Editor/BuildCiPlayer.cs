using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;

public static class BuildCiPlayer
{
    [MenuItem("Tools/CI/Build Linux Player")]
    public static void BuildLinux()
    {
        if (!BuildPipeline.IsBuildTargetSupported(BuildTargetGroup.Standalone, BuildTarget.StandaloneLinux64))
            throw new BuildFailedException("Install Linux Build Support (Mono) for this Unity version in Unity Hub.");
        const string scene = "Assets/Scenes/CiLidarWorld.unity";
        if (!File.Exists(scene)) throw new BuildFailedException("Create CiLidarWorld first.");
        var output = Environment.GetEnvironmentVariable("CI_PLAYER_OUTPUT");
        if (string.IsNullOrEmpty(output)) output = "Builds/CI/Linux/CiRover.x86_64";
        Directory.CreateDirectory(Path.GetDirectoryName(output));
        var target = UnityEditor.Build.NamedBuildTarget.Standalone;
        var oldBackend = PlayerSettings.GetScriptingBackend(target);
        try
        {
            PlayerSettings.SetScriptingBackend(target, ScriptingImplementation.Mono2x);
            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = new[] { scene },
                locationPathName = output,
                target = BuildTarget.StandaloneLinux64,
                options = BuildOptions.None
            });
            if (report.summary.result != BuildResult.Succeeded)
                throw new BuildFailedException("CI Linux build failed: " + report.summary.result);
            UnityEngine.Debug.Log("CI_PLAYER_BUILT " + Path.GetFullPath(output));
        }
        finally
        {
            PlayerSettings.SetScriptingBackend(target, oldBackend);
        }
    }
}

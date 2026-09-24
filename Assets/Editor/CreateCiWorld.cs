using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;

public static class CreateCiWorld
{
    const string ScenePath = "Assets/Scenes/CiLidarWorld.unity";

    [InitializeOnLoadMethod]
    static void CreateWhenImported()
    {
        EditorApplication.delayCall += () =>
        {
            if (!EditorApplication.isPlayingOrWillChangePlaymode && !File.Exists(ScenePath)) Create();
        };
    }

    [MenuItem("Tools/CI/Create Lidar World")]
    public static void Create()
    {
        if (File.Exists(ScenePath))
        {
            Debug.Log("CI world already exists: " + ScenePath);
            return;
        }
        var previousScene = SceneManager.GetActiveScene();
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
        SceneManager.SetActiveScene(scene);
        Directory.CreateDirectory("Assets/Materials/CI");
        AssetDatabase.Refresh();
        var ground = Material("Ground", new Color(0.22f, 0.27f, 0.31f));
        var wall = Material("Walls", new Color(0.72f, 0.77f, 0.80f));
        var obstacle = Material("Obstacles", new Color(0.95f, 0.48f, 0.12f));
        Box("Floor", new Vector3(0, -0.1f, 0), new Vector3(12, 0.2f, 12), ground);
        Box("North wall", new Vector3(0, 1, 6), new Vector3(12.4f, 2, 0.2f), wall);
        Box("South wall", new Vector3(0, 1, -6), new Vector3(12.4f, 2, 0.2f), wall);
        Box("East wall", new Vector3(6, 1, 0), new Vector3(0.2f, 2, 12), wall);
        Box("West wall", new Vector3(-6, 1, 0), new Vector3(0.2f, 2, 12), wall);
        Box("Box obstacle", new Vector3(2, 0.5f, 2), new Vector3(1, 1, 1), obstacle);
        Box("Long obstacle", new Vector3(-3, 0.5f, 1), new Vector3(0.6f, 1, 2), obstacle);
        Box("Low obstacle", new Vector3(1, 0.3f, -3), new Vector3(1.5f, 0.6f, 0.7f), obstacle);

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Rover/A4WD3_Rover.prefab");
        if (prefab == null) throw new System.InvalidOperationException("Rover prefab missing");
        var rover = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
        rover.transform.SetPositionAndRotation(new Vector3(0, 0.02f, 0), Quaternion.identity);
        var remapper = rover.GetComponent<UnityTopicRemapper>();
        if (remapper != null) remapper.Remap = false;
        foreach (var lidar in rover.GetComponentsInChildren<UnityLidarPublisher>(true))
        {
            for (var t = lidar.transform; t != rover.transform; t = t.parent) t.gameObject.SetActive(true);
            lidar.enabled = true;
            lidar.scanTopic = "/scan_raw";
            lidar.rangeMax = 15;
            lidar.fieldOfViewFDegrees = 360;
            lidar.sampleCount = 360;
            lidar.drawDebugRays = false;
            lidar.showHitMarkers = false;
        }
        foreach (var camera in rover.GetComponentsInChildren<Camera>(true)) camera.gameObject.SetActive(false);
        foreach (var ouster in rover.GetComponentsInChildren<UnityOusterOs1Publisher>(true)) ouster.enabled = false;
        foreach (var t in rover.GetComponentsInChildren<Transform>(true))
            if (t.name == "Ouster OS-1") t.gameObject.SetActive(false);
        PrefabUtility.RecordPrefabInstancePropertyModifications(rover.transform);
        foreach (var component in rover.GetComponentsInChildren<Component>(true))
            if (component != null) PrefabUtility.RecordPrefabInstancePropertyModifications(component);
        foreach (var t in rover.GetComponentsInChildren<Transform>(true))
            PrefabUtility.RecordPrefabInstancePropertyModifications(t.gameObject);

        var ros = new GameObject("ROS Connection").AddComponent<ROSConnection>();
        ros.RosIPAddress = "127.0.0.1";
        ros.RosPort = 10000;
        ros.ConnectOnStart = true;
        new GameObject("Simulation Clock").AddComponent<UnityClockPublisher>();
        var light = new GameObject("Sun").AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.5f;
        light.transform.rotation = Quaternion.Euler(50, -30, 0);
        var view = new GameObject("Overview Camera").AddComponent<Camera>();
        view.tag = "MainCamera";
        view.transform.position = new Vector3(10, 12, -10);
        view.transform.LookAt(Vector3.zero);
        view.nearClipPlane = 0.1f;
        view.farClipPlane = 50;
        EditorSceneManager.SaveScene(scene, ScenePath);
        AssetDatabase.SaveAssets();
        if (previousScene.IsValid()) SceneManager.SetActiveScene(previousScene);
        EditorSceneManager.CloseScene(scene, true);
        Debug.Log("CI_WORLD_CREATED " + ScenePath);
    }

    static Material Material(string name, Color color)
    {
        var material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        material.color = color;
        AssetDatabase.CreateAsset(material, "Assets/Materials/CI/" + name + ".mat");
        return material;
    }

    static void Box(string name, Vector3 position, Vector3 scale, Material material)
    {
        var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = name;
        box.transform.position = position;
        box.transform.localScale = scale;
        box.GetComponent<Renderer>().sharedMaterial = material;
        box.isStatic = true;
    }
}

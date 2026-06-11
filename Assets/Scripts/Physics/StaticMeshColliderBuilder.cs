using UnityEngine;

public class StaticMeshColliderBuilder : MonoBehaviour
{
    [Header("Scene Collider Generation")]
    public bool buildOnStart = true;
    public bool logSummary = true;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateForLoadedScene()
    {
        if (FindObjectOfType<StaticMeshColliderBuilder>() != null)
        {
            return;
        }

        GameObject builder = new GameObject("Static Mesh Collider Builder");
        builder.AddComponent<StaticMeshColliderBuilder>();
    }

    void Start()
    {
        if (buildOnStart)
        {
            BuildStaticMeshColliders();
        }
    }

    public void BuildStaticMeshColliders()
    {
        int added = 0;
        MeshFilter[] meshFilters = FindObjectsOfType<MeshFilter>();

        foreach (MeshFilter meshFilter in meshFilters)
        {
            if (!ShouldAddCollider(meshFilter))
            {
                continue;
            }

            MeshCollider collider = meshFilter.gameObject.AddComponent<MeshCollider>();
            collider.sharedMesh = meshFilter.sharedMesh;
            collider.convex = false;
            added++;
        }

        if (logSummary)
        {
            Debug.Log($"[StaticMeshColliderBuilder] Added {added} mesh colliders for lidar raycasts.");
        }
    }

    bool ShouldAddCollider(MeshFilter meshFilter)
    {
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            return false;
        }

        GameObject target = meshFilter.gameObject;

        if (!target.activeInHierarchy || target.GetComponent<Collider>() != null)
        {
            return false;
        }

        Renderer renderer = target.GetComponent<Renderer>();
        if (renderer == null || !renderer.enabled)
        {
            return false;
        }

        // The rover already has wheel/body colliders. Adding mesh colliders under
        // a Rigidbody creates noisy self-hits for the simulated lidar.
        if (target.GetComponentInParent<Rigidbody>() != null)
        {
            return false;
        }

        return true;
    }
}

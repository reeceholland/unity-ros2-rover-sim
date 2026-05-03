using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using System.Collections;

[RequireComponent(typeof(Camera))]
public class UnityImagePublisher : MonoBehaviour
{
    public string imageTopic = "/camera/image/compressed";
    public string frameId = "camera_link";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float publishRate = 10f;
    public int jpegQuality = 75;
    public bool logDebugMessages = true;
    public bool publishTestPattern = false;

    [Header("Debug")]
    public bool drawCameraForwardRay = true;
    public float forwardRayLength = 2f;

    ROSConnection ros;
    Camera cameraSensor;
    RenderTexture renderTexture;
    Texture2D readTexture;
    float lastPublishTime;
    bool loggedFirstPublish;
    bool isPublishing;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(imageTopic);

        cameraSensor = GetComponent<Camera>();
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        readTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        cameraSensor.targetTexture = renderTexture;

        if (logDebugMessages)
        {
            Debug.Log($"UnityImagePublisher registered {imageTopic} ({imageWidth}x{imageHeight})");
            Debug.Log($"UnityImagePublisher camera world pose: position={transform.position}, rotation={transform.rotation.eulerAngles}");
        }
    }

    void Update()
    {
        if (drawCameraForwardRay)
        {
            Debug.DrawRay(transform.position, transform.forward * forwardRayLength, Color.magenta);
        }

        if (isPublishing || Time.time - lastPublishTime < 1f / publishRate)
        {
            return;
        }

        lastPublishTime = Time.time;
        StartCoroutine(PublishImageAtEndOfFrame());
    }

    IEnumerator PublishImageAtEndOfFrame()
    {
        isPublishing = true;
        yield return new WaitForEndOfFrame();
        PublishImage();
        isPublishing = false;
    }

    void PublishImage()
    {
        if (ros == null || cameraSensor == null || renderTexture == null || readTexture == null)
        {
            Debug.LogWarning("UnityImagePublisher is not initialized yet.");
            return;
        }

        byte[] jpegBytes;

        if (publishTestPattern)
        {
            FillTestPattern();
            jpegBytes = readTexture.EncodeToJPG(jpegQuality);
        }
        else
        {
            RenderTexture previousActive = RenderTexture.active;
            RenderTexture previousTarget = cameraSensor.targetTexture;

            cameraSensor.targetTexture = renderTexture;
            RenderTexture.active = renderTexture;
            GL.Clear(true, true, cameraSensor.backgroundColor);
            cameraSensor.Render();

            readTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
            readTexture.Apply();

            RenderTexture.active = previousActive;
            cameraSensor.targetTexture = previousTarget;

            jpegBytes = readTexture.EncodeToJPG(jpegQuality);
        }

        ros.Publish(imageTopic, new CompressedImageMsg(
            MakeHeader(),
            "jpeg",
            jpegBytes));

        if (logDebugMessages && !loggedFirstPublish)
        {
            loggedFirstPublish = true;
            Debug.Log($"UnityImagePublisher published first frame on {imageTopic}: {jpegBytes.Length} bytes");
        }
    }

    void FillTestPattern()
    {
        for (int y = 0; y < imageHeight; y++)
        {
            for (int x = 0; x < imageWidth; x++)
            {
                float u = (float)x / Mathf.Max(1, imageWidth - 1);
                float v = (float)y / Mathf.Max(1, imageHeight - 1);
                readTexture.SetPixel(x, y, new Color(u, v, 1f - u));
            }
        }

        readTexture.Apply();
    }

    HeaderMsg MakeHeader()
    {
        double now = Time.realtimeSinceStartupAsDouble;
        int seconds = (int)now;
        uint nanoseconds = (uint)((now - seconds) * 1e9);

        TimeMsg stamp = new TimeMsg();
        stamp.sec = seconds;
        stamp.nanosec = nanoseconds;

        HeaderMsg header = new HeaderMsg();
        header.stamp = stamp;
        header.frame_id = frameId;
        return header;
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }

        if (cameraSensor != null)
        {
            cameraSensor.targetTexture = null;
        }
    }

    void OnDrawGizmos()
    {
        if (!drawCameraForwardRay)
        {
            return;
        }

        Gizmos.color = Color.magenta;
        Gizmos.DrawRay(transform.position, transform.forward * forwardRayLength);
        Gizmos.DrawWireSphere(transform.position + transform.forward * forwardRayLength, 0.05f);
    }
}

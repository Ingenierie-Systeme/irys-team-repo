using System;
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// Capture la caméra à intervalle régulier et publie les images
/// sur un topic ROS 2 (sensor_msgs/msg/Image) via ROS-TCP-Connector.
/// Placez ce composant sur un GameObject avec un Camera.
/// </summary>
public class CameraRosPublisher : MonoBehaviour
{
    [Header("Topic ROS")]
    [Tooltip("Nom du topic :")]
    public string topicName = "/camera/image";
    [Tooltip("Frame ID pour l'entête ROS (ex: camera_link)")]
    public string frameId = "camera_link";

    [Header("Capture")]
    [Tooltip("fréquence de capture (en Hz)")]
    public int frequence = 10;
    [Tooltip("Largeur de l'image publiée")]
    public int width = 640;
    [Tooltip("Hauteur de l'image publiée")]
    public int height = 480;

    ROSConnection ros;
    RenderTexture renderTexture;
    Texture2D readableTexture;
    float lastCaptureTime;
    bool isPublishing;
    private float captureInterval;

    // byte[] -> msg.data est byte[] natif côté ROS-TCP, pas besoin de copie ici
    ImageMsg imageMsg;

    void Start(){
        float captureInterval = 1 / frequence;
        var cam = GetComponent<Camera>();
        if (cam == null)
        {
            Debug.LogError("CameraRosPublisher doit être sur un GameObject avec une Camera.");
            enabled = false;
            return;
        }

        // Connexion ROS (paramétrable dans Unity via ROSConnectionPrefab / ROS TCP endpoint)
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // RenderTexture de travail
        renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.RGB565);
        cam.targetTexture = renderTexture;

        readableTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        }

    void Update()
    {
        if (Time.time - lastCaptureTime < captureInterval) return;
        lastCaptureTime = Time.time;
        CaptureAndPublish();
    }

    void CaptureAndPublish()
    {
        if (isPublishing) return;
        isPublishing = true;

        // Rendu de la caméra dans la RenderTexture (fait automatiquement par Unity
        // puisque targetTexture est assigné), puis lecture des pixels.
        var prev = RenderTexture.active;
        RenderTexture.active = renderTexture;
        readableTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        readableTexture.Apply();
        RenderTexture.active = prev;

        byte[] rawBytes = readableTexture.GetRawTextureData();

        // sensor_msgs/msg/Image : encodage RGB8
        // Unity rend "à l'envers" (origine en bas à gauche), on inverse la ligne
        // en publiant height par ligne pour rester simple : step = width * 3.
        // Pour un vrai repère ROS (origine en haut à gauche), inverser les lignes
        // avec FlipVertical() ou paramétrer le subscriber.

        imageMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId
            },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(width * 3),
            // ROS s'attend à un tableau row-major : Unity fournit la texture inversée,
            // on retourne les lignes pour respecter la convention ROS.
            data = FlipVertical(rawBytes, width, height, 3)
        };

        ros.Publish(topicName, imageMsg);
        isPublishing = false;
    }

    static byte[] FlipVertical(byte[] src, int w, int h, int bytesPerPixel)
    {
        int rowLen = w * bytesPerPixel;
        var dst = new byte[src.Length];
        for (int y = 0; y < h; y++)
            Array.Copy(src, y * rowLen, dst, (h - 1 - y) * rowLen, rowLen);
        return dst;
    }

    void OnDestroy()
    {
        if (renderTexture != null) Destroy(renderTexture);
        if (readableTexture != null) Destroy(readableTexture);
    }
}
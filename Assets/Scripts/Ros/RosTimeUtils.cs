using RosMessageTypes.BuiltinInterfaces;
using System;
using UnityEngine;

public static class RosTimeUtils
{
    public static TimeMsg Now()
    {
        float unityTime = Time.time;

        int sec = Mathf.FloorToInt(unityTime);
        uint nanosec = (uint)((unityTime - sec) * 1_000_000_000f);

        return new TimeMsg(sec, nanosec);
    }
}

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Extensions
{
    /// <summary>
    /// Returns true if LayerMask mask contains int layer.
    /// </summary>
    /// <param name="mask"></param>
    /// <param name="layer"></param>
    /// <returns>Boolean</returns>
    public static bool Contains(this LayerMask mask, int layer)
    {
        return mask == (mask | (1 << layer));
    }

    /// <summary>
    /// Maps float value between float in_min and float in_max to the corresponding value in range between float out_min and float out_max.
    /// </summary>
    /// <param name="in_min"></param>
    /// <param name="in_max"></param>
    /// <param name="out_min"></param>
    /// <param name="out_max"></param>
    /// <returns>float</returns>
    public static float Map(this float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}

public static class Run
{
    /// <summary>
    /// Provides an IEnumerator to use with StartCoroutine that executes Action action after float time in seconds.
    /// </summary>
    /// <param name="time"></param>
    /// <param name="action"></param>
    /// <returns>IEnumerator</returns>
    public static IEnumerator Delayed(float time, Action action)
    {
        yield return new WaitForSeconds(time);
        action();
    }
}

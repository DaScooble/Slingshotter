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

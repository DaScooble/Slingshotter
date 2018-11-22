using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Run
{
    public static IEnumerator Delayed(float time, Action action)
    {
        yield return new WaitForSeconds(time);
        action();
    }
}

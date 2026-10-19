// Walls in the scene (ADR 0009): plan-view polylines, ISO world (x, y), that
// the car's footprint cannot cross. Every TwinCar hands every TwinWalls in the
// scene to rc_twin when it wakes. The scene builders fill these wherever a
// face is taller than the ground query's climbable step; colliders are not
// scanned for them.
using System;
using System.Collections.Generic;
using UnityEngine;

namespace RcTwin
{
    public class TwinWalls : MonoBehaviour
    {
        [Serializable]
        public class Wall
        {
            public string name;
            [Tooltip("ISO world (x, y) [m], in order.")]
            public Vector2[] points;
            [Tooltip("Join the last point back to the first.")]
            public bool closed;
        }

        public List<Wall> walls = new List<Wall>();

        [Tooltip("Height the walls are drawn at in the Scene view [m].")]
        public float gizmoHeight = 0.05f;

        public void Add(string name, bool closed, params Vector2[] points) =>
            walls.Add(new Wall { name = name, points = points, closed = closed });

        /// <summary>One wall's points as rc_twin wants them: x, y pairs.</summary>
        public static double[] Flatten(Wall w)
        {
            var xy = new double[2 * w.points.Length];
            for (int i = 0; i < w.points.Length; i++)
            {
                xy[2 * i] = w.points[i].x;
                xy[2 * i + 1] = w.points[i].y;
            }
            return xy;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = new Color(1f, 0.3f, 0.2f);
            foreach (var w in walls)
            {
                if (w.points == null || w.points.Length < 2) continue;
                int n = w.closed ? w.points.Length : w.points.Length - 1;
                for (int i = 0; i < n; i++)
                {
                    Vector2 a = w.points[i], b = w.points[(i + 1) % w.points.Length];
                    Gizmos.DrawLine(TwinCar.ToUnity(a.x, a.y, gizmoHeight), TwinCar.ToUnity(b.x, b.y, gizmoHeight));
                }
            }
        }
    }
}

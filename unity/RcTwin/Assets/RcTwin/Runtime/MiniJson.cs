// Just enough JSON for the twin's own files (campus/campus_network.json,
// the replay's drives.json): objects, arrays, numbers, strings, true, false,
// null. Objects parse to Dictionary<string, object>, arrays to List<object>,
// numbers to double.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace RcTwin
{
    public static class MiniJson
    {
        public static object Parse(string s)
        {
            int i = 0;
            object v = Value(s, ref i);
            Ws(s, ref i);
            if (i != s.Length) throw new FormatException($"trailing characters at {i}");
            return v;
        }

        private static void Ws(string s, ref int i)
        {
            while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
        }

        private static object Value(string s, ref int i)
        {
            Ws(s, ref i);
            if (i >= s.Length) throw new FormatException("unexpected end");
            char c = s[i];
            if (c == '{')
            {
                var o = new Dictionary<string, object>();
                i++; Ws(s, ref i);
                if (s[i] == '}') { i++; return o; }
                while (true)
                {
                    Ws(s, ref i);
                    string k = Str(s, ref i);
                    Ws(s, ref i);
                    Expect(s, ref i, ':');
                    o[k] = Value(s, ref i);
                    Ws(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    Expect(s, ref i, '}');
                    return o;
                }
            }
            if (c == '[')
            {
                var a = new List<object>();
                i++; Ws(s, ref i);
                if (s[i] == ']') { i++; return a; }
                while (true)
                {
                    a.Add(Value(s, ref i));
                    Ws(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    Expect(s, ref i, ']');
                    return a;
                }
            }
            if (c == '"') return Str(s, ref i);
            if (string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return true; }
            if (string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return false; }
            if (string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
            int start = i;
            while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
            if (i == start) throw new FormatException($"unexpected '{c}' at {i}");
            return double.Parse(s.Substring(start, i - start), NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        private static void Expect(string s, ref int i, char c)
        {
            if (i >= s.Length || s[i] != c) throw new FormatException($"expected '{c}' at {i}");
            i++;
        }

        private static string Str(string s, ref int i)
        {
            Expect(s, ref i, '"');
            var sb = new StringBuilder();
            while (s[i] != '"')
            {
                if (s[i] == '\\')
                {
                    i++;
                    char e = s[i];
                    if (e == 'u') { sb.Append((char)Convert.ToInt32(s.Substring(i + 1, 4), 16)); i += 4; }
                    else sb.Append(e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e == 'b' ? '\b' : e == 'f' ? '\f' : e);
                }
                else sb.Append(s[i]);
                i++;
            }
            i++;
            return sb.ToString();
        }
    }
}

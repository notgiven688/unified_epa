namespace GJKEPADemo;

public struct MinkowskiDifference
{
    public ISupportMappable SupportA, SupportB;
    public JMatrix OrientationA, OrientationB;
    public JVector PositionA, PositionB;

    private void SupportMapTransformedA(in JVector direction, out JVector result)
    {
        JVector.TransposedTransform(direction, OrientationA, out JVector tmp);
        SupportA.SupportMapping(tmp, out result);
        JVector.Transform(result, OrientationA, out result);
        JVector.Add(result, PositionA, out result);
    }

    private void SupportMapTransformedB(in JVector direction, out JVector result)
    {
        JVector.TransposedTransform(direction, OrientationB, out JVector tmp);
        SupportB.SupportMapping(tmp, out result);
        JVector.Transform(result, OrientationB, out result);
        JVector.Add(result, PositionB, out result);
    }

    public void Support(in JVector direction, out JVector vA, out JVector vB, out JVector v)
    {
        // Calculates the support function S_{A-B}(d) = S_{A}(d) - S_{B}(-d),
        // where 'd' represents the direction.
        SupportMapTransformedA(direction, out vA);
        SupportMapTransformedB(-direction, out vB);
        JVector.Subtract(vA, vB, out v);
    }

    public void SupportMapping(in JVector direction, out JVector result)
    {
        this.Support(direction, out _, out _, out result);
    }

    public void SupportCenter(out JVector center)
    {
        JVector.Subtract(PositionA, PositionB, out center);
    }
}

using System;

namespace CubSim
{
    public interface IC1ScanSource
    {
        C1Scan LatestScan { get; }
        event Action<C1Scan> ScanReady;
    }
}

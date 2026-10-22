// A student's ride request, from one campus stop to another.
namespace RcTwin
{
    public enum RequestState { Waiting, Assigned, Riding, Delivered, GaveUp }

    public sealed class RideRequest
    {
        public int Id;
        public string Origin, Destination;
        public double RaisedAt;                    // [s] game time
        public double PickedUpAt = double.NaN, DroppedAt = double.NaN;
        public RequestState State = RequestState.Waiting;
        public bool UserRaised;

        public override string ToString() => $"#{Id} {Origin}→{Destination} {State}";
    }
}

package OSMDataManager;

public class NetworkNode {
    private long networkNodeId;
    private long roadOsmId;
    private String roadName;
    private double networkNodeLongitude;
    private double networkNodeLatitude;

    public NetworkNode(long networkNodeId, long roadOsmId, String roadName, double networkNodeLongitude,
                       double networkNodeLatitude) {
        this.networkNodeId = networkNodeId;
        this.roadOsmId = roadOsmId;
        this.roadName = roadName;
        this.networkNodeLongitude = networkNodeLongitude;
        this.networkNodeLatitude = networkNodeLatitude;
    }

    public long getNetworkNodeId() {
        return this.networkNodeId;
    }

    public long getRoadOsmId() {
        return this.roadOsmId;
    }

    public String getRoadName() {
        return this.roadName;
    }

    public double getNetworkNodeLongitude() {
        return this.networkNodeLongitude;
    }

    public double getNetworkNodeLatitude() {
        return this.networkNodeLatitude;
    }

    @Override
    public String toString() {
        return ("Network node details:\n" +
                "Network node ID: " + this.networkNodeId + ", " + "OSM Road ID: " + this.roadOsmId + ", " +
                "Road name: " + this.roadName + ", " + "Network node longitude: " + this.networkNodeLongitude + ", " +
                "Network node latitude: " + this.networkNodeLatitude);
    }
}
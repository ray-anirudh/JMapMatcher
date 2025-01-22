package GPSDataManager;

import OSMDataManager.NetworkNode;

import java.util.ArrayList;

public class GPSNode {
    private long gPSNodeId;
    private double gPSNodeLongitude;
    private double gPSNodeLatitude;
    private String dateTimeStamp;
    private ArrayList<NetworkNode> candidateStateNodes;
    private ArrayList<Double> emissionProbabilities;
    private double matchedNodeLongitude;
    private double matchedNodeLatitude;
    private double matchedNodeElevationM;
    private double distanceFromPreviousPointM;
    private Long matchedNodeOsmWayId;
    String gPSNodeAtDepot;
    String gPSNodeAtDepotLag;
    String gPSNodeAtDepotNext;
    String gPSNodeHubStay;
    String gPSNodeHubStart;
    String gPSNodeHubStop;
    int gPSNodeTourNumber;
    String gPSNodeIsStop;
    double jointProbability;

    GPSNode(long gPSNodeId, double gPSNodeLongitude, double gPSNodeLatitude, String dateTimeStamp,
            String gPSNodeAtDepot, String gPSNodeAtDepotLag, String gPSNodeAtDepotNext, String gPSNodeHubStay,
            String gPSNodeHubStart, String gPSNodeHubStop, int gPSNodeTourNumber, String gPSNodeIsStop) {
        this.gPSNodeId = gPSNodeId;
        this.gPSNodeLongitude = gPSNodeLongitude;
        this.gPSNodeLatitude = gPSNodeLatitude;
        this.dateTimeStamp = dateTimeStamp;
        this.candidateStateNodes = new ArrayList<>();
        this.gPSNodeAtDepot = gPSNodeAtDepot;
        this.gPSNodeAtDepotLag = gPSNodeAtDepotLag;
        this.gPSNodeAtDepotNext = gPSNodeAtDepotNext;
        this.gPSNodeHubStay = gPSNodeHubStay;
        this.gPSNodeHubStart = gPSNodeHubStart;
        this.gPSNodeHubStop = gPSNodeHubStop;
        this.gPSNodeTourNumber = gPSNodeTourNumber;
        this.gPSNodeIsStop = gPSNodeIsStop;
    }

    public void setEmissionProbabilities(ArrayList<Double> emissionProbabilities) {
        this.emissionProbabilities = emissionProbabilities;
    }
    public void setMatchedNodeLongitude(double matchedNodeLongitude) {
        this.matchedNodeLongitude = matchedNodeLongitude;
    }
    public void setMatchedNodeElevationM(double matchedNodeElevationM) {
        this.matchedNodeElevationM = matchedNodeElevationM;
    }
    public void setDistanceFromPreviousGPSPointM(double distanceFromPreviousPointM) {
        this.distanceFromPreviousPointM = distanceFromPreviousPointM;
    }
    public void setMatchedNodeOsmWayId(Long matchedNodeOsmWayId) {
        this.matchedNodeOsmWayId = matchedNodeOsmWayId;
    }
    public void setMatchedNodeLatitude(double matchedNodeLatitude) {
        this.matchedNodeLatitude = matchedNodeLatitude;
    }
    public void setJointProbability(double jointProbability) { this.jointProbability = jointProbability; }

    public long getGPSNodeId() {
        return this.gPSNodeId;
    }
    public double getGPSNodeLongitude() {
        return this.gPSNodeLongitude;
    }
    public double getGPSNodeLatitude() {
        return this.gPSNodeLatitude;
    }
    public String getDateTimeStamp() {
        return this.dateTimeStamp;
    }
    public ArrayList<NetworkNode> getCandidateStateNodes() {
        return this.candidateStateNodes;
    }
    public ArrayList<Double> getEmissionProbabilities() {
        return this.emissionProbabilities;
    }
    public double getMatchedNodeLongitude() {
        return this.matchedNodeLongitude;
    }
    public double getMatchedNodeLatitude() {
        return this.matchedNodeLatitude;
    }
    public double getMatchedNodeElevationM() {
        return this.matchedNodeElevationM;
    }
    public double getDistanceFromPreviousPointM() {
        return this.distanceFromPreviousPointM;
    }
    public Long getMatchedNodeOsmWayId() {
        return this.matchedNodeOsmWayId;
    }
    public String getgPSNodeAtDepot() {
        return gPSNodeAtDepot;
    }
    public String getgPSNodeAtDepotLag() {
        return gPSNodeAtDepotLag;
    }
    public String getgPSNodeAtDepotNext() {
        return gPSNodeAtDepotNext;
    }
    public String getgPSNodeHubStay() {
        return gPSNodeHubStay;
    }
    public String getgPSNodeHubStart() {
        return gPSNodeHubStart;
    }
    public String getgPSNodeHubStop() {
        return gPSNodeHubStop;
    }
    public int getgPSNodeTourNumber() {
        return gPSNodeTourNumber;
    }
    public String getgPSNodeIsStop() {
        return gPSNodeIsStop;
    }
    public double getJointProbability() { return jointProbability; }

    @Override
    public String toString() {
        return ("GPS node details:\n" +
                "GPS node ID: " + this.gPSNodeId + "," + this.gPSNodeLongitude + "," + this.gPSNodeLatitude + "," +
                this.dateTimeStamp + "," + this.candidateStateNodes + "," + this.gPSNodeAtDepot + "," +
                this.gPSNodeAtDepotLag + "," + this.gPSNodeAtDepotNext + "," + this.gPSNodeHubStay + "," +
                this.gPSNodeHubStart + "," + this.gPSNodeHubStop + "," + this.gPSNodeTourNumber + "," +
                this.gPSNodeIsStop);
    }
}
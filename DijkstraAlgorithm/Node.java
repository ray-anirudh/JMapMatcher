/**
 * Author: Anirudh Ray
 * Institution: Professorship of Traffic Engineering and Control, Technical University of Munich
 * Department: Mobility Systems Engineering, School of Engineering and Design
 * E-mail Address: Anirudh.Ray@tum.de
 * Purpose: Component of a Java-based multi-modal routing algorithm, built using RAPTOR, Dijkstra-algorithm, and
 * KD-Trees
 */

package DijkstraAlgorithm;

import java.util.ArrayList;

public class Node {    // Node IDs are present in the relevant hashmap, as well
    private long nodeId;
    private double nodeLongitude;
    private double nodeLatitude;
    private final ArrayList<Long> linkIdList;
    private long predecessorNodeId;
    private long previousGPSNodeId;
    private double previousGPSNodeLongitude;
    private double previousGPSNodeLatitude;
    private long nextGPSNodeId;
    private double nextGPSNodeLongitude;
    private double nextGPSNodeLatitude;
    private int tourNumber;
    private String previousGPSNodeDateTime;
    private String nextGPSNodeDateTime;
    private String previousGPSNodeAtDepot;
    private String previousGPSNodeAtDepotLag;
    private String previousGPSNodeAtDepotNext;
    private String previousGPSNodeHubStay;
    private String previousGPSNodeHubStart;
    private String previousGPSNodeHubStop;
    private String previousGPSNodeIsStop;
    private long osmWayIdAscribedForRoute;
    private double distanceToPreviousNetworkNode;
    private boolean isIntersection;

    public Node(ArrayList<Long> linkIdList) {
        this.linkIdList = linkIdList;
    }

    public double equiRectangularDistanceTo(double otherPointLongitude, double otherPointLatitude) {
        final int EARTH_RADIUS_M = 6_371_000;
        double longitudeDifference = Math.toRadians(this.nodeLongitude - otherPointLongitude);
        double latitudeDifference = Math.toRadians(this.nodeLatitude - otherPointLatitude);

        double x = longitudeDifference * Math.cos(Math.toRadians((this.nodeLatitude + otherPointLatitude) / 2));
        return EARTH_RADIUS_M * (Math.sqrt(x * x + latitudeDifference * latitudeDifference));
    }

    void setNodeId(long nodeId) {
        this.nodeId = nodeId;
    }
    void setNodeLongitude(double nodeLongitude) {
        this.nodeLongitude = nodeLongitude;
    }
    void setNodeLatitude(double nodeLatitude) {
        this.nodeLatitude = nodeLatitude;
    }
    void setPredecessorNodeId(long predecessorNodeId) {
        this.predecessorNodeId = predecessorNodeId;
    }
    public void setPreviousGPSNodeId(long previousGPSNodeId) {
        this.previousGPSNodeId = previousGPSNodeId;
    }
    public void setNextGPSNodeId(long nextGPSNodeId) {
        this.nextGPSNodeId = nextGPSNodeId;
    }
    public void setTourNumber(int tourNumber) {
        this.tourNumber = tourNumber;
    }

    public double getPreviousGPSNodeLongitude() {
        return previousGPSNodeLongitude;
    }

    public void setPreviousGPSNodeLongitude(double previousGPSNodeLongitude) {
        this.previousGPSNodeLongitude = previousGPSNodeLongitude;
    }

    public double getPreviousGPSNodeLatitude() {
        return previousGPSNodeLatitude;
    }

    public void setPreviousGPSNodeLatitude(double previousGPSNodeLatitude) {
        this.previousGPSNodeLatitude = previousGPSNodeLatitude;
    }

    public double getNextGPSNodeLongitude() {
        return nextGPSNodeLongitude;
    }

    public void setNextGPSNodeLongitude(double nextGPSNodeLongitude) {
        this.nextGPSNodeLongitude = nextGPSNodeLongitude;
    }

    public double getNextGPSNodeLatitude() {
        return nextGPSNodeLatitude;
    }

    public void setNextGPSNodeLatitude(double nextGPSNodeLatitude) {
        this.nextGPSNodeLatitude = nextGPSNodeLatitude;
    }

    public int getTourNumber() {
        return tourNumber;
    }

    public boolean isIntersection() {
        return this.isIntersection;
    }

    public void setIsIntersection(boolean isIntersection) {
        this.isIntersection = isIntersection;
    }

    public long getNodeId() {
        return this.nodeId;
    }
    public double getNodeLongitude() {
        return this.nodeLongitude;
    }
    public double getNodeLatitude() {
        return this.nodeLatitude;
    }
    public ArrayList<Long> getLinkIdList() {
        return this.linkIdList;
    }
    public long getPredecessorNodeId() {
        return this.predecessorNodeId;
    }
    public long getPreviousGPSNodeId() {
        return this.previousGPSNodeId;
    }
    public long getNextGPSNodeId() {
        return this.nextGPSNodeId;
    }

    public String getPreviousGPSNodeDateTime() {
        return previousGPSNodeDateTime;
    }

    public void setPreviousGPSNodeDateTime(String previousGPSNodeDateTime) {
        this.previousGPSNodeDateTime = previousGPSNodeDateTime;
    }

    public String getNextGPSNodeDateTime() {
        return nextGPSNodeDateTime;
    }

    public void setNextGPSNodeDateTime(String nextGPSNodeDateTime) {
        this.nextGPSNodeDateTime = nextGPSNodeDateTime;
    }

    public String getPreviousGPSNodeAtDepot() {
        return previousGPSNodeAtDepot;
    }

    public void setPreviousGPSNodeAtDepot(String previousGPSNodeAtDepot) {
        this.previousGPSNodeAtDepot = previousGPSNodeAtDepot;
    }

    public String getPreviousGPSNodeAtDepotLag() {
        return previousGPSNodeAtDepotLag;
    }

    public void setPreviousGPSNodeAtDepotLag(String previousGPSNodeAtDepotLag) {
        this.previousGPSNodeAtDepotLag = previousGPSNodeAtDepotLag;
    }

    public String getPreviousGPSNodeAtDepotNext() {
        return previousGPSNodeAtDepotNext;
    }

    public void setPreviousGPSNodeAtDepotNext(String previousGPSNodeAtDepotNext) {
        this.previousGPSNodeAtDepotNext = previousGPSNodeAtDepotNext;
    }

    public String getPreviousGPSNodeHubStay() {
        return previousGPSNodeHubStay;
    }

    public void setPreviousGPSNodeHubStay(String previousGPSNodeHubStay) {
        this.previousGPSNodeHubStay = previousGPSNodeHubStay;
    }

    public String getPreviousGPSNodeHubStart() {
        return previousGPSNodeHubStart;
    }

    public void setPreviousGPSNodeHubStart(String previousGPSNodeHubStart) {
        this.previousGPSNodeHubStart = previousGPSNodeHubStart;
    }

    public String getPreviousGPSNodeHubStop() {
        return previousGPSNodeHubStop;
    }

    public void setPreviousGPSNodeHubStop(String previousGPSNodeHubStop) {
        this.previousGPSNodeHubStop = previousGPSNodeHubStop;
    }

    public String getPreviousGPSNodeIsStop() {
        return previousGPSNodeIsStop;
    }

    public void setPreviousGPSNodeIsStop(String previousGPSNodeIsStop) {
        this.previousGPSNodeIsStop = previousGPSNodeIsStop;
    }

    public long getOsmWayIdAscribedForRoute() {
        return osmWayIdAscribedForRoute;
    }

    public void setOsmWayIdAscribedForRoute(long osmWayIdAscribedForRoute) {
        this.osmWayIdAscribedForRoute = osmWayIdAscribedForRoute;
    }

    public double getDistanceToPreviousNetworkNode() {
        return distanceToPreviousNetworkNode;
    }

    public void setDistanceToPreviousNetworkNode(double distanceToPreviousNetworkNode) {
        this.distanceToPreviousNetworkNode = distanceToPreviousNetworkNode;
    }

    @Override
    public String toString() {
        return ("Node ID: " + this.nodeId + ", Node Latitude: " + this.nodeLatitude + ", Node Longitude: " +
                this.nodeLatitude + ", Tour Number: " + this.tourNumber + ", Previous GPS Node ID: " +
                this.previousGPSNodeId + ", Next GPS Node ID: " + this.nextGPSNodeId);
    }
}
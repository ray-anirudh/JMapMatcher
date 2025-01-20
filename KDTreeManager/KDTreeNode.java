package KDTreeManager;

import OSMDataManager.NetworkNode;

public class KDTreeNode {   // Use for network node-based KD-Trees
    private NetworkNode networkNode;  // KD-Tree nodes are modelled on top of network nodes
    private KDTreeNode left;
    private KDTreeNode right;

    KDTreeNode(NetworkNode networkNode) {
        this.networkNode = networkNode;
        this.left = this.right = null;
    }

    public double equiRectangularDistanceTo(double otherPointLongitude, double otherPointLatitude) {
        final int EARTH_RADIUS_M = 6_371_000;

        double longitudeDifference = Math.toRadians(this.networkNode.getNetworkNodeLongitude() - otherPointLongitude);
        double latitudeDifference = Math.toRadians(this.networkNode.getNetworkNodeLatitude() - otherPointLatitude);

        double x = longitudeDifference * Math.cos(Math.toRadians((this.networkNode.getNetworkNodeLatitude() +
                otherPointLatitude) / 2));

        return EARTH_RADIUS_M * Math.sqrt(x * x + latitudeDifference * latitudeDifference);
    }

    public void setLeft(KDTreeNode left) {
        this.left = left;
    }

    public void setRight(KDTreeNode right) {
        this.right = right;
    }

    public NetworkNode getNetworkNode() {
        return this.networkNode;
    }

    public KDTreeNode getLeft() {
        return this.left;
    }

    public KDTreeNode getRight() {
        return this.right;
    }
}
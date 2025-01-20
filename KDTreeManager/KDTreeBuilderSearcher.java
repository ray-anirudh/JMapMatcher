package KDTreeManager;

import java.util.*;

import OSMDataManager.NetworkNode;

public class KDTreeBuilderSearcher {
    private KDTreeNode kDTreeRootNode;    // Represents the root (highest level) node of the tree

    /**
     * BEHAVIOUR DEFINITIONS
     * For fast nearest-neighbour searches, the below methods are executed to build and query node-based KD-Trees
     */

    private KDTreeNode buildKDTreeForNetworkNodes(NetworkNode[] networkNodes, int depth) {
        if ((networkNodes == null) || (networkNodes.length == 0)) {
            return null;
        }

        int axis = depth % 2;
        Arrays.sort(networkNodes, Comparator.comparingDouble(networkNode -> (axis == 0) ? networkNode.
                        getNetworkNodeLatitude() : networkNode.getNetworkNodeLongitude()));

        int medianIndex = networkNodes.length / 2;     // Indexing for roots of new subtrees
        KDTreeNode networkNode = new KDTreeNode(networkNodes[medianIndex]);
        // Setting up node-based roots of new subtrees

        networkNode.setLeft(buildKDTreeForNetworkNodes(Arrays.copyOfRange(networkNodes, 0, medianIndex),
                depth + 1));
        networkNode.setRight(buildKDTreeForNetworkNodes(Arrays.copyOfRange(networkNodes, medianIndex + 1,
                        networkNodes.length), depth + 1));

        return networkNode;
    }

    public void buildNodeBasedKDTree(NetworkNode[] networkNodes) {
        this.kDTreeRootNode = buildKDTreeForNetworkNodes(networkNodes, 0);
        System.out.println("KD-Tree created for network nodes");
    }

    private void nearestNeighboursSearchForNetworkNodes(double sourceLongitude, double sourceLatitude,
                                                        KDTreeNode kDTreeNode, PriorityQueue<KDTreeNode> nearestNodes,
                                                        int depth, int n) {
        if (kDTreeNode == null) {
            return;
        }

        double distance = kDTreeNode.equiRectangularDistanceTo(sourceLongitude, sourceLatitude);

        // Add current node to the priority queue if it's closer or the queue isn't full
        if (nearestNodes.size() < n) {
            nearestNodes.offer(kDTreeNode);
        } else {
            assert nearestNodes.peek() != null;
            if (distance < nearestNodes.peek().equiRectangularDistanceTo(sourceLongitude, sourceLatitude)) {
                nearestNodes.poll(); // Remove the farthest node
                nearestNodes.offer(kDTreeNode); // Add the closer node
            }
        }

        int axis = depth % 2;
        KDTreeNode nextKDTreeNode = ((axis == 0) ? (sourceLatitude < kDTreeNode.getNetworkNode().
                getNetworkNodeLatitude()) : (sourceLongitude < kDTreeNode.getNetworkNode().getNetworkNodeLongitude()))
                ? kDTreeNode.getLeft() : kDTreeNode.getRight();
        KDTreeNode otherKDTreeNode = (nextKDTreeNode == kDTreeNode.getLeft()) ? kDTreeNode.getRight() : kDTreeNode.
                getLeft();

        // Search the next node
        nearestNeighboursSearchForNetworkNodes(sourceLongitude, sourceLatitude, nextKDTreeNode, nearestNodes,
                depth + 1, n);

        // Check if we need to explore the other branch
        double axisDistance = (axis == 0) ?
                Math.abs(kDTreeNode.getNetworkNode().getNetworkNodeLatitude() - sourceLatitude) * 111_320 :
                Math.abs(kDTreeNode.getNetworkNode().getNetworkNodeLongitude() - sourceLongitude) * 111_320 *
                        Math.cos(Math.toRadians(kDTreeNode.getNetworkNode().getNetworkNodeLatitude()));

        if (nearestNodes.size() < n || axisDistance < Objects.requireNonNull(nearestNodes.peek()).
                equiRectangularDistanceTo(sourceLongitude, sourceLatitude)) {
            nearestNeighboursSearchForNetworkNodes(sourceLongitude, sourceLatitude, otherKDTreeNode, nearestNodes,
                    depth + 1, n);
        }
    }

    // Find the n nearest nodes to a source point from amongst a set of nodes
    public List<NetworkNode> findNearestNodes(double sourceLongitude, double sourceLatitude, int n) {
        if (this.kDTreeRootNode == null) {
            throw new IllegalStateException("Network node-based KD-Tree is empty.");
        }

        PriorityQueue<KDTreeNode> nearestNodes = new PriorityQueue<>(
                Comparator.comparingDouble(node -> -node.equiRectangularDistanceTo(sourceLongitude, sourceLatitude))
        );

        nearestNeighboursSearchForNetworkNodes(sourceLongitude, sourceLatitude, this.kDTreeRootNode, nearestNodes,
                0, n);

        List<NetworkNode> result = new ArrayList<>();
        while (!nearestNodes.isEmpty()) {
            result.add(nearestNodes.poll().getNetworkNode());
        }

        // Reverse to get closest first (priority queue is max-heap, so elements are in reverse order)
        Collections.reverse(result);
        return result;
    }
}